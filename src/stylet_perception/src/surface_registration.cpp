#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pcl/common/centroid.h" // for pcl::compute3DCentroid
#include "pcl/common/pca.h"      // for pcl::PCA
#include "pcl/registration/gicp.h"
#include "pcl/common/transforms.h" // for pcl::transformPointCloud
#include <vector>
#include <limits>
#include <chrono>
#include <cmath>

class SurfaceRegistration : public rclcpp::Node
{
public:
    SurfaceRegistration() : rclcpp::Node("surface_registration")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/stylet/perception/filtered_cloud", 10,
            std::bind(&SurfaceRegistration::callback, this, std::placeholders::_1));

        // Visual debug: the reference cloud transformed by T_final, published
        // in the same frame ("world") as filtered_cloud, to visually check in
        // RViz whether the registration actually fits the observed object.
        aligned_debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stylet/perception/aligned_reference_debug", 10);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/stylet/perception/target_pose", 10);
        fitness_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/stylet/perception/fitness_score", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Automatic rejection: if the best candidate's fitness exceeds this
        // threshold, the pose is neither published nor used as the tracking
        // starting point for the next callback - better to publish nothing
        // than a wrong pose.
        this->declare_parameter("fitness_reject_threshold", 0.01);

        reference_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        std::string path = ament_index_cpp::get_package_share_directory("stylet_perception") + "/config/target_reference.pcd";
        pcl::io::loadPCDFile(path, *reference_cloud);

        reference_centroid_ = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*reference_cloud, reference_centroid_);

        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(reference_cloud);
        reference_axes_ = ensureProperRotation(pca.getEigenVectors());

        RCLCPP_INFO_STREAM(this->get_logger(),
            "reference_cloud loaded: " << reference_cloud->size() << " points (from " << path << ")");
    };

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    // eigh/PCA doesn't guarantee det=+1 (it can return a reflection, not a
    // true rotation) - this forces it by flipping the last axis if needed.
    // Without this, the "4 valid sign combinations" below would actually be
    // 4 reflections.
    static Eigen::Matrix3f ensureProperRotation(Eigen::Matrix3f axes)
    {
        if (axes.determinant() < 0.0f)
        {
            axes.col(2) *= -1.0f;
        }
        return axes;
    }

    // The 4 sign combinations (out of 8 possible) that preserve a true
    // rotation (determinant +1) rather than a reflection (determinant -1).
    static std::vector<Eigen::Vector3f> validSignFlips()
    {
        std::vector<Eigen::Vector3f> flips;
        for (float sx : {1.0f, -1.0f})
            for (float sy : {1.0f, -1.0f})
                for (float sz : {1.0f, -1.0f})
                    if (sx * sy * sz > 0.0f)
                        flips.push_back(Eigen::Vector3f(sx, sy, sz));
        return flips; // always exactly 4 elements
    }

    static Eigen::Matrix4f transformFromFlip(
        const Eigen::Vector3f &flip, const Eigen::Matrix3f &observed_axes,
        const Eigen::Matrix3f &reference_axes, const Eigen::Vector4f &observed_centroid,
        const Eigen::Vector4f &reference_centroid)
    {
        Eigen::Matrix3f candidate_axes = observed_axes;
        candidate_axes.col(0) *= flip[0];
        candidate_axes.col(1) *= flip[1];
        candidate_axes.col(2) *= flip[2];

        Eigen::Matrix3f R = candidate_axes * reference_axes.transpose();
        Eigen::Vector3f t = observed_centroid.head<3>() - R * reference_centroid.head<3>();

        Eigen::Matrix4f T_init = Eigen::Matrix4f::Identity();
        T_init.block<3, 3>(0, 0) = R;
        T_init.block<3, 1>(0, 3) = t;
        return T_init;
    }

    // Fitness beyond which fast tracking (2 candidates) is considered to have
    // failed, falling back to the full search (4 starts). Provisional value,
    // to be validated empirically like the rest of this node's thresholds.
    static constexpr double kTrackingLostFitness = 0.008;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
    Eigen::Matrix3f reference_axes_;
    Eigen::Vector4f reference_centroid_;

    // Tracking state carried from one callback to the next (static scene for
    // now: the correct transform changes very little between two frames).
    bool has_previous_transformation_ = false;
    Eigen::Matrix4f previous_transformation_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f previous_best_flip_ = Eigen::Vector3f(1.0f, 1.0f, 1.0f);
};

void SurfaceRegistration::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    auto start_time = std::chrono::steady_clock::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr observed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *observed_cloud);

    Eigen::Vector4f observed_centroid;
    pcl::compute3DCentroid(*observed_cloud, observed_centroid);

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(observed_cloud);
    Eigen::Matrix3f observed_axes = ensureProperRotation(pca.getEigenVectors());

    bool best_converged = false;
    double best_fitness = std::numeric_limits<double>::max();
    Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();
    Eigen::Vector3f best_flip = previous_best_flip_;

    // gicp created ONCE: reference_cloud and observed_cloud don't change
    // between attempts, only T_init does.
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(reference_cloud);
    gicp.setInputTarget(observed_cloud);
    gicp.setMaxCorrespondenceDistance(0.01);
    gicp.setMaximumIterations(10);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    auto try_T_init = [&](const Eigen::Matrix4f &T_init)
    {
        pcl::PointCloud<pcl::PointXYZ> aligned;
        gicp.align(aligned, T_init);
        double fitness = gicp.getFitnessScore();
        if (fitness < best_fitness)
        {
            best_fitness = fitness;
            best_converged = gicp.hasConverged();
            best_transformation = gicp.getFinalTransformation();
        }
        return fitness;
    };

    auto try_flip = [&](const Eigen::Vector3f &flip)
    {
        Eigen::Matrix4f T_init = transformFromFlip(
            flip, observed_axes, reference_axes_, observed_centroid, reference_centroid_);
        double fitness_before = best_fitness;
        double fitness = try_T_init(T_init);
        if (fitness < fitness_before)
        {
            best_flip = flip;
        }
    };

    std::string mode;
    if (!has_previous_transformation_)
    {
        // First callback: no previous transform, full search across all 4
        // starts (ADR-023/ADR-024).
        for (const auto &flip : validSignFlips())
        {
            try_flip(flip);
        }
        mode = "full search (first callback)";
    }
    else
    {
        // Tracking: the scene is static, the previous transform is already
        // close to the right answer. Only 2 candidates:
        //   - direct reuse of the previous transform (warm start)
        //   - fresh PCA on the current cloud, with the sign that won last
        //     time (PCA's "natural" sign can vary slightly frame to frame
        //     due to noise, hence this 2nd, independent candidate)
        try_T_init(previous_transformation_);
        try_flip(previous_best_flip_);

        if (best_fitness > kTrackingLostFitness)
        {
            // Tracking lost (abnormally bad fitness): fall back to the full
            // search to relocalize correctly.
            for (const auto &flip : validSignFlips())
            {
                try_flip(flip);
            }
            mode = "full search (tracking lost)";
        }
        else
        {
            mode = "tracking (2 candidates)";
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Ground truth (simulation only): the target is placed in stylet.world at
    // <pose>0.15 0.15 0 0 0 0</pose>, so with no rotation. The raw GICP
    // fitness conflates pose accuracy with the observed cloud's partial
    // coverage (full reference vs. partial view), which makes it hard to
    // interpret on its own - a direct comparison against this known pose is
    // used instead to judge the actual registration quality. Fitness remains
    // the only criterion used to CHOOSE the best candidate (the only
    // information available outside simulation).
    const Eigen::Vector3f ground_truth_translation(0.15f, 0.15f, 0.0f);
    Eigen::Vector3f estimated_translation = best_transformation.block<3, 1>(0, 3);
    double translation_error_mm = (estimated_translation - ground_truth_translation).norm() * 1000.0;

    Eigen::Matrix3f R_est = best_transformation.block<3, 3>(0, 0);
    double cos_angle = (R_est.trace() - 1.0f) / 2.0f;
    cos_angle = std::max(-1.0, std::min(1.0, static_cast<double>(cos_angle)));
    double rotation_error_deg = std::acos(cos_angle) * 180.0 / std::acos(-1.0);

    RCLCPP_INFO_STREAM(this->get_logger(),
        "GICP has converged: " << best_converged
        << " translation error: " << translation_error_mm << " mm"
        << " rotation error: " << rotation_error_deg << " deg"
        << " mode: " << mode
        << " time: " << elapsed_ms << " ms");

    // fitness_score always published (useful for monitoring/debugging), even
    // if the pose ends up rejected below.
    std_msgs::msg::Float64 fitness_msg;
    fitness_msg.data = best_fitness;
    fitness_pub_->publish(fitness_msg);

    double fitness_reject_threshold = this->get_parameter("fitness_reject_threshold").as_double();
    if (best_fitness > fitness_reject_threshold)
    {
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Registration rejected (fitness " << best_fitness << " > threshold " << fitness_reject_threshold
            << ") - pose/TF not published, full search on next callback");
        has_previous_transformation_ = false; // don't restart from a rejected pose
        return;
    }

    has_previous_transformation_ = true;
    previous_transformation_ = best_transformation;
    previous_best_flip_ = best_flip;

    Eigen::Quaternionf orientation(R_est);
    orientation.normalize();

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = msg->header.stamp;
    pose_msg.pose.position.x = best_transformation(0, 3);
    pose_msg.pose.position.y = best_transformation(1, 3);
    pose_msg.pose.position.z = best_transformation(2, 3);
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    pose_pub_->publish(pose_msg);

    // TF world -> target_frame (ADR-014 mentions an intermediate patient_base
    // frame, not yet implemented in this project - published directly from
    // world for now).
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "world";
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.child_frame_id = "target_frame";
    tf_msg.transform.translation.x = pose_msg.pose.position.x;
    tf_msg.transform.translation.y = pose_msg.pose.position.y;
    tf_msg.transform.translation.z = pose_msg.pose.position.z;
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);

    // Publishes reference_cloud transformed by best_transformation, in the
    // same frame as observed_cloud/filtered_cloud (world): meant to be
    // compared visually in RViz (2 PointCloud2 displays, Fixed Frame = world)
    // to judge the registration.
    pcl::PointCloud<pcl::PointXYZ> aligned_reference;
    pcl::transformPointCloud(*reference_cloud, aligned_reference, best_transformation);
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned_reference, aligned_msg);
    aligned_msg.header = msg->header; // same frame_id (world) and same stamp
    aligned_debug_pub_->publish(aligned_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurfaceRegistration>());
    rclcpp::shutdown();
    return 0;
}
