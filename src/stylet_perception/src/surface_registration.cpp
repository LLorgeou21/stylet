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
#include "pcl/common/centroid.h" // pour pcl::compute3DCentroid
#include "pcl/common/pca.h"      // pour pcl::PCA
#include "pcl/registration/gicp.h"
#include "pcl/common/transforms.h" // pour pcl::transformPointCloud
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

        // Debug visuel : nuage de reference transforme par T_final, publie dans le
        // meme repere ("world") que filtered_cloud, pour verifier a l'oeil dans RViz
        // si le recalage colle bien a l'objet observe.
        aligned_debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stylet/perception/aligned_reference_debug", 10);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/stylet/perception/target_pose", 10);
        fitness_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/stylet/perception/fitness_score", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Rejet automatique : si le fitness du meilleur candidat depasse ce seuil,
        // la pose n'est pas publiee (ni utilisee comme point de depart du suivi
        // au prochain callback) - mieux vaut ne rien publier qu'une pose fausse.
        this->declare_parameter("fitness_reject_threshold", 0.01);

        reference_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        path = ament_index_cpp::get_package_share_directory("stylet_perception") + "/config/target_reference.pcd";
        pcl::io::loadPCDFile(path, *reference_cloud);

        reference_centroid_ = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*reference_cloud, reference_centroid_);

        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(reference_cloud);
        reference_axes_ = ensureProperRotation(pca.getEigenVectors());

        RCLCPP_INFO_STREAM(this->get_logger(),
            "reference_cloud charge : " << reference_cloud->size() << " points (depuis " << path << ")");
    };

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

    // eigh/PCA ne garantit pas det=+1 (peut renvoyer une reflexion, pas une vraie
    // rotation) - on force ca en inversant le dernier axe si besoin. Sans ca, les
    // "4 combinaisons de signes valides" ci-dessous seraient en fait 4 reflexions.
    static Eigen::Matrix3f ensureProperRotation(Eigen::Matrix3f axes)
    {
        if (axes.determinant() < 0.0f)
        {
            axes.col(2) *= -1.0f;
        }
        return axes;
    }

    // Les 4 combinaisons de signes (sur 8 possibles) qui preservent une vraie
    // rotation (determinant +1) plutot qu'une reflexion (determinant -1).
    static std::vector<Eigen::Vector3f> validSignFlips()
    {
        std::vector<Eigen::Vector3f> flips;
        for (float sx : {1.0f, -1.0f})
            for (float sy : {1.0f, -1.0f})
                for (float sz : {1.0f, -1.0f})
                    if (sx * sy * sz > 0.0f)
                        flips.push_back(Eigen::Vector3f(sx, sy, sz));
        return flips; // toujours exactement 4 elements
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

    // Fitness au-dela duquel on considere que le suivi rapide (2 candidats) a
    // echoue et qu'il faut retomber sur la recherche complete (4 departs).
    // Valeur provisoire, a valider empiriquement comme le reste des seuils de ce
    // noeud (voir session du 2026-07-06).
    static constexpr double kTrackingLostFitness = 0.008;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_debug_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fitness_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string path;
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud;
    Eigen::Matrix3f reference_axes_;
    Eigen::Vector4f reference_centroid_;

    // Etat de suivi d'un callback a l'autre (scene statique pour l'instant : la
    // bonne transformation change tres peu entre deux frames).
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

    // gicp cree UNE SEULE FOIS : reference_cloud et observed_cloud ne changent
    // pas entre les tentatives, seul T_init change.
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
        // Premier callback : pas de transformation precedente, recherche complete
        // sur les 4 departs (voir ADR-023 / session du 2026-07-06).
        for (const auto &flip : validSignFlips())
        {
            try_flip(flip);
        }
        mode = "recherche complete (premier callback)";
    }
    else
    {
        // Suivi : la scene est statique, la transformation precedente est deja
        // proche de la bonne reponse. 2 candidats seulement :
        //   - reprise directe de la transformation precedente (warm start)
        //   - PCA fraiche sur le nuage courant, avec le signe qui gagnait avant
        //     (le signe "naturel" de la PCA peut varier legerement d'une frame
        //     a l'autre a cause du bruit, d'ou ce 2eme candidat independant)
        try_T_init(previous_transformation_);
        try_flip(previous_best_flip_);

        if (best_fitness > kTrackingLostFitness)
        {
            // Suivi perdu (fitness anormalement mauvais) : on retombe sur la
            // recherche complete pour se re-localiser correctement.
            for (const auto &flip : validSignFlips())
            {
                try_flip(flip);
            }
            mode = "recherche complete (suivi perdu)";
        }
        else
        {
            mode = "suivi (2 candidats)";
        }
    }

    auto end_time = std::chrono::steady_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    // Verite terrain (simulation uniquement) : la cible est placee dans
    // stylet.world a <pose>0.15 0.15 0 0 0 0</pose>, donc sans rotation. Le
    // fitness GICP brut melange precision de pose et couverture partielle du
    // nuage observe (reference complete vs vue partielle), ce qui le rend peu
    // parlant : on lui prefere ici une comparaison directe a cette pose connue
    // pour juger la qualite reelle du recalage. Le fitness reste le seul critere
    // utilise pour CHOISIR le meilleur candidat (seule info dispo hors simulation).
    const Eigen::Vector3f ground_truth_translation(0.15f, 0.15f, 0.0f);
    Eigen::Vector3f estimated_translation = best_transformation.block<3, 1>(0, 3);
    double translation_error_mm = (estimated_translation - ground_truth_translation).norm() * 1000.0;

    Eigen::Matrix3f R_est = best_transformation.block<3, 3>(0, 0);
    double cos_angle = (R_est.trace() - 1.0f) / 2.0f;
    cos_angle = std::max(-1.0, std::min(1.0, static_cast<double>(cos_angle)));
    double rotation_error_deg = std::acos(cos_angle) * 180.0 / std::acos(-1.0);

    RCLCPP_INFO_STREAM(this->get_logger(),
        "GICP has converged: " << best_converged
        << " erreur translation: " << translation_error_mm << " mm"
        << " erreur rotation: " << rotation_error_deg << " deg"
        << " mode: " << mode
        << " temps: " << elapsed_ms << " ms");

    // fitness_score toujours publie (utile pour le monitoring/debug), meme si la
    // pose finit par etre rejetee ci-dessous.
    std_msgs::msg::Float64 fitness_msg;
    fitness_msg.data = best_fitness;
    fitness_pub_->publish(fitness_msg);

    double fitness_reject_threshold = this->get_parameter("fitness_reject_threshold").as_double();
    if (best_fitness > fitness_reject_threshold)
    {
        RCLCPP_WARN_STREAM(this->get_logger(),
            "Recalage rejete (fitness " << best_fitness << " > seuil " << fitness_reject_threshold
            << ") - pose/TF non publiees, recherche complete au prochain callback");
        has_previous_transformation_ = false; // ne pas repartir d'une pose rejetee
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

    // TF world -> target_frame (ADR-014 mentionne un intermediaire patient_base,
    // pas encore implemente dans le projet - publie directement depuis world pour
    // l'instant).
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.frame_id = "world";
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.child_frame_id = "target_frame";
    tf_msg.transform.translation.x = pose_msg.pose.position.x;
    tf_msg.transform.translation.y = pose_msg.pose.position.y;
    tf_msg.transform.translation.z = pose_msg.pose.position.z;
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);

    // Publie reference_cloud transforme par best_transformation, dans le meme
    // repere que observed_cloud/filtered_cloud (world) : a comparer visuellement
    // dans RViz (2 PointCloud2, Fixed Frame = world) pour juger du recalage.
    pcl::PointCloud<pcl::PointXYZ> aligned_reference;
    pcl::transformPointCloud(*reference_cloud, aligned_reference, best_transformation);
    sensor_msgs::msg::PointCloud2 aligned_msg;
    pcl::toROSMsg(aligned_reference, aligned_msg);
    aligned_msg.header = msg->header; // meme frame_id (world) et meme stamp
    aligned_debug_pub_->publish(aligned_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurfaceRegistration>());
    rclcpp::shutdown();
    return 0;
}
