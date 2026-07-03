#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

class PointCloudMerger : public rclcpp::Node
{
public:
    PointCloudMerger() : rclcpp::Node("point_cloud_merger")
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        this->declare_parameter("sync_max_interval_ms", 50);
        int sync_max_interval_ms = this->get_parameter("sync_max_interval_ms").as_int();

        sub1_.subscribe(this, "/lidar_1/points");
        sub2_.subscribe(this, "/lidar_2/points");
        sub3_.subscribe(this, "/lidar_3/points");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub1_, sub2_, sub3_);
        sync_->registerCallback(std::bind(&PointCloudMerger::callback, this,
                                          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_max_interval_ms / 1000.0));

        merged_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stylet/perception/merged_cloud", 10);
    }

private:
    void callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg3);

    // TF2 : pour interroger les transformations (world <- capteur)
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Les 3 abonnements "spéciaux" compatibles message_filters
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub2_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub3_;

    // Le synchroniseur qui déclenche callback() quand les 3 sont alignés
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // Le publisher du nuage fusionné
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pub_;
};

void PointCloudMerger::callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg3)
{
    try
    {
        geometry_msgs::msg::TransformStamped transform1 =
            tf_buffer_->lookupTransform("world", msg1->header.frame_id, msg1->header.stamp);

        sensor_msgs::msg::PointCloud2 transformed1;
        tf2::doTransform(*msg1, transformed1, transform1);

        geometry_msgs::msg::TransformStamped transform2 =
            tf_buffer_->lookupTransform("world", msg2->header.frame_id, msg2->header.stamp);

        sensor_msgs::msg::PointCloud2 transformed2;
        tf2::doTransform(*msg2, transformed2, transform2);

        geometry_msgs::msg::TransformStamped transform3 =
            tf_buffer_->lookupTransform("world", msg3->header.frame_id, msg3->header.stamp);

        sensor_msgs::msg::PointCloud2 transformed3;
        tf2::doTransform(*msg3, transformed3, transform3);

        sensor_msgs::msg::PointCloud2 merged = transformed1; // copie l'en-tête/les champs de cloud 1
        merged.data.insert(merged.data.end(), transformed2.data.begin(), transformed2.data.end());
        merged.data.insert(merged.data.end(), transformed3.data.begin(), transformed3.data.end());

        // Il faut mettre à jour ces 2 champs pour que le nuage combiné soit valide :
        merged.height = 1;
        merged.width = transformed1.width * transformed1.height + transformed2.width * transformed2.height + transformed3.width * transformed3.height; // à toi de trouver : le nombre total de points des 3 nuages
        merged.row_step = merged.point_step * merged.width;

        merged_pub_->publish(merged);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
        return;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudMerger>());
    rclcpp::shutdown();
    return 0;
}