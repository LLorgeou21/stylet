#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"

class CloudPreprocessor : public rclcpp::Node
{
public:
    CloudPreprocessor() : rclcpp::Node("cloud_preprocessor")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/stylet/perception/merged_cloud", 10,
            std::bind(&CloudPreprocessor::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/stylet/perception/filtered_cloud", 10);
    };

private:
    void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

void CloudPreprocessor::callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PassThrough<pcl::PointXYZ> passx;
    passx.setInputCloud(cloud);
    passx.setFilterFieldName("x");
    passx.setFilterLimits(0.10, 0.8);
    passx.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> passy;
    passy.setInputCloud(cloud);
    passy.setFilterFieldName("y");
    passy.setFilterLimits(-0.10, 0.8);
    passy.filter(*cloud);

    pcl::PassThrough<pcl::PointXYZ> passz;
    passz.setInputCloud(cloud);
    passz.setFilterFieldName("z");
    passz.setFilterLimits(0.01, 1.0);
    passz.filter(*cloud);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.002f, 0.002f, 0.002f);
    voxel.filter(*cloud);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header = msg->header;
    pub_->publish(output);
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudPreprocessor>());
    rclcpp::shutdown();
    return 0;
}