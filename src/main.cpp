#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <chrono>
#include <thread>

#include "../include/calibrator.hpp"



class ImagePointCloudSubscriber : public rclcpp::Node {
  public:
    ImagePointCloudSubscriber(std::string img_left_topic,
                              std::string img_right_topic,
                              std::string cloud_topic,
                              std::shared_ptr<Calibrator> calibrator)
        : Node("image_pointcloud_subscriber"){
        calibrator_ = calibrator;
        image_sub1_ = create_subscription<sensor_msgs::msg::Image>(
            img_left_topic, 1, std::bind(&ImagePointCloudSubscriber::imageCallback1, this, std::placeholders::_1));
        image_sub2_ = create_subscription<sensor_msgs::msg::Image>(
            img_right_topic, 1, std::bind(&ImagePointCloudSubscriber::imageCallback2, this, std::placeholders::_1));
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&ImagePointCloudSubscriber::pointCloudCallback, this, std::placeholders::_1));
    }

  private:
    void imageCallback1(const sensor_msgs::msg::Image &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (calibrator_->getLeftImage().empty()) {
            calibrator_->setLeftImage(cv_ptr->image);
        }
    }

    void imageCallback2(const sensor_msgs::msg::Image &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (calibrator_->getRightImage().empty()) {
            calibrator_->setRightImage(cv_ptr->image);
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2 &msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(msg, *cloud);

        if (calibrator_->getPointCloud() == nullptr) {
            calibrator_->setPointCloud(cloud);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub2_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    std::shared_ptr<Calibrator> calibrator_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Load topics from YAML file
    std::string config_file       = ament_index_cpp::get_package_share_directory("lidar_stereo_calib") + "/config.yaml";
    std::string stereo_config_file = ament_index_cpp::get_package_share_directory("lidar_stereo_calib") + "/cam_stereo.yaml";
    YAML::Node config             = YAML::LoadFile(config_file);
    std::string image_left_topic  = config["topics"]["image_left_topic"].as<std::string>();
    std::string image_right_topic = config["topics"]["image_right_topic"].as<std::string>();
    std::string pointcloud_topic  = config["topics"]["pointcloud_topic"].as<std::string>();

    // Create a shared pointer to the Calibrator object
    std::shared_ptr<Calibrator> calibrator = std::make_shared<Calibrator>(config_file, stereo_config_file);

    std::thread calib_thread(&Calibrator::calibrate, calibrator);
    calib_thread.detach();

    rclcpp::spin(
        std::make_shared<ImagePointCloudSubscriber>(image_left_topic, image_right_topic, pointcloud_topic, calibrator));

    return 0;
}
