#ifndef CALIBRATOR_HPP
#define CALIBRATOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "../include/denseStereo.hpp"

class Calibrator {
  public:
    Calibrator(std::string config_file, std::string stereo_config_file) {
        _leftImage  = cv::Mat();
        _rightImage = cv::Mat();
        _pcl_cloud_lidar = nullptr;

        // Load DS parameters
        YAML::Node config = YAML::LoadFile(config_file);
        _ds               = std::make_shared<denseStereo>(stereo_config_file);
        _ds->_vfov        = config["vfov"].as<double>();
        _ds->_ndisp       = config["ndisp"].as<double>();
        _ds->_wsize       = config["wsize"].as<double>();
        _ds->InitRectifyMap();

        // Load T_lidar_cam0
        std::vector<double> data_T(16);
        data_T                 = config["T_lidar_cam0"].as<std::vector<double>>();
        _T_lidar_cam0 = Eigen::Map<Eigen::Affine3d::MatrixType>(&data_T[0], 4, 4).transpose();
    };

    void setLeftImage(const cv::Mat &leftImage);
    void setRightImage(const cv::Mat &rightImage);
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);

    cv::Mat getLeftImage() const;
    cv::Mat getRightImage() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud() const;

    bool calibrate();

  private:
    cv::Mat _leftImage;
    cv::Mat _rightImage;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _pcl_cloud_lidar;
    Eigen::Matrix4d _T_lidar_cam0;

    std::shared_ptr<denseStereo> _ds;
};

#endif // CALIBRATOR_HPP