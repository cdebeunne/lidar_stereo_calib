#include "../include/calibrator.hpp"

void Calibrator::setLeftImage(const cv::Mat &leftImage) { _leftImage = leftImage; }

void Calibrator::setRightImage(const cv::Mat &rightImage) { _rightImage = rightImage; }

void Calibrator::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) { _pointCloud = pointCloud; }

cv::Mat Calibrator::getLeftImage() const { return _leftImage; }

cv::Mat Calibrator::getRightImage() const { return _rightImage; }

pcl::PointCloud<pcl::PointXYZ>::Ptr Calibrator::getPointCloud() const { return _pointCloud; }

bool Calibrator::calibrate() {

    while (_leftImage.empty() || _rightImage.empty() || _pointCloud == nullptr) {
        std::cout << "Calibration failed. Retrying..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Downsample image
    cv::Mat small_left_img, small_right_img;
    cv::resize(_leftImage, small_left_img, cv::Size(), 1, 1);
    cv::resize(_rightImage, small_right_img, cv::Size(), 1, 1);

    cv::Mat rect_imgl, rect_imgr;
    cv::remap(small_left_img, rect_imgl, _ds->smap[0][0], _ds->smap[0][1], 1, 0);
    cv::remap(small_right_img, rect_imgr, _ds->smap[1][0], _ds->smap[1][1], 1, 0);

    // Disparity computation
    cv::Mat disp_img, depth_map;
    _ds->DisparityImage(rect_imgl, rect_imgr, disp_img, depth_map);

    // Depth image filtering
    cv::Mat depth_filtered;
    cv::medianBlur(depth_map, depth_filtered, 5);

    // Pointcloud computation
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_stereo = _ds->pcFromDepthMap(depth_filtered);

    // Downsample dense stereo
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud_stereo );
    sor.setLeafSize(0.025f, 0.025f, 0.025f);
    sor.filter(*pcl_cloud_stereo );

    // Perform ICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl_cloud_stereo);
    icp.setInputTarget(_pointCloud);
    icp.setMaximumIterations(10);
    icp.setMaxCorrespondenceDistance(5);

    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp.align(final_cloud, _T_cam_lidar.cast<float>());

    if (!icp.hasConverged())
    {
        std::cerr << "ICP did not converge" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;

        // Visualization
        pcl::visualization::PCLVisualizer viewer("ICP Alignment");
        viewer.addPointCloud(_pointCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(_pointCloud, 255, 0, 0), "_pointCloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "_pointCloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*pcl_cloud_stereo));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final_cloud_ptr, 0, 0, 255);
        pcl::transformPointCloud(*pcl_cloud_stereo, *final_cloud_ptr, _T_cam_lidar.cast<float>());
        viewer.addPointCloud(final_cloud_ptr, final_color, "final_cloud");

        viewer.addCoordinateSystem();

        viewer.spin();
        return false;
    } else
    {
        std::cout << "ICP converged with fitness score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix:\n"
                  << icp.getFinalTransformation() << std::endl;

        // Visualization
        pcl::visualization::PCLVisualizer viewer("ICP Alignment");
        viewer.addPointCloud(_pointCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(_pointCloud, 255, 0, 0), "_pointCloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "_pointCloud");

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*pcl_cloud_stereo));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final_cloud_ptr, 0, 0, 255);
        pcl::transformPointCloud(*pcl_cloud_stereo, *final_cloud_ptr, icp.getFinalTransformation());
        viewer.addPointCloud(final_cloud_ptr, final_color, "final_cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "final_cloud");

        viewer.addCoordinateSystem();

        viewer.spin();
    }

    return true;
}
