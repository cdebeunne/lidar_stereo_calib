#include "../include/calibrator.hpp"

void Calibrator::setLeftImage(const cv::Mat &leftImage) { _leftImage = leftImage; }

void Calibrator::setRightImage(const cv::Mat &rightImage) { _rightImage = rightImage; }

void Calibrator::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud) { _pcl_cloud_lidar = pointCloud; }

cv::Mat Calibrator::getLeftImage() const { return _leftImage; }

cv::Mat Calibrator::getRightImage() const { return _rightImage; }

pcl::PointCloud<pcl::PointXYZ>::Ptr Calibrator::getPointCloud() const { return _pcl_cloud_lidar; }

void torrusFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                  double min,
                  double max) {

    cloud_out->clear();
    cloud_out->width = 1;
    cloud_out->height = cloud_in->points.size();
    for (auto pt : cloud_in->points)
        if (sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) > min && sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z) < max)
            cloud_out->points.push_back(pt);
}

bool Calibrator::calibrate() {

    while (_leftImage.empty() || _rightImage.empty() || _pcl_cloud_lidar == nullptr) {
        std::cout << "Wait for scans and image, don't move the robot!" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

    // Display disparity image
    cv::imshow("Disparity Image", disp_img);
    cv::waitKey(10000);

    // Pointcloud computation
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_stereo = _ds->pcFromDepthMap(depth_filtered);

    // Downsample dense stereo
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud_stereo);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*pcl_cloud_stereo);

    // Filter dense stereo
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_stereo_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    torrusFilter(pcl_cloud_stereo, pcl_cloud_stereo_filtered, _min_dist, _max_dist);

    // Filter LiDAR too
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_lidar_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    torrusFilter(_pcl_cloud_lidar, pcl_cloud_lidar_filtered, _min_dist, _max_dist);

    // Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_cloud_stereo_filtered, *pcl_cloud_stereo_filtered, indices);
    pcl::removeNaNFromPointCloud(*pcl_cloud_lidar_filtered, *pcl_cloud_lidar_filtered, indices);

    // Perform ICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(pcl_cloud_stereo_filtered);
    icp.setInputTarget(pcl_cloud_lidar_filtered);
    icp.setMaximumIterations(_max_iterations);
    icp.setMaxCorrespondenceDistance(_max_correspondence_distance);

    // We perform ICP i.e. find the transformation matrix that aligns the stereo pointcloud with the lidar pointcloud
    pcl::PointCloud<pcl::PointXYZ> final_cloud;
    icp.align(final_cloud, _T_lidar_cam0.cast<float>());

    if (!icp.hasConverged()) {
        std::cerr << "ICP did not converge" << std::endl;
        std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;

        // Visualization
        pcl::visualization::PCLVisualizer viewer("ICP Alignment");
        viewer.addPointCloud(
            pcl_cloud_lidar_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_cloud_lidar_filtered, 255, 0, 0),
            "cloud_lidar");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_lidar");

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_ptr(
            new pcl::PointCloud<pcl::PointXYZ>(*pcl_cloud_stereo_filtered));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final_cloud_ptr, 0, 0, 255);
        pcl::transformPointCloud(*pcl_cloud_stereo_filtered, *final_cloud_ptr, _T_lidar_cam0.cast<float>());
        viewer.addPointCloud(pcl_cloud_stereo_filtered, final_color, "final_cloud");

        viewer.addCoordinateSystem();

        viewer.spin();
        return false;
    } else {
        std::cout << "ICP converged with fitness score: " << icp.getFitnessScore() << std::endl;
        std::cout << "Transformation matrix T_lidar_cam0 :\n" << icp.getFinalTransformation() << std::endl;

        // Visualization
        pcl::visualization::PCLVisualizer viewer("ICP Alignment");
        viewer.addPointCloud(
            pcl_cloud_lidar_filtered,
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pcl_cloud_lidar_filtered, 255, 0, 0),
            "cloud_lidar");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_lidar");

        pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(*pcl_cloud_stereo_filtered));
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final_cloud_ptr, 0, 0, 255);
        pcl::transformPointCloud(*pcl_cloud_stereo_filtered, *final_cloud_ptr, icp.getFinalTransformation());
        viewer.addPointCloud(final_cloud_ptr, final_color, "final_cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "final_cloud");

        viewer.addCoordinateSystem();

        viewer.spin();
    }

    return true;
}
