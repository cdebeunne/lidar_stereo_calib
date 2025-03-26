# Lidar Stereo Calibration

This ROS 2 package provides tools for calibrating a LiDAR sensor with a stereo camera system. It is based on a simple ICP between dense stereo and LiDAR point clouds.

## Features

- Support common camera models, including fisheye (pinhole, omnidirectionnal and double sphere)
- Visualization of calibration results.
- ROS 2 integration for seamless sensor data handling.

## Installation

1. Clone the repository into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/your-repo/lidar_stereo_calib.git
    ```
2. Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

## Usage

Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash

```
Run the calibration node:
```bash
ros2 run lidar_stereo_calib calib
```

## Configuration

Modify the `config/params.yaml` file to set parameters such as:
- LiDAR topic
- Stereo camera topics
- Parameters of the dense stereo algorithm and the ICP
- An initial transform to properly warmstart the ICP