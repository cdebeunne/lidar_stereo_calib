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
3. Source the workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

## Usage

1. Launch the calibration node:
    ```bash
    ros2 launch lidar_stereo_calib calibration.launch.py
    ```
2. Provide the required sensor topics and parameters in the launch file.
3. Follow the instructions in the terminal or use the provided visualization tools to complete the calibration process.

## Configuration

Modify the `config/params.yaml` file to set parameters such as:
- LiDAR topic
- Stereo camera topics
- Parameters of the dense stereo algorithm and the ICP
- 

## Visualization

Use the included Rviz2 configuration to visualize sensor data and calibration results:
```bash
ros2 launch lidar_stereo_calib rviz.launch.py
```

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This package is licensed under the [MIT License](LICENSE).

## Acknowledgments

Special thanks to the open-source robotics community for their contributions to sensor calibration tools.