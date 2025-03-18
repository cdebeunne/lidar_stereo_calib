cmake_minimum_required(VERSION 3.8)
project(lidar_stereo_calib)

# -------------------------------------------------------------------
# Compiler Options
# -------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------

# set(cvbridge_INCLUDE_DIRS /opt/ros/humble/include/cv_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${YAML_CPP_INCLUDE_DIR} 
  ${Boost_SYSTEM_LIBRARY}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src)

# -------------------------------------------------------------------
# Library
# -------------------------------------------------------------------

add_library(calib-lib SHARED
  src/calibrator.cpp
  src/denseStereo.cpp)
target_link_libraries(calib-lib ${OpenCV_LIBS} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

# -------------------------------------------------------------------
# Executables generation
# -------------------------------------------------------------------

add_executable(calib src/main.cpp)
ament_target_dependencies(calib rclcpp sensor_msgs pcl_conversions cv_bridge)
target_link_libraries(calib calib-lib)

install(TARGETS
  calib
  DESTINATION lib/${PROJECT_NAME})

install(FILES config.yaml cam_stereo.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
