SET(CMAKE_BUILD_TYPE RELEASE)

project(lidar_stereo_calib)
cmake_minimum_required(VERSION 3.14)

# -------------------------------------------------------------------
# Compiler Options
# -------------------------------------------------------------------
add_definitions("-Wall -std=c++17 -O3")

if(CMAKE_BUILD_TYPE STREQUAL "DEBUG")
  message(STATUS "Debug configuration")
  add_definitions("-g")
else(CMAKE_BUILD_TYPE STREQUAL "RELEASE")
  message(STATUS "Release configuration")
  add_definitions("-O3")
endif(CMAKE_BUILD_TYPE STREQUAL "DEBUG")

add_definitions(-D_USE_MATH_DEFINES)


# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIR} ${Boost_SYSTEM_LIBRARY})

# -------------------------------------------------------------------
# Library
# -------------------------------------------------------------------

file(GLOB_RECURSE LIB_SOURCES src/*.cpp include/*.cpp)
file(GLOB_RECURSE LIB_HEADERS include/*.h include/*.hpp)
add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES} ${LIB_HEADERS})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
)

# -------------------------------------------------------------------
# Executables generation
# -------------------------------------------------------------------

add_executable(calib src/main.cpp)
ament_target_dependencies(calib rclcpp sensor_msgs image_transport cv_bridge)
target_link_libraries(calib ${PROJECT_NAME})

install(TARGETS
  calib
  DESTINATION lib/${PROJECT_NAME})

install(FILES config.yaml cam_stereo.yaml
  DESTINATION share/${PROJECT_NAME})

ament_package()
