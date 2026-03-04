cmake_minimum_required(VERSION 3.0.2)
project(air_video_receiver)

## 1. 查找依赖库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

# 查找系统中的 GStreamer 库
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

## 2. 声明 ROS 包配置
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

## 3. 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

## 4. 编译可执行文件
add_executable(h265_node src/h265_node.cpp)

## 5. 链接库文件
target_link_libraries(h265_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
)
# --- 新增：IMU 节点配置 ---
add_executable(imu_node src/imu_node.cpp)
target_link_libraries(imu_node
  ${catkin_LIBRARIES}
)