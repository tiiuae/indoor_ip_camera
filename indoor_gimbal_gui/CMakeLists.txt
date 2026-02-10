cmake_minimum_required(VERSION 3.8)
project(indoor_gimbal_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Qt5
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# GStreamer via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14)
pkg_check_modules(GSTVIDEO REQUIRED gstreamer-video-1.0>=1.14)

add_executable(indoor_gimbal_gui_node
  src/indoor_gimbal_gui_node.cpp
)


target_compile_features(indoor_gimbal_gui_node PUBLIC cxx_std_17)

target_include_directories(indoor_gimbal_gui_node PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GSTVIDEO_INCLUDE_DIRS}
)

target_link_libraries(indoor_gimbal_gui_node
  Qt5::Widgets
  ${GST_LIBRARIES}
  ${GSTVIDEO_LIBRARIES}
)

target_compile_options(indoor_gimbal_gui_node PRIVATE
  ${GST_CFLAGS_OTHER}
  ${GSTVIDEO_CFLAGS_OTHER}
)

ament_target_dependencies(indoor_gimbal_gui_node
  rclcpp std_msgs geometry_msgs
)

install(TARGETS indoor_gimbal_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
