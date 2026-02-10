cmake_minimum_required(VERSION 3.5.6)
project(indoor_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)

# OpenCV (VideoCapture lives in videoio)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)

set(node_plugins "")

# Component library (plugin)
add_library(rtsp_camera SHARED
  src/rtsp_camera.cpp
)
target_compile_definitions(rtsp_camera
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

target_include_directories(rtsp_camera PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(rtsp_camera
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  cv_bridge
  sensor_msgs
  camera_info_manager
  camera_calibration_parsers
  image_transport
)

# IMPORTANT: plain signature (no PRIVATE/PUBLIC) to match ament_target_dependencies() usage
target_link_libraries(rtsp_camera
  ${OpenCV_LIBS}
)

rclcpp_components_register_nodes(rtsp_camera "indoor_camera::RTSPCamera")
set(node_plugins "${node_plugins}indoor_camera::RTSPCamera;$<TARGET_FILE:rtsp_camera>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

# Optional composition executable
add_executable(composition
  src/main.cpp
)

# composition needs your package headers too
target_include_directories(composition PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(composition
  rtsp_camera
)

ament_target_dependencies(composition
  rclcpp
)

install(TARGETS
  rtsp_camera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  composition
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
