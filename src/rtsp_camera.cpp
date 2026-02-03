// Copyright (c) 2019 Tasuku Miura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include "indoor_camera/rtsp_camera.hpp"


namespace indoor_camera
{
  RTSPCamera::RTSPCamera(const std::string & node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    qos_(rclcpp::QoS(rclcpp::KeepLast(2)).best_effort().durability_volatile())
  {
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "middleware: %s", rmw_get_implementation_identifier());

    // Declare parameters.
    this->initialize_parameters();
    this->configure();

    // Run pipeline
    // this->gstreamerPipeline();

    // Image publisher
    this->pub_ = image_transport::create_camera_publisher(
      this, "image_raw", qos_.get_rmw_qos_profile());
    this->execute();
      
    }

  RTSPCamera::RTSPCamera(const rclcpp::NodeOptions & options)
  : RTSPCamera::RTSPCamera("ipcamera", options)
  {}

  void
  RTSPCamera::configure()
  {
    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    this->cap_.open(source_, cv::CAP_FFMPEG); 

    // Try to keep internal buffer tiny
    this->cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // Set the width and height based on command line arguments.
    // The width, height has to match the available resolutions of the IP camera.
    this->cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    this->cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));

    if (!this->cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if (cinfo_manager_->validateURL(camera_calibration_file_param_)) {
      cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
    } else {
      RCLCPP_WARN(this->get_logger(), "CameraInfo URL not valid.");
      RCLCPP_WARN(this->get_logger(), "URL IS %s", camera_calibration_file_param_.c_str());
    }
  }

  void
  RTSPCamera::initialize_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
    rtsp_uri_descriptor.name = "rtsp_uri";
    rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
    rtsp_uri_descriptor.additional_constraints = "Should be of the form 'rtsp://";
    this->declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

    rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
    camera_calibration_file_descriptor.name = "camera_calibration_file";
    camera_calibration_file_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter(
      "camera_calibration_file", "", camera_calibration_file_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
    image_width_descriptor.name = "image_width";
    image_width_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_width", 640, image_width_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
    image_height_descriptor.name = "image_height";
    image_height_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_height", 480, image_height_descriptor);
        
    // Camera parameters
    this->get_parameter<std::string>("rtsp_uri", source_);
    RCLCPP_INFO(this->get_logger(), "rtsp_uri: %s", source_.c_str());

    this->get_parameter<std::string>("camera_calibration_file", camera_calibration_file_param_);
    RCLCPP_INFO(this->get_logger(), "camera_calibration_file: %s",
                camera_calibration_file_param_.c_str());

    this->get_parameter<int>("image_width", width_);
    RCLCPP_INFO(this->get_logger(), "image_width: %d", width_);

    this->get_parameter<int>("image_height", height_);
    RCLCPP_INFO(this->get_logger(), "image_height: %d", height_);

    // GStreamer Pipeline
    this->declare_parameter<bool>("operator_view", true);
    this->declare_parameter<std::string>("operator_view_cmd",
      "gst-launch-1.0 -q rtspsrc location=RTSP_URI protocols=udp latency=0 drop-on-latency=true ! "
      "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
      "queue max-size-buffers=1 leaky=downstream ! autovideosink sync=false"
    );

    this->get_parameter("operator_view", operator_view_);
    this->get_parameter("operator_view_cmd", operator_view_cmd_);
    RCLCPP_INFO(this->get_logger(), "gstreamer_pipeline: %s", operator_view_cmd_.c_str());
  }


  void
  RTSPCamera::gstreamerPipeline()
  {
    if (operator_view_) {
      std::string cmd = operator_view_cmd_;
      // replace token with actual URI
      const std::string token = "RTSP_URI";
      if (cmd.find(token) != std::string::npos) {
        cmd.replace(cmd.find(token), token.size(), source_);
      }

      // Spawn in background (fork/exec is better than system("&"), but system works too)
      cmd += " &";
      RCLCPP_INFO(this->get_logger(), "Starting operator view: %s", cmd.c_str());
      (void)std::system(cmd.c_str());
    }
  }

  void
  RTSPCamera::execute()
  {
    rclcpp::Rate loop_rate(freq_);

    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());

    // Initialize OpenCV image matrices.
    cv::Mat frame;

    size_t frame_id = 0;
    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok()) {
      // Initialize a shared pointer to an Image message.
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->is_bigendian = false;

      // Drain queued frames (drop latency)
      for (int i = 0; i < 3; ++i) {
        if (!this->cap_.grab()) break;
      }
      this->cap_.retrieve(frame);

      // Check if the frame was grabbed correctly
      if (!frame.empty()) {
        // Convert to a ROS image
        convert_frame_to_message(frame, frame_id, *msg, *camera_info_msg);
        // Publish the image message and increment the frame_id.
        this->pub_.publish(std::move(msg), camera_info_msg);
        ++frame_id;
      }
      loop_rate.sleep();
    }
  }



  std::string
  RTSPCamera::mat_type2encoding(int mat_type)
  {
    switch (mat_type) {
      case CV_8UC1:
        return "mono8";
      case CV_8UC3:
        return "bgr8";
      case CV_16SC1:
        return "mono16";
      case CV_8UC4:
        return "rgba8";
      default:
        throw std::runtime_error("Unsupported encoding type");
    }
  }

  void
  RTSPCamera::convert_frame_to_message(
    const cv::Mat & frame,
    size_t frame_id,
    sensor_msgs::msg::Image & msg,
    sensor_msgs::msg::CameraInfo & camera_info_msg)
  {
    // copy cv information into ros message
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = mat_type2encoding(frame.type());
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    size_t size = frame.step * frame.rows;
    msg.data.resize(size);
    memcpy(&msg.data[0], frame.data, size);

    rclcpp::Time timestamp = this->get_clock()->now();

    msg.header.frame_id = std::to_string(frame_id);
    msg.header.stamp = timestamp;
    camera_info_msg.header.frame_id = std::to_string(frame_id);
    camera_info_msg.header.stamp = timestamp;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(indoor_camera::RTSPCamera)
