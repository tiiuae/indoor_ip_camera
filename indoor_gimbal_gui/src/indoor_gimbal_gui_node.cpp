#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <QApplication>
#include <QWidget>
#include <QMainWindow>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QTimer>
#include <QMessageBox>

#include <gst/gst.h>
#include <gst/video/videooverlay.h>

using namespace std::chrono_literals;

static std::string to_lower(std::string s) {
  for (auto &c : s) c = static_cast<char>(::tolower(c));
  return s;
}

class RosBridge : public rclcpp::Node {
public:
  RosBridge()
  : rclcpp::Node("indoor_gimbal_gui")
  {
    pub_ptz_ = this->create_publisher<std_msgs::msg::String>("ptz_cmd", 10);
    pub_angles_ = this->create_publisher<geometry_msgs::msg::Vector3>("gimbal_angles_gyro_deg", 10);

    rtsp_uri_ = this->declare_parameter<std::string>("rtsp_uri", "rtsp://192.168.1.108:554/stream=1");
    rtsp_protocol_ = this->declare_parameter<std::string>("rtsp_protocol", "udp");
    use_gl_sink_ = this->declare_parameter<bool>("use_gl_sink", true);

    RCLCPP_INFO(get_logger(), "GUI node up. Topics: ptz_cmd (String), gimbal_angles_gyro_deg (Vector3)");
    RCLCPP_INFO(get_logger(), "RTSP: %s proto=%s use_gl_sink=%s",
                rtsp_uri_.c_str(), rtsp_protocol_.c_str(), use_gl_sink_ ? "true" : "false");
  }

  void send_ptz(const std::string &cmd) {
    std_msgs::msg::String m;
    m.data = cmd;
    pub_ptz_->publish(m);
    RCLCPP_INFO(get_logger(), "PTZ -> %s", cmd.c_str());
  }

  void send_angles_deg(double yaw, double pitch, double roll) {
    geometry_msgs::msg::Vector3 v;
    v.x = yaw;
    v.y = pitch;
    v.z = roll;
    pub_angles_->publish(v);
    RCLCPP_INFO(get_logger(), "ANGLES(deg) -> yaw=%.2f pitch=%.2f roll=%.2f", yaw, pitch, roll);
  }

  const std::string &rtsp_uri() const { return rtsp_uri_; }
  const std::string &rtsp_protocol() const { return rtsp_protocol_; }
  bool use_gl_sink() const { return use_gl_sink_; }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ptz_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_angles_;
  std::string rtsp_uri_;
  std::string rtsp_protocol_;
  bool use_gl_sink_{true};
};

class VideoWidget : public QWidget {
public:
  VideoWidget(QWidget *parent = nullptr) : QWidget(parent) {
    // critical: native window handle for GstVideoOverlay
    setAttribute(Qt::WA_NativeWindow);
    setAttribute(Qt::WA_DontCreateNativeAncestors);
    setMinimumSize(960, 540);
  }
};

class GstreamerPlayer {
public:
  GstreamerPlayer() = default;

  bool start(const std::string &rtsp_uri,
             const std::string &rtsp_protocol,
             bool prefer_gl_sink,
             WId window_id,
             std::string *err_out)
  {
    stop();

    const std::string proto = to_lower(rtsp_protocol);
    if (proto != "udp" && proto != "tcp") {
      if (err_out) *err_out = "rtsp_protocol must be 'udp' or 'tcp'";
      return false;
    }

    // Low-latency pipeline (matches your working gst-launch approach)
    //
    // IMPORTANT: we must use a sink supporting GstVideoOverlay to render into QWidget.
    // We'll try glimagesink first (if prefer_gl_sink), else ximagesink.
    //
    // queue leaky downstream max-size-buffers=1 prevents creeping latency.
    // sync=false keeps it “live”.
    //
    std::string sink = prefer_gl_sink ? "glimagesink" : "ximagesink";

    std::ostringstream ss;
    ss
      << "rtspsrc location=\"" << rtsp_uri << "\" protocols=" << proto
      << " latency=0 drop-on-latency=true "
      << "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
      << "! queue max-size-buffers=1 leaky=downstream "
      << "! " << sink << " sync=false";

    std::string pipeline_str = ss.str();

    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (!pipeline_) {
      if (err_out) *err_out = error ? error->message : "gst_parse_launch failed";
      if (error) g_error_free(error);
      return false;
    }
    if (error) g_error_free(error);

    // Find the sink that will do video overlay: last element is usually our sink
    // We'll search for the first element implementing GstVideoOverlay.
    GstIterator *it = gst_bin_iterate_recurse(GST_BIN(pipeline_));
    GValue item = G_VALUE_INIT;
    bool found_overlay = false;

    while (gst_iterator_next(it, &item) == GST_ITERATOR_OK) {
      GstElement *elem = GST_ELEMENT(g_value_get_object(&item));
      if (GST_IS_VIDEO_OVERLAY(elem)) {
        video_sink_ = elem; // borrowed reference; keep pipeline alive
        found_overlay = true;
        g_value_reset(&item);
        break;
      }
      g_value_reset(&item);
    }
    g_value_unset(&item);
    gst_iterator_free(it);

    if (!found_overlay) {
      // Fallback: try ximagesink if glimagesink didn't produce overlay-capable sink
      if (prefer_gl_sink) {
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return start(rtsp_uri, rtsp_protocol, /*prefer_gl_sink=*/false, window_id, err_out);
      }

      if (err_out) *err_out = "No GstVideoOverlay sink found. Try installing gstreamer GL/X11 plugins.";
      stop();
      return false;
    }

    // Bind the video sink to our Qt window handle
    gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(video_sink_), static_cast<guintptr>(window_id));

    // Listen for errors
    bus_ = gst_element_get_bus(pipeline_);

    // Start
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    running_.store(true);
    return true;
  }

  void stop() {
    running_.store(false);

    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (bus_) {
      gst_object_unref(bus_);
      bus_ = nullptr;
    }
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    video_sink_ = nullptr;
  }

  bool running() const { return running_.load(); }

  // Call periodically from Qt timer to drain bus messages (errors/eos)
  std::string poll_bus() {
    if (!bus_) return "";
    while (true) {
      GstMessage *msg = gst_bus_pop(bus_);
      if (!msg) break;

      switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
          GError *err = nullptr;
          gchar *dbg = nullptr;
          gst_message_parse_error(msg, &err, &dbg);
          std::string out = std::string("GStreamer ERROR: ") + (err ? err->message : "(unknown)");
          if (dbg) {
            out += std::string(" | debug: ") + dbg;
          }
          if (err) g_error_free(err);
          if (dbg) g_free(dbg);
          gst_message_unref(msg);
          stop();
          return out;
        }
        case GST_MESSAGE_EOS: {
          gst_message_unref(msg);
          stop();
          return "GStreamer EOS (end of stream).";
        }
        default:
          gst_message_unref(msg);
          break;
      }
    }
    return "";
  }

private:
  GstElement *pipeline_{nullptr};
  GstElement *video_sink_{nullptr};
  GstBus *bus_{nullptr};
  std::atomic<bool> running_{false};
};

class MainWindow : public QMainWindow {
public:
  MainWindow(std::shared_ptr<RosBridge> node)
  : node_(std::move(node))
  {
    auto *central = new QWidget();
    auto *root = new QVBoxLayout(central);

    // --- Video group
    auto *video_group = new QGroupBox("Video (RTSP embedded)");
    auto *video_layout = new QVBoxLayout(video_group);

    video_widget_ = new VideoWidget();
    video_layout->addWidget(video_widget_);

    auto *viewer_controls = new QHBoxLayout();
    btn_start_ = new QPushButton("Start Video");
    btn_stop_  = new QPushButton("Stop Video");
    viewer_controls->addWidget(btn_start_);
    viewer_controls->addWidget(btn_stop_);
    viewer_controls->addStretch();
    video_layout->addLayout(viewer_controls);

    root->addWidget(video_group);

    // --- Controls group
    auto *ctrl_group = new QGroupBox("Gimbal Control");
    auto *ctrl_layout = new QHBoxLayout(ctrl_group);

    // PTZ pad
    auto *ptz_box = new QGroupBox("PTZ");
    auto *ptz_grid = new QGridLayout(ptz_box);

    auto *btn_up = new QPushButton("Up");
    auto *btn_down = new QPushButton("Down");
    auto *btn_left = new QPushButton("Left");
    auto *btn_right = new QPushButton("Right");
    auto *btn_stop_ptz = new QPushButton("Stop");
    auto *btn_home = new QPushButton("Home");

    ptz_grid->addWidget(btn_up, 0, 1);
    ptz_grid->addWidget(btn_left, 1, 0);
    ptz_grid->addWidget(btn_stop_ptz, 1, 1);
    ptz_grid->addWidget(btn_right, 1, 2);
    ptz_grid->addWidget(btn_down, 2, 1);
    ptz_grid->addWidget(btn_home, 3, 1);

    ctrl_layout->addWidget(ptz_box);

    // Angles
    auto *ang_box = new QGroupBox("Set Angles (gyro deg)");
    auto *ang_layout = new QGridLayout(ang_box);

    yaw_ = new QDoubleSpinBox(); yaw_->setRange(-180, 180); yaw_->setDecimals(2);
    pitch_ = new QDoubleSpinBox(); pitch_->setRange(-90, 90); pitch_->setDecimals(2);
    roll_ = new QDoubleSpinBox(); roll_->setRange(-90, 90); roll_->setDecimals(2);

    auto *btn_send = new QPushButton("Send Angles");

    ang_layout->addWidget(new QLabel("Yaw"), 0, 0);
    ang_layout->addWidget(yaw_, 0, 1);
    ang_layout->addWidget(new QLabel("Pitch"), 1, 0);
    ang_layout->addWidget(pitch_, 1, 1);
    ang_layout->addWidget(new QLabel("Roll"), 2, 0);
    ang_layout->addWidget(roll_, 2, 1);
    ang_layout->addWidget(btn_send, 3, 0, 1, 2);

    ctrl_layout->addWidget(ang_box);
    ctrl_layout->addStretch();

    root->addWidget(ctrl_group);

    setCentralWidget(central);
    setWindowTitle("Indoor Operator Station (embedded video)");
    resize(1200, 800);

    // --- wiring: PTZ publishes strings
    connect(btn_up, &QPushButton::clicked, this, [&]{ node_->send_ptz("up"); });
    connect(btn_down, &QPushButton::clicked, this, [&]{ node_->send_ptz("down"); });
    connect(btn_left, &QPushButton::clicked, this, [&]{ node_->send_ptz("left"); });
    connect(btn_right, &QPushButton::clicked, this, [&]{ node_->send_ptz("right"); });
    connect(btn_stop_ptz, &QPushButton::clicked, this, [&]{ node_->send_ptz("stop"); });
    connect(btn_home, &QPushButton::clicked, this, [&]{ node_->send_ptz("home"); });

    // Angles publish Vector3 (x=yaw,y=pitch,z=roll)
    connect(btn_send, &QPushButton::clicked, this, [&]{
      node_->send_angles_deg(yaw_->value(), pitch_->value(), roll_->value());
    });

    // Viewer controls
    connect(btn_start_, &QPushButton::clicked, this, [&]{ this->start_video(); });
    connect(btn_stop_,  &QPushButton::clicked, this, [&]{ this->stop_video(); });

    // Timers:
    // - spin ROS
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, [&]{
      rclcpp::spin_some(node_);
    });
    ros_timer_->start(10);

    // - poll gstreamer bus
    gst_timer_ = new QTimer(this);
    connect(gst_timer_, &QTimer::timeout, this, [&]{
      const std::string err = player_.poll_bus();
      if (!err.empty()) {
        QMessageBox::critical(this, "GStreamer Error", QString::fromStdString(err));
      }
    });
    gst_timer_->start(50);
  }

  ~MainWindow() override {
    stop_video();
  }

  void start_video() {
    if (player_.running()) return;

    // Ensure widget has a valid window id
    video_widget_->show();
    video_widget_->raise();
    const WId wid = video_widget_->winId();

    std::string err;
    if (!player_.start(node_->rtsp_uri(), node_->rtsp_protocol(), node_->use_gl_sink(), wid, &err)) {
      QMessageBox::critical(this, "Failed to start video", QString::fromStdString(err));
    }
  }

  void stop_video() {
    player_.stop();
  }

private:
  std::shared_ptr<RosBridge> node_;

  VideoWidget *video_widget_{nullptr};
  QPushButton *btn_start_{nullptr};
  QPushButton *btn_stop_{nullptr};

  QDoubleSpinBox *yaw_{nullptr};
  QDoubleSpinBox *pitch_{nullptr};
  QDoubleSpinBox *roll_{nullptr};

  QTimer *ros_timer_{nullptr};
  QTimer *gst_timer_{nullptr};

  GstreamerPlayer player_;
};

int main(int argc, char **argv) {
  // ROS2 init
  rclcpp::init(argc, argv);

  // Qt app (NOTE: Qt will also look at argc/argv; fine)
  QApplication app(argc, argv);

  // GStreamer init (safe to call once)
  gst_init(&argc, &argv);

  auto node = std::make_shared<RosBridge>();
  MainWindow win(node);
  win.show();

  int ret = app.exec();

  rclcpp::shutdown();
  return ret;
}
