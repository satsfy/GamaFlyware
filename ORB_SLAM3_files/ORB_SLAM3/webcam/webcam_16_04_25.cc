#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <deque> // Use deque for frame queue and IMU buffer
#include <thread>
#include <signal.h>
#include <vector>
#include <atomic>
#include <csignal>
#include <stdlib.h>
#include <mutex> // For thread-synchronization

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std;
using namespace std::chrono_literals;

// Global flags and timing parameters.
bool ENABLED_IMU = true;
double FRAME_TIME_OFFSET = 0.0;
std::chrono::milliseconds main_loop_duration = 20ms;
const int MIN_STARTUP_DELAY_MS = 3000; // 3 seconds
const int MAX_STORED_IMAGES = 5;
const int MAX_STORED_IMU_MESSAGES = 10;
const int MIN_IMU_MSGS_REQUIRED = 2;

std::atomic<bool> g_interrupt_requested(false);
void signalHandler(int signal)
{
  std::cout << "Caught signal " << signal << ", shutting down gracefully." << std::endl;
  g_interrupt_requested.store(true);
  rclcpp::shutdown();
  std::thread([]
              {
    std::cerr << "Shutdown timeout exceeded. Forcing exit." << std::endl;
    exit(1); })
      .detach();
}

class WebcamSLAMNode : public rclcpp::Node
{
public:
  WebcamSLAMNode(const std::string &vocab_path, const std::string &settings_path)
      : Node("webcam_slam_node"),
        imu_count_(0),
        imu_initialized_(false),
        last_frame_timestamp_(0),
        has_received_image_(false),
        timer_callback_index(0)
  {
    initial_timestamp_ = std::chrono::high_resolution_clock::now();

    // Create dedicated callback groups for the IMU and image subscriptions.
    imu_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    image_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions imu_sub_options;
    imu_sub_options.callback_group = imu_callback_group_;

    rclcpp::SubscriptionOptions image_sub_options;
    image_sub_options.callback_group = image_callback_group_;

    // Set up IMU subscription.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data_raw",
        rclcpp::SensorDataQoS().keep_last(100), // Increase QoS history
        std::bind(&WebcamSLAMNode::imu_callback, this, std::placeholders::_1),
        imu_sub_options);

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera",
        rclcpp::SensorDataQoS().keep_last(10),
        std::bind(&WebcamSLAMNode::image_callback, this, std::placeholders::_1),
        image_sub_options);

    post_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
        "/orbslam3/pose", rclcpp::SensorDataQoS());

    if (ENABLED_IMU)
    {
      slam_system_ = std::make_unique<ORB_SLAM3::System>(
          vocab_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true);
    }
    else
    {
      slam_system_ = std::make_unique<ORB_SLAM3::System>(
          vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);
    }
    image_scale_ = slam_system_->GetImageScale();
    RCLCPP_INFO(this->get_logger(), "Initialized SLAM with image scale: %f", image_scale_);

    // main_timer_ = this->create_wall_timer(
    //     main_loop_duration, std::bind(&WebcamSLAMNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Node initialization complete. Waiting for IMU and image data...");
  }

  ~WebcamSLAMNode()
  {
    std::cout << "WebcamSLAMNode destructor called, shutting down SLAM system..." << std::endl;
    if (slam_system_)
    {
      try
      {
        slam_system_->Shutdown();
        slam_system_.reset();
      }
      catch (const std::exception &e)
      {
        std::cerr << "Error during SLAM system shutdown: " << e.what() << std::endl;
      }
      catch (...)
      {
        std::cerr << "Unknown error during SLAM system shutdown" << std::endl;
      }
    }
    cv::destroyAllWindows();
    std::cout << "SLAM system shutdown completed." << std::endl;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr post_pub_;
  rclcpp::TimerBase::SharedPtr main_timer_;

  int timer_callback_index;
  cv::Mat latest_frame_;
  std::mutex image_mutex_;
  bool has_received_image_;
  std::deque<std::pair<cv::Mat, double>> frame_queue_;
  std::mutex frame_queue_mutex_;

  std::unique_ptr<ORB_SLAM3::System> slam_system_;
  float image_scale_;

  std::chrono::time_point<std::chrono::high_resolution_clock> initial_timestamp_;

  rclcpp::CallbackGroup::SharedPtr imu_callback_group_;
  rclcpp::CallbackGroup::SharedPtr image_callback_group_;

  int imu_count_;
  bool imu_initialized_;
  double last_frame_timestamp_;
  std::deque<ORB_SLAM3::IMU::Point> imu_deque_;
  std::mutex imu_mutex_;

  ORB_SLAM3::IMU::Point last_point = ORB_SLAM3::IMU::Point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  bool last_point_defined = false;

  int64_t get_elapsed_ms()
  {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - initial_timestamp_).count();
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const std_msgs::msg::Header &header = msg->header;
    // const double timestamp = static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) / 1e9;
    double timestamp = this->now().seconds();

    // rclcpp::Time current_ros_time = this->now();
    // double current_time = current_ros_time.seconds();
    // const double timestamp = current_time;

    static double last_frame_ts = 0;
    if (timestamp <= last_frame_ts)
    {
      RCLCPP_WARN(this->get_logger(), "Non-monotonic frame timestamp detected. Skipping frame.");
      return;
    }
    last_frame_ts = timestamp;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      last_frame_timestamp_ = timestamp;
    }

    cv::Mat image;
    if (msg->encoding == "bgr8")
    {
      image = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data())).clone();
    }
    else if (msg->encoding == "rgb8")
    {
      cv::Mat temp = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data())).clone();
      cv::cvtColor(temp, image, cv::COLOR_RGB2BGR);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
      return;
    }

    cv::resize(image, image, cv::Size(854, 480));
    cv::Rect roi(70, 0, image.cols - 140, image.rows);
    if (roi.x >= 0 && roi.y >= 0 && (roi.x + roi.width) <= image.cols && (roi.y + roi.height) <= image.rows)
    {
      image = image(roi).clone();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "ROI out of bounds, skipping cropping.");
    }
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_frame_ = image.clone();
      has_received_image_ = true;
    }
    {
      std::lock_guard<std::mutex> lock(frame_queue_mutex_);
      frame_queue_.push_back({image, timestamp});
      if (frame_queue_.size() > MAX_STORED_IMAGES)
        frame_queue_.pop_front();
    }

    timer_callback();
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
  {
    const std_msgs::msg::Header &header = imuMsg->header;
    // const double timestamp = static_cast<double>(header.stamp.sec) +
    //                          static_cast<double>(header.stamp.nanosec) / 1e9;
    double timestamp = this->now().seconds();

    const geometry_msgs::msg::Vector3 &angularVelocity = imuMsg->angular_velocity;
    const geometry_msgs::msg::Vector3 &linearAceleration = imuMsg->linear_acceleration;

    double ax = linearAceleration.x, ay = linearAceleration.y, az = linearAceleration.z;
    double gx = angularVelocity.x, gy = angularVelocity.y, gz = angularVelocity.z;

    cv::Point3f acc(ax, ay, az);
    cv::Point3f gyr(gx, gy, gz);
    ORB_SLAM3::IMU::Point point(acc, gyr, timestamp);
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      imu_deque_.push_back(point);
      if (imu_deque_.size() > MAX_STORED_IMU_MESSAGES)
        imu_deque_.pop_front();
    }
    imu_count_++;
  }

  void timer_callback()
  {
    timer_callback_index++;

    if (timer_callback_index < 5)
    {
      RCLCPP_INFO(this->get_logger(), "Skipping first calls to timer_callback...");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      size_t imu_queue_size = imu_deque_.size();
      double latest_imu_ts = imu_queue_size > 0 ? imu_deque_.back().t : 0.0;
      RCLCPP_INFO(this->get_logger(), "IMU queue size: %zu, latest IMU timestamp: %f", imu_queue_size, latest_imu_ts);

      // Print timestamps of each element in the IMU list.
      RCLCPP_INFO(this->get_logger(), "IMU timestamps:");
      for (const auto &imu_pt : imu_deque_)
      {
        RCLCPP_INFO(this->get_logger(), "  %f", imu_pt.t);
      }
    }

    {
      std::lock_guard<std::mutex> lock(frame_queue_mutex_);
      size_t img_queue_size = frame_queue_.size();
      double latest_img_ts = img_queue_size > 0 ? frame_queue_.back().second : 0.0;
      RCLCPP_INFO(this->get_logger(), "Image queue size: %zu, latest image timestamp: %f", img_queue_size, latest_img_ts);

      // Print timestamps of each element in the image list.
      RCLCPP_INFO(this->get_logger(), "Image timestamps:");
      for (const auto &img_pair : frame_queue_)
      {
        RCLCPP_INFO(this->get_logger(), "  %f", img_pair.second);
      }
    }

    if (g_interrupt_requested.load())
    {
      RCLCPP_WARN(this->get_logger(), "Interrupt requested; skipping timer callback.");
      return;
    }

    auto now = std::chrono::high_resolution_clock::now();

    if (!imu_initialized_)
    {
      if (imu_count_ < MIN_IMU_MSGS_REQUIRED || get_elapsed_ms() < MIN_STARTUP_DELAY_MS)
      {
        RCLCPP_INFO(this->get_logger(), "Waiting for IMU initialization: %d msgs, elapsed: %ld ms",
                    imu_count_, get_elapsed_ms());
        return;
      }
      else
      {
        imu_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "IMU Initialized after %ld ms with %d messages",
                    get_elapsed_ms(), imu_count_);
      }
    }

    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      if (!has_received_image_)
      {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Waiting for camera images... IMU buffer size: %zu", imu_deque_.size());
        return;
      }
    }

    std::vector<ORB_SLAM3::IMU::Point> imu_points;
    cv::Mat candidate_frame;
    double candidate_timestamp = 0.0;
    int candidate_index = -1;

    {
      std::lock_guard<std::mutex> lock(frame_queue_mutex_);

      for (size_t i = frame_queue_.size() - 1; i >= 0; --i)
      {
        RCLCPP_INFO(this->get_logger(), "Im deliberating about image %d", i);
        double img_ts = frame_queue_[i].second;
        std::vector<ORB_SLAM3::IMU::Point> temp_imu_points;
        {
          std::lock_guard<std::mutex> imu_lock(imu_mutex_);
          for (const auto &imu_pt : imu_deque_)
          {
            if (imu_pt.t > img_ts)
              temp_imu_points.push_back(imu_pt);
          }
        }
        if (temp_imu_points.size() >= MIN_IMU_MSGS_REQUIRED)
        {
          candidate_index = i;
          imu_points = temp_imu_points;
          break;
        }
        if (i == 0)
        {
          RCLCPP_WARN(this->get_logger(), "I was forced to choose the oldest image because no matches were found");
          candidate_index = i;
          imu_points = temp_imu_points;
          break;
        }
      }

      candidate_frame = frame_queue_[candidate_index].first;
      candidate_timestamp = frame_queue_[candidate_index].second;
    }

    if (candidate_frame.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Candidate frame is empty. -------------> Skipping iteration");
      return;
    }
    if (imu_points.size() < MIN_IMU_MSGS_REQUIRED)
    {
      RCLCPP_ERROR(this->get_logger(), "Not enough IMU points (%zu) for integration. ------------> Skipping iteration", imu_points.size());
      return;
    }

    std::sort(imu_points.begin(), imu_points.end(),
              [](const ORB_SLAM3::IMU::Point &a, const ORB_SLAM3::IMU::Point &b)
              {
                return a.t < b.t;
              });

    RCLCPP_INFO(this->get_logger(), "Feeding candidate frame (timestamp: %f) to SLAM with %zu IMU points",
                candidate_timestamp, imu_points.size());

    if (ENABLED_IMU)
    {
      RCLCPP_INFO(this->get_logger(), "TrackMonocular input: frame timestamp: %f, dimensions: %dx%d, IMU count: %zu",
                  candidate_timestamp, candidate_frame.cols, candidate_frame.rows, imu_points.size());
      for (const auto &pt : imu_points)
      {
        RCLCPP_INFO(this->get_logger(),
                    "IMU Point: t = %f, Acc = (%f, %f, %f), Gyro = (%f, %f, %f)",
                    pt.t, pt.a.x(), pt.a.y(), pt.a.z(), pt.w.x(), pt.w.y(), pt.w.z());
      }
      try
      {
        slam_system_->TrackMonocular(candidate_frame, candidate_timestamp, imu_points);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Exception in TrackMonocular: %s", e.what());
        return;
      }
    }
    else
    {
      slam_system_->TrackMonocular(candidate_frame, candidate_timestamp);
    }

    // remove every frame up through the one you just processed
    {
      std::lock_guard<std::mutex> fl(frame_queue_mutex_);
      frame_queue_.erase(
          frame_queue_.begin(),
          frame_queue_.begin() + (candidate_index + 1));
    }

    {
      double last_imu_t = imu_points.back().t;
      std::lock_guard<std::mutex> il(imu_mutex_);
      // remove everything ≤ last_imu_t
      imu_deque_.erase(
          imu_deque_.begin(),
          std::upper_bound(
              imu_deque_.begin(), imu_deque_.end(), last_imu_t,
              [](double t, const ORB_SLAM3::IMU::Point &p)
              { return t < p.t; }));
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - now);
    RCLCPP_INFO(this->get_logger(), "Timer callback duration: %ld ms", duration_ms.count());
  }
};

int main(int argc, char **argv)
{
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGABRT, signalHandler);

  std::cout << "COMPILED VERSION V3 - Enhanced IMU handling with candidate frame rule" << std::endl;

  if (argc < 3 || argc > 4)
  {
    std::cerr << "Usage: ./webcam_slam_node path_to_vocabulary path_to_settings (trajectory_file_name)" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<WebcamSLAMNode>(argv[1], argv[2]);

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);

  try
  {
    executor.spin();
  }
  catch (const std::exception &e)
  {
    std::cerr << "Exception in executor: " << e.what() << std::endl;
  }

  std::cout << "Executor finished spinning" << std::endl;

  if (!g_interrupt_requested.load())
    rclcpp::shutdown();

  node.reset();

  std::cout << "Exiting program" << std::endl;
  return 0;
}
