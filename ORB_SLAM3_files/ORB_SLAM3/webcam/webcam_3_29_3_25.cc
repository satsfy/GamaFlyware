#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <signal.h>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std;
using namespace std::chrono_literals;

bool ENABLED_IMU = true;

// Define a helper struct to store IMU messages with our computed timestamp.
struct ImuData
{
    double timestamp_ms; // Monotonic timestamp (milliseconds) computed via steady_clock.
    sensor_msgs::msg::Imu::SharedPtr msg;
};

class WebcamSLAMNode : public rclcpp::Node
{
public:
    WebcamSLAMNode(const std::string &vocab_path, const std::string &settings_path)
        : Node("webcam_slam_node"),
          imu_count_(0),
          MIN_IMU_MSGS_REQUIRED(50),
          imu_initialized_(false),
          last_frame_timestamp_ms_(0)
    {
        // Record an initial reference time using the steady clock.
        initial_steady_time_ = std::chrono::steady_clock::now();

        // Set up the IMU subscription.
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&WebcamSLAMNode::imu_callback, this, std::placeholders::_1));

        // Publisher for ORB-SLAM3 pose output.
        post_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/orbslam3/pose", rclcpp::SensorDataQoS());

        // Open the video stream.
        cap_.open("http://localhost:8080/video_feed");
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open the video stream");
            rclcpp::shutdown();
            return;
        }
        width_img_ = 854;
        height_img_ = 480;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_img_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_img_);

        // Create ORB-SLAM3 system (with IMU if enabled).
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

        // Create a timer for processing frames every 50ms.
        main_timer_ = this->create_wall_timer(
            50ms, std::bind(&WebcamSLAMNode::timer_callback, this));
    }

    ~WebcamSLAMNode()
    {
        if (slam_system_)
        {
            slam_system_->Shutdown();
        }
        cv::destroyAllWindows();
    }

private:
    // --- Member variables ---
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr post_pub_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    cv::VideoCapture cap_;
    int width_img_, height_img_;
    float image_scale_;
    std::unique_ptr<ORB_SLAM3::System> slam_system_;

    // We use the steady clock for a common, monotonic time basis.
    std::chrono::time_point<std::chrono::steady_clock> initial_steady_time_;

    // IMU synchronization variables.
    int imu_count_;
    const int MIN_IMU_MSGS_REQUIRED; // e.g., 50 messages required before processing frames.
    bool imu_initialized_;
    uint64_t last_frame_timestamp_ms_;

    std::mutex imu_mutex_;
    std::queue<ImuData> imu_queue_;

    // --- IMU callback: record a monotonic timestamp and enqueue the message.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto now_steady = std::chrono::steady_clock::now();
        double ts = std::chrono::duration_cast<std::chrono::milliseconds>(now_steady - initial_steady_time_).count();
        ImuData data{ts, msg};

        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_queue_.push(data);
        imu_count_++;
    }

    // --- Timer callback: process frames after waiting for sufficient IMU data.
    void timer_callback()
    {
        // Compute elapsed time (in ms) relative to the initial steady time.
        auto now_steady = std::chrono::steady_clock::now();
        uint64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_steady - initial_steady_time_).count();

        // Delay frame processing until we've collected enough IMU messages and waited a minimum delay.
        const int MIN_STARTUP_DELAY_MS = 3000; // 3 seconds delay
        if (!imu_initialized_)
        {
            if (imu_count_ < MIN_IMU_MSGS_REQUIRED || elapsed_ms < MIN_STARTUP_DELAY_MS)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Waiting for IMU initialization: %d msgs, elapsed: %ld ms",
                            imu_count_, elapsed_ms);
                return;
            }
            else
            {
                imu_initialized_ = true;
                RCLCPP_INFO(this->get_logger(),
                            "IMU Initialized after %ld ms with %d messages",
                            elapsed_ms, imu_count_);
            }
        }

        // Use the elapsed time (in ms) as the frame timestamp.
        uint64_t frame_timestamp_ms = elapsed_ms;

        // Ensure timestamps are strictly increasing.
        if (frame_timestamp_ms <= last_frame_timestamp_ms_)
        {
            RCLCPP_WARN(this->get_logger(), "Non-monotonic frame timestamp detected. Skipping frame.");
            return;
        }
        last_frame_timestamp_ms_ = frame_timestamp_ms;

        // Capture a frame.
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Webcam frame is empty");
            rclcpp::shutdown();
            return;
        }
        cv::resize(frame, frame, cv::Size(width_img_, height_img_));

        // Optionally display the frame.
        cv::imshow("Webcam Feed", frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            RCLCPP_INFO(this->get_logger(), "Quit key pressed, shutting down.");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "frame_timestamp_ms: %lld", frame_timestamp_ms);
        // Process all queued IMU messages using our computed (steady) timestamps.
        std::vector<ORB_SLAM3::IMU::Point> imu_points;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            while (!imu_queue_.empty())
            {
                ImuData data = imu_queue_.front();
                imu_queue_.pop();

                // Use our computed timestamp (relative to the same reference) for each IMU measurement.
                double imu_timestamp_ms = data.timestamp_ms;
                float acc_x = data.msg->linear_acceleration.x;
                float acc_y = data.msg->linear_acceleration.y;
                float acc_z = data.msg->linear_acceleration.z;
                float ang_vel_x = data.msg->angular_velocity.x;
                float ang_vel_y = data.msg->angular_velocity.y;
                float ang_vel_z = data.msg->angular_velocity.z;

                RCLCPP_INFO(this->get_logger(), "imu_timestamp_ms: %d", imu_timestamp_ms);
                ORB_SLAM3::IMU::Point pt(acc_x, acc_y, acc_z,
                                         ang_vel_x, ang_vel_y, ang_vel_z, imu_timestamp_ms);
                imu_points.push_back(pt);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Feeding frame to SLAM with %ld IMU points", imu_points.size());

        // Feed the frame (and IMU measurements if enabled) to ORB‑SLAM3.
        if (ENABLED_IMU)
        {
            slam_system_->TrackMonocular(frame, frame_timestamp_ms, imu_points);
        }
        else
        {
            slam_system_->TrackMonocular(frame, frame_timestamp_ms);
        }

        // (Optional) Retrieve and publish tracked map points.
        auto map_points = slam_system_->GetTrackedMapPoints();
        bool found_point = false;
        for (auto point : map_points)
        {
            if (!point)
                continue;
            found_point = true;
            auto worldPos = point->GetWorldPos();
            geometry_msgs::msg::Point out_msg;
            out_msg.x = worldPos.x();
            out_msg.y = worldPos.y();
            out_msg.z = worldPos.z();
            post_pub_->publish(out_msg);
            RCLCPP_INFO(this->get_logger(), "Published map point: [%.3f, %.3f, %.3f]",
                        out_msg.x, out_msg.y, out_msg.z);
        }
        if (!found_point)
        {
            RCLCPP_WARN(this->get_logger(), "Warning: no valid map point found.");
        }

        auto end_steady = std::chrono::steady_clock::now();
        auto callback_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_steady - now_steady).count();
        std::cout << "Timer callback duration: " << callback_duration << " ms" << std::endl;
    }
};

int main(int argc, char **argv)
{
    cout << "COMPILED VERSION V2" << endl;
    if (argc < 3 || argc > 4)
    {
        std::cerr << "Usage: ./webcam_slam_node path_to_vocabulary path_to_settings (trajectory_file_name)" << std::endl;
        return 1;
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebcamSLAMNode>(argv[1], argv[2]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
