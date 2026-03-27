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

class WebcamSLAMNode : public rclcpp::Node
{
public:
    // Constructor takes vocabulary and settings file paths.
    WebcamSLAMNode(const std::string &vocab_path, const std::string &settings_path)
        : Node("webcam_slam_node")
    {
        initial_timestamp = std::chrono::high_resolution_clock::now();

        // Set up IMU subscription.
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&WebcamSLAMNode::imu_callback, this, std::placeholders::_1));

        // Create a publisher for geometry_msgs::msg::Point messages on "/orbslam3/pose"
        post_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/orbslam3/pose", rclcpp::SensorDataQoS());

        // Open the video stream (adjust URL or device index as needed).
        cap_.open("http://localhost:8080/video_feed");
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Unable to open the video stream");
            rclcpp::shutdown();
            return;
        }
        // Set desired resolution.
        width_img_ = 854;
        height_img_ = 480;
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_img_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_img_);

        if (ENABLED_IMU)
        {
            slam_system_ = std::make_unique<ORB_SLAM3::System>(vocab_path, settings_path, ORB_SLAM3::System::IMU_MONOCULAR, true);
        }
        else
        {
            slam_system_ = std::make_unique<ORB_SLAM3::System>(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, true);
        }

        // Get image scale (if needed).
        image_scale_ = slam_system_->GetImageScale();
        RCLCPP_INFO(this->get_logger(), "Initialized SLAM with image scale: %f", image_scale_);

        // Create a timer for the main loop
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
    // IMU subscription callback: push incoming messages onto a queue.
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_queue_.push(msg);
    }

    // Timer callback: read webcam image, process queued IMU messages, and feed frame to SLAM.
    void timer_callback()
    {
        auto start = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::system_clock::now();
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        // Capture a frame.
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error: Webcam frame is empty");
            rclcpp::shutdown();
            return;
        }
        // Resize frame to desired resolution.
        cv::resize(frame, frame, cv::Size(width_img_, height_img_));

        // std::cout << "frame.cols=" << frame.cols << ", frame.rows=" << frame.rows << std::endl;

        // cv::Rect gearRegion(
        //     0,               // x
        //     frame.rows - 140, // y
        //     60,      // width
        //     140               // height
        // );
        // cv::rectangle(frame, gearRegion, cv::Scalar(0, 0, 0), cv::FILLED);

        // cv::Rect gearRegion(
        //     frame.rows-140,               // x
        //     frame.rows - 140, // y
        //     60,      // width
        //     140               // height
        // );
        // cv::rectangle(frame, gearRegion, cv::Scalar(0, 0, 0), cv::FILLED);

        // 2) Now show the result
        // Display the captured frame.
        cv::imshow("Webcam Feed", frame);
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            RCLCPP_INFO(this->get_logger(), "Quit key pressed, shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Process any queued IMU messages.
        sensor_msgs::msg::Imu::SharedPtr last_imu_msg = nullptr;
        std::vector<ORB_SLAM3::IMU::Point> points;

        if (ENABLED_IMU)
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            while (!imu_queue_.empty())
            {
                last_imu_msg = imu_queue_.front();
                imu_queue_.pop();

                float acc_x = last_imu_msg->linear_acceleration.x;
                float acc_y = last_imu_msg->linear_acceleration.y;
                float acc_z = last_imu_msg->linear_acceleration.z;
                float ang_vel_x = last_imu_msg->angular_velocity.x;
                float ang_vel_y = last_imu_msg->angular_velocity.y;
                float ang_vel_z = last_imu_msg->angular_velocity.z;
                double imu_timestamp = double(last_imu_msg->header.stamp.sec);

                auto t1 = std::chrono::high_resolution_clock::now();
                float incT = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - initial_timestamp).count();

                std::cout << "Linear Acceleration: ["
                          << acc_x << ", " << acc_y << ", " << acc_z << "]\n"
                          << "Angular Velocity: ["
                          << ang_vel_x << ", " << ang_vel_y << ", " << ang_vel_z << "]\n"
                          << "IMU Timestamp: " << timestamp_ms << "\n"
                          << std::endl;

                auto newpt = ORB_SLAM3::IMU::Point(acc_x, acc_y, acc_z,
                                                   ang_vel_x, ang_vel_y, ang_vel_z, timestamp_ms);
                points = std::vector<ORB_SLAM3::IMU::Point>();
                points.push_back(newpt);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Feeding frame to SLAM with %ld IMU points", points.size());

        if (ENABLED_IMU)
        {
            slam_system_->TrackMonocular(frame, timestamp_ms, points);
        }
        else
        {
            slam_system_->TrackMonocular(frame, timestamp_ms);
        }

        auto map_points = slam_system_->GetTrackedMapPoints();
        bool found_point = false;
        for (auto point : map_points)
        {
            if (!point)
                continue;

            found_point = true;
            auto worldPos = point->GetWorldPos();

            geometry_msgs::msg::Point msg;
            msg.x = worldPos.x();
            msg.y = worldPos.y();
            msg.z = worldPos.z();
            post_pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published this: [%.3f, %.3f, %.3f]",
                        msg.x, msg.y, msg.z);
        }
        if (!found_point)
        {
            cout << "RETRIEVED A LIST OF " << map_points.size() << " TRACKED MAP POINTS" << endl;
            RCLCPP_WARN(this->get_logger(), "Warning: not valid map point found.");
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Timer callback duration: " << duration << " ms" << std::endl;
    }

    // Member variables.
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr post_pub_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    cv::VideoCapture cap_;
    int width_img_, height_img_;
    float image_scale_;
    std::unique_ptr<ORB_SLAM3::System> slam_system_;

    std::chrono::time_point<std::chrono::high_resolution_clock> initial_timestamp;

    std::mutex imu_mutex_;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
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
