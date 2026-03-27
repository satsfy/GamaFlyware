

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <queue>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#include <System.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;

bool b_continue_session;

// Global IMU queue and mutex (consider encapsulating these in a class)
std::mutex imu_mutex;
std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue;

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(imu_mutex);
    imu_queue.push(msg);
}

void exit_loop_handler(int s)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    // ros::init(argc, argv, "orb_slam3_webcam");
    // ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<geometry_msgs::Point>("tracked_position", 1000);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("orb_slam3_webcam");
    // auto pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("orb_slam3/pose", 10);
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", 10, imu_callback);

    if (argc < 3 || argc > 4)
    {
        cerr << endl
             << "Usage: ./mono_webcam path_to_vocabulary path_to_settings (trajectory_file_name)" << endl;
        return 1;
    }

    string file_name;
    bool bFileName = false;

    if (argc == 4)
    {
        file_name = string(argv[argc - 1]);
        bFileName = true;
    }

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Initialize network stream capture
    cv::VideoCapture cap("http://localhost:8080/video_feed"); // Replace with your stream URL
    if (!cap.isOpened())
    {
        cerr << "Error: Unable to open the stream" << endl;
        return -1;
    }

    cout << "init1" << endl;

    // Initialize USB webcam
    // cv::VideoCapture cap(4); // Use device 0 (default camera)
    // if (!cap.isOpened())
    // {
    //     cerr << "Error: Unable to open webcam" << endl;
    //     return -1;
    // }

    // // // Set camera resolution (optional)
    int width_img = 854;  // Set this based on your webcam
    int height_img = 480; // Set this based on your webcam
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, width_img);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, height_img);
    // cap.set(cv::CAP_PROP_EXPOSURE, -6); // Decrease exposure for faster shutter speed
    // cap.set(cv::CAP_PROP_FPS, 30);      // Set frame rate to 60 FPS

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    // ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    cout << "init2" << endl;
    float imageScale = SLAM.GetImageScale();
    cout << "init3" << endl;

    cv::Mat imCV;
    double t_resize = 0.f;
    double t_track = 0.f;

    cout << "image scale " << imageScale << endl;
    double last_timestamp_ms = 0;

    while (rclcpp::ok() && b_continue_session)
    {
        rclcpp::spin_some(node);

        double timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count();
        cout << "time since last loop = " << timestamp_ms - last_timestamp_ms << endl;
        last_timestamp_ms = timestamp_ms;

        cap >> imCV; // Capture a frame
        if (imCV.empty())
        {
            cerr << "Error: Webcam frame is empty" << endl;
            break;
        }
        cout << "got frame" << endl;

        cv::resize(imCV, imCV, cv::Size(width_img, height_img));

        // cout << "resized to " << width_img << 'x' << height_img << endl;

        // if (imageScale != 1.f)
        // {
        //     cout << "init10" << endl;
        //     int width = imCV.cols * imageScale;
        //     int height = imCV.rows * imageScale;
        //     cv::resize(imCV, imCV, cv::Size(width_img, height_img));
        // }
        // Display the captured frame
        cv::imshow("Webcam Feed", imCV);
        cout << "init6" << endl;

        // Wait for 1 ms and check if 'q' key is pressed to exit
        if (cv::waitKey(1) == 'q')
        {
            b_continue_session = false;
        }

        cout << "init9" << endl;
        // Resize if necessary
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif
        cout << "feeding to slam" << endl;

        std::lock_guard<std::mutex> lock(imu_mutex);
        sensor_msgs::msg::Imu::SharedPtr imu_msg;
        bool got = false;
        while (!imu_queue.empty())
        {
            got = true;
            auto imu_msg = imu_queue.front();
            cout << "GOT IMU MESSAGE!!" << imu_msg << endl;
            imu_queue.pop();
        }

        if (got && imu_msg != nullptr)
        {
            cout << "b4 21312" << endl;
            cout << imu_msg << endl;
            cout << "aft 12312" << endl;
        }
        else
        {
            cout << "DID NOT GET ANY IMU MESSAGE --------------" << endl;
        }

        SLAM.TrackMonocular(imCV, timestamp_ms);
        // TODO: PASS IMU DATA HERE!!!!!
        // vector<ORB_SLAM3::IMU::Point> imu_point;
        // the IMU thread (passed on initializer) must be present here
        // SLAM.TrackMonocular(imCV, timestamp_ms, imu_point);

        cout << "getting points" << endl;
        for (auto point : SLAM.GetTrackedMapPoints())
        {
            if (!point)
            {
                cerr << "Warning: Encountered a null map point." << endl;
                continue;
            }
            // cout << point << endl;
            // cout << "A" << endl;
            // cout << point->GetNormal() << endl;
            // cout << "B" << endl;
            auto worldPos = point->GetWorldPos();
            cout << "getting points 2" << endl;
            auto x = worldPos.x();
            auto y = worldPos.y();
            auto z = worldPos.z();
            cout << "getting points 3" << endl;
            cout << x << ' ' << y << ' ' << z << endl;
        }

        // geometry_msgs::Point msg;
        // msg.x = x;
        // msg.y = y;
        // msg.z = z;
        // pub.publish(msg);

        // TODO: COLLECT AND SEND IS LOST INFORMATION FROM SLAM.isLost()

        // // Retrieve and publish the current pose.
        // cv::Mat Tcw = SLAM.GetCurrentPose();
        // if (!Tcw.empty())
        // {
        //   geometry_msgs::msg::PoseStamped pose_msg;
        //   pose_msg.header.stamp = node->now();
        //   pose_msg.header.frame_id = "map";
        //   pose_msg.pose.position.x = Tcw.at<double>(0, 3);
        //   pose_msg.pose.position.y = Tcw.at<double>(1, 3);
        //   pose_msg.pose.position.z = Tcw.at<double>(2, 3);

        //   cv::Mat Rcw = Tcw(cv::Rect(0, 0, 3, 3));
        //   cv::Mat rvec;
        //   cv::Rodrigues(Rcw, rvec);
        //   double angle = cv::norm(rvec);
        //   cv::Vec3d axis = rvec / angle;

        //   // Using TF2 to create a quaternion (include tf2/LinearMath/Quaternion.h)
        //   tf2::Quaternion q;
        //   q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);
        //   pose_msg.pose.orientation.x = q.x();
        //   pose_msg.pose.orientation.y = q.y();
        //   pose_msg.pose.orientation.z = q.z();
        //   pose_msg.pose.orientation.w = q.w();

        //   pose_pub->publish(pose_msg);
        // }

        cout << "image sent" << endl;

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }

    // Stop all threads
    SLAM.Shutdown();
    rclcpp::shutdown();
    return 0;
}
