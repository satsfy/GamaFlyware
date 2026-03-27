#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class CombinedNode : public rclcpp::Node
{
public:
  CombinedNode()
      : Node("combined_node"), counter_(0)
  {
    // Create a publisher for "increment_topic" with a queue size of 10.
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("increment_topic", 10);

    // Create a subscription for the IMU data on "/mavros/imu/data" with SensorDataQoS.
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", rclcpp::SensorDataQoS(),
        std::bind(&CombinedNode::imu_callback, this, std::placeholders::_1));

    // Timer to publish the incrementing integer every 1 second.
    increment_timer_ = this->create_wall_timer(
        1s, std::bind(&CombinedNode::publish_increment, this));

    // Timer to process and print any IMU messages every 100 ms.
    imu_timer_ = this->create_wall_timer(
        100ms, std::bind(&CombinedNode::process_imu_queue, this));
  }

private:
  // Callback for the increment timer.
  void publish_increment()
  {
    auto message = std_msgs::msg::Int32();
    message.data = counter_;
    RCLCPP_INFO(this->get_logger(), "Publishing increment: %d", counter_);
    publisher_->publish(message);
    counter_++;
  }

  // Callback for the IMU subscription.
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_queue_.push(msg);
  }

  // Timer callback to process the IMU queue.
  void process_imu_queue()
  {
    sensor_msgs::msg::Imu::SharedPtr last_imu_msg = nullptr;
    bool got_message = false;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      while (!imu_queue_.empty())
      {
        last_imu_msg = imu_queue_.front();
        std::cout << "GOT IMU MESSAGE 2!!" << std::endl;
        std::cout << "Orientation: ["
                  << last_imu_msg->orientation.x << ", "
                  << last_imu_msg->orientation.y << ", "
                  << last_imu_msg->orientation.z << ", "
                  << last_imu_msg->orientation.w << "]" << std::endl;
        imu_queue_.pop();
        got_message = true;
      }
    }

    if (got_message)
    {
      std::cout << "Processed an IMU message." << std::endl;
    }
    else
    {
      std::cout << "DID NOT GET ANY IMU MESSAGE --------------" << std::endl;
    }
  }

  // Member variables.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr increment_timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  int counter_;

  std::mutex imu_mutex_;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CombinedNode>());
  rclcpp::shutdown();
  return 0;
}
