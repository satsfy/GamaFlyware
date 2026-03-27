#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped

class SimpleTakeoffNode(Node):
    def __init__(self):
        super().__init__('simple_takeoff_node')

        # Current drone state
        self.current_state = State()
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # Current position
        self.current_pose = PoseStamped()
        self.local_position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.local_position_callback, 10)

        # Setpoint publisher
        self.local_position_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to become available
        self.get_logger().info("Waiting for arming and mode services...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        self.get_logger().info("Arming and mode services are available.")

        # Setpoint message
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.5  # Desired x position
        self.pose.pose.position.y = -0.5  # Desired y position
        self.pose.pose.position.z = 2.0  # Desired altitude (z position)

        # Setpoint publishing must be faster than 2Hz
        self.setpoint_timer_period = 0.1  # 10 Hz
        self.setpoint_timer = self.create_timer(self.setpoint_timer_period, self.setpoint_publish)

        # Timer for state checking and mode setting
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_request = self.get_clock().now()

    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_pose = msg

    def distance_to_target(self):
        dx = self.pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.pose.pose.position.y - self.current_pose.pose.position.y
        dz = self.pose.pose.position.z - self.current_pose.pose.position.z
        return (dx**2 + dy**2 + dz**2) ** 0.5

    def setpoint_publish(self):
        distance = self.distance_to_target()
        if distance > 0.1:  # Tolerance of 10 cm
            self.local_position_pub.publish(self.pose)
        else:
            self.get_logger().info("Reached target position.")
            # Optionally, land or hover
            # self.land()
            # self.setpoint_timer.cancel()  # Stop the timer if desired

    def timer_callback(self):
        now = self.get_clock().now()
        if self.current_state.mode != "OFFBOARD" and (now - self.last_request) > Duration(seconds=1.0):
            # Set OFFBOARD mode
            self.get_logger().info("Setting OFFBOARD mode...")
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.set_mode_response_callback)
            self.last_request = now
        elif not self.current_state.armed and (now - self.last_request) > Duration(seconds=1.0):
            # Arm the vehicle
            self.get_logger().info("Arming the vehicle...")
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response_callback)
            self.last_request = now

    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode set successfully")
            else:
                self.get_logger().error("Failed to set OFFBOARD mode")
        except Exception as e:
            self.get_logger().error(f"SetMode call failed: {e}")

    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().error("Failed to arm vehicle")
        except Exception as e:
            self.get_logger().error(f"Arming call failed: {e}")

    # Optional: Implement landing
    def land(self):
        self.get_logger().info("Initiating landing...")
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.land_response_callback)

    def land_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("Landing initiated successfully")
            else:
                self.get_logger().error("Failed to initiate landing")
        except Exception as e:
            self.get_logger().error(f"Landing call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoffNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
