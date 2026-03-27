import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ImuPlotter(Node):
    def __init__(self):
        super().__init__("imu_plotter")
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=15,
        )
        self.subscription = self.create_subscription(
            Imu, "/mavros/imu/data", self.imu_callback, qos_profile
        )
        self.times = []

        # Orientation components (quaternion)
        self.orientation_x = []
        self.orientation_y = []
        self.orientation_z = []
        self.orientation_w = []

        # Angular velocity components
        self.angular_velocity_x = []
        self.angular_velocity_y = []
        self.angular_velocity_z = []

        # Linear acceleration components
        self.linear_acceleration_x = []
        self.linear_acceleration_y = []
        self.linear_acceleration_z = []

        self.start_time = time.time()

    def imu_callback(self, msg):
        current_time = time.time() - self.start_time
        self.times.append(current_time)

        # Orientation (quaternion)
        self.orientation_x.append(msg.orientation.x)
        self.orientation_y.append(msg.orientation.y)
        self.orientation_z.append(msg.orientation.z)
        self.orientation_w.append(msg.orientation.w)

        # Angular velocity (Vector3)
        self.angular_velocity_x.append(msg.angular_velocity.x)
        self.angular_velocity_y.append(msg.angular_velocity.y)
        self.angular_velocity_z.append(msg.angular_velocity.z)

        # Linear acceleration (Vector3)
        self.linear_acceleration_x.append(msg.linear_acceleration.x)
        self.linear_acceleration_y.append(msg.linear_acceleration.y)
        self.linear_acceleration_z.append(msg.linear_acceleration.z)

        self.get_logger().debug(
            f"Time: {current_time:.2f} s, "
            f"Orient: [{msg.orientation.x:.2f}, {msg.orientation.y:.2f}, {msg.orientation.z:.2f}, {msg.orientation.w:.2f}], "
            f"AngVel: [{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}], "
            f"LinAcc: [{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}]"
        )
        print("Received IMU data:", msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    imu_plotter = ImuPlotter()

    # Set up live plot with three subplots
    plt.ion()
    fig, (ax2, ax3) = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
    ax2.set_title("Angular Velocity (rad/s)")
    ax3.set_title("Linear Acceleration (m/s²)")
    ax3.set_xlabel("Time (s)")

    try:
        # Run indefinitely until ROS2 shutdown or keyboard interrupt
        while rclpy.ok():
            # Process incoming messages
            rclpy.spin_once(imu_plotter, timeout_sec=0.1)

            # Clear previous plots
            ax2.cla()
            ax3.cla()

            # Reapply labels and titles
            ax2.set_title("Angular Velocity (rad/s)")
            ax3.set_title("Linear Acceleration (m/s²)")
            ax3.set_xlabel("Time (s)")
            # Plot Angular Velocity components
            ax2.scatter(
                imu_plotter.times,
                imu_plotter.angular_velocity_x,
                color="red",
                s=15,
                label="x",
            )
            ax2.scatter(
                imu_plotter.times,
                imu_plotter.angular_velocity_y,
                color="green",
                s=15,
                label="y",
            )
            ax2.scatter(
                imu_plotter.times,
                imu_plotter.angular_velocity_z,
                color="blue",
                s=15,
                label="z",
            )
            ax2.legend(loc="upper right")

            # Plot Linear Acceleration components
            ax3.scatter(
                imu_plotter.times,
                imu_plotter.linear_acceleration_x,
                color="red",
                s=15,
                label="x",
            )
            ax3.scatter(
                imu_plotter.times,
                imu_plotter.linear_acceleration_y,
                color="green",
                s=15,
                label="y",
            )
            ax3.scatter(
                imu_plotter.times,
                imu_plotter.linear_acceleration_z,
                color="blue",
                s=15,
                label="z",
            )
            ax3.legend(loc="upper right")

            plt.pause(0.0001)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up ROS node and plot
        imu_plotter.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
