#!/usr/bin/env python3

# Biblioteca padrão
import time
import os
import logging
from datetime import datetime

# Outras bibliotecas
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import logging  # Add this line to import the logging module
from scipy.spatial.transform import (
    Rotation as R,
)  # Use scipy for quaternion to Euler conversion

# Arquivos locais
from .attitude import Attitude
from .trajectory import Trajectory
import math
import numpy as np
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

SECOND_IN_NS = 1e9
MILLISECOND_IN_NS = 1e6
MICROSECOND_IN_NS = 1e3

TOPICS_MAVROS_MODE = True
REG_LOGFILE = False


class RosConfig(Node):
    """
    Classe base para cuidar de todas as configurações internas de ROS padrões
    Isso evitar poluição do código principal.
    """

    def __init__(self, name: str = "ros_config") -> None:
        super().__init__(name)

        self.trajectory = Trajectory()

        # Configure QoS profile for publishing and subscribing
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1,
        # )

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # if not TOPICS_MAVROS_MODE:
        #     # Create publishers
        #     self.offboard_control_mode_publisher = self.create_publisher(
        #         OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile
        #     )
        #     self.trajectory_setpoint_publisher = self.create_publisher(
        #         TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile
        #     )
        #     self.vehicle_command_publisher = self.create_publisher(
        #         VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        #     )

        if TOPICS_MAVROS_MODE:
            self.current_pose = PoseStamped()

            self.local_position_sub = self.create_subscription(
                PoseStamped,
                "/mavros/local_position/pose",
                self.vehicle_local_position_callback,
                qos_profile,
            )
        else:
            self.vehicle_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition,
                "/fmu/out/vehicle_local_position",
                self.vehicle_local_position_callback,
                qos_profile,
            )

        # self.vehicle_status_subscriber = self.create_subscription(
        #     VehicleStatus,
        #     "/fmu/out/vehicle_status",
        #     self.vehicle_status_callback,
        #     qos_profile,
        # )

        self.camera_subscription = self.create_subscription(
            Image, "camera", self.camera_callback, 10
        )
        self.camera_subscription  # prevent unused variable warning

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.bridge = CvBridge()

        self.log_level = 4

        self.enable_simple_setpoint_publish = True
        self.last_setpoint = None
        self.last_setpoint2 = None
        self.last_published_setpoint = None
        self.setpoint_last_call = 0
        self.heartbeat_last_call = 0

        self.setpoint_publish_period = 200 * MILLISECOND_IN_NS
        self.heartbeat_publish_period = 10 * MILLISECOND_IN_NS
        self.min_setpoint_delta = 0

        self.configure_logging()
        self.get_logger().setLevel(logging.DEBUG)

        self.create_timer(0.1, self.loop_principal)

        self.create_timer(0.01, self.ros_loop)

        # ------------------------------------------------------------

        # Current drone state
        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_callback, 10
        )

        # Setpoint publisher
        self.local_position_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        self.velocity_target_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )

        # ================================= POS STATE
        # ------------------------------------------------------------
        # Additional states for velocity-based control
        self.estimated_position = None
        self.prev_time_ns = None
        self.max_velocity = 1.0  # m/s
        self.threshold = 0.15  # meters - stop distance

        # ================================= END POS STATE

        # Service clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services to become available
        self.get_logger().info("Waiting for arming and mode services...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        self.get_logger().info("Arming and mode services are available.")

        # Setpoint message
        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0  # Desired x position
        self.pose.pose.position.y = 0.0  # Desired y position
        self.pose.pose.position.z = 2.0  # Desired altitude (z position)

        # Timer for state checking and mode setting
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_request = self.get_clock().now()

    def set_setpoint_message(self, msg, force: bool = False):
        self.last_setpoint = msg
        self.logger.debug(f"set_setpoint_message called {msg}")
        if (
            not TOPICS_MAVROS_MODE
            and self.last_published_setpoint is not None
            and not force
        ):
            if (
                abs(self.last_published_setpoint.position[0] - msg.position[0])
                < self.min_setpoint_delta
                and abs(self.last_published_setpoint.position[1] - msg.position[1])
                < self.min_setpoint_delta
                and abs(self.last_published_setpoint.position[2] - msg.position[2])
                < self.min_setpoint_delta
            ):
                self.logger.warning(
                    f"ignoring setpoint because it is too close to last setpoint msg={msg.position} last={self.last_published_setpoint.position}"
                )

    def get_last_setpoint_x_y_z(self) -> tuple[float, float, float]:
        if self.last_setpoint is None:
            raise ValueError("No setpoint available")
        return tuple(self.last_setpoint.position)

    def ros_loop(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.enable_simple_setpoint_publish:
            self.handle_setpoint_publish()
        self.handle_setpoint_publish2()

    def loop(self):
        while True:
            self.handle_setpoint_publish2()
            time.sleep(0.01)
            

    def publish_position_setpoint2(self, x: float, y: float, z: float, heading: float = None, force: bool = False):
        self.enable_simple_setpoint_publish = False

        # Avoid absolute zeros which might cause issues.
        x = x if x != 0 else 1e-6
        y = y if y != 0 else 1e-6
        z = z if z != 0 else 1e-6

        # Create target position vector.
        target_pos = np.array([x, y, z])
        
        # Use the last published setpoint as the current position.
        # If not available, initialize current_pos to target_pos.
        if hasattr(self, 'last_published_setpoint') and self.last_published_setpoint is not None:
            current_pos = np.array(self.last_published_setpoint.position)
        else:
            current_pos = target_pos

        # Smoothing factor (0 < alpha <= 1): lower values yield smoother, slower updates.
        alpha = 0.05  # Adjust this value based on how smooth you need the motion to be.

        # Compute the new, smoothed position by interpolating between current and target.
        smoothed_pos = current_pos + alpha * (target_pos - current_pos)

        # Optionally, if the difference is minimal, jump directly to the target to avoid endless interpolation.
        if np.linalg.norm(target_pos - smoothed_pos) < 1e-3:
            smoothed_pos = target_pos

        # Create a new trajectory setpoint message.
        msg = TrajectorySetpoint()
        msg.position = smoothed_pos.tolist()

        # Set the yaw (heading) as before.
        msg.yaw = float(heading) if heading is not None else 1.57079

        # Compute a velocity command based on the change in position.
        dt = 0.01  # Time step (in seconds); ensure this matches your loop rate.
        velocity = (smoothed_pos - current_pos) / dt
        msg.velocity = velocity.tolist()

        # Set the timestamp (assuming get_timestamp_nanoseconds returns nanoseconds).
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)

        # Store the setpoint and log the update.
        self.last_setpoint2 = msg
        self.logger.debug(f"set_setpoint_message2 called {msg}")


    def handle_setpoint_publish2(self):
        # self.handle_setpoint_publish_with_vel()
        # return
        timenow = self.get_timestamp_nanoseconds()
        if (
            self.setpoint_last_call + self.setpoint_publish_period < timenow
            and self.last_setpoint2 is not None
        ):
            x, y, z = self.last_setpoint2.position
            self.logger.debug(f"2 actually publishing setpoint x={x}, y={y}, z={z}")

            self.pose.pose.position.x = float(x)
            self.pose.pose.position.y = float(y)
            self.pose.pose.position.z = float(z)
            print("self.pose2:", self.pose)
            self.local_position_pub.publish(self.pose)

            self.setpoint_last_call = timenow
            self.last_published_setpoint = self.last_setpoint2


    def handle_setpoint_publish(self):
        # self.handle_setpoint_publish_with_vel()
        # return
        timenow = self.get_timestamp_nanoseconds()
        if (
            self.setpoint_last_call + self.setpoint_publish_period < timenow
            and self.last_setpoint is not None
        ):
            x, y, z = self.last_setpoint.position
            self.logger.debug(f"actually publishing setpoint x={x}, y={y}, z={z}")
            if TOPICS_MAVROS_MODE:
                self.pose.pose.position.x = float(x)
                self.pose.pose.position.y = float(y)
                self.pose.pose.position.z = float(z)

                print("self.pose:", self.pose)
                self.local_position_pub.publish(self.pose)
            else:
                self.trajectory_setpoint_publisher.publish(self.last_setpoint)
            self.setpoint_last_call = timenow
            self.last_published_setpoint = self.last_setpoint

    # def handle_setpoint_publish(self):
    #     """
    #     Publish velocity setpoints that move toward 'last_setpoint' in a smoother way
    #     without overcorrecting. We track our own 'estimated_position' each cycle
    #     so we can decide when to stop.
    #     """
    #     timenow = self.get_timestamp_nanoseconds()

    #     # Only publish if enough time has passed AND we actually have a target
    #     if (
    #         self.setpoint_last_call + self.setpoint_publish_period < timenow
    #         and self.last_setpoint is not None
    #     ):
    #         # If this is the first time we're publishing for a new target,
    #         # or we haven't initialized our estimated_position yet...
    #         if self.estimated_position is None:
    #             # Initialize our "estimated_position" to the actual current_pose
    #             # we have from MAVROS (or zero if none available).
    #             curr_x = self.current_pose.pose.position.x
    #             curr_y = self.current_pose.pose.position.y
    #             curr_z = self.current_pose.pose.position.z

    #             self.estimated_position = np.array([curr_x, curr_y, curr_z], dtype=float)
    #             self.prev_time_ns = timenow

    #         # Compute time delta in seconds since last publish
    #         dt = (timenow - self.prev_time_ns) * 1e-9  # ns -> seconds
    #         if dt <= 0:
    #             dt = 0.01  # fallback to avoid division by zero

    #         # Extract the target position
    #         target_x, target_y, target_z = self.last_setpoint.position
    #         target_pos = np.array([target_x, target_y, target_z], dtype=float)

    #         # Current estimated position
    #         curr_pos = self.estimated_position

    #         # Compute distance to goal
    #         diff = target_pos - curr_pos
    #         dist = np.linalg.norm(diff)

    #         # Decide on velocity
    #         if dist < self.threshold:
    #             # Close enough: hover in place (zero velocity)
    #             vx, vy, vz = 0.0, 0.0, 0.0
    #         else:
    #             # Move at a fixed speed in the direction of the target
    #             direction = diff / dist  # unit vector
    #             vx = direction[0] * self.max_velocity
    #             vy = direction[1] * self.max_velocity
    #             vz = direction[2] * self.max_velocity

    #         # Update the internal estimate of position
    #         # (assuming the drone actually moves at the commanded velocity for dt)
    #         self.estimated_position += np.array([vx, vy, vz]) * dt

    #         # Build a PositionTarget message
    #         velocity_target = PositionTarget()
    #         velocity_target.header.stamp = self.get_clock().now().to_msg()
    #         velocity_target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

    #         # We want to ignore position, acceleration, yaw
    #         velocity_target.type_mask = (
    #             PositionTarget.IGNORE_PX
    #             | PositionTarget.IGNORE_PY
    #             | PositionTarget.IGNORE_PZ
    #             | PositionTarget.IGNORE_AFX
    #             | PositionTarget.IGNORE_AFY
    #             | PositionTarget.IGNORE_AFZ
    #             | PositionTarget.IGNORE_YAW
    #             | PositionTarget.IGNORE_YAW_RATE
    #         )

    #         velocity_target.velocity.x = vx
    #         velocity_target.velocity.y = vy
    #         velocity_target.velocity.z = vz

    #         # Publish velocity command
    #         self.velocity_target_pub.publish(velocity_target)

    #         # Update timing
    #         self.prev_time_ns = timenow
    #         self.setpoint_last_call = timenow
    #         self.last_published_setpoint = self.last_setpoint

    #         # Optional: Log debug info
    #         self.logger.debug(
    #             f"[handle_setpoint_publish] dt={dt:.3f}s dist={dist:.2f} -> "
    #             f"vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, "
    #             f"est_pos={self.estimated_position}"
    #         )

    def publish_offboard_control_heartbeat_signal(self):
        timenow = self.get_timestamp_nanoseconds()
        if self.heartbeat_last_call + self.heartbeat_publish_period < timenow:
            msg = OffboardControlMode()
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
            msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
            if TOPICS_MAVROS_MODE:
                pass
                # TODO: implement
            else:
                self.offboard_control_mode_publisher.publish(msg)
            self.heartbeat_last_call = timenow

    def camera_callback(self, msg):
        raise NotImplementedError("Método camera_callback não implementado")

    def loop_principal(self):
        raise NotImplementedError("Método loop_principal não implementado")

    def custom_sleep(self, seconds):
        start = self.get_timestamp_nanoseconds()
        while self.get_timestamp_nanoseconds() - start < seconds * 1e9:
            self.ros_loop()
            time.sleep(0.01)

    def vehicle_local_position_callback(
        # self, vehicle_local_position: VehicleLocalPosition
        self,
        vehicle_local_position,
    ):
        """Callback function for vehicle_local_position topic subscriber."""

        x, y, z = [-1000000, -1000000, -1000000]
        heading = 0

        # self.logger.info(f"vehicle_local_position_callback msg={vehicle_local_position}")
        if TOPICS_MAVROS_MODE:
            self.current_pose = vehicle_local_position
            x = vehicle_local_position.pose.position.x
            y = vehicle_local_position.pose.position.y
            z = vehicle_local_position.pose.position.z

            orientation_q = vehicle_local_position.pose.orientation
            # Quaternion order in ROS is [x, y, z, w]
            orientation_list = [
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w,
            ]
            r = R.from_quat(orientation_list)
            (roll, pitch, yaw) = r.as_euler("xyz", degrees=False)
            heading = yaw
        else:
            self.vehicle_local_position = vehicle_local_position
            x = vehicle_local_position.x
            y = vehicle_local_position.y
            z = vehicle_local_position.z
            heading = vehicle_local_position.heading
        time_ns = self.get_timestamp_nanoseconds()

        # self.logger.info(f"ADICIONA NO HISTORICO!!! ")
        self.trajectory.add(Attitude(time_ns, x, y, z, heading))
        self.logger.debug(
            f"Current position: x={x}, y={y}, z={z}, heading={heading}",
        )

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def publish_position_setpoint(
        self, x: float, y: float, z: float, heading: float = None, force: bool = False
    ):
        """Publish the trajectory setpoint."""
        # self.logger.info(
        #     f"Overwrite position setpoint: pos={[x, y, z]}" ,
        # )
        msg = TrajectorySetpoint()
        if x == 0:
            x = 0.000001
        if y == 0:
            y = 0.000001
        if z == 0:
            z = 0.000001
        if heading is not None:
            msg.yaw = float(heading)
        else:
            msg.yaw = 1.57079  # (90 degree)

        msg.position = [x, y, z]
        msg.velocity = [0.0, 0.0, 0.0]  # Set the desired velocity components
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
        self.set_setpoint_message(msg, force)

    def publish_position_setpoint_with_interpolation(
        self,
        x: float,
        y: float,
        z: float,
        num_steps: int = 50,
        delay: float = 0.05,
    ):
        """Publish the trajectory setpoint with slower movement."""
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z

        for i in range(1, num_steps + 1):
            interp_x = current_x + (x - current_x) * (i / num_steps)
            interp_y = current_y + (y - current_y) * (i / num_steps)
            interp_z = current_z + (z - current_z) * (i / num_steps)

            self.logger.debug(
                f"Publishing intermediate position setpoint: {[interp_x, interp_y, interp_z]}",
            )

            self.publish_position_setpoint(interp_x, interp_y, interp_z)
            self.custom_sleep(delay)

        self.publish_position_setpoint(x, y, z)
        self.logger.info(
            f"Reached final position setpoint: {[x, y, z]}",
        )

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        """Publish the trajectory setpoint with velocity components."""
        msg = TrajectorySetpoint()
        msg.position = [
            float("nan"),
            float("nan"),
            float("nan"),
        ]  # Use NaN to indicate no position setpoint
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
        self.set_setpoint_message(msg)
        # self.trajectory_setpoint_publisher.publish(msg)
        self.logger.info(f"Publishing velocity setpoint: {[vx, vy, vz]}, yaw: {yaw}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
        if TOPICS_MAVROS_MODE:
            pass
            # TODO: implement
        else:
            self.vehicle_command_publisher.publish(msg)

    def get_timestamp_nanoseconds(self) -> int:
        return self.get_clock().now().nanoseconds

    def arm(self):
        if TOPICS_MAVROS_MODE:
            now = self.get_clock().now()
            if not self.current_state.armed and (now - self.last_request) > Duration(
                seconds=1.0
            ):
                self.get_logger().info("Arming the vehicle...")
                req = CommandBool.Request()
                req.value = True
                future = self.arming_client.call_async(req)
                future.add_done_callback(self.arm_response_callback)
                self.last_request = now
        else:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
                param2=21196.0,
            )
        self.logger.info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.logger.info("Disarm command sent")

    def takeoff(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF)
        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_VTOL_TAKEOFF)
        self.logger.info("Takeoff command sent")

    def engage_offboard_mode(self):
        if TOPICS_MAVROS_MODE:
            now = self.get_clock().now()
            if self.current_state.mode != "OFFBOARD" and (
                now - self.last_request
            ) > Duration(seconds=1.0):
                # Set OFFBOARD mode
                self.get_logger().info("Setting OFFBOARD mode...")
                req = SetMode.Request()
                req.custom_mode = "OFFBOARD"
                future = self.set_mode_client.call_async(req)
                future.add_done_callback(self.set_mode_response_callback)
                self.last_request = now
        else:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )
        self.logger.info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.logger.info("Switching to land mode")

    def configure_logging(self):
        self.logger = logging.getLogger("ros_config_logger")

        if REG_LOGFILE:
            # Create logs directory if it doesn't exist
            if not os.path.exists("logs"):
                os.makedirs("logs")

            debug_log_filename = datetime.now().strftime("logs/%Y%m%d_%H%M%S_debug.log")
            info_log_filename = datetime.now().strftime("logs/%Y%m%d_%H%M%S_info.log")

            # File handler for INFO level logs
            file_handler1 = logging.FileHandler(info_log_filename)
            file_handler1.setLevel(logging.INFO)
            file_handler1.setFormatter(
                logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
            )
            self.logger.addHandler(file_handler1)

            # File handler for DEBUG level logs
            file_handler2 = logging.FileHandler(debug_log_filename)
            file_handler2.setLevel(logging.DEBUG)
            file_handler2.setFormatter(
                logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
            )
            self.logger.addHandler(file_handler2)

        # Console handler for DEBUG level logs
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.DEBUG)
        console_handler.setFormatter(
            logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        )
        self.logger.addHandler(console_handler)

    def get_logger(self):
        return self.logger

    def state_callback(self, msg):
        self.current_state = msg

    def local_position_callback(self, msg):
        self.current_pose = msg

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
