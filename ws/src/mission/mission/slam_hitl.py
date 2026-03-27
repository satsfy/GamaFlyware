import sys
import select
import math
import time
import os
import logging
from datetime import datetime
import numpy as np

import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

from mavros_msgs.msg import State, PositionTarget, GPSRAW
from mavros_msgs.srv import CommandBool, SetMode

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Image

from std_msgs.msg import Header
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from scipy.spatial.transform import Rotation as R

# Local modules
from .attitude import Attitude
from .trajectory import Trajectory

import time
import sys, termios, tty, select, threading
from pymavlink import mavutil
import signal
from enum import Enum


def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


TAKEOFF_HEIGHT = 2.3
SECOND_IN_NS = 1e9
MILLISECOND_IN_NS = 1e6
MICROSECOND_IN_NS = 1e3

TOPICS_MAVROS_MODE = True
REG_LOGFILE = True
KEYBOARD_CMD_ENABLED = True


class RuntimeGPSState(Enum):
    GPS_MANUAL = 1
    SLAM_MANUAL = 2
    FEEDBACK = 3
    NONE = 4


class Coordenadas:
    def __init__(self, override_bases):
        self.new_bases = []

        self.bases = [
            [-10.0, -10.0, -10.0, 1005, 0, 0],
            [-10.0, +10.0, -10.0, 1005, 0, 0],
            [5.4772255750, -10.0, -10.0, 1005, 0, 0],
        ]
        cx = 1
        cy = 1
        dx = +1.1
        dy = +1.5

        for i in range(len(self.bases)):
            self.new_bases.append([-1000000000, -10000000000, -1000000000])
            self.new_bases[i][0] = (self.bases[i][0] + dx) * cx
            self.new_bases[i][1] = (self.bases[i][1] + dy) * cy
            self.new_bases[i][2] = self.bases[i][2]

        self.caixas_visitadas = []
        self.indice_caixa_atual = None

    def find_next_base(self) -> int:
        next_base = 0
        while next_base in self.caixas_visitadas:
            next_base += 1
        if next_base >= len(self.new_bases):
            # REMOVE
            self.caixas_visitadas = []
            return self.find_next_base()
            # END OF REMOVE
            self.indice_base_atual = None
            return None
        self.indice_base_atual = next_base
        return next_base

    def get_posicao_base_atual(self) -> tuple[float, float, float]:
        """
        retorna x,y,z da base atual
        """
        if (
            self.indice_base_atual is None
            or self.indice_base_atual >= len(self.new_bases)
            or self.indice_base_atual < 0
        ):
            raise LookupError("Indice da base atual não definido")
        return (
            float(self.new_bases[self.indice_base_atual][0]),
            float(self.new_bases[self.indice_base_atual][1]),
            float(self.new_bases[self.indice_base_atual][2]),
        )

    def marca_base_como_visitada(self):
        if self.indice_base_atual is None:
            raise LookupError("Indice da base atual não definido")
        self.caixas_visitadas.append(self.indice_base_atual)


class RosConfig(Node):
    """
    Classe base para cuidar de todas as configurações internas de ROS padrões.
    Isso evita a poluição do código principal.
    """

    def __init__(self, name: str = "ros_config") -> None:
        super().__init__(name)

        self.trajectory = Trajectory()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.gps_sub = self.create_subscription(
            GPSRAW,
            "/mavros/gpsstatus/gps1/raw",
            self.gps_raw_callback,
            qos_profile,
        )
        self.gps_raw_latest = None

        self.gps_sub = self.create_subscription(
            Point,
            "/orbslam3/pose",
            self.slam_raw_callback,
            qos_profile,
        )
        self.slam_raw_latest = None

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

        self.camera_subscription = self.create_subscription(
            Image, "camera", self.camera_callback, 10
        )
        # prevent unused variable warning
        self.camera_subscription

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
        # self.create_timer(0.15, self.send_vision_estimate)

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, "/mavros/state", self.state_callback, 10
        )

        self.local_position_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

        self.velocity_target_pub = self.create_publisher(
            PositionTarget, "/mavros/setpoint_raw/local", 10
        )

        self.gps_sent_pub = self.create_publisher(GPSRAW, "/gpssent", 10)

        self.estimated_position = None
        self.prev_time_ns = None
        self.max_velocity = 1.0  # m/s
        self.threshold = 0.15  # meters - stop distance

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        self.get_logger().info("Waiting for arming and mode services...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        self.get_logger().info("Arming and mode services are available.")

        self.pose = PoseStamped()
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 2.0

        self.last_request = self.get_clock().now()

        # Other GPS parameters remain unchanged
        self.mavlink_master = None
        self.runtime_var_rate = 10.0
        self.runtime_var_interval = 1.0 / self.runtime_var_rate
        self.runtime_var_fix_type = 3
        self.runtime_var_eph = 0
        self.runtime_var_epv = 0
        self.runtime_var_vel = 0
        self.runtime_var_vn = 0
        self.runtime_var_ve = 0
        self.runtime_var_vd = 0
        self.runtime_var_cog = 65535
        self.runtime_var_satellites_visible = 255
        self.runtime_var_gps_id = 0
        self.runtime_var_yaw = 36000
        self.runtime_var_alt = 0  # Altitude (MSL) [mm]
        self.runtime_var_lat = 473979932  # Latitude
        self.runtime_var_lon = 85461621  # Longitude
        self.runtime_var_running = True
        self.runtime_var_old_settings = None
        self.runtime_var_x = 0.0
        self.runtime_var_y = 0.0
        self.runtime_var_z = 0.0
        self.runtime_var_roll = 0.0
        self.runtime_var_pitch = 0.0
        self.runtime_var_yaw = 0.0
        self.runtime_var_covariance = 0
        self.runtime_var_reset_counter = 0

        self.runtime_var_gps_state: RuntimeGPSState = RuntimeGPSState.FEEDBACK

        self.init_runtime_configs()
        runtime_config_loop_thread = threading.Thread(
            target=self.runtime_config_loop, daemon=True
        )
        runtime_config_loop_thread.start()

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
        if not self.runtime_var_running:
            exit(0)
        self.publish_offboard_control_heartbeat_signal()
        if self.enable_simple_setpoint_publish:
            self.handle_setpoint_publish()
        self.handle_setpoint_publish2()

    def publish_position_setpoint2(
        self, x: float, y: float, z: float, heading: float = None, force: bool = False
    ):
        self.enable_simple_setpoint_publish = False

        x = x if x != 0 else 1e-6
        y = y if y != 0 else 1e-6
        z = z if z != 0 else 1e-6

        target_pos = np.array([x, y, z])
        if (
            hasattr(self, "last_published_setpoint")
            and self.last_published_setpoint is not None
        ):
            current_pos = np.array(self.last_published_setpoint.position)
        else:
            current_pos = target_pos

        alpha = 0.05
        smoothed_pos = current_pos + alpha * (target_pos - current_pos)
        if np.linalg.norm(target_pos - smoothed_pos) < 1e-3:
            smoothed_pos = target_pos

        msg = TrajectorySetpoint()
        msg.position = smoothed_pos.tolist()
        msg.yaw = float(heading) if heading is not None else 1.57079
        dt = 0.01
        velocity = (smoothed_pos - current_pos) / dt
        msg.velocity = velocity.tolist()
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
        self.last_setpoint2 = msg
        self.logger.debug(f"set_setpoint_message2 called {msg}")

    def handle_setpoint_publish2(self):
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
            self.logger.debug(f"self.pose2={self.pose}")
            self.local_position_pub.publish(self.pose)
            self.setpoint_last_call = timenow
            self.last_published_setpoint = self.last_setpoint2

    def handle_setpoint_publish(self):
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
                self.logger.debug(f"self.pose={self.pose}")
                self.local_position_pub.publish(self.pose)
            else:
                self.trajectory_setpoint_publisher.publish(self.last_setpoint)
            self.setpoint_last_call = timenow
            self.last_published_setpoint = self.last_setpoint

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
                pass  # TODO: implement if needed
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

    def vehicle_local_position_callback(self, vehicle_local_position):
        x, y, z = [-1000000, -1000000, -1000000]
        heading = 0
        if TOPICS_MAVROS_MODE:
            self.current_pose = vehicle_local_position
            x = vehicle_local_position.pose.position.x
            y = vehicle_local_position.pose.position.y
            z = vehicle_local_position.pose.position.z
            orientation_q = vehicle_local_position.pose.orientation
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
        self.trajectory.add(Attitude(time_ns, x, y, z, heading))
        self.logger.debug(f"Current position: x={x}, y={y}, z={z}, heading={heading}")

    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def publish_position_setpoint(
        self, x: float, y: float, z: float, heading: float = None, force: bool = False
    ):
        msg = TrajectorySetpoint()
        if x == 0:
            x = 0.000001
        if y == 0:
            y = 0.000001
        if z == 0:
            z = 0.000001
        msg.yaw = float(heading) if heading is not None else 1.57079
        msg.position = [x, y, z]
        msg.velocity = [0.0, 0.0, 0.0]
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
        current_x = self.vehicle_local_position.x
        current_y = self.vehicle_local_position.y
        current_z = self.vehicle_local_position.z

        for i in range(1, num_steps + 1):
            interp_x = current_x + (x - current_x) * (i / num_steps)
            interp_y = current_y + (y - current_y) * (i / num_steps)
            interp_z = current_z + (z - current_z) * (i / num_steps)
            self.logger.debug(
                f"Publishing intermediate position setpoint: {[interp_x, interp_y, interp_z]}"
            )
            self.publish_position_setpoint(interp_x, interp_y, interp_z)
            self.custom_sleep(delay)

        self.publish_position_setpoint(x, y, z)
        self.logger.info(f"Reached final position setpoint: {[x, y, z]}")

    def publish_velocity_setpoint(self, vx: float, vy: float, vz: float, yaw: float):
        msg = TrajectorySetpoint()
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [vx, vy, vz]
        msg.yaw = yaw
        msg.timestamp = int(self.get_timestamp_nanoseconds() / 1000)
        self.set_setpoint_message(msg)
        self.logger.info(f"Publishing velocity setpoint: {[vx, vy, vz]}, yaw: {yaw}")

    def publish_vehicle_command(self, command, **params) -> None:
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
        self.logger.info("Takeoff command sent")

    def engage_offboard_mode(self):
        if TOPICS_MAVROS_MODE:
            now = self.get_clock().now()
            if self.current_state.mode != "OFFBOARD" and (
                now - self.last_request
            ) > Duration(seconds=1.0):
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

    def configure_logging(self):
        self.logger = logging.getLogger("ros_config_logger")
        if REG_LOGFILE:
            if not os.path.exists("logs"):
                os.makedirs("logs")
            debug_log_filename = datetime.now().strftime("logs/%Y%m%d_%H%M%S_debug.log")
            info_log_filename = datetime.now().strftime("logs/%Y%m%d_%H%M%S_info.log")
            file_handler1 = logging.FileHandler(info_log_filename)
            file_handler1.setLevel(logging.INFO)
            file_handler1.setFormatter(
                logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
            )
            self.logger.addHandler(file_handler1)
            file_handler2 = logging.FileHandler(debug_log_filename)
            file_handler2.setLevel(logging.DEBUG)
            file_handler2.setFormatter(
                logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
            )
            self.logger.addHandler(file_handler2)
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.ERROR)
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

    def gps_raw_callback(self, msg):
        self.logger.debug(f"GPS Raw: {msg}")
        self.gps_raw_latest = msg

    def slam_raw_callback(self, msg):
        self.logger.debug(f"SLAM Raw: {msg}")
        self.slam_raw_callback = msg

    def init_runtime_configs(self):
        # Register signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)

        # Connection setup
        connection = "udp:127.0.0.1:14550"
        print("Connecting to MAVLink at:", connection)
        self.mavlink_master = mavutil.mavlink_connection(connection, source_system=1)

        print("Waiting for heartbeat...")
        if not self.mavlink_master.wait_heartbeat(timeout=30):
            print("No heartbeat received. Exiting.")
            return
        print("Heartbeat received from system:", self.mavlink_master.target_system)

        # Start the key capture thread
        key_thread = threading.Thread(target=self.key_capture, daemon=True)
        key_thread.start()

        print("Starting real-time HIL_GPS message sending.")
        print(
            "Press 'a' to increase latitude, 'd' to decrease latitude, 'w' to increase longitude, and 's' to decrease longitude."
        )
        print("Press Ctrl+C to exit.")

    def feeback_lat_long(self):
        coord = self.get_gps_coord()
        if coord is None:
            return

        time_usec = int(time.time() * 1e6)
        self.runtime_var_alt = coord.alt
        # self.runtime_var_eph = coord.eph
        # self.runtime_var_epv = coord.epv
        # self.runtime_var_vel = coord.vel
        # self.runtime_var_eph = 65535
        # self.runtime_var_epv = 65535
        # self.runtime_var_vel = 65535
        # self.runtime_var_cog = 65535
        self.runtime_var_satellites_visible = 255

        # self.runtime_var_ve = coord.ve
        # self.runtime_var_vd = coord.vd
        # self.runtime_var_cog = coord.cog
        # self.runtime_var_satellites_visible = coord.satellites_visible

        msg = GPSRAW()

        header = Header()
        timestamp_ns = self.get_timestamp_nanoseconds()
        header.stamp.sec = int(timestamp_ns // SECOND_IN_NS)
        header.stamp.nanosec = int(timestamp_ns % SECOND_IN_NS)

        msg.header = header
        msg.lon = self.runtime_var_lon
        msg.lat = self.runtime_var_lat
        msg.fix_type = self.runtime_var_fix_type
        msg.lat = self.runtime_var_lat
        msg.lon = self.runtime_var_lon
        msg.alt = self.runtime_var_alt
        msg.eph = self.runtime_var_eph
        msg.epv = self.runtime_var_epv
        msg.vel = self.runtime_var_vel

        self.gps_sent_pub.publish(msg)

        # self.mavlink_master.mav.hil_gps_send(
        #     time_usec,
        #     self.runtime_var_fix_type,
        #     self.runtime_var_lat,
        #     self.runtime_var_lon,
        #     self.runtime_var_alt,
        #     self.runtime_var_eph,
        #     self.runtime_var_epv,
        #     self.runtime_var_vel,
        #     self.runtime_var_vn,
        #     self.runtime_var_ve,
        #     self.runtime_var_vd,
        #     self.runtime_var_cog,
        #     self.runtime_var_satellites_visible,
        #     self.runtime_var_gps_id,
        #     self.runtime_var_yaw,
        # )

        # self.mavlink_master.mav.gps_raw_int_send(
        #     time_usec,
        #     self.runtime_var_fix_type,
        #     self.runtime_var_lat,
        #     self.runtime_var_lon,
        #     self.runtime_var_alt,
        #     self.runtime_var_eph,
        #     self.runtime_var_epv,
        #     self.runtime_var_vel,
        #     self.runtime_var_cog,
        #     self.runtime_var_satellites_visible,
        # )

        # self.send_fake_hil_sensor()
        self.send_vision_estimate()
        time.sleep(0.1)

        msg = self.mavlink_master.recv_match(blocking=False, timeout=0.1)
        if msg is None:
            self.logger.debug("No MAVLink message received.")
            return

        msg_type = msg.get_type()
        log_file = "ekf_messages.log"
        log_entry = ""
        if msg_type in [
            "EKF2_INNOVATIONS",
            "EKF2_INNOV",
            "ESTIMATOR_STATUS",
            "SENSOR_BIAS",
            "ODOMETRY",
        ]:
            log_entry = f"{msg_type}: " + str(msg.to_dict())
            with open(log_file, "a") as f:
                f.write(log_entry + "\n")
                self.logger.info("EFK Log: " + log_entry)

        # time.sleep(self.runtime_var_interval)

    def get_gps_coord(self) -> GPSRAW:
        # return latest from /mavros/gpsstatus/gps1/raw
        return self.gps_raw_latest

    def get_slam_coord(self):

        pass

    def send_vision_estimate(self):
        # return
        if self.mavlink_master is None:
            return

        time_usec = int(time.time() * 1e6)

        if (
            self.runtime_var_gps_state == RuntimeGPSState.SLAM_MANUAL and
            self.get_slam_coord() is not None
        ):
            coord = self.get_slam_coord()
            print("coords are", coords)
            # x = coords.lat - 473979272
            # y = coords.lon - 85462077
            # z = coords.alt - -1000
            # # amplitude 3.5
            # # factor 5.714285714285714
            # cm_divfac = 100
            # mm_divfac = 1000
            # x, y, z = x / cm_divfac, y / cm_divfac, z / mm_divfac
            # print("x, y, z, alt", x, y, z, coords.alt)
            # # print("coords lat lon", coords.lat, coords.lon, coords.alt)
            # # x, y = 0, 0
            # self.runtime_var_x = x  # 0.0
            # self.runtime_var_y = y  # 0.0
            # self.runtime_var_z = z  # 0.0
        elif (
            self.runtime_var_gps_state == RuntimeGPSState.GPS_MANUAL
            and self.get_gps_coord() is not None
        ):
            coords = self.get_gps_coord()
            # print("coords are", coords)
            x = coords.lat - 473979272
            y = coords.lon - 85462077
            z = coords.alt - -1000
            # amplitude 3.5
            # factor 5.714285714285714
            cm_divfac = 100
            mm_divfac = 1000
            x, y, z = x / cm_divfac, y / cm_divfac, z / mm_divfac
            print("x, y, z, alt", x, y, z, coords.alt)
            # print("coords lat lon", coords.lat, coords.lon, coords.alt)
            # x, y = 0, 0
            self.runtime_var_x = x  # 0.0
            self.runtime_var_y = y  # 0.0
            self.runtime_var_z = z  # 0.0
        else:
            print("[WARN] no slam or gps coords to base ev on")

            
        self.runtime_var_roll = 0.0
        self.runtime_var_pitch = 0.0
        self.runtime_var_yaw = 0.0

        self.logger.debug(
            f"published vision_position_estimate_send: time_usec={time_usec}, x={self.runtime_var_x}, y={self.runtime_var_y}, z={self.runtime_var_z}, roll={self.runtime_var_roll}, pitch={self.runtime_var_pitch}, yaw={self.runtime_var_yaw}, covariance={self.runtime_var_covariance}, reset_counter={self.runtime_var_reset_counter}"
        )

        self.runtime_var_reset_counter = 1

        linear_accel_cov = 0.01
        angular_vel_cov = 0.01
        tracker_confidence = 1.0
        cov_pose = linear_accel_cov * pow(10, 3 - int(tracker_confidence))
        cov_twist = angular_vel_cov * pow(10, 1 - int(tracker_confidence))
        covariance = np.array(
            [
                cov_pose,
                0,
                0,
                0,
                0,
                0,
                cov_pose,
                0,
                0,
                0,
                0,
                cov_pose,
                0,
                0,
                0,
                cov_twist,
                0,
                0,
                cov_twist,
                0,
                cov_twist,
            ]
        )

        self.mavlink_master.mav.vision_position_estimate_send(
            time_usec,
            self.runtime_var_x,
            self.runtime_var_y,
            self.runtime_var_z,
            self.runtime_var_roll,
            self.runtime_var_pitch,
            self.runtime_var_yaw,
            covariance,
            self.runtime_var_reset_counter,
        )

    def send_fake_hil_sensor(self):
        """
        Send a falsified HIL_SENSOR message to override the real IMU, baro, and mag data.
        Adjust the values as needed to achieve the desired effect.
        """
        # Get the current time in microseconds
        time_usec = int(time.time() * 1e6)

        # Fake accelerometer readings (in m/s^2):
        # Normally ~9.81 m/s^2 is gravity; here we inject high values to simulate extreme acceleration.
        xacc = 50.0
        yacc = 50.0
        zacc = 50.0

        # Fake gyroscope readings (in rad/s):
        # Typical values are small during normal flight; we inject high rotation rates.
        xgyro = 5.0
        ygyro = 5.0
        zgyro = 5.0

        # Fake magnetometer readings (in, e.g., microTesla):
        xmag = 0.5
        ymag = 0.5
        zmag = 0.5

        # Fake barometer data:
        # Absolute pressure: typical sea-level pressure is ~101325 Pa.
        # Here, choose a pressure that is far from expected (e.g., 80000 Pa) to simulate a wrong altitude.
        abs_pressure = 80000.0
        # Differential pressure can be given an arbitrary value
        diff_pressure = 100.0

        # Fake temperature in Celsius:
        temperature = 25.0

        pressure_alt = 0.0

        # fields_present: set all fields as present (commonly 0xFFFF)
        fields_present = 0xFFFF

        # Send the HIL_SENSOR MAVLink message
        self.mavlink_master.mav.hil_sensor_send(
            time_usec,
            xacc,
            yacc,
            zacc,
            xgyro,
            ygyro,
            zgyro,
            xmag,
            ymag,
            zmag,
            abs_pressure,
            diff_pressure,
            pressure_alt,
            temperature,
            fields_present,
        )
        self.get_logger().debug(
            f"Sent fake HIL_SENSOR: time_usec={time_usec}, xacc={xacc}, yacc={yacc}, zacc={zacc}, "
            f"xgyro={xgyro}, ygyro={ygyro}, zgyro={zgyro}, xmag={xmag}, ymag={ymag}, zmag={zmag}, "
            f"abs_pressure={abs_pressure}, diff_pressure={diff_pressure}, temperature={temperature}"
        )

    def runtime_config_loop(self):
        # if state==GPS_MANUAL is pressed, feed GPS from topic to PX4
        # if state==SLAM_MANUAL is pressed, feed SLAM as GPS
        # if state==FEEDBACK is pressed, keep feeding same GPS coords
        # if state==NONE prevent HITL message

        # TODDO talvez você precisa fazer que a funcao feeback_lat_long() seja
        # chamada em loop com frequencia fixa

        try:
            while True:
                if not self.runtime_var_running:
                    print(
                        "[WARNING] skip runtime_config_loop, THE SCRIPT HAS BEEN KILLED"
                    )
                    self.restore_terminal()
                    print("\nExiting...")
                    exit(0)
                    continue

                if self.mavlink_master is None:
                    print(
                        "[ERROR] mavlink_master is None, please call init_runtime_configs() first"
                    )
                    continue

                # time.sleep(0.08)

                if self.runtime_var_gps_state == RuntimeGPSState.NONE:
                    # do not publish mavlink msgs
                    continue

                if self.runtime_var_gps_state == RuntimeGPSState.FEEDBACK:
                    self.feeback_lat_long()
                    continue

                if self.runtime_var_gps_state == RuntimeGPSState.GPS_MANUAL:
                    coord = self.get_gps_coord()
                    if coord is None:
                        print("[ERROR] GPS coord is None in GPS_MANUAL mode")
                        continue
                    self.runtime_var_lat = coord.lat
                    self.runtime_var_lon = coord.lon
                    self.feeback_lat_long()
                    continue

                if self.runtime_var_gps_state == RuntimeGPSState.SLAM_MANUAL:
                    coord = self.get_slam_coord()
                    if coord is None:
                        print("[ERROR] SLAM coord is None in SLAM_MANUAL mode")
                        continue
                    self.feeback_lat_long()

        except Exception as e:
            print(f"\nError in main loop: {e}")
        finally:
            self.runtime_var_running = False
            self.restore_terminal()
            print("\nExiting...")

    def restore_terminal(
        self,
    ):
        """Restore terminal settings"""
        if self.runtime_var_old_settings:
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, self.runtime_var_old_settings
            )
            print("\nTerminal settings restored")

    def signal_handler(self, sig, frame):
        """Handle signals like SIGINT (Ctrl+C)"""
        print("\nSignal received, exiting gracefully...")
        self.runtime_var_running = False
        self.restore_terminal()
        sys.exit(0)

    def key_capture(self):
        """Continuously capture key presses at a high frequency and update lat/lon."""
        # Save terminal settings
        if not KEYBOARD_CMD_ENABLED:
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        try:
            # Run loop until running flag is False
            while self.runtime_var_running:
                # Use a very short timeout for select for high-frequency capture
                rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == "1":
                        self.runtime_var_gps_state = RuntimeGPSState.GPS_MANUAL
                        print("GPS_MANUAL mode activated")
                    elif key == "2":
                        self.runtime_var_gps_state = RuntimeGPSState.SLAM_MANUAL
                        print("SLAM_MANUAL mode activated")
                    elif key == "3":
                        self.runtime_var_gps_state = RuntimeGPSState.FEEDBACK
                        print("FEEDBACK mode activated")
                    elif key == "4":
                        self.runtime_var_gps_state = RuntimeGPSState.NONE
                        print("NONE mode activated")

                    if key == "a":
                        self.runtime_var_lat += 200
                        print("Increased latitude:", self.runtime_var_lat)
                    elif key == "d":
                        self.runtime_var_lat -= 200
                        print("Decreased latitude:", self.runtime_var_lat)
                    elif key == "w":
                        self.runtime_var_lon += 200
                        print("Increased longitude:", self.runtime_var_lon)
                    elif key == "s":
                        self.runtime_var_lon -= 200
                        print("Decreased longitude:", self.runtime_var_lon)
                    elif key == "\x03":  # Ctrl+C
                        self.runtime_var_running = False
                        break
                # Small sleep to yield control and avoid high CPU usage
                time.sleep(0.001)
        except Exception as e:
            print(f"\nError in key capture: {e}")
        finally:
            self.restore_terminal()


class NoFase3(RosConfig):
    """
    Nó do ROS principal que controla nosso drone
    """

    def __init__(self, name: str = "mission", override_bases=None):
        super().__init__(name)
        self.funcao_estado_atual = self.inicia
        self.coordenadas = Coordenadas(override_bases)

    def loop_principal(self):
        self.publish_offboard_control_heartbeat_signal()
        self.verificar_erro()
        self.logger.info(
            f">>>>>>>>>>> Chamando função: {self.funcao_estado_atual.__name__}"
        )
        output_funcao = self.funcao_estado_atual()
        self.verificar_erro()
        estado = self.get_maquina_de_estados()[self.funcao_estado_atual]
        funcao_proximo_estado = None
        if "proximo" in estado:
            funcao_proximo_estado = estado["proximo"]["funcao"]
        elif "sim" in estado and "nao" in estado:
            if output_funcao is True:
                self.logger.info("Output da função booleana: True")
                funcao_proximo_estado = estado["sim"]["funcao"]
            elif output_funcao is False:
                self.logger.info("Output da função booleana: False")
                funcao_proximo_estado = estado["nao"]["funcao"]
            else:
                self.estado_de_erro(f"Output da função não é booleano: {output_funcao}")
        else:
            self.estado_de_erro("Estado não possui próximo estado")
        if funcao_proximo_estado is None:
            self.estado_de_erro("Proximo estado indefinido")
        self.funcao_estado_atual = funcao_proximo_estado

    def estado_de_erro(self, erro):
        self.get_logger().error("Estado de erro ===============")
        self.get_logger().error(f"erro: {erro}")
        exit(1)

    def verificar_erro(self):
        erro = False
        if erro:
            self.estado_de_erro("Erro detectado")

    def inicia(self):
        self.logger.info("Inicia called")

    def armar(self):
        self.arm()

    def disarma(self):
        self.disarm()

    def pousa(self):
        self.land()
        self.custom_sleep(7)

    def sobe_voo(self):
        x, y, z, _ = self.trajectory.get_drone_x_y_z_heading(should_fail=False)
        self.get_logger().info(
            f"Envia comando de decolar para x={x} y={y} z={TAKEOFF_HEIGHT}"
        )
        self.get_logger().info(f"Posição atual                 x={x} y={y} z={z}")
        self.publish_position_setpoint(x, y, TAKEOFF_HEIGHT, force=True)
        self.custom_sleep(3)

    def esta_no_ar_estavel(self) -> bool:
        segundos_de_estabilidade = 1
        distancia_maxima_de_estabilidade = 4  # metros
        tempo_atual = self.get_timestamp_nanoseconds()
        if self.trajectory.is_stable(
            segundos_de_estabilidade, distancia_maxima_de_estabilidade, tempo_atual
        ):
            spx, spy, spz = self.get_last_setpoint_x_y_z()
            try:
                if self.trajectory.is_in_position_3d(spx, spy, spz, 0.1):
                    return True
            except Exception as e:
                self.logger.error(e)
        return False

    def tem_alguma_caixa_nao_visitada(self) -> bool:
        if self.coordenadas.find_next_base() is None:
            self.get_logger().info("Todas as caixas já estão visitadas")
            return False
        return True

    def navegue_para_base(self):
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        self.publish_position_setpoint(x, y, TAKEOFF_HEIGHT)

    def esta_em_cima_da_base(self) -> bool:
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        return self.trajectory.is_in_position_2d(x, y, 0.5)

    def esta_segurando_uma_caixa(self) -> bool:
        raise NotImplementedError()

    def aproxima_da_base(self):
        raise NotImplementedError()

    def libera_a_caixa(self):
        raise NotImplementedError()

    def navage_para_proxima_caixa(self):
        raise NotImplementedError()

    def esta_em_cima_da_caixa_odometria(self) -> bool:
        raise NotImplementedError()

    def esta_em_cima_da_caixa_camera(self) -> bool:
        raise NotImplementedError()

    def marque_caixa_como_visitada(self):
        raise NotImplementedError()

    def aproximar_da_caixa(self):
        raise NotImplementedError()

    def ligue_o_ima(self):
        raise NotImplementedError()

    def levanta_voo(self):
        self.sobe_voo()

    def esta_carregando_caixa(self) -> bool:
        raise NotImplementedError()

    def ajustar_aproximacao(self):
        raise NotImplementedError()

    def camera_callback(self, msg):
        pass


SHOULD_SAVE_IMGS = False
MILLISECONDS_IN_NS = 1e6


class DetectaQRCodes(NoFase3):
    """
    Nó do ROS principal que controla nosso drone
    """

    def __init__(self, name: str = "detecta_qrcodes"):
        super().__init__(name)
        self.standard_height = 20.0
        self.funcao_estado_atual = self.inicia
        self.origem = [0.22532819211483002, 0.1024281308054924, 0.7090049386024475]
        self.last_image = None
        self.capture_image_flag = False
        self.model = YOLO("src/mission/mission/edra_colab_model.pt", verbose=False)
        self.qr_detector = cv2.QRCodeDetector()
        self.approach_height = None
        self.approach_height_delta = 0.006
        self.min_base_drone_distance = 1.83
        self.max_base_drone_distance = 6
        self.approach_height_reduce_last = 0
        self.approach_height_reduce_period = 10 * MILLISECONDS_IN_NS
        self.last_qrcodes = None
        self.alternativas_codigo_qrcodes = ["A", "B", "C", "D", "E"]

    def tem_alguma_base_nao_visitada(self) -> bool:
        next_base = self.coordenadas.find_next_base()
        if next_base is None:
            self.logger.info("Todas as bases já estão visitadas")
            return False
        self.logger.info(
            f"Próxima base: {next_base} {self.coordenadas.new_bases[next_base]}"
        )
        return True

    def navegue_para_origem(self):
        x, y, _ = self.origem
        self.publish_position_setpoint(x, y, self.standard_height)

    def esta_em_cima_da_origem(self) -> bool:
        x, y, _ = self.origem
        return self.trajectory.is_in_position_2d(x, y, 0.5)

    def esta_segurando_uma_caixa(self) -> bool:
        raise NotImplementedError()

    def aproxima_da_base(self):
        raise NotImplementedError()

    def navage_para_proxima_base(self):
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        dx, dy, _, yaw = self.trajectory.get_drone_x_y_z_heading(should_fail=False)
        self.logger.info(f"Posicao base   [{x}, {y}]\n")
        self.logger.info(f"Posicao drone ---------------- [{dx}, {dy}]\n")
        self.publish_position_setpoint2(x, y, self.standard_height)

    def esta_em_cima_da_base_odometria(self) -> bool:
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        if self.trajectory.is_in_position_2d(x, y, 1.0):
            for i in range(3):
                self.custom_sleep(1)
                if not self.trajectory.is_in_position_2d(x, y, 1.0):
                    return False
            return True
        return False

    def esta_em_cima_de_base_pela_camera(self) -> bool:
        frame = self.last_image
        height, width, _ = frame.shape
        self.logger.debug(f"image height: {height} width: {width}")
        results = self.model(frame, conf=0.9, verbose=False)
        annotated_frame = results[0].plot()
        bounding_boxes = [caixa.xyxy[0] for caixa in results[0].boxes]
        cv2.circle(annotated_frame, (width // 2, height // 2), 5, (0, 255, 0), -1)
        sp = self.trajectory.get_drone_x_y_z_heading()
        if SHOULD_SAVE_IMGS:
            cv2.imwrite(
                f"src/mission/mission/saved/{round(sp[0],2)}_{round(sp[1],2)}_{round(sp[2],2)}.png",
                annotated_frame,
            )
        resized_frame = cv2.resize(annotated_frame, (width, height))
        cv2.imshow(f"BUSCANDO BASE...", resized_frame)
        cv2.waitKey(1)
        self.logger.info(f"{len(bounding_boxes)} bases detectadas")
        if len(bounding_boxes) == 0:
            return False
        center_image = (width // 2, height // 2)
        for base in bounding_boxes:
            x0, y0, x1, y1 = base
            x0, x1 = sorted([x0, x1])
            y0, y1 = sorted([y0, y1])
            if not (x0 < center_image[0] < x1 and y0 < center_image[1] < y1):
                return False
        return True

    def centralizar_posicao_sobre_a_base(self):
        pass

    def afaste_drone_para_encontrar_base(self):
        self.logger.debug("afaste_drone_para_encontrar_base")
        if self.approach_height is None:
            self.logger.debug("approach_height is None")
            self.approach_height = self.standard_height
        if (
            self.approach_height_reduce_last + self.approach_height_reduce_period
            < self.get_timestamp_nanoseconds()
        ):
            self.approach_height_reduce_last = self.get_timestamp_nanoseconds()
            self.approach_height += self.approach_height_delta
            self.logger.info(
                f"Afastando-se para encontrar base. z={self.approach_height}"
            )
        _, _, base_z = self.coordenadas.get_posicao_base_atual()
        x, y, drone_z, _ = self.trajectory.get_drone_x_y_z_heading()
        self.logger.debug(
            "math.fabs(base_z - drone_z) > self.max_base_drone_distance",
            math.fabs(base_z - drone_z) > self.max_base_drone_distance,
        )
        if math.fabs(base_z - drone_z) > self.max_base_drone_distance:
            self.logger.info("Base está muito longe do drone, parando de afastar")
            self.approach_height = base_z + self.max_base_drone_distance
        self.publish_position_setpoint(x, y, self.approach_height)

    def marque_base_como_visitada(self):
        self.coordenadas.marca_base_como_visitada()

    def algum_qrcode_detectado(self) -> bool:
        frame = self.last_image
        height, width, _ = frame.shape
        _, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(frame)
        if points is not None:
            for bbox, data in zip(points, decoded_info):
                bbox = bbox.astype(int)
                n = len(bbox)
                for i in range(n):
                    start_point = tuple(bbox[i])
                    end_point = tuple(bbox[(i + 1) % n])
                    cv2.line(frame, start_point, end_point, (255, 0, 0), 2)
                if data:
                    FONT_SIZE = 2.2
                    text_size, _ = cv2.getTextSize(
                        data, cv2.FONT_HERSHEY_SIMPLEX, FONT_SIZE, 2
                    )
                    text_position = tuple(bbox[0])
                    cv2.rectangle(
                        frame,
                        (text_position[0], text_position[1] - text_size[1]),
                        (text_position[0] + text_size[0], text_position[1]),
                        (255, 0, 0),
                        -1,
                    )
                    cv2.putText(
                        frame,
                        data,
                        text_position,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        FONT_SIZE,
                        (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                if not data or data not in self.alternativas_codigo_qrcodes:
                    return False
        qrcodes = len(points) if points is not None else 0
        resized_frame = cv2.resize(frame, (width, height))
        cv2.imshow("QR Code Detector", resized_frame)
        cv2.waitKey(1)
        if points is not None and qrcodes > 0:
            self.last_qrcodes = zip(points, decoded_info)
            return True
        return False

    def sobe_da_base_depois_de_ler_qrcodes(self):
        self.approach_height = None
        x, y, _ = self.coordenadas.get_posicao_base_atual()
        self.publish_position_setpoint(x, y, self.standard_height, force=True)

    def conseguiu_processar_qrcodes_detectados(self):
        if self.last_qrcodes is None:
            return False
        detected = False
        for bbox, data in self.last_qrcodes:
            detected = True
            self.logger.fatal(f"DETECTED QRCODE !!!!!!!!!!!!!!!: {data}")
        return detected

    def camera_callback(self, msg):
        self.logger.debug(
            f"[CAMERA] camera_callback called with self.capture_image_flag={self.capture_image_flag}"
        )
        self.last_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    def aproximar_do_chao_para_ler_qrcode(self):
        if self.approach_height is None:
            self.logger.debug("approach_height is None")
            self.approach_height = self.standard_height
        if (
            self.approach_height_reduce_last + self.approach_height_reduce_period
            < self.get_timestamp_nanoseconds()
        ):
            self.approach_height_reduce_last = self.get_timestamp_nanoseconds()
            self.approach_height -= self.approach_height_delta
            self.logger.debug(
                "Aproximando da base pra ler qrcode.... ALTURA: %f",
                self.approach_height,
            )
        x, y, base_z = self.coordenadas.get_posicao_base_atual()
        _, _, drone_z, _ = self.trajectory.get_drone_x_y_z_heading()
        self.logger.info(
            "math.fabs(base_z - drone_z) < self.min_base_drone_distance: %f",
            math.fabs(base_z - drone_z) < self.min_base_drone_distance,
        )
        if math.fabs(base_z - drone_z) < self.min_base_drone_distance:
            self.logger.debug("Base está muito perto do drone, parando de aproximar")
            self.approach_height = base_z + self.min_base_drone_distance
        self.publish_position_setpoint(x, y, self.approach_height)

    def echo_camera(self):
        try:
            frame = self.last_image
            height, width, _ = frame.shape
            self.logger.debug(f"image height: {height} width: {width}")
            resized_frame = cv2.resize(frame, (int(480 * 16 / 9), 480))
            cv2.imshow("BUSCANDO BASE...", resized_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.logger.error(f"SEM IMAGEM DA CAMERA >>>>> Error: {e}")

    def get_maquina_de_estados(self) -> dict[callable, dict]:
        return {
            self.inicia: {"proximo": {"funcao": self.engage_offboard_mode}},
            self.engage_offboard_mode: {"proximo": {"funcao": self.armar}},
            self.armar: {"proximo": {"funcao": self.sobe_voo}},
            self.sobe_voo: {"proximo": {"funcao": self.esta_no_ar_estavel}},
            self.esta_no_ar_estavel: {
                "sim": {"funcao": self.tem_alguma_base_nao_visitada},
                "nao": {"funcao": self.engage_offboard_mode},
            },
            self.tem_alguma_base_nao_visitada: {
                "sim": {"funcao": self.navage_para_proxima_base},
                "nao": {"funcao": self.navage_para_proxima_base},
            },
            self.navage_para_proxima_base: {
                "proximo": {"funcao": self.esta_em_cima_da_base_odometria},
            },
            self.esta_em_cima_da_base_odometria: {
                "sim": {"funcao": self.marque_base_como_visitada},
                "nao": {"funcao": self.navage_para_proxima_base},
            },
            self.marque_base_como_visitada: {
                "proximo": {"funcao": self.tem_alguma_base_nao_visitada},
            },
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_control = DetectaQRCodes()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(e)
