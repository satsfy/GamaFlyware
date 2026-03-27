import multiprocessing as mp

if __name__ == "__main__":
    mp.set_start_method("fork", force=True)

import signal
import threading
import sys
import termios
import copy

# One global “please-stop” switch that every thread can see
_EXIT_EVENT = threading.Event()
from multiprocessing import get_context, Manager

ctx = get_context("spawn")
manager = Manager()
_EXIT_EVENT = manager.Event()


def _sigint_handler(signum, frame):
    _EXIT_EVENT.set()
    try:
        import rclpy

        rclpy.shutdown()
    except Exception:
        pass
    try:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            getattr(_EXIT_EVENT, "old_termios", None) or termios.tcgetattr(sys.stdin),
        )
    except Exception:
        pass
    sys.exit(0)


import sys
from geometry_msgs.msg import Quaternion
import select
import math
import time
import os
import logging
from datetime import datetime
import numpy as np
from collections import deque
import time
import sys, termios, tty, select, threading
from pymavlink import mavutil
from enum import Enum
from typing import Callable
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from mavros_msgs.msg import State, PositionTarget, GPSRAW
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from scipy.spatial.transform import Rotation as R
import time
import sys, termios, tty, select, threading
from pymavlink import mavutil
import signal
import traceback
from matplotlib.gridspec import GridSpec
import functools
from copy import deepcopy

TAKEOFF_HEIGHT = 4.0
SECOND_IN_NS = 1e9
MILLISECOND_IN_NS = 1e6
KEYBOARD_CMD_ENABLED = False
SHOULD_SAVE_IMGS = False
MILLISECONDS_IN_NS = 1e6
ENABLE_DEBUG_LOGGER = True
TIMESTAMP_LOGS = False
ENABLE_LOG_CALL = True
ENABLE_PLOTTING = False

# YOU CAN SHARE THE MAVLINK CONNECTION IN QGROUNDCONTROL APPLICATION SETTINGS!
# MAVLINK_URL = "udp:127.0.0.1:14550"
MAVLINK_URL = "udp:localhost:14445"

# =================== LOG CONFIG ======================

timestamp_format = logging.Formatter("[%(levelname)s] %(message)s %(asctime)s ")
notimestamp_format = logging.Formatter("[%(levelname)s] %(message)s")
stdformat = notimestamp_format
if TIMESTAMP_LOGS:
    stdformat = timestamp_format


class MockLogger:
    def debug(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def warning(self, *args, **kwargs):
        pass


mock_logger = MockLogger()


def setup_logger(name, filename=None, level=logging.INFO):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    if filename:
        fh = logging.FileHandler(filename, mode="w")
        fh.setLevel(level)
        fh.setFormatter(stdformat)
        logger.addHandler(fh)
    return logger


os.makedirs("logs", exist_ok=True)
general_logger = setup_logger("general_logger", "logs/info.log", logging.INFO)

if ENABLE_DEBUG_LOGGER:
    debug_handler = logging.FileHandler("logs/debug.log", mode="w")
    debug_handler.setLevel(logging.DEBUG)
    debug_handler.setFormatter(stdformat)
    general_logger.addHandler(debug_handler)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.WARNING)
console_handler.setFormatter(stdformat)

general_logger.addHandler(console_handler)
general_logger.setLevel(logging.DEBUG)


call_count = {}


def log_call(func=None, *, only_every=1):
    if func is None:
        return lambda f: log_call(f, only_every=only_every)

    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        key = func.__qualname__
        count = call_count.get(key, 0) + 1
        call_count[key] = count

        should_log = ENABLE_LOG_CALL and (only_every <= 1 or count % only_every == 1)
        notice = ""
        if only_every > 1:
            notice = f" (call #{count})"
        if should_log:
            try:
                general_logger.debug(
                    f"Calling {func.__name__} with args={args}, kwargs={kwargs} {notice}"
                )
            except Exception:
                general_logger.error(
                    f"Error logging call to {func.__name__} with args={args}, kwargs={kwargs} {notice}"
                )
        ret = func(self, *args, **kwargs)
        if should_log:
            try:
                general_logger.debug(
                    f"Finished calling {func.__name__} with args={args}, kwargs={kwargs}, return={ret} {notice}"
                )
            except Exception:
                general_logger.error(
                    f"Error logging call to {func.__name__} with args={args}, kwargs={kwargs} {notice}"
                )
        return ret

    return wrapper


# =================== END OF LOG CONFIG ======================


class RuntimeGPSState(Enum):
    GPS_MANUAL = 1
    SLAM_MANUAL = 2
    WITH_SLAM_RET = 3
    NONE = 4
    CALIB_SLAM_MANUAL = 5


def plot_process_inner(traj_queue, data_queue, exit_event, plot_logger):
    plot_logger.debug(f"plot_process_inner called")
    import signal

    signal.signal(signal.SIGINT, signal.SIG_IGN)

    import matplotlib

    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import time
    import queue

    plt.ion()

    # now 2x2 grid: XY, Z, slam_control, and info
    fig = plt.figure(figsize=(17, 12))

    # 4 rows, 2 columns.  First row is “taller” than each control row—
    # next three rows are equal height.
    gs = GridSpec(
        nrows=4,
        ncols=3,
        figure=fig,
        height_ratios=[3, 1, 1, 1],
        hspace=0.3,
        wspace=0.2,
    )

    # Top‐left: XY
    ax_xy = fig.add_subplot(gs[0, 0])
    # Top‐right: Z
    ax_z = fig.add_subplot(gs[0, 1])

    # Info panel spans rows 1–3 in the right column
    ax_info = fig.add_subplot(gs[2:4, 1])

    plt.show(block=False)

    # --- XY plot setup (unchanged) --- #
    (gps_xy_line,) = ax_xy.plot([], [], "b-", label="GPS")
    (slam_xy_line,) = ax_xy.plot([], [], "r-", label="SLAM")
    (setpoint_xy_line,) = ax_xy.plot([], [], "g-", label="Setpoint")
    ax_xy.set_xlabel("X")
    ax_xy.set_ylabel("Y")
    ax_xy.legend()
    gps_current = ax_xy.scatter([], [], s=30, c="blue", marker="o", zorder=11)
    slam_current = ax_xy.scatter([], [], s=30, c="red", marker="o", zorder=11)
    setpoint_current = ax_xy.scatter([], [], s=30, c="green", marker="o", zorder=11)

    # --- Z vs time setup (unchanged) --- #
    (gps_z_line,) = ax_z.plot([], [], "b-", label="GPS Z")
    (slam_z_line,) = ax_z.plot([], [], "r-", label="SLAM Z")
    (setpoint_z_line,) = ax_z.plot([], [], "g-", label="Setpoint Z")
    ax_z.set_xlabel("Time (s)")
    ax_z.set_ylabel("Z")
    ax_z.legend()
    gps_z_current = ax_z.scatter([], [], s=30, c="blue", marker="o", zorder=11)
    slam_z_current = ax_z.scatter([], [], s=30, c="red", marker="o", zorder=11)
    setpoint_z_current = ax_z.scatter([], [], s=30, c="green", marker="o", zorder=11)

    # ─── ctrl ───────────────────────────────────────────────
    ax_ctrl = fig.add_subplot(gs[1, 0])
    ax_ctrl.set_ylabel("Control")
    # three empty data buffers
    ctrl_times = []
    ctrl_sc = []
    ctrl_se = []
    ctrl_cal = []

    (line_sc,) = ax_ctrl.plot([], [], "g-", label="slam_control")
    (line_se,) = ax_ctrl.plot([], [], "m-", label="slam_enabled")
    (line_cal,) = ax_ctrl.plot([], [], "c-", label="calibrating")
    ax_ctrl.legend(loc="center left")

    # ─── 1 placeholders ───────────────────────────────────────────────
    ctrl_ax_blank1 = []
    blank1_times = []

    ax_blank1 = fig.add_subplot(gs[2, 0])
    ax_blank1.set_ylabel("Time since last SLAM")
    (line_cal2,) = ax_blank1.plot([], [], "c-", label="calibrating")

    # ─── 2 placeholders ───────────────────────────────────────────────
    ctrl_ax_blank2 = []
    blank2_times = []
    ax_blank2 = fig.add_subplot(gs[3, 0])
    ax_blank2.set_ylabel("time since last mavlink")
    (line_cal3,) = ax_blank2.plot([], [], "c-", label="calibrating")
    ax_blank2.set_xlabel("Time (s)")

    # ─── info ───────────────────────────────────────────────
    ax_info.axis("off")

    # shared buffers & flags for trajectories
    STALE_THRESH = 1.0  # seconds
    gps_times, gps_zs = [], []
    slam_times, slam_zs = [], []
    setpoint_times, setpoint_zs = [], []
    last_slam_change = None
    slam_started = False
    prev_slam_pt = None
    stale_marked = False

    while not exit_event.is_set():
        plot_logger.debug(f"plot_process_inner loop iter")
        iterstart = time.time()
        try:
            data = traj_queue.get(timeout=0.1)
        except queue.Empty:
            data = None
        if data is None:
            continue
        slam_pts, gps_pts, setpoint_pts, now = data

        if gps_pts:
            gx, gy, gz = zip(*gps_pts)
            gps_xy_line.set_data(gx, gy)
            gps_times.append(now)
            gps_zs.append(gps_pts[-1][2])
            gps_z_line.set_data(gps_times, gps_zs)
            gps_current.set_offsets([[gps_pts[-1][0], gps_pts[-1][1]]])
            gps_z_current.set_offsets([[gps_times[-1], gps_zs[-1]]])

        if slam_pts:
            sx, sy, sz = zip(*slam_pts)
            slam_xy_line.set_data(sx, sy)
            slam_times.append(now)
            slam_zs.append(slam_pts[-1][2])
            slam_z_line.set_data(slam_times, slam_zs)
            slam_current.set_offsets([[slam_pts[-1][0], slam_pts[-1][1]]])
            slam_z_current.set_offsets([[slam_times[-1], slam_zs[-1]]])

            slam_started = True
            last = slam_pts[-1]
            if prev_slam_pt is None or last != prev_slam_pt:
                last_slam_change = now
                stale_marked = False
            prev_slam_pt = last

        if setpoint_pts:
            gx, gy, gz = zip(*setpoint_pts)
            setpoint_xy_line.set_data(gx, gy)
            setpoint_times.append(now)
            setpoint_zs.append(setpoint_pts[-1][2])
            setpoint_z_line.set_data(setpoint_times, setpoint_zs)
            setpoint_current.set_offsets([[setpoint_pts[-1][0], setpoint_pts[-1][1]]])
            setpoint_z_current.set_offsets([[setpoint_times[-1], setpoint_zs[-1]]])

        is_stale = slam_started and (now - (last_slam_change or now)) > STALE_THRESH
        if is_stale and not stale_marked:
            stale_marked = True
            if gps_pts:
                gx, gy, gz = gps_pts[-1]
                ax_xy.scatter(gx, gy, s=200, c="blue", marker="X", zorder=10)
                ax_z.scatter(gps_times[-1], gz, s=200, c="blue", marker="X", zorder=10)
            if slam_pts:
                sx, sy, sz = slam_pts[-1]
                ax_xy.scatter(sx, sy, s=200, c="red", marker="X", zorder=10)
                ax_z.scatter(slam_times[-1], sz, s=200, c="red", marker="X", zorder=10)

        for ax in (ax_xy, ax_z):
            ax.relim()
            ax.autoscale_view()

        # --- now pull and render the data_hist info --- #
        try:
            info = data_queue.get(timeout=0.1)
        except queue.Empty:
            info = None
        if info is None:
            continue
        now, data_hist = info
        latest = data_hist[-1]

        ctrl_times.append(now)

        ctrl_sc.append(latest["slam_control"])
        ctrl_se.append(latest["slam_enabled"])
        ctrl_cal.append(latest["calibrating"])

        try:
            plot_logger.debug(
                f'min(latest["time_since_last_slam"], 5)={min(latest["time_since_last_slam"], 5)}',
            )
            ctrl_ax_blank1.append(min(latest["time_since_last_slam"], 5))
            blank1_times.append(now)
        except Exception as e:
            plot_logger.debug(f"Error in blank1: {e}\n{traceback.format_exc()}")

        try:
            plot_logger.debug(latest["time_since_last_mavlink"])
            ctrl_ax_blank2.append(latest["time_since_last_mavlink"])
            blank2_times.append(now)
        except Exception as e:
            plot_logger.debug(f"Error in blank2: {e}\n{traceback.format_exc()}")

        line_sc.set_data(ctrl_times, ctrl_sc)
        line_se.set_data(ctrl_times, ctrl_se)
        line_cal.set_data(ctrl_times, ctrl_cal)
        line_cal2.set_data(
            blank1_times, ctrl_ax_blank1 if len(ctrl_ax_blank1) > 0 else [0]
        )
        line_cal3.set_data(
            blank2_times, ctrl_ax_blank2 if len(ctrl_ax_blank2) > 0 else [0]
        )

        for ax in (
            ax_xy,
            ax_z,
            ax_ctrl,
            ax_blank1,
            ax_blank2,
        ):
            ax.relim()
            ax.autoscale_view()

        # ---- update the text panel ---- #
        ax_info.clear()
        ax_info.axis("off")

        lines = []
        for key, val in latest.items():
            if hasattr(val, "tolist"):
                val = val.tolist()
            lines.append(f"{key:20}: {val}")
        txt = "\n".join(lines)
        ax_info.text(0, 1, txt, va="top", family="monospace", fontsize=8)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        plot_logger.debug(
            f"plot_process_inner loop iter finish in {time.time() - iterstart}s"
        )

    plt.close(fig)


def plot_process(traj_queue, data_queue, exit_event, plot_logger):
    plot_logger.debug(f"plot_process called")
    try:
        plot_process_inner(traj_queue, data_queue, exit_event, plot_logger)
    except Exception as e:
        plot_logger.error(f"Error in plot_process: {e}.{traceback.format_exc()}")


class IntelligentSLAM:
    MIN_CALIB_SAMPLES = 200  # don’t calibrate until we have this many
    WINDOW_SIZE = 50000  # keep only last 50000 samples
    EPS_MOVE_THRESH = 1e-3  # require SLAM move to record new sample
    TIME_WINDOW = 200.0  # seconds: throw away samples older than this
    TIME_DECAY = 0.1  # per-second decay for time-weighting

    def __init__(self):
        self.logger = general_logger

        # similarity‐transform params
        self.scale = 16.0211
        self.rot = np.array(
            [
                [0.99868, 0.051297, 0.00053443],
                [-0.051272, 0.99844, -0.022112],
                [-0.0016678, 0.022055, 0.99976],
            ],
        )
        self.trans = np.array([7.5836, -7.7536, 19.0262])

        self._samples = []  # list of (slam_numpy, gps_numpy)
        self._last_slam_p = None
        self.last_slam = None

    def _add_sample(self, slam_p: np.ndarray, gps_p: np.ndarray, slam_ts: float):
        # skip near‐duplicate SLAM points
        if (
            self._last_slam_p is not None
            and np.linalg.norm(slam_p - self._last_slam_p) < self.EPS_MOVE_THRESH
        ):
            return
        self._last_slam_p = slam_p

        # append and then prune by window size & age
        now = time.time()
        self._samples.append((slam_p, gps_p, slam_ts))
        # drop oldest if too many
        if len(self._samples) > self.WINDOW_SIZE:
            self._samples.pop(0)
        # drop by age
        self._samples = [
            (s, g, ts) for (s, g, ts) in self._samples if now - ts <= self.TIME_WINDOW
        ]

    def calibrate(self):
        """Robust similarity fit (IRLS with Huber weights)."""
        now = time.time()
        # build arrays
        S_list, G_list, T_list = zip(*self._samples)
        S = np.stack(S_list)
        G = np.stack(G_list)
        ts = np.array(T_list)
        n = S.shape[0]
        if n < self.MIN_CALIB_SAMPLES:
            return

        # compute per‐sample time weights
        ages = now - ts  # age in seconds
        time_w = np.exp(-self.TIME_DECAY * ages)

        def umeyama_weighted(A, B, w):
            W = w.sum()
            mu_A = (w[:, None] * A).sum(axis=0) / W
            mu_B = (w[:, None] * B).sum(axis=0) / W
            X = A - mu_A
            Y = B - mu_B
            cov = (Y * w[:, None]).T @ X / W
            U, D, Vt = np.linalg.svd(cov)
            C = np.eye(3)
            if np.linalg.det(U) * np.linalg.det(Vt) < 0:
                C[2, 2] = -1
            R = U @ C @ Vt
            var_A = (w * (X**2).sum(axis=1)).sum() / W
            s = np.trace(np.diag(D) @ C) / var_A
            t = mu_B - s * (R @ mu_A)
            return s, R, t

        # initialize residual weights = 1
        w = np.ones(n)

        huber_k = 1.0  # meters
        s, R, t = 1.0, np.eye(3), np.zeros(3)

        for _ in range(10):
            # fit with combined weight = time_w * w_residual
            s, R, t = umeyama_weighted(S, G, time_w * w)
            preds = (s * (R @ S.T).T) + t
            res = np.linalg.norm(preds - G, axis=1)
            # Huber residual weights
            w_res = np.where(res <= huber_k, 1.0, huber_k / res)
            w = w_res

        # commit final
        self.scale = s
        self.rot = R
        self.trans = t

        general_logger.info(
            f"Calibration done (time‐decay IRLS) → scale={s:.4f}, "
            f"trans=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]\nRotation:\n{R}"
        )

    def get_processed_slam(
        self, gps: PoseStamped, slam: PoseStamped, calibrate=False
    ) -> list[float]:
        if slam is None:
            return None

        slam_pos = slam.pose.position
        gps_pos = gps.pose.position
        slam_timestamp = slam.header.stamp.sec

        if slam_timestamp is not None:
            time_diff = time.time() - slam_timestamp
            # general_logger.debug(f"time diff: {time_diff} seconds")
            if time_diff < 0.5:
                p = np.array([slam_pos.x, slam_pos.y, slam_pos.z])
                g = np.array([gps_pos.x, gps_pos.y, gps_pos.z])
                self._add_sample(p, g, slam_timestamp)

                if calibrate:
                    self.calibrate()

                    general_logger.debug(
                        f"CALIBRATED!!\n"
                        f"self.scale = {self.scale}\n"
                        f"self.rot   = np.array({self.rot.tolist()})\n"
                        f"self.trans = np.array({self.trans.tolist()})"
                    )
            else:
                general_logger.debug(
                    f"not calibrating, time diff too high: {time_diff} seconds"
                )

        p = np.array([slam_pos.x, slam_pos.y, slam_pos.z])
        mapped = self.scale * (self.rot @ p) + self.trans
        self.last_slam = mapped.tolist()
        return mapped.tolist()


class Attitude:
    """
    Attitude (posição e orientação) do drone.
    """

    def __init__(self, t: int, x: float, y: float, z: float, heading: float):
        self.t = t
        self.x = x
        self.y = y
        self.z = z
        self.heading = heading


class Trajectory:

    def __init__(
        self,
    ):
        self.history = []

    def add(self, attitude: Attitude):
        self.history.append(attitude)

    def get(self, index) -> Attitude:
        return self.history[index]

    def get_drone_attitude(self) -> Attitude:
        if len(self.history) == 0:
            raise LookupError("Nenhuma attitude no histórico")
        return self.history[-1]

    def get_drone_x_y_z_heading(
        self, should_fail=True
    ) -> tuple[float, float, float, float]:
        if len(self.history) == 0:
            if not should_fail:
                return (0, 0, 0, 0)
            raise LookupError("Nenhuma attitude no histórico")
        return (
            float(self.history[-1].x),
            float(self.history[-1].y),
            float(self.history[-1].z),
            float(self.history[-1].heading),
        )

    def is_stable(self, seconds: float, max_distance: float, time_now: int) -> bool:
        reversed_hist = copy.deepcopy(self.history)
        reversed_hist.reverse()
        for i, p in enumerate(reversed_hist):
            for q in reversed_hist[i:]:
                if q.t + seconds * 1e9 < time_now:
                    break
                if (
                    math.sqrt((p.x - q.x) ** 2 + (p.y - q.y) ** 2 + (p.z - q.z) ** 2)
                    > max_distance
                ):
                    return False
        return True

    def is_in_position_3d(self, x: float, y: float, z: float, max_dist: float) -> bool:
        attitude = self.get_drone_attitude()
        # distância de manhattan
        if (
            abs(attitude.x - x) <= max_dist
            and abs(attitude.y - y) <= max_dist
            and abs(attitude.z - z) <= max_dist
        ):
            # distância euclidiana
            if (
                math.sqrt(
                    (attitude.x - x) ** 2
                    + (attitude.y - y) ** 2
                    + (attitude.z - z) ** 2
                )
                <= max_dist
            ):
                return True
        return False

    def is_in_position_2d(self, x: float, y: float, max_dist: float) -> bool:
        attitude = self.get_drone_attitude()
        manhattan_x = abs(attitude.x - x)
        manhattan_y = abs(attitude.y - y)
        if manhattan_x <= max_dist and manhattan_y <= max_dist:
            euclidean = math.sqrt((attitude.x - x) ** 2 + (attitude.y - y) ** 2)
            if euclidean <= max_dist:
                return True
        return False


class DroneState:
    def __init__(self):
        self.trajectory = Trajectory()


class Camera:
    def __init__(self, ros_node: Node):
        self.ros_node = ros_node
        self.last_image = None
        self.camera_subscription = self.ros_node.create_subscription(
            Image, "camera", self.camera_callback, 10
        )

    def camera_callback(self, msg):
        self.last_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")


class ROSNode(Node):
    @log_call
    def __init__(self, state: DroneState) -> None:
        super().__init__("ros_node")
        self.state = state

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
        self.gps_raw_latest: GPSRAW = None

        self.slam_sub = self.create_subscription(
            PoseStamped,
            "/orbslam3/pose",
            self.slam_raw_callback,
            qos_profile,
        )
        self.slam_raw_latest: PoseStamped = None

        self.current_pose = PoseStamped()
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.vehicle_local_position_callback,
            qos_profile,
        )

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.bridge = CvBridge()

        self.last_setpoint = None
        self.last_published_setpoint = None
        self.setpoint_last_call = 0
        self.heartbeat_last_call = 0

        self.setpoint_publish_period = 200 * MILLISECOND_IN_NS
        self.heartbeat_publish_period = 10 * MILLISECOND_IN_NS
        self.min_setpoint_delta = 0

        self.create_timer(0.01, self.ros_loop)

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

        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        general_logger.info("Waiting for arming and mode services...")
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            general_logger.info("Waiting for arming service...")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            general_logger.info("Waiting for set_mode service...")
        general_logger.info("Arming and mode services are available.")

        self.setpoint: PoseStamped = PoseStamped()
        self.setpoint.pose.position.x = 0.0
        self.setpoint.pose.position.y = 0.0
        self.setpoint.pose.position.z = 2.0

        self.last_request = self.get_clock().now()

        self.terminal_settings = None

        self.mavlink_master = None
        self.runtime_var_lat = 473979932  # Latitude
        self.runtime_var_lon = 85461621  # Longitude
        self.runtime_var_roll = 0.0
        self.runtime_var_pitch = 0.0
        self.runtime_var_yaw = 0.0

        self.runtime_var_gps_state: RuntimeGPSState = RuntimeGPSState.SLAM_MANUAL

        self.init_runtime_configs()
        self.vision_estimate_loop_thread = threading.Thread(
            target=self.vision_estimate_loop, daemon=True
        )
        self.vision_estimate_loop_thread.start()

        traj_history_size = 10000
        self.slam_traj = deque(maxlen=traj_history_size)
        self.gps_traj = deque(maxlen=traj_history_size)
        self.setpoint_traj = deque(maxlen=traj_history_size)

        self.intelligent_slam = IntelligentSLAM()

        self.data_queue_max = 100
        self.data_queue = deque(maxlen=self.data_queue_max)

        if ENABLE_PLOTTING:
            self.should_plot = True
            self.create_timer(0.1, self.update_live_plot)

            self.plot_q = mp.Queue(maxsize=1)
            self.plot_data = mp.Queue(maxsize=1)

            self.plot_proc = mp.Process(
                target=plot_process,
                args=(
                    self.plot_q,
                    self.plot_data,
                    _EXIT_EVENT,
                    general_logger,
                ),
            )
            self.plot_proc.daemon = True
            self.plot_proc.start()

            self.mavlink_times = deque(maxlen=1000)

        self.initial_coord_set = False
        self.initial_coord = GPSRAW()
        self.initial_coord.lat = 473979272
        self.initial_coord.lon = 85462077
        self.initial_coord.alt = -1000
        self.factor_lat_long = 100
        self.factor_alt = 1000

    def destroy_node(self):
        super().destroy_node()
        try:
            self.plot_q.put_nowait(None)
        except:
            pass
        self.plot_proc.join(timeout=1.0)
        if self.plot_proc.is_alive():
            self.plot_proc.terminate()
            self.plot_proc.join()
        _EXIT_EVENT.set()
        if self.vision_estimate_loop_thread.is_alive():
            general_logger.error("Waiting for vision_estimate_loop_thread to exit…")
            self.vision_estimate_loop_thread.join(timeout=1.0)
            if self.vision_estimate_loop_thread.is_alive():
                general_logger.error(
                    "vision_estimate_loop_thread still alive; it will die with the process"
                )
            else:
                general_logger.error("vision_estimate_loop_thread exited cleanly")

    @log_call
    def set_setpoint_message(self, msg: TrajectorySetpoint):
        self.last_setpoint = msg
        x, y, z = msg.position
        yaw = msg.yaw
        general_logger.info(
            f"set_setpoint_message called x={x}, y={y}, z={z}, yaw={yaw}"
        )

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
            self.heartbeat_last_call = timenow

    @log_call(only_every=100)
    def ros_loop(self):
        self.publish_offboard_control_heartbeat_signal()
        if _EXIT_EVENT.is_set():
            return

        self.handle_setpoint_publish(self.last_setpoint)

    @log_call(only_every=100)
    def handle_setpoint_publish(self, msg: TrajectorySetpoint):
        if msg is None:
            return

        timenow = self.get_timestamp_nanoseconds()
        if self.setpoint_last_call + self.setpoint_publish_period >= timenow:
            return

        x, y, z = msg.position
        self.setpoint.pose.position.x = float(x)
        self.setpoint.pose.position.y = float(y)
        self.setpoint.pose.position.z = float(z)

        self.setpoint.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(msg.yaw / 2),
            w=math.cos(msg.yaw / 2),
        )

        self.local_position_pub.publish(self.setpoint)

        self.setpoint_last_call = timenow
        self.last_published_setpoint = msg

    @log_call
    def custom_sleep(self, seconds):
        start = self.get_timestamp_nanoseconds()
        while self.get_timestamp_nanoseconds() - start < seconds * 1e9:
            self.ros_loop()
            time.sleep(0.01)

    @log_call
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def get_timestamp_nanoseconds(self) -> int:
        return self.get_clock().now().nanoseconds

    @log_call
    def arm(self):
        now = self.get_clock().now()
        if not self.current_state.armed and (now - self.last_request) > Duration(
            seconds=1.0
        ):
            general_logger.info("Arming the vehicle...")
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response_callback)
            self.last_request = now

        general_logger.info("Arm command sent")

    @log_call
    def engage_offboard_mode(self):
        now = self.get_clock().now()
        if self.current_state.mode != "OFFBOARD" and (
            now - self.last_request
        ) > Duration(seconds=1.0):
            general_logger.info("Setting OFFBOARD mode...")
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.set_mode_response_callback)
            self.last_request = now
        general_logger.info("Switching to offboard mode")

    @log_call
    def land(self):
        general_logger.info("Initiating landing...")
        req = SetMode.Request()
        req.custom_mode = "AUTO.LAND"
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.land_response_callback)

    @log_call
    def land_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                general_logger.info("Landing initiated successfully")
            else:
                general_logger.error("Failed to initiate landing")
        except Exception as e:
            general_logger.error(f"Landing call failed: {e}")

    @log_call
    def state_callback(self, msg):
        self.current_state = msg

    @log_call
    def set_mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                general_logger.info("OFFBOARD mode set successfully")
            else:
                general_logger.error("Failed to set OFFBOARD mode")
        except Exception as e:
            general_logger.error(f"SetMode call failed: {e}")

    @log_call
    def arm_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                general_logger.info("Vehicle armed successfully")
            else:
                general_logger.error("Failed to arm vehicle")
        except Exception as e:
            general_logger.error(f"Arming call failed: {e}")

    @log_call
    def gps_raw_callback(self, msg):
        if not self.initial_coord_set:
            self.initial_coord_set = True
            self.initial_coord = msg
            general_logger.info(
                f"Initial GPS coordinates set: lat={msg.lat}, lon={msg.lon}, alt={msg.alt}"
            )
        # general_logger.info(f"GPS Raw: {msg}")
        self.gps_raw_latest = msg

    @log_call
    def slam_raw_callback(self, msg):
        # general_logger.info(f"SLAM Raw: {msg}")
        self.slam_raw_latest = msg

    @log_call
    def init_runtime_configs(self):
        connection = MAVLINK_URL
        general_logger.error(f"Connecting to MAVLink at: {connection}")
        self.mavlink_master = mavutil.mavlink_connection(connection, source_system=1)

        general_logger.error("Waiting for heartbeat...")
        if not self.mavlink_master.wait_heartbeat(timeout=30):
            general_logger.error("No heartbeat received. Exiting.")
            return
        general_logger.error(
            f"Heartbeat received from system {self.mavlink_master.target_system}"
        )

        if KEYBOARD_CMD_ENABLED:
            general_logger.error("Starting key capture thread.")
            key_thread = threading.Thread(target=self.key_capture, daemon=True)
            key_thread.start()

    @log_call
    def get_slam_coord(self) -> PoseStamped:
        # returned from /orbslam3/pose
        return self.slam_raw_latest

    @log_call(only_every=100)
    def send_mavlink_message(
        self,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        reset_counter: int,
    ):
        try:
            self.mavlink_times.append(time.time())

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

            time_usec = int(time.time() * 1e6)
            self.mavlink_master.mav.vision_position_estimate_send(
                time_usec,
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
                covariance,
                reset_counter,
            )
        except Exception as e:
            general_logger.error(
                f"Error sending MAVLink message: {e}\n{traceback.format_exc()}"
            )

    @log_call
    def quit(
        self,
    ):
        general_logger.error("quit called")

        general_logger.error("Restore terminal settings")
        if self.terminal_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
            general_logger.error("\nTerminal settings restored")

        general_logger.error("kill thread")
        # only join if *this* is not the vision_estimate_loop thread
        if threading.current_thread() is not self.vision_estimate_loop_thread:
            general_logger.error("joining vision_estimate_loop_thread")
            self.vision_estimate_loop_thread.join(timeout=1.0)
            if self.vision_estimate_loop_thread.is_alive():
                general_logger.error("thread remained alive, terminating it")
            else:
                general_logger.error("thread exited cleanly")
        general_logger.error("thread exited")

    @log_call
    def key_capture(self):
        if not KEYBOARD_CMD_ENABLED:
            return

        old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        try:
            while not _EXIT_EVENT.is_set():
                # Use a very short timeout for select for high-frequency capture
                rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == "1":
                        self.runtime_var_gps_state = RuntimeGPSState.GPS_MANUAL
                        general_logger.error("GPS_MANUAL mode activated")
                    elif key == "2":
                        self.runtime_var_gps_state = RuntimeGPSState.SLAM_MANUAL
                        general_logger.error("SLAM_MANUAL mode activated")
                    elif key == "3":
                        self.runtime_var_gps_state = RuntimeGPSState.WITH_SLAM_RET
                        general_logger.error("WITH_SLAM_RET mode activated")
                    elif key == "4":
                        self.runtime_var_gps_state = RuntimeGPSState.NONE
                        general_logger.error("NONE mode activated")
                    elif key == "5":
                        self.runtime_var_gps_state = RuntimeGPSState.CALIB_SLAM_MANUAL
                        general_logger.error("calib slam mode activated")

                    elif key == "p":
                        if self.should_plot:
                            self.should_plot = False
                        else:
                            self.should_plot = True
                    if key == "a":
                        self.runtime_var_lat += 200
                        general_logger.error(
                            "Increased latitude:", self.runtime_var_lat
                        )
                    elif key == "d":
                        self.runtime_var_lat -= 200
                        general_logger.error(
                            "Decreased latitude:", self.runtime_var_lat
                        )
                    elif key == "w":
                        self.runtime_var_lon += 200
                        general_logger.error(
                            "Increased longitude:", self.runtime_var_lon
                        )
                    elif key == "s":
                        self.runtime_var_lon -= 200
                        general_logger.error(
                            "Decreased longitude:", self.runtime_var_lon
                        )
                    elif key == "\x03":  # Ctrl-C pressed while in raw mode
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                        _EXIT_EVENT.set()
                        return
                # Small sleep to yield control and avoid high CPU usage
                time.sleep(0.01)
        except Exception as e:
            general_logger.error(f"\nError in key capture: {e}")
        finally:
            general_logger.error("exit key_capture")
            return

    @log_call
    def update_live_plot(self):
        try:
            general_logger.debug("update_live_plot")
            if _EXIT_EVENT.is_set():
                general_logger.debug("update_live_plot: exit event set")
                return
            if not self.should_plot:
                general_logger.debug("update_live_plot: self.should_plot=False")
                return

            general_logger.debug("update_live_plot: queuing frame")

            if self.plot_q.full():
                try:
                    self.plot_q.get_nowait()
                except mp.queues.Empty:
                    pass
            self.plot_q.put_nowait(
                (
                    list(self.slam_traj),
                    list(self.gps_traj),
                    list(self.setpoint_traj),
                    time.time(),  # time stamp for Z-plot
                )
            )

            data = {}

            if len(self.slam_traj) > 0 and len(self.gps_traj) > 0:
                # compute error between latest SLAM and GPS X coordinates
                data["last_slam_x"] = self.slam_traj[-1][0]
                data["last_gps_x"] = self.gps_traj[-1][0]
                data["slam_to_gps_error_x"] = data["last_slam_x"] - data["last_gps_x"]
                data["last_slam_y"] = self.slam_traj[-1][1]
                data["last_gps_y"] = self.gps_traj[-1][1]
                data["slam_to_gps_error_y"] = data["last_slam_y"] - data["last_gps_y"]
                data["last_slam_z"] = self.slam_traj[-1][2]
                data["last_gps_z"] = self.gps_traj[-1][2]
                data["slam_to_gps_error_z"] = data["last_slam_z"] - data["last_gps_z"]

            data["slam_control"] = 0
            if self.runtime_var_gps_state == RuntimeGPSState.WITH_SLAM_RET:
                data["slam_control"] = 1

            data["slam_enabled"] = 0
            if (
                self.runtime_var_gps_state == RuntimeGPSState.SLAM_MANUAL
                or self.runtime_var_gps_state == RuntimeGPSState.WITH_SLAM_RET
                or self.runtime_var_gps_state == RuntimeGPSState.CALIB_SLAM_MANUAL
            ):
                data["slam_enabled"] = 1
            data["gps_enabled"] = 1

            data["rot"] = self.intelligent_slam.rot.copy()
            data["scale"] = self.intelligent_slam.scale
            data["trans"] = self.intelligent_slam.trans.copy()

            data["calibrating"] = 0
            if self.runtime_var_gps_state == RuntimeGPSState.CALIB_SLAM_MANUAL:
                data["calibrating"] = 1

            data["time_since_last_slam"] = float(
                0
                if self.get_slam_coord() is None
                else time.time() - self.get_slam_coord().header.stamp.sec
            )

            data["roll"] = self.runtime_var_roll
            data["pitch"] = self.runtime_var_pitch
            data["yaw"] = self.runtime_var_yaw

            if len(self.mavlink_times) >= 2:
                data["time_since_last_mavlink"] = float(
                    self.mavlink_times[-1] - self.mavlink_times[-2]
                )

            # data["published_pose"] = [
            #     round(self.setpoint.pose.position.x, 2),
            #     round(self.setpoint.pose.position.y, 2),
            #     round(self.setpoint.pose.position.z, 2),
            # ]

            # # get system RAM usage in MB
            # mem = psutil.virtual_memory()
            # used_mb = mem.used / (1024 ** 2)

            # # send to the plot_mem queue (drop old if full)
            # if self.plot_mem.full():
            #     try:
            #         self.plot_mem.get_nowait()
            #     except Exception:
            #         pass
            # self.plot_mem.put_nowait((time.time(), used_mb))

            data["timestamp"] = time.time()

            self.data_queue.append(data)
            general_logger.debug(f"data ready: {data}")
            general_logger.debug("update_live_plot: queuing data frame")

            if self.plot_data.full():  # non-blocking: don’t wait
                try:
                    self.plot_data.get_nowait()  # drop the previous packet
                except mp.queues.Empty:
                    pass

            self.plot_data.put_nowait((data["timestamp"], self.data_queue))

            general_logger.debug(f"data sent")
        except Exception as e:
            general_logger.error(f"Error in update_live_plot: {e}")
            general_logger.error(traceback.format_exc())

    @log_call
    def vehicle_local_position_callback(self, _):
        raise NotImplementedError()



    @log_call
    def get_ekf_log(self):
        msg = self.ros_node.mavlink_master.recv_match(blocking=False, timeout=0.1)
        if msg is None:
            general_logger.info("No MAVLink message received.")
            return
        msg_type = msg.get_type()
        log_file = "logs/ekf_messages.log"
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
                general_logger.info("EFK Log: " + log_entry)



class DroneControl:
    @log_call
    def __init__(self, state: DroneState, ros_node: ROSNode):
        self.state = state
        self.ros_node = ros_node

    @log_call
    def vision_estimate_loop(self):
        try:
            while not _EXIT_EVENT.is_set():
                if self.ros_node.mavlink_master is None:
                    general_logger.error(
                        "[ERROR] mavlink_master is None, please call init_runtime_configs() first"
                    )
                    continue
                self.send_vision_estimate()
        except Exception as e:
            general_logger.error(f"\nError in main loop: {e}")
            general_logger.error("Exception in vision_estimate_loop:")
            general_logger.error(traceback.format_exc())
        finally:
            general_logger.error("[WARNING] set exit event...")
            _EXIT_EVENT.set()
            general_logger.error("[WARNING] shutting down rclpy...")
            try:
                import rclpy

                if rclpy.ok():
                    rclpy.shutdown()
            except RuntimeError:
                pass

            general_logger.error("[WARNING] rclpy shutdown complete")


    @log_call
    def publish_position_setpoint_smoothed(
        self,
        x: float,
        y: float,
        z: float,
        heading: float = None,
        smooth_factor: float = 0.93,
        yaw_smooth_factor: float = 0.93,
    ):
        """
        smooth_factor varies from 0 to 1, where 0 is no smoothing and 1 is full smoothing.
        Applies exponential smoothing to x, y, z **and** yaw.
        """
        if not (0 <= smooth_factor <= 1):
            raise ValueError("smooth_factor must be between 0 and 1")
        if not (0 <= yaw_smooth_factor <= 1):
            raise ValueError("yaw_smooth_factor must be between 0 and 1")

        # Avoid zero-vector
        x = x or 1e-6
        y = y or 1e-6
        z = z or 1e-6

        # Prepare target vectors
        target_pos = np.array([x, y, z])
        target_yaw = float(heading) if heading is not None else 1.57079

        # Initialize current from last setpoint, or jump straight to target
        if hasattr(self, "last_published_setpoint") and self.last_published_setpoint:
            curr_pos = np.array(self.ros_node.last_published_setpoint.position)
            curr_yaw = self.ros_node.last_published_setpoint.yaw
        else:
            curr_pos = target_pos.copy()
            curr_yaw = target_yaw

        # smoothing coefficient
        α = 1.0 - smooth_factor

        # --- position smoothing ---
        smoothed_pos = curr_pos + α * (target_pos - curr_pos)
        if np.linalg.norm(target_pos - smoothed_pos) < 1e-3:
            smoothed_pos = target_pos

        # --- yaw smoothing ---
        # compute shortest delta angle in [-pi, +pi]
        Δyaw = (target_yaw - curr_yaw + np.pi) % (2 * np.pi) - np.pi
        smoothed_yaw = curr_yaw + (1.0 - yaw_smooth_factor) * Δyaw
        # if we’re very close, just snap
        if abs(Δyaw) < 1e-3:
            smoothed_yaw = target_yaw

        # build message
        msg = TrajectorySetpoint()
        msg.position = smoothed_pos.tolist()
        msg.yaw = float(smoothed_yaw)

        # velocity from position delta / dt
        dt = 0.01
        velocity = (smoothed_pos - curr_pos) / dt
        msg.velocity = velocity.tolist()

        msg.timestamp = int(self.ros_node.get_timestamp_nanoseconds() / 1000)

        self.ros_node.last_setpoint = msg

    @log_call
    def publish_position_setpoint(
        self, x: float, y: float, z: float, heading: float = None
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
        msg.timestamp = int(self.ros_node.get_timestamp_nanoseconds() / 1000)
        self.ros_node.set_setpoint_message(msg)

    # @log_call
    def vehicle_local_position_callback(self, vehicle_local_position: PoseStamped):
        self.current_pose = vehicle_local_position
        x = vehicle_local_position.pose.position.x
        y = vehicle_local_position.pose.position.y
        z = vehicle_local_position.pose.position.z
        roll, pitch, yaw = self.get_roll_pitch_yaw(vehicle_local_position)
        general_logger.info(
            f"Vehicle local position: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}"
        )
        time_ns = self.ros_node.get_timestamp_nanoseconds()
        self.state.trajectory.add(Attitude(time_ns, x, y, z, yaw))

    def get_roll_pitch_yaw(self, pose: PoseStamped) -> tuple[float, float, float]:
        orientation_q = pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        r = R.from_quat(orientation_list)
        (roll, pitch, yaw) = r.as_euler("xyz", degrees=False)
        return roll, pitch, yaw

    def get_gps_raw_coord(self) -> GPSRAW:
        return self.ros_node.gps_raw_latest

    @log_call(only_every=100)
    def get_gps_processed_coord(self) -> PoseStamped:
        gps_coords: GPSRAW = self.get_gps_raw_coord()
        # delta_longitude_in_degrees * 111320 * cos(latitude_in_degrees) = distance_in_meters
        # meters = delta_latitude * 111111

        # x = (gps_coords.lat - 473979272) / 100
        # y = (gps_coords.lon - 85462077) / 100
        # z = (gps_coords.alt - -1000) / 1000
        x = (gps_coords.lat - self.ros_node.initial_coord.lat) / self.ros_node.factor_lat_long
        y = (gps_coords.lon - self.ros_node.initial_coord.lon) / self.ros_node.factor_lat_long
        z = (gps_coords.alt - self.ros_node.initial_coord.alt) / self.ros_node.factor_alt
        pose = PoseStamped()
        pose.header = gps_coords.header
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = deepcopy(self.ros_node.current_pose.pose.orientation)
        return pose

    @log_call
    def get_last_setpoint_x_y_z(self) -> tuple[float, float, float]:
        if self.ros_node.last_setpoint is None:
            raise ValueError("No setpoint available")
        return tuple(self.ros_node.last_setpoint.position)

    def send_vision_estimate(self):
        if self.get_gps_raw_coord() is None:
            return

        if self.mavlink_master is None:
            return

        state = self.runtime_var_gps_state
        pose = None
        slam_calibrate = False
        slam_return = False
        slam_mode = True

        if self.slam_raw_latest is not None:
            time_diff = time.time() - self.slam_raw_latest.header.stamp.sec
            # general_logger.info(f"time diff: {time_diff} seconds")

        if state == RuntimeGPSState.SLAM_MANUAL:
            pass
        elif state == RuntimeGPSState.CALIB_SLAM_MANUAL:
            slam_calibrate = True
        elif state == RuntimeGPSState.WITH_SLAM_RET:
            slam_return = True
        elif state == RuntimeGPSState.GPS_MANUAL:
            slam_mode = False
        else:
            general_logger.error(
                f"Invalid GPS state, cannot send vision estimate state={state}"
            )
            return

        raw_gps = self.get_gps_raw_coord()
        processed_gps = self.get_gps_processed_coord()
        raw_slam = self.ros_node.slam_raw_latest
        processed_slam = self.intelligent_slam.get_processed_slam(
            processed_gps, self.ros_node.slam_raw_latest, calibrate=slam_calibrate
        )
        setpoint = self.ros_node.setpoint

        # general_logger.debug(f"raw_gps        ={raw_gps}")
        # general_logger.debug(f"processed_gps  ={processed_gps}")
        # general_logger.debug(f"raw_slam       ={raw_slam}")
        # general_logger.debug(f"processed_slam ={processed_slam}")
        # general_logger.debug(f"setpoint       ={setpoint}")
        # ns = [setpoint.pose.position.y, setpoint.pose.position.x, setpoint.pose.position.z]

        # PLEASE FIX HERE X Y Z !!!
        ns = [
            setpoint.pose.position.y,
            setpoint.pose.position.x,
            setpoint.pose.position.z,
        ]

        for a, b in [
            (ns, self.ros_node.setpoint_traj),
            (processed_slam, self.ros_node.slam_traj),
            (processed_gps, self.ros_node.gps_traj),
        ]:
            if a:
                if type(a) is list:
                    b.append(
                        (
                            a[0],
                            a[1],
                            a[2],
                        )
                    )
                else:
                    b.append(
                        (
                            a.pose.position.x,
                            a.pose.position.y,
                            a.pose.position.z,
                        )
                    )

        pose = processed_gps
        if slam_return and processed_slam is not None:
            general_logger.debug("sending slam pose")
            pose = processed_slam

            # finge que o GPS sempre esta disponivel
            # usa o SLAM para fazer algo interessante

            _, setpoint_pixhawk = self.fazer_algo_mais_inteligente(processed_slam)

        if type(pose) is list:
            x, y, z = pose
            roll, pitch, yaw = 0.0, 0.0, 0.0
        else:
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z
            roll, pitch, yaw = self.get_roll_pitch_yaw(pose)

        # general_logger.debug(
        #     f"data for send vision estimate: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw} slam_mode={slam_mode}, slam_calibrate={slam_calibrate}, slam_return={slam_return}"
        # )
        reset_counter = 0
        # self.send_mavlink_message(
        #     x,
        #     y,
        #     z,
        #     roll,
        #     pitch,
        #     yaw,
        #     reset_counter,
        # )

        # time it takes for mavlink to 'get it'
        # general_logger.debug("MAVLink wait")
        time.sleep(0.1)
        # self.get_ekf_log()


class Mission:

    def __init__(self, ros_node: Node, camera: Camera, state: DroneState, drone_control: DroneControl):
        self.ros_node = ros_node
        self.drone_control = drone_control
        self.camera = camera
        self.state = state

        self.standard_height = 20.0
        self.next_state_procedure = self.init
        self.camera.last_image = None
        self.model = YOLO("src/mission/mission/edra_colab_model.pt", verbose=False)
        self.approach_height = None
        self.last_bounding_boxes = None
        self.current_find_attempts = 0
        self.max_find_attempts = 10
        self.final_approach_attempts = 10

        self.target_setpoints = []
        self.current_setpoint_index = 0
        height = 7.0
        yaw = 1.57079
        foward_direction = True
        for i in range(0, 10, 3):
            if foward_direction:
                for j in range(-10, 0, 3):
                    self.target_setpoints.append(
                        [+float(i), +float(j), height, yaw],
                    )
            else:
                for j in range(0, -10, -3):
                    self.target_setpoints.append(
                        [+float(i), +float(j), height, yaw],
                    )
            foward_direction = not foward_direction


    def init(self):
        self.ros_node.create_timer(0.03, self.main_loop)
        general_logger.info("init called")

    def engage_offboard_mode(self):
        self.ros_node.engage_offboard_mode()

    def arm(self):
        self.ros_node.arm()

    def main_loop(self):
        general_logger.info(
            f">>>>>>>>>>> Calling function: {self.next_state_procedure.__name__}"
        )
        procedure_output = self.next_state_procedure()
        estado = self.get_finite_state_machine()[self.next_state_procedure]
        next_state_procedure = None
        if "next" in estado:
            next_state_procedure = estado["next"]["call"]
        elif "yes" in estado and "no" in estado:
            if procedure_output is True:
                general_logger.info("Output da função booleana: True")
                next_state_procedure = estado["yes"]["call"]
            elif procedure_output is False:
                general_logger.info("Output da função booleana: False")
                next_state_procedure = estado["no"]["call"]
            else:
                self.abort(f"Procedure output not boolean: {procedure_output}")
        else:
            self.abort("Current state has no next state")
        if next_state_procedure is None:
            self.abort("Next state procedure not defined")
        self.next_state_procedure = next_state_procedure

    def abort(self, error):
        general_logger.error(f"ABORT CALLED: {error}")
        exit(1)

    def takeoff(self):
        x, y, z, _ = self.state.trajectory.get_drone_x_y_z_heading(should_fail=False)
        general_logger.info(f"Send takeoff command para x={x} y={y} z={TAKEOFF_HEIGHT}")
        general_logger.info(f"Current position                 x={x} y={y} z={z}")
        self.drone_control.publish_position_setpoint(x, y, TAKEOFF_HEIGHT)
        self.ros_node.custom_sleep(3)

    def is_airborne(self) -> bool:
        stability_seconds = 0.7
        max_dist_stability = 4
        current_time = self.ros_node.get_timestamp_nanoseconds()
        if self.state.trajectory.is_stable(
            stability_seconds, max_dist_stability, current_time
        ):
            spx, spy, spz = self.drone_control.get_last_setpoint_x_y_z()
            try:
                if self.state.trajectory.is_in_position_3d(spx, spy, spz, 0.3):
                    return True
            except Exception as e:
                general_logger.error(e)
        return False

    def navigate_to_next_destination(self):
        x, y, z, yaw = self.target_setpoints[self.current_setpoint_index]
        dx, dy, dz, dyaw = self.state.trajectory.get_drone_x_y_z_heading(
            should_fail=False
        )
        general_logger.info(
            f"position base   [{round(x,3)}, {round(y,3)}, {round(z,3)}]\n"
        )
        general_logger.info(
            f"position drone  [{round(dx,3)}, {round(dy,3)}, {round(dz,3)}]\n"
        )
        self.drone_control.publish_position_setpoint_smoothed(
            x, y, z, heading=yaw, smooth_factor=0.985, yaw_smooth_factor=0.994
        )

    def is_on_top_of_destination(self) -> bool:
        x, y, _, _ = self.target_setpoints[self.current_setpoint_index]
        if self.state.trajectory.is_in_position_2d(x, y, 3.0):
            for i in range(2):
                self.ros_node.custom_sleep(0.5)
                if not self.state.trajectory.is_in_position_2d(x, y, 3.0):
                    return False
            return True
        return False

    def fly_with_delta_height(self, delta=0.0001):
        x, y, z, _ = self.state.trajectory.get_drone_x_y_z_heading()
        self.drone_control.publish_position_setpoint_smoothed(x, y, z + delta)

    def set_destination_visited(self):
        self.current_setpoint_index = (self.current_setpoint_index + 1) % len(
            self.target_setpoints
        )

    def echo_camera(self):
        try:
            frame = self.camera.last_image
            height, width, _ = frame.shape
            general_logger.info(f"image height: {height} width: {width}")
            resized_frame = cv2.resize(frame, (int(480 * 16 / 9), 480))
            cv2.imshow("BUSCANDO BASE NESSA FOTO...", resized_frame)
            cv2.waitKey(1)
        except Exception as e:
            general_logger.error(f"SEM IMAGEM DA CAMERA >>>>> Error: {e}")

    def show_frame(
        self,
        name,
        yolo_results=None,
        bounding_boxes=None,
        center_point=False,
        resize=False,
        vector=None,
    ):
        frame = self.camera.last_image
        height, width, _ = frame.shape
        if yolo_results:
            frame = yolo_results[0].plot()
        if center_point:
            cv2.circle(frame, (width // 2, height // 2), 5, (0, 255, 0), -1)
        if resize:
            frame = cv2.resize(frame, (width, height))
        if bounding_boxes:
            for bb in bounding_boxes:
                x0, y0, x1, y1 = bb
                x0, x1 = sorted([x0, x1])
                y0, y1 = sorted([y0, y1])
                cv2.rectangle(
                    frame, (int(x0), int(y0)), (int(x1), int(y1)), (255, 0, 0), 2
                )
        if vector:
            vx, vy = vector
            # Draw the vector as an arrow from the center of the image
            start_point = (width // 2, height // 2)
            end_point = (int(start_point[0] + vx), int(start_point[1] + vy))
            cv2.arrowedLine(
                frame, start_point, end_point, (0, 0, 255), thickness=2, tipLength=0.2
            )
        cv2.imshow(name, frame)
        cv2.waitKey(1)

    def get_bounding_boxes(self, frame: np.ndarray):
        yolo_results = self.model(frame, conf=0.9, verbose=False)
        bounding_boxes = [caixa.xyxy[0] for caixa in yolo_results[0].boxes]
        if len(bounding_boxes) > 0:
            self.last_bounding_boxes = bounding_boxes
        return bounding_boxes, yolo_results

    def find_landing_pad(self):
        bbxs, _ = self.get_bounding_boxes(self.camera.last_image)
        if len(bbxs) > 0:
            self.current_find_attempts = 0
        else:
            self.current_find_attempts += 1
            self.fly_with_delta_height(delta=0.0005)

        if self.current_find_attempts >= self.max_find_attempts:
            return False
        return True

    def is_landing_pad_in_sight(self) -> bool:
        frame = self.camera.last_image
        bounding_boxes, _ = self.get_bounding_boxes(frame)
        if len(bounding_boxes) > 0:
            general_logger.info(f"{len(bounding_boxes)} bases detectadas")
            self.current_find_attempts = 0
            return True
        return False

    def take_closest_landing_pad(self, cx, cy, bounding_boxes):
        # Find the bounding box whose center is closest to (cx, cy)
        min_dist = float("inf")
        closest_bb = None
        x = None
        y = None
        for bb in bounding_boxes:
            x0, y0, x1, y1 = bb.tolist()
            bb_cx = (x0 + x1) / 2
            bb_cy = (y0 + y1) / 2
            dist = math.hypot(bb_cx - cx, bb_cy - cy)
            if dist < min_dist:
                min_dist = dist
                closest_bb = bb
                x, y = bb_cx, bb_cy
        return x, y, closest_bb

    def zero_in_landing_pad(self):
        bounding_boxes = self.last_bounding_boxes
        if len(bounding_boxes) == 0:
            general_logger.error("Nenhuma base detectada, não é possível centralizar")
            return
        x, y, z, yaw = self.state.trajectory.get_drone_x_y_z_heading(should_fail=False)
        height, width, _ = self.camera.last_image.shape
        cx, cy = width // 2, height // 2
        bbx, bby, closest_bb = self.take_closest_landing_pad(cx, cy, bounding_boxes)
        if closest_bb is not None:
            vx, vy = bbx - cx, bby - cy
        else:
            vx, vy = 0, 0

        self.show_frame(
            name="zero_in_landing_pad",
            resize=True,
            bounding_boxes=bounding_boxes,
            center_point=True,
            vector=[vx, vy],
        )

        magnitude = math.hypot(vx, vy)
        general_logger.info(f"Vector magnitude: {magnitude}")
        bb_size = math.hypot(
            closest_bb[2] - closest_bb[0], closest_bb[3] - closest_bb[1]
        )
        general_logger.info(f"Bounding box size: {bb_size}")

        self.final_approach_attempts = 10
        # too close, dont move
        if bb_size > 550:
            vx, vy = 0, 0
            self.final_approach_attempts = 30

        # too close, dont move
        if magnitude < 30:
            vx, vy = 0, 0

        DELTA = 0.009

        # bb_size moderates how close the drone is to base, so small changes in up-close imagine do not cause big moves.
        nx = x + vx * ((700 - bb_size) / 700) * DELTA
        ny = y - vy * ((700 - bb_size) / 700) * DELTA

        self.drone_control.publish_position_setpoint_smoothed(
            nx, ny, z, heading=yaw, smooth_factor=0.9
        )

    def is_on_top_of_landing_pad(self):
        bounding_boxes = self.last_bounding_boxes
        general_logger.info(f"{len(bounding_boxes)} bases detectadas")
        if len(bounding_boxes) == 0:
            return False
        center_image = (
            self.camera.last_image.shape[1] // 2,
            self.camera.last_image.shape[0] // 2,
        )
        for base in bounding_boxes:
            x0, y0, x1, y1 = base
            x0, x1 = sorted([x0, x1])
            y0, y1 = sorted([y0, y1])
            if not (x0 < center_image[0] < x1 and y0 < center_image[1] < y1):
                return False
        return True

    def approach_for_landing(self):
        for i in range(0, self.final_approach_attempts):
            self.fly_with_delta_height(delta=-0.09)
            self.ros_node.custom_sleep(0.1)

    def is_landed(self):
        return False

    def get_finite_state_machine(self) -> dict[callable, dict]:
        return {
            # ======== SCANNING PHASE
            self.init: {"next": {"call": self.engage_offboard_mode}},
            self.engage_offboard_mode: {"next": {"call": self.arm}},
            self.arm: {"next": {"call": self.takeoff}},
            self.takeoff: {"next": {"call": self.is_airborne}},
            self.is_airborne: {
                "yes": {"call": self.navigate_to_next_destination},
                "no": {"call": self.engage_offboard_mode},
            },
            self.navigate_to_next_destination: {
                "next": {"call": self.is_landing_pad_in_sight},
            },
            self.is_landing_pad_in_sight: {
                "yes": {"call": self.find_landing_pad},
                "no": {"call": self.is_on_top_of_destination},
            },
            self.is_on_top_of_destination: {
                "yes": {"call": self.set_destination_visited},
                "no": {"call": self.navigate_to_next_destination},
            },
            self.set_destination_visited: {
                "next": {"call": self.navigate_to_next_destination},
            },
            # ======== LANDING PHASE
            self.find_landing_pad: {
                "yes": {"call": self.zero_in_landing_pad},
                "no": {"call": self.is_on_top_of_destination},
            },
            self.zero_in_landing_pad: {
                "next": {"call": self.is_on_top_of_landing_pad},
            },
            self.is_on_top_of_landing_pad: {
                "yes": {"call": self.approach_for_landing},
                "no": {"call": self.find_landing_pad},
            },
            self.approach_for_landing: {
                "next": {"call": self.is_landed},
            },
            self.is_landed: {
                "yes": {"call": self.takeoff},
                "no": {"call": self.find_landing_pad},
            },
        }


def main(args=None):
    rclpy.init(args=args)

    # logger = DroneLogger()
    # config = Config()
    state = DroneState()
    # plot <- state
    # logger <- state
    ros_node = ROSNode(state)
    drone_control = DroneControl(state, ros_node)
    camera = Camera(drone_control)
    mission = Mission(ros_node, camera, state, drone_control)
    # key <- keyboard_capture
    # action <- mission, keyboard_capture
    
    try:
        rclpy.spin(drone_control)
        mission.init()
    finally:
        drone_control.destroy_node()
        rclpy.shutdown(context=drone_control.context)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _sigint_handler)
    try:
        main()
    except Exception as e:
        general_logger.error(e)
