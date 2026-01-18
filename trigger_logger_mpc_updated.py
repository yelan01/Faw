#!/usr/bin/env python3
import argparse
import math
import os
import signal
import subprocess
import sys
import time
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_system_msgs.srv import ChangeOperationMode, ChangeAutowareControl

# ===== 固定起點 / 終點 / 最高速（你提供的值）=====
START = {
    "frame_id": "map",
    "x": 8.674476623535156,
    "y": -7.661574840545654,
    "yaw": -1.5179005546573276,  # rad
}

GOAL = {
    "frame_id": "map",
    "x": 29.963605880737305,
    "y": 0.25403621792793274,
    "yaw": 3.123180900871389,  # rad
}

MAX_SPEED_KMH = 10.0
MAX_SPEED_MS = MAX_SPEED_KMH / 3.6  # m/s

# ===== 固定 Topic/Service（跟你現有 AW setup 一致）=====
TOPIC_INITIALPOSE3D = "/initialpose3d"
TOPIC_INITIALPOSE2D = "/initialpose"
TOPIC_GOAL = "/planning/mission_planning/goal"
TOPIC_MAX_VELOCITY = "/planning/scenario_planning/max_velocity"
TOPIC_OP_MODE_STATE = "/system/operation_mode/state"

SRV_CHANGE_OP_MODE = "/system/operation_mode/change_operation_mode"
SRV_CHANGE_AW_CONTROL = "/system/operation_mode/change_autoware_control"


def yaw_to_quat_z_w(yaw: float) -> Tuple[float, float]:
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


class TriggerLogger(Node):
    def __init__(self, args):
        super().__init__("trigger_logger_mpc")
        self.args = args

        qos_tl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_init3d = self.create_publisher(PoseWithCovarianceStamped, TOPIC_INITIALPOSE3D, 10)
        self.pub_init2d = self.create_publisher(PoseWithCovarianceStamped, TOPIC_INITIALPOSE2D, 10)
        self.pub_goal = self.create_publisher(PoseStamped, TOPIC_GOAL, 10)
        self.pub_vlim = self.create_publisher(VelocityLimit, TOPIC_MAX_VELOCITY, qos_tl)

        self.op_state: Optional[OperationModeState] = None
        self.create_subscription(OperationModeState, TOPIC_OP_MODE_STATE, self._cb_op, qos_tl)

        self.cli_mode = self.create_client(ChangeOperationMode, SRV_CHANGE_OP_MODE)
        self.cli_ctrl = self.create_client(ChangeAutowareControl, SRV_CHANGE_AW_CONTROL)

        self.logger_proc = None

    def _cb_op(self, msg: OperationModeState):
        self.op_state = msg

    def publish_start_goal_speed(self):
        # ---- START ----
        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = START["frame_id"]
        init_msg.header.stamp = self.get_clock().now().to_msg()
        init_msg.pose.pose.position.x = float(START["x"])
        init_msg.pose.pose.position.y = float(START["y"])
        init_msg.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quat_z_w(float(START["yaw"]))
        init_msg.pose.pose.orientation.z = qz
        init_msg.pose.pose.orientation.w = qw
        init_msg.pose.covariance = [0.0] * 36

        # 兩個都發，避免不同 stack 吃不同 topic
        for _ in range(3):
            self.pub_init3d.publish(init_msg)
            self.pub_init2d.publish(init_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"Published START -> {TOPIC_INITIALPOSE3D} / {TOPIC_INITIALPOSE2D}")

        # ---- GOAL ----
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = GOAL["frame_id"]
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = float(GOAL["x"])
        goal_msg.pose.position.y = float(GOAL["y"])
        goal_msg.pose.position.z = 0.0
        qz, qw = yaw_to_quat_z_w(float(GOAL["yaw"]))
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        for _ in range(3):
            self.pub_goal.publish(goal_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(f"Published GOAL -> {TOPIC_GOAL}")

        # ---- MAX VELOCITY ----
        vmsg = VelocityLimit()
        vmsg.stamp = self.get_clock().now().to_msg()
        vmsg.max_velocity = float(MAX_SPEED_MS)
        vmsg.use_constraints = False
        vmsg.sender = "trigger_logger_mpc"

        for _ in range(3):
            self.pub_vlim.publish(vmsg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            f"Published MAX_SPEED {MAX_SPEED_KMH:.1f} km/h ({MAX_SPEED_MS:.6f} m/s) -> {TOPIC_MAX_VELOCITY}"
        )

    def wait_for_services(self, timeout_sec: float = 60.0) -> bool:
        t0 = time.time()
        last_print = 0.0
        while (time.time() - t0) < timeout_sec and rclpy.ok():
            ok1 = self.cli_mode.wait_for_service(timeout_sec=0.2)
            ok2 = self.cli_ctrl.wait_for_service(timeout_sec=0.2)
            if ok1 and ok2:
                return True

            now = time.time()
            if (now - last_print) > 2.0:
                self.get_logger().info(
                    f"Waiting op_mode services... mode={ok1} control={ok2} ({now - t0:.1f}/{timeout_sec:.1f}s)"
                )
                last_print = now
        return False

    def wait_until_autonomous_available(self, timeout_sec: float = 20.0) -> bool:
        t0 = time.time()
        while (time.time() - t0) < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.op_state and self.op_state.is_autonomous_mode_available and (not self.op_state.is_in_transition):
                return True
        return False

    def set_stop(self) -> bool:
        req = ChangeOperationMode.Request()
        req.mode = 1  # STOP
        fut = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().status.success:
            msg = fut.result().status.message if fut.result() else "no response"
            self.get_logger().warn(f"ChangeOperationMode(STOP) failed: {msg}")
            return False
        self.get_logger().info("Changed operation mode -> STOP")
        return True

    def enable_control(self) -> bool:
        req = ChangeAutowareControl.Request()
        req.autoware_control = True
        fut = self.cli_ctrl.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().status.success:
            msg = fut.result().status.message if fut.result() else "no response"
            self.get_logger().error(f"ChangeAutowareControl failed: {msg}")
            return False
        self.get_logger().info("Enabled Autoware control")
        return True

    def set_autonomous(self) -> bool:
        req = ChangeOperationMode.Request()
        req.mode = 2  # AUTONOMOUS
        fut = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None or not fut.result().status.success:
            msg = fut.result().status.message if fut.result() else "no response"
            self.get_logger().error(f"ChangeOperationMode(AUTONOMOUS) failed: {msg}")
            return False
        self.get_logger().info("Changed operation mode -> AUTONOMOUS")
        return True

    def wait_until_autonomous(self, timeout_sec: float = 10.0) -> bool:
        t0 = time.time()
        while (time.time() - t0) < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.op_state is None:
                continue
            if (
                self.op_state.mode == 2
                and self.op_state.is_autoware_control_enabled
                and (not self.op_state.is_in_transition)
            ):
                return True
        return False

    def start_logger(self, algo: str, dump_path: str) -> None:
        algo = (algo or "mpc").lower()
        if algo not in ("mpc", "pp"):
            self.get_logger().warn(f"Unknown algo='{algo}', fallback to 'mpc'")
            algo = "mpc"

        # 這裡用 mpc_logger（你的 package 內部會記錄：traj/odom/steer/control_cmd）
        cmd = [
            "ros2", "run", "mpc_io_logger", "mpc_logger",
            "--ros-args",
            "-p", f"traj_topic:={self.args.traj_topic}",
            "-p", f"odom_topic:={self.args.odom_topic}",
            "-p", f"steer_topic:={self.args.steer_topic}",
            "-p", f"control_topic:={self.args.control_topic}",
            "-p", f"dump_path:={dump_path}",
            "-p", f"algo:={algo}",
            "-p", f"rate_hz:={self.args.rate_hz}",
            "-p", f"head_k:={self.args.head_k}",
        ]
        self.get_logger().info("Starting logger:\n  " + " ".join(cmd))
        self.logger_proc = subprocess.Popen(cmd)

    def stop_logger(self):
        if self.logger_proc is not None and self.logger_proc.poll() is None:
            self.get_logger().info("Stopping logger...")
            self.logger_proc.terminate()
            try:
                self.logger_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.logger_proc.kill()
        self.logger_proc = None


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="MPC/PP one-shot trigger: start+goal+speed+autonomous+log")
    p.add_argument("--algo", default="mpc", choices=["mpc", "pp"])
    p.add_argument("--dump-path", default="", help="Default: /tmp/<algo>_log.jsonl")
    p.add_argument("--truncate", action="store_true", help="Remove dump_path before starting (avoid mixing logs).")

    # ✅ 跟 controller_node_exe 對齊的預設
    p.add_argument("--traj_topic", default="/planning/trajectory")
    p.add_argument("--odom_topic", default="/localization/kinematic_state")
    p.add_argument("--steer_topic", default="/vehicle/status/steering_status")
    p.add_argument("--control_topic", default="/control/trajectory_follower/control_cmd")

    p.add_argument("--rate_hz", type=float, default=33.333333)
    p.add_argument("--head_k", type=int, default=10)

    p.add_argument("--service-timeout", type=float, default=60.0)
    p.add_argument("--wait-auto-available", type=float, default=20.0)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    algo = args.algo.lower()
    dump_path = args.dump_path if args.dump_path else f"/tmp/{algo}_log.jsonl"

    if args.truncate and dump_path:
        try:
            if os.path.exists(dump_path):
                os.remove(dump_path)
        except Exception as e:
            print(f"[trigger_logger_mpc] WARN: failed to truncate '{dump_path}': {e}", file=sys.stderr)

    rclpy.init()
    node = TriggerLogger(args)

    def _sig_handler(sig, frame):
        node.stop_logger()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _sig_handler)
    signal.signal(signal.SIGTERM, _sig_handler)

    time.sleep(0.3)

    # 1) 固定 start/goal/speed
    node.publish_start_goal_speed()

    # 2) 等 op_mode services
    if not node.wait_for_services(timeout_sec=args.service_timeout):
        node.get_logger().error("Operation mode services not available.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 3) 先切 STOP
    node.set_stop()

    # 4) enable control
    if not node.enable_control():
        node.get_logger().error("Failed to enable Autoware control.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 5) 等 autonomous available
    if not node.wait_until_autonomous_available(timeout_sec=args.wait_auto_available):
        node.get_logger().error(
            "Autonomous mode not available. Likely localization not initialized. "
            "Please initialize localization (RViz) then rerun."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    # 6) 切 AUTONOMOUS
    if not node.set_autonomous():
        node.get_logger().error("Failed to enter autonomous.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.wait_until_autonomous(timeout_sec=10.0)

    # 7) start logger
    node.start_logger(algo=algo, dump_path=dump_path)

    node.get_logger().info("Running. Press Ctrl-C to stop.")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.logger_proc is not None and node.logger_proc.poll() is not None:
            node.get_logger().error(f"Logger exited with code {node.logger_proc.returncode}")
            break

    node.stop_logger()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

