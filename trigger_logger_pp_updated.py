#!/usr/bin/env python3
import argparse
import math
import os
import subprocess
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_internal_planning_msgs.msg import VelocityLimit
from autoware_system_msgs.srv import ChangeOperationMode, ChangeAutowareControl
from autoware_adapi_v1_msgs.msg import OperationModeState


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class TriggerLoggerPP(Node):
    def __init__(self, args):
        super().__init__("trigger_logger_pp")
        self.args = args

        self.last_odom: Optional[Odometry] = None
        self.last_traj: Optional[Trajectory] = None
        self.last_mode: Optional[OperationModeState] = None

        # publishers
        self.pub_init3d = self.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", 10)
        self.pub_init2d = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.pub_goal = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 10)

        vel_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_vel = self.create_publisher(VelocityLimit, "/planning/scenario_planning/max_velocity", vel_qos)

        # subscribers (readiness gating)
        self.create_subscription(Odometry, self.args.odom_topic, self.on_odom, 10)
        self.create_subscription(Trajectory, self.args.traj_topic, self.on_traj, 10)

        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OperationModeState, "/system/operation_mode/state", self.on_mode, mode_qos)

        # service clients
        self.cli_mode = self.create_client(ChangeOperationMode, "/system/operation_mode/change_operation_mode")
        self.cli_ctrl = self.create_client(ChangeAutowareControl, "/system/operation_mode/change_autoware_control")

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def on_traj(self, msg: Trajectory):
        self.last_traj = msg

    def on_mode(self, msg: OperationModeState):
        self.last_mode = msg

    def wait_services(self, timeout_sec: float = 15.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            ok1 = self.cli_mode.wait_for_service(timeout_sec=0.2)
            ok2 = self.cli_ctrl.wait_for_service(timeout_sec=0.2)
            if ok1 and ok2:
                return True
        return False

    def publish_start_goal_speed(self):
        # START
        start_x = 8.674476623535156
        start_y = -7.661574840545654
        start_yaw = -1.5179005546573276

        msg_init = PoseWithCovarianceStamped()
        msg_init.header.frame_id = "map"
        msg_init.pose.pose.position.x = float(start_x)
        msg_init.pose.pose.position.y = float(start_y)
        msg_init.pose.pose.position.z = 0.0
        msg_init.pose.pose.orientation = yaw_to_quat(start_yaw)
        msg_init.pose.covariance = [0.0] * 36

        for _ in range(3):
            self.pub_init3d.publish(msg_init)
            self.pub_init2d.publish(msg_init)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Published START -> /initialpose3d /initialpose")

        # GOAL
        goal_x = 29.963605880737305
        goal_y = 0.25403621792793274
        goal_yaw = 3.123180900871389

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = "map"
        msg_goal.pose.position.x = float(goal_x)
        msg_goal.pose.position.y = float(goal_y)
        msg_goal.pose.position.z = 0.0
        msg_goal.pose.orientation = yaw_to_quat(goal_yaw)

        for _ in range(3):
            self.pub_goal.publish(msg_goal)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Published GOAL -> /planning/mission_planning/goal")

        # MAX SPEED 10 km/h
        v_kmh = 10.0
        v_ms = v_kmh / 3.6

        msg_v = VelocityLimit()
        msg_v.stamp.sec = 0
        msg_v.stamp.nanosec = 0
        msg_v.max_velocity = float(v_ms)
        msg_v.use_constraints = False
        msg_v.sender = "trigger_logger_pp"

        for _ in range(3):
            self.pub_vel.publish(msg_v)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            f"Published MAX_SPEED {v_kmh:.1f} km/h ({v_ms:.6f} m/s) -> /planning/scenario_planning/max_velocity"
        )

    def wait_ready_conditions(self, timeout_sec: float = 20.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

            odom_ok = self.last_odom is not None
            traj_ok = (self.last_traj is not None) and (len(self.last_traj.points) > 0)
            mode_ok = (
                self.last_mode is not None
                and bool(self.last_mode.is_autonomous_mode_available)
                and (not bool(self.last_mode.is_in_transition))
            )

            if odom_ok and traj_ok and mode_ok:
                self.get_logger().info("Ready: odom OK, trajectory OK, autonomous available.")
                return True

        self.get_logger().error(
            f"Not ready in time. odom={self.last_odom is not None}, "
            f"traj={(self.last_traj is not None)} size={(len(self.last_traj.points) if self.last_traj else 0)}, "
            f"mode={(self.last_mode.mode if self.last_mode else None)}, "
            f"auto_avail={(self.last_mode.is_autonomous_mode_available if self.last_mode else None)}, "
            f"in_trans={(self.last_mode.is_in_transition if self.last_mode else None)}"
        )
        return False

    def call_change_mode(self, mode: int, timeout_sec: float = 5.0) -> bool:
        req = ChangeOperationMode.Request()
        req.mode = int(mode)
        fut = self.cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            if getattr(res.status, "success", False):
                return True
            self.get_logger().error(f"ChangeOperationMode failed: {res.status.message}")
            return False
        self.get_logger().error("ChangeOperationMode no response (timeout).")
        return False

    def call_change_control(self, enable: bool, timeout_sec: float = 5.0) -> bool:
        req = ChangeAutowareControl.Request()
        req.autoware_control = bool(enable)
        fut = self.cli_ctrl.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            if getattr(res.status, "success", False):
                return True
            self.get_logger().error(f"ChangeAutowareControl failed: {res.status.message}")
            return False
        self.get_logger().error("ChangeAutowareControl no response (timeout).")
        return False

    def enter_autonomous(self) -> bool:
        STOP = 1
        AUTONOMOUS = 2

        if not self.call_change_mode(STOP):
            return False
        self.get_logger().info("Changed operation mode -> STOP")

        if not self.call_change_control(True):
            return False
        self.get_logger().info("Enabled Autoware control")

        if not self.wait_ready_conditions(timeout_sec=20.0):
            return False

        for _ in range(10):
            if self.call_change_mode(AUTONOMOUS):
                self.get_logger().info("Changed operation mode -> AUTONOMOUS")
                return True
            time.sleep(0.3)

        return False

    def start_pp_logger_process(self) -> int:
        dump_path = self.args.dump_path
        rate_hz = self.args.rate_hz
        head_k = self.args.head_k

        if self.args.truncate and dump_path:
            try:
                os.remove(dump_path)
            except FileNotFoundError:
                pass

        cmd = [
            "ros2", "run", "mpc_io_logger", "pp_logger",
            "--ros-args",
            "-p", f"traj_topic:={self.args.traj_topic}",
            "-p", f"odom_topic:={self.args.odom_topic}",
            "-p", f"steer_topic:={self.args.steer_topic}",
            "-p", f"control_topic:={self.args.control_topic}",
            "-p", f"dump_path:={dump_path}",
            "-p", "algo:=pp",
            "-p", f"rate_hz:={rate_hz}",
            "-p", f"head_k:={head_k}",
        ]

        self.get_logger().info("Starting logger:\n  " + " ".join(cmd))
        proc = subprocess.Popen(cmd)
        self.get_logger().info("Running. Press Ctrl-C to stop.")

        try:
            while proc.poll() is None:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass

        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()

        return int(proc.returncode or 0)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="PP one-shot trigger: start+goal+speed+autonomous+log")
    p.add_argument("--algo", default="pp", choices=["pp"])

    # ✅ 跟 controller_node_exe 對齊（重點就在這行）
    p.add_argument("--traj_topic", default="/planning/trajectory")

    p.add_argument("--odom_topic", default="/localization/kinematic_state")
    p.add_argument("--steer_topic", default="/vehicle/status/steering_status")
    p.add_argument("--control_topic", default="/control/trajectory_follower/control_cmd")

    p.add_argument("--dump_path", default="/tmp/pp_log.jsonl")
    p.add_argument("--rate_hz", type=float, default=33.333333)
    p.add_argument("--head_k", type=int, default=10)
    p.add_argument("--truncate", action="store_true", help="Remove dump_path first")

    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    rclpy.init()
    node = TriggerLoggerPP(args)

    node.publish_start_goal_speed()

    if not node.wait_services(timeout_sec=15.0):
        node.get_logger().error("Operation mode services not available.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not node.enter_autonomous():
        node.get_logger().error("Failed to enter autonomous.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    rc = node.start_pp_logger_process()

    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())

