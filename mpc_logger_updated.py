#!/usr/bin/env python3
import json
import math
from dataclasses import dataclass, asdict
from typing import Optional, List, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from autoware_planning_msgs.msg import Trajectory
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_control_msgs.msg import Control  # ✅ your actual type

CONTROL_KIND = "autoware_control_msgs/Control"


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


@dataclass
class MPCLogSample:
    t_nsec: int

    # timing (receive time, to estimate controller latency)
    traj_rx_nsec: int
    ctrl_rx_nsec: int
    latency_ns: int
    latency_ms: float

    # vehicle state (from odom)
    x: float
    y: float
    yaw: float
    v: float

    # steering feedback (from SteeringReport)
    steer_tire_angle: float

    # trajectory snapshot (from Trajectory)
    traj_frame: str
    traj_size: int
    traj_head_xy: List[List[float]]
    traj_head_vel: List[float]

    # control output snapshot (from /control_cmd)
    control_kind: str
    target_steer_tire_angle: float
    target_steer_rate: float
    target_velocity: float
    target_accel: float
    target_jerk: float
    is_defined_acceleration: bool
    is_defined_jerk: bool


class MPCLogger(Node):
    def __init__(self):
        super().__init__("mpc_logger")

        # ----- params -----
        self.declare_parameter("traj_topic", "/planning/scenario_planning/trajectory")
        self.declare_parameter("odom_topic", "/localization/kinematic_state")
        self.declare_parameter("steer_topic", "/vehicle/status/steering_status")
        self.declare_parameter("control_topic", "/control/trajectory_follower/control_cmd")

        self.declare_parameter("dump_path", "/tmp/mpc_log.jsonl")
        self.declare_parameter("head_k", 10)
        self.declare_parameter("rate_hz", 33.333333)  # dt ≈ 0.03s
        self.declare_parameter("algo", "mpc")  # "mpc" / "pp" just a tag

        self.traj_topic = str(self.get_parameter("traj_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.steer_topic = str(self.get_parameter("steer_topic").value)
        self.control_topic = str(self.get_parameter("control_topic").value)

        self.dump_path = str(self.get_parameter("dump_path").value)
        self.head_k = int(self.get_parameter("head_k").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.algo = str(self.get_parameter("algo").value)

        # ----- last msgs -----
        self.last_traj: Optional[Trajectory] = None
        self.last_odom: Optional[Odometry] = None
        self.last_steer: Optional[SteeringReport] = None
        self.last_ctrl: Optional[Control] = None

        # ---- timing ----
        self.last_traj_rx_nsec: Optional[int] = None
        self.last_ctrl_rx_nsec: Optional[int] = None
        self._lat_sum_ns: int = 0
        self._lat_n: int = 0

        # optional in-memory buffer
        self.MPC_OUTPUT: List[Dict[str, Any]] = []

        # ----- QoS -----
        qos10 = QoSProfile(depth=10)

        # publisher is RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1)
        ctrl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ----- subs -----
        self.create_subscription(Trajectory, self.traj_topic, self.on_traj, qos10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos10)
        self.create_subscription(SteeringReport, self.steer_topic, self.on_steer, qos10)
        self.create_subscription(Control, self.control_topic, self.on_ctrl, ctrl_qos)

        # ----- timer -----
        period = 1.0 / max(self.rate_hz, 1e-6)
        self.create_timer(period, self.try_pack)

        # ----- log -----
        self.get_logger().info("=== MPC/PP Logger Started ===")
        self.get_logger().info(f"algo:         {self.algo}")
        self.get_logger().info(f"traj_topic:   {self.traj_topic}")
        self.get_logger().info(f"odom_topic:   {self.odom_topic}")
        self.get_logger().info(f"steer_topic:  {self.steer_topic}")
        self.get_logger().info(f"control_topic:{self.control_topic}")
        self.get_logger().info(f"dump_path:    {self.dump_path}")
        self.get_logger().info(f"rate_hz:      {self.rate_hz} (dt≈{1.0/self.rate_hz:.3f}s)")
        self.get_logger().info(f"head_k:       {self.head_k}")
        self.get_logger().info(f"control_kind: {CONTROL_KIND}")

    # ----- callbacks -----
    def on_traj(self, msg: Trajectory):
        self.last_traj = msg
        self.last_traj_rx_nsec = int(self.get_clock().now().nanoseconds)

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def on_steer(self, msg: SteeringReport):
        self.last_steer = msg

    def on_ctrl(self, msg: Control):
        self.last_ctrl = msg
        self.last_ctrl_rx_nsec = int(self.get_clock().now().nanoseconds)

    # ----- helpers -----
    def _extract_control(self) -> Dict[str, Any]:
        m = self.last_ctrl
        if m is None:
            return {
                "steer": 0.0,
                "steer_rate": 0.0,
                "vel": 0.0,
                "acc": 0.0,
                "jerk": 0.0,
                "is_def_acc": False,
                "is_def_jerk": False,
            }

        steer = float(m.lateral.steering_tire_angle)
        steer_rate = float(m.lateral.steering_tire_rotation_rate)
        vel = float(m.longitudinal.velocity)
        acc = float(m.longitudinal.acceleration)
        jerk = float(m.longitudinal.jerk)
        is_def_acc = bool(m.longitudinal.is_defined_acceleration)
        is_def_jerk = bool(m.longitudinal.is_defined_jerk)

        return {
            "steer": steer,
            "steer_rate": steer_rate,
            "vel": vel,
            "acc": acc,
            "jerk": jerk,
            "is_def_acc": is_def_acc,
            "is_def_jerk": is_def_jerk,
        }

    # ----- main pack -----
    def try_pack(self):
        # wait until we have everything
        if self.last_traj is None or self.last_odom is None or self.last_steer is None or self.last_ctrl is None:
            return

        now = int(self.get_clock().now().nanoseconds)

        odom = self.last_odom
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        yaw = quat_to_yaw(q)
        v = float(odom.twist.twist.linear.x)

        steer_fb = float(self.last_steer.steering_tire_angle)

        traj = self.last_traj
        pts = traj.points
        k = min(self.head_k, len(pts))

        head_xy: List[List[float]] = []
        head_vel: List[float] = []
        for i in range(k):
            tp = pts[i]
            head_xy.append([float(tp.pose.position.x), float(tp.pose.position.y)])
            vel_i = getattr(tp, "longitudinal_velocity_mps", 0.0)
            head_vel.append(float(vel_i))

        ctrl = self._extract_control()
        # latency estimate: time from latest Trajectory reception -> latest Control reception
        traj_rx = int(self.last_traj_rx_nsec or 0)
        ctrl_rx = int(self.last_ctrl_rx_nsec or 0)
        latency_ns = int(ctrl_rx - traj_rx) if (traj_rx and ctrl_rx and ctrl_rx >= traj_rx) else -1
        latency_ms = (latency_ns / 1e6) if latency_ns >= 0 else -1.0


        sample = MPCLogSample(
            t_nsec=now,
            traj_rx_nsec=traj_rx,
            ctrl_rx_nsec=ctrl_rx,
            latency_ns=latency_ns,
            latency_ms=float(latency_ms),
            x=float(p.x),
            y=float(p.y),
            yaw=float(yaw),
            v=v,
            steer_tire_angle=steer_fb,
            traj_frame=str(traj.header.frame_id),
            traj_size=int(len(pts)),
            traj_head_xy=head_xy,
            traj_head_vel=head_vel,
            control_kind=CONTROL_KIND,
            target_steer_tire_angle=float(ctrl["steer"]),
            target_steer_rate=float(ctrl["steer_rate"]),
            target_velocity=float(ctrl["vel"]),
            target_accel=float(ctrl["acc"]),
            target_jerk=float(ctrl["jerk"]),
            is_defined_acceleration=bool(ctrl["is_def_acc"]),
            is_defined_jerk=bool(ctrl["is_def_jerk"]),
        )

        d = asdict(sample)
        d["algo"] = self.algo  # tag

        self.MPC_OUTPUT.append(d)

        if latency_ns >= 0:
            self._lat_sum_ns += int(latency_ns)
            self._lat_n += 1
            if (self._lat_n % 100) == 0:
                avg_ms = (self._lat_sum_ns / max(self._lat_n, 1)) / 1e6
                self.get_logger().info(f\"[latency] avg over {self._lat_n} samples: {avg_ms:.3f} ms\")

        with open(self.dump_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(d, ensure_ascii=False) + "\n")


def main():
    rclpy.init()
    node = MPCLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

