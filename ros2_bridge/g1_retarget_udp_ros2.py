"""Expose GMR and Isaac G1 telemetry as standard ROS 2 messages.

The node is visualization-only: it does not publish Unitree DDS commands.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


SCHEMAS = {
    "zed_gmr_g1_23dof_live/v1",
    "zed_gmr_g1_23dof_isaac_telemetry/v1",
}

G1_23DOF_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
]


def args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15053)
    parser.add_argument("--frame-id", default="odom")
    parser.add_argument("--base-frame", default="pelvis")
    parser.add_argument("--stale-after", type=float, default=0.5)
    return parser.parse_args()


def finite_vec(value, size=3) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == size
        and all(isinstance(v, (int, float)) and math.isfinite(v) for v in value)
    )


def color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class G1RetargetRosBridge(Node):
    def __init__(self, options: argparse.Namespace) -> None:
        super().__init__("g1_retarget_udp_bridge")
        self.options = options
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((options.listen_host, options.listen_port))
        self.sock.setblocking(False)
        reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.raw_joint_pub = self.create_publisher(JointState, "/g1/retarget/raw_joint_states", reliable)
        self.safe_joint_pub = self.create_publisher(JointState, "/g1/retarget/safe_joint_states", reliable)
        self.actual_joint_pub = self.create_publisher(JointState, "/g1/isaac/joint_states", reliable)
        self.display_joint_pub = self.create_publisher(JointState, "/g1/display/joint_states", reliable)
        self.human_marker_pub = self.create_publisher(MarkerArray, "/g1/retarget/human_markers", reliable)
        self.raw_marker_pub = self.create_publisher(MarkerArray, "/g1/retarget/raw_markers", reliable)
        self.safe_marker_pub = self.create_publisher(MarkerArray, "/g1/retarget/safe_markers", reliable)
        self.actual_marker_pub = self.create_publisher(MarkerArray, "/g1/isaac/actual_markers", reliable)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/g1/retarget/diagnostics", reliable)
        self.safety_pub = self.create_publisher(String, "/g1/retarget/safety", transient)
        self.tf = TransformBroadcaster(self)
        self.last_rx = 0.0
        self.last_sequence = -1
        self.rx_count = 0
        self.accepted = 0
        self.arrivals = []
        self.stale_sent = False
        self.create_timer(1.0 / 120.0, self.poll)
        self.create_timer(1.0, self.status_tick)
        self.get_logger().info(
            f"GMR/Isaac UDP {options.listen_host}:{options.listen_port} -> ROS 2 G1 topics"
        )

    def destroy_node(self):
        self.sock.close()
        return super().destroy_node()

    def poll(self) -> None:
        newest = None
        while True:
            try:
                payload, _ = self.sock.recvfrom(262144)
            except BlockingIOError:
                break
            except OSError:
                return
            try:
                packet = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            self.rx_count += 1
            if packet.get("schema") in SCHEMAS:
                newest = packet
        if newest is not None:
            sequence = int(newest.get("sequence", -1))
            if sequence != self.last_sequence or newest.get("schema", "").endswith("telemetry/v1"):
                self.last_sequence = sequence
                self.last_rx = time.monotonic()
                self.arrivals.append(self.last_rx)
                self.accepted += 1
                self.stale_sent = False
                self.publish_packet(newest)

    def publish_joint_state(self, publisher, names, values, stamp) -> bool:
        if not isinstance(names, list) or not isinstance(values, list) or len(names) != len(values):
            return False
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            return False
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [str(name) for name in names]
        msg.position = [float(v) for v in values]
        publisher.publish(msg)
        return True

    def publish_packet(self, packet: dict) -> None:
        stamp = self.get_clock().now().to_msg()
        names = packet.get("joint_names", [])
        raw = packet.get("raw_joint_position_rad", [])
        safe = packet.get("safe_joint_position_rad", packet.get("joint_position_rad", []))
        self.publish_joint_state(self.raw_joint_pub, names, raw, stamp)
        safe_ok = self.publish_joint_state(self.safe_joint_pub, names, safe, stamp)

        isaac = packet.get("isaac_metrics") or {}
        actual_names = isaac.get("joint_names", names)
        actual = isaac.get("actual_joint_position_rad", [])
        actual_ok = self.publish_joint_state(self.actual_joint_pub, actual_names, actual, stamp)
        if actual_ok:
            self.publish_joint_state(self.display_joint_pub, actual_names, actual, stamp)
        elif safe_ok:
            self.publish_joint_state(self.display_joint_pub, names, safe, stamp)

        skeleton = packet.get("g1_skeleton") or {}
        edges = skeleton.get("edges", [])
        self.publish_skeleton(self.raw_marker_pub, "g1_raw", skeleton.get("raw_positions_m"), edges, (1.0, 0.55, 0.10), stamp)
        self.publish_skeleton(self.safe_marker_pub, "g1_safe", skeleton.get("safe_positions_m"), edges, (0.15, 0.95, 0.35), stamp)
        self.publish_skeleton(self.actual_marker_pub, "g1_actual", skeleton.get("actual_positions_m"), edges, (0.90, 0.20, 0.95), stamp)
        comparison = packet.get("retarget_comparison") or {}
        self.publish_skeleton(
            self.human_marker_pub,
            "human_retarget",
            comparison.get("human_positions_m"),
            comparison.get("human_edges", []),
            (0.10, 0.65, 1.0),
            stamp,
        )

        self.publish_base_tf(skeleton.get("actual_positions_m") or skeleton.get("safe_positions_m"), isaac, stamp)
        safety = packet.get("safety") or {}
        self.safety_pub.publish(String(data=json.dumps(safety, separators=(",", ":"))))
        self.publish_diag(packet, stale=False)

    def publish_skeleton(self, publisher, namespace, positions, edges, rgb, stamp) -> None:
        if not isinstance(positions, dict) or not positions:
            return
        valid = {str(k): v for k, v in positions.items() if finite_vec(v)}
        array = MarkerArray()
        line = Marker()
        line.header.frame_id, line.header.stamp = self.options.frame_id, stamp
        line.ns, line.id, line.type, line.action = namespace + "_bones", 0, Marker.LINE_LIST, Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.022
        line.color = color(*rgb)
        line.lifetime.nanosec = 350_000_000
        for pair in edges if isinstance(edges, list) else []:
            if not isinstance(pair, (list, tuple)) or len(pair) != 2:
                continue
            if str(pair[0]) not in valid or str(pair[1]) not in valid:
                continue
            for key in pair:
                p = valid[str(key)]
                line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        array.markers.append(line)
        joints = Marker()
        joints.header.frame_id, joints.header.stamp = self.options.frame_id, stamp
        joints.ns, joints.id, joints.type, joints.action = namespace + "_joints", 1, Marker.SPHERE_LIST, Marker.ADD
        joints.pose.orientation.w = 1.0
        joints.scale.x = joints.scale.y = joints.scale.z = 0.045
        joints.color = color(*rgb)
        joints.lifetime.nanosec = 350_000_000
        for p in valid.values():
            joints.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        array.markers.append(joints)
        publisher.publish(array)

    def publish_base_tf(self, positions, isaac, stamp) -> None:
        pelvis = (positions or {}).get("pelvis", [0.0, 0.0, 0.793])
        if not finite_vec(pelvis):
            pelvis = [0.0, 0.0, 0.793]
        roll = float(isaac.get("base_roll_rad", 0.0))
        pitch = float(isaac.get("base_pitch_rad", 0.0))
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, 0.0)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.options.frame_id
        transform.child_frame_id = self.options.base_frame
        transform.transform.translation.x = float(pelvis[0])
        transform.transform.translation.y = float(pelvis[1])
        transform.transform.translation.z = float(pelvis[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf.sendTransform(transform)

    def publish_diag(self, packet: dict, stale: bool) -> None:
        safety = packet.get("safety") or {}
        level_name = str(safety.get("level", "UNKNOWN"))
        ros_level = DiagnosticStatus.WARN if level_name in ("YELLOW", "ORANGE") else DiagnosticStatus.ERROR if level_name == "RED" or stale else DiagnosticStatus.OK
        bridge = packet.get("bridge_metrics") or {}
        isaac = packet.get("isaac_metrics") or {}
        status = DiagnosticStatus(
            level=ros_level,
            name="g1_retarget_udp_bridge",
            hardware_id="G1-23DOF-simulation",
            message="STALE" if stale else level_name,
        )
        status.values = [
            KeyValue(key="sequence", value=str(packet.get("sequence", self.last_sequence))),
            KeyValue(key="safety_reasons", value=",".join(map(str, safety.get("reasons", [])))),
            KeyValue(key="gmr_solve_ms", value=str(bridge.get("solve_ms", "nan"))),
            KeyValue(key="joint_tracking_rmse_rad", value=str(isaac.get("joint_tracking_rmse_rad", "nan"))),
            KeyValue(key="body_tracking_mpjpe_m", value=str(isaac.get("body_tracking_mpjpe_m", "nan"))),
            KeyValue(key="physical_robot_output", value="false"),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)

    def status_tick(self) -> None:
        now = time.monotonic()
        self.arrivals = [t for t in self.arrivals if now - t <= 1.0]
        if not self.last_rx:
            stamp = self.get_clock().now().to_msg()
            self.publish_joint_state(self.display_joint_pub, G1_23DOF_ORDER, [0.0] * 23, stamp)
            self.publish_base_tf(None, {}, stamp)
        if self.last_rx and now - self.last_rx > self.options.stale_after and not self.stale_sent:
            self.publish_diag({"sequence": self.last_sequence, "safety": {"level": "RED", "reasons": ["stale_packet"]}}, stale=True)
            self.stale_sent = True
        self.get_logger().info(f"rx={self.rx_count} accepted={self.accepted} hz={len(self.arrivals)} sequence={self.last_sequence}")


def main() -> int:
    options = args()
    rclpy.init()
    node = G1RetargetRosBridge(options)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
