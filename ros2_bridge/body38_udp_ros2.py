"""Publish the ZED BODY_38 UDP stream as ROS 2 visualization messages.

This node is perception-only. It publishes markers, poses and diagnostics and
has no Unitree DDS or robot command interface.
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


EDGES = (
    ("PELVIS", "SPINE_1"),
    ("SPINE_1", "SPINE_2"),
    ("SPINE_2", "SPINE_3"),
    ("SPINE_3", "NECK"),
    ("NECK", "NOSE"),
    ("SPINE_3", "LEFT_CLAVICLE"),
    ("LEFT_CLAVICLE", "LEFT_SHOULDER"),
    ("LEFT_SHOULDER", "LEFT_ELBOW"),
    ("LEFT_ELBOW", "LEFT_WRIST"),
    ("SPINE_3", "RIGHT_CLAVICLE"),
    ("RIGHT_CLAVICLE", "RIGHT_SHOULDER"),
    ("RIGHT_SHOULDER", "RIGHT_ELBOW"),
    ("RIGHT_ELBOW", "RIGHT_WRIST"),
    ("PELVIS", "LEFT_HIP"),
    ("LEFT_HIP", "LEFT_KNEE"),
    ("LEFT_KNEE", "LEFT_ANKLE"),
    ("LEFT_ANKLE", "LEFT_HEEL"),
    ("LEFT_ANKLE", "LEFT_BIG_TOE"),
    ("LEFT_BIG_TOE", "LEFT_SMALL_TOE"),
    ("PELVIS", "RIGHT_HIP"),
    ("RIGHT_HIP", "RIGHT_KNEE"),
    ("RIGHT_KNEE", "RIGHT_ANKLE"),
    ("RIGHT_ANKLE", "RIGHT_HEEL"),
    ("RIGHT_ANKLE", "RIGHT_BIG_TOE"),
    ("RIGHT_BIG_TOE", "RIGHT_SMALL_TOE"),
    ("LEFT_WRIST", "LEFT_HAND_THUMB_4"),
    ("LEFT_WRIST", "LEFT_HAND_INDEX_1"),
    ("LEFT_WRIST", "LEFT_HAND_MIDDLE_4"),
    ("LEFT_WRIST", "LEFT_HAND_PINKY_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_THUMB_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_INDEX_1"),
    ("RIGHT_WRIST", "RIGHT_HAND_MIDDLE_4"),
    ("RIGHT_WRIST", "RIGHT_HAND_PINKY_1"),
)

LABELS = (
    "PELVIS",
    "NECK",
    "LEFT_WRIST",
    "RIGHT_WRIST",
    "LEFT_ANKLE",
    "RIGHT_ANKLE",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=15054)
    parser.add_argument("--frame-id", default="zed_camera")
    parser.add_argument("--confidence", type=float, default=35.0)
    parser.add_argument("--stale-after", type=float, default=0.5)
    return parser.parse_args()


def valid_point(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 3
        and all(isinstance(component, (int, float)) and math.isfinite(component)
                for component in value)
    )


def finite_quaternion(value) -> bool:
    return (
        isinstance(value, (list, tuple))
        and len(value) == 4
        and all(isinstance(component, (int, float)) and math.isfinite(component)
                for component in value)
    )


def rgba(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    return ColorRGBA(r=red, g=green, b=blue, a=alpha)


class Body38RosBridge(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("zed_body38_udp_bridge")
        self.args = args
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((args.listen_host, args.listen_port))
        self.socket.setblocking(False)

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, "/zed/body38/markers", reliable
        )
        self.pose_publisher = self.create_publisher(
            PoseArray, "/zed/body38/keypoints", sensor_qos
        )
        self.orientation_publisher = self.create_publisher(
            PoseArray, "/zed/body38/local_orientations", sensor_qos
        )
        self.root_publisher = self.create_publisher(
            PoseStamped, "/zed/body38/root_pose", sensor_qos
        )
        self.confidence_publisher = self.create_publisher(
            Float32MultiArray, "/zed/body38/keypoint_confidence", sensor_qos
        )
        self.diagnostic_publisher = self.create_publisher(
            DiagnosticArray, "/zed/body38/diagnostics", reliable
        )
        self.last_packet_monotonic = 0.0
        self.last_sequence = -1
        self.packet_count = 0
        self.accepted_count = 0
        self.arrivals: list[float] = []
        self.last_status_print = time.monotonic()
        self.stale_published = False
        self.timer = self.create_timer(1.0 / 60.0, self.poll)
        self.get_logger().info(
            f"BODY_38 UDP {args.listen_host}:{args.listen_port} -> "
            "/zed/body38/markers, /zed/body38/keypoints"
        )

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()

    def poll(self) -> None:
        newest = None
        while True:
            try:
                payload, _source = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:
                return
            try:
                packet = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            self.packet_count += 1
            if packet.get("schema") == "zed_body38_live/status/v1":
                self.publish_status_event(packet)
                if packet.get("status") == "NO_BODY":
                    self.publish_delete_all()
                continue
            if packet.get("schema") == "zed_body38_live/v1":
                newest = packet

        if newest is not None:
            sequence = int(newest.get("sequence", -1))
            if sequence != self.last_sequence:
                self.last_sequence = sequence
                self.last_packet_monotonic = time.monotonic()
                self.arrivals.append(self.last_packet_monotonic)
                self.accepted_count += 1
                self.stale_published = False
                self.publish_body(newest)

        now = time.monotonic()
        self.arrivals = [arrival for arrival in self.arrivals if now - arrival <= 1.0]
        if (
            self.last_packet_monotonic
            and now - self.last_packet_monotonic > self.args.stale_after
            and not self.stale_published
        ):
            self.publish_delete_all()
            self.publish_diagnostics(
                level=DiagnosticStatus.WARN,
                message="STALE",
                body_id="-",
                confidence=0.0,
                valid_points=0,
            )
            self.stale_published = True
        if now - self.last_status_print >= 1.0:
            self.get_logger().info(
                f"rx={self.packet_count} accepted={self.accepted_count} "
                f"hz={len(self.arrivals)} sequence={self.last_sequence}"
            )
            self.last_status_print = now

    def publish_body(self, packet: dict) -> None:
        names = [str(name) for name in packet.get("keypoint_names", [])]
        values = packet.get("keypoints_3d_m", [])
        confidence_values = packet.get("keypoint_confidence", [])
        points = {}
        confidence = {}
        for index, name in enumerate(names):
            if index >= len(values) or not valid_point(values[index]):
                continue
            points[name] = tuple(float(component) for component in values[index])
            confidence[name] = (
                float(confidence_values[index])
                if index < len(confidence_values)
                and isinstance(confidence_values[index], (int, float))
                else 0.0
            )

        stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        lines = Marker()
        lines.header.frame_id = self.args.frame_id
        lines.header.stamp = stamp
        lines.ns = "body38_bones"
        lines.id = 0
        lines.type = Marker.LINE_LIST
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.018
        lines.color = rgba(0.25, 0.95, 0.45, 1.0)
        lines.lifetime.nanosec = 300_000_000
        for first, second in EDGES:
            if first not in points or second not in points:
                continue
            if min(confidence.get(first, 0.0), confidence.get(second, 0.0)) < (
                self.args.confidence
            ):
                continue
            for name in (first, second):
                x, y, z = points[name]
                lines.points.append(Point(x=x, y=y, z=z))
        marker_array.markers.append(lines)

        joints = Marker()
        joints.header.frame_id = self.args.frame_id
        joints.header.stamp = stamp
        joints.ns = "body38_joints"
        joints.id = 1
        joints.type = Marker.SPHERE_LIST
        joints.action = Marker.ADD
        joints.pose.orientation.w = 1.0
        joints.scale.x = 0.045
        joints.scale.y = 0.045
        joints.scale.z = 0.045
        joints.lifetime.nanosec = 300_000_000

        pose_array = PoseArray()
        pose_array.header.frame_id = self.args.frame_id
        pose_array.header.stamp = stamp
        for name in names:
            if name not in points:
                continue
            x, y, z = points[name]
            joints.points.append(Point(x=x, y=y, z=z))
            score = confidence.get(name, 0.0)
            joints.colors.append(
                rgba(0.20, 0.80, 1.0, 1.0)
                if score >= self.args.confidence
                else rgba(1.0, 0.45, 0.15, 0.85)
            )
            pose = Pose()
            pose.position = Point(x=x, y=y, z=z)
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
        marker_array.markers.append(joints)

        for label_id, name in enumerate(LABELS, start=100):
            if name not in points:
                continue
            x, y, z = points[name]
            text = Marker()
            text.header.frame_id = self.args.frame_id
            text.header.stamp = stamp
            text.ns = "body38_labels"
            text.id = label_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=x, y=y, z=z + 0.055)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.06
            text.color = rgba(0.90, 0.96, 1.0, 0.95)
            text.text = name
            text.lifetime.nanosec = 300_000_000
            marker_array.markers.append(text)

        self.marker_publisher.publish(marker_array)
        self.pose_publisher.publish(pose_array)
        local_q = packet.get("local_orientation_per_joint_xyzw", [])
        orientation_array = PoseArray()
        orientation_array.header = pose_array.header
        for index, name in enumerate(names):
            if name not in points or index >= len(local_q) or not finite_quaternion(local_q[index]):
                continue
            pose = Pose()
            pose.position = Point(x=points[name][0], y=points[name][1], z=points[name][2])
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, local_q[index])
            orientation_array.poses.append(pose)
        self.orientation_publisher.publish(orientation_array)
        root = PoseStamped()
        root.header = pose_array.header
        root_position = packet.get("root_position_m", points.get("PELVIS", (0.0, 0.0, 0.0)))
        if valid_point(root_position):
            root.pose.position = Point(x=float(root_position[0]), y=float(root_position[1]), z=float(root_position[2]))
        root_q = packet.get("global_root_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
        if finite_quaternion(root_q):
            root.pose.orientation.x, root.pose.orientation.y, root.pose.orientation.z, root.pose.orientation.w = map(float, root_q)
        else:
            root.pose.orientation.w = 1.0
        self.root_publisher.publish(root)
        self.confidence_publisher.publish(Float32MultiArray(data=[float(confidence.get(name, 0.0)) for name in names]))
        self.publish_diagnostics(
            level=DiagnosticStatus.OK,
            message="LIVE",
            body_id=str(packet.get("body_id", "-")),
            confidence=float(packet.get("body_confidence", 0.0)),
            valid_points=len(points),
        )

    def publish_delete_all(self) -> None:
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_publisher.publish(MarkerArray(markers=[marker]))

    def publish_status_event(self, packet: dict) -> None:
        status = str(packet.get("status", "STATUS"))
        self.publish_diagnostics(
            level=(
                DiagnosticStatus.WARN
                if status in ("NO_BODY", "LOW_QUALITY")
                else DiagnosticStatus.ERROR
            ),
            message=status,
            body_id="-",
            confidence=0.0,
            valid_points=0,
        )

    def publish_diagnostics(
        self,
        *,
        level: int,
        message: str,
        body_id: str,
        confidence: float,
        valid_points: int,
    ) -> None:
        status = DiagnosticStatus()
        status.level = level
        status.name = "zed_body38_udp_bridge"
        status.hardware_id = "ZED2i"
        status.message = message
        status.values = [
            KeyValue(key="body_id", value=body_id),
            KeyValue(key="body_confidence", value=f"{confidence:.1f}"),
            KeyValue(key="valid_keypoints", value=str(valid_points)),
            KeyValue(key="udp_hz", value=str(len(self.arrivals))),
            KeyValue(key="physical_robot_output", value="false"),
        ]
        message_array = DiagnosticArray()
        message_array.header.stamp = self.get_clock().now().to_msg()
        message_array.status = [status]
        self.diagnostic_publisher.publish(message_array)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Body38RosBridge(args)
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
