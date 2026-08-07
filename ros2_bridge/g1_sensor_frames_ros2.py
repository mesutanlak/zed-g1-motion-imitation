"""Publish the official G1 sensor child frames and mount visualization."""

from __future__ import annotations

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy


class SensorFrames(Node):
    def __init__(self):
        super().__init__("g1_sensor_frames")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MarkerArray, "/g1/sensors/mount_markers", qos)
        # Register the canonical interfaces even before a simulator/physical driver
        # supplies samples. Drivers may publish the same message types concurrently.
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lidar_interface = self.create_publisher(PointCloud2, "/livox/lidar", sensor_qos)
        self.depth_points_interface = self.create_publisher(PointCloud2, "/camera/camera/depth/color/points", sensor_qos)
        self.depth_image_interface = self.create_publisher(Image, "/camera/camera/depth/image_rect_raw", sensor_qos)
        self.depth_info_interface = self.create_publisher(CameraInfo, "/camera/camera/depth/camera_info", sensor_qos)
        self.diag = self.create_publisher(DiagnosticArray, "/g1/sensors/diagnostics", qos)
        self.static = StaticTransformBroadcaster(self)
        self.send_frames()
        self.create_timer(1.0, self.publish_markers)
        self.get_logger().info("Official G1 d435_link/mid360_link child frames active")

    def transform(self, parent, child, xyz=(0, 0, 0), rpy=(0, 0, 0)):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
        q = quat(*rpy)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def send_frames(self):
        # d435_link and mid360_link themselves are supplied by the official Unitree URDF.
        self.static.sendTransform([
            self.transform("mid360_link", "livox_frame"),
            self.transform("d435_link", "camera_link"),
            self.transform("camera_link", "camera_depth_frame"),
            self.transform("camera_depth_frame", "camera_depth_optical_frame", rpy=(-math.pi/2, 0.0, -math.pi/2)),
            # ZED 2i is the external observer; override this extrinsic after calibration.
            self.transform("odom", "zed_camera"),
        ])

    def publish_markers(self):
        now = self.get_clock().now().to_msg()
        out = MarkerArray()
        specs = [
            ("mid360_link", 0, Marker.CYLINDER, (0.085, 0.085, 0.065), (0.18, 0.55, 1.0)),
            ("d435_link", 1, Marker.CUBE, (0.09, 0.025, 0.025), (0.20, 0.95, 0.45)),
        ]
        for frame, marker_id, marker_type, scale, rgb in specs:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = frame, now
            marker.ns, marker.id, marker.type, marker.action = "g1_sensor_mounts", marker_id, marker_type, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = scale
            marker.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=0.85)
            out.markers.append(marker)
        self.pub.publish(out)
        status = DiagnosticStatus(
            level=DiagnosticStatus.OK,
            name="g1_sensor_frames",
            hardware_id="G1-official-mounts",
            message="MOUNT_TF_READY_WAITING_FOR_SENSOR_SOURCE",
            values=[
                KeyValue(key="mid360_mount_source", value="Unitree official g1_23dof.urdf"),
                KeyValue(key="d435_mount_source", value="Unitree official g1_23dof.urdf"),
            ],
        )
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = now
        diagnostic.status = [status]
        self.diag.publish(diagnostic)


def main():
    rclpy.init()
    node = SensorFrames()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
