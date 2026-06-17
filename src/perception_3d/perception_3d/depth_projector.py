import json
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class DepthProjector(Node):
    """Projects 2D detections into 3D camera-frame points using aligned depth."""

    def __init__(self):
        super().__init__("depth_projector")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("default_depth_m", 0.6)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("min_confidence", 0.5)

        self.camera_info: Optional[CameraInfo] = None
        self.depth_image: Optional[Image] = None

        self.create_subscription(CameraInfo, "camera_info", self.on_camera_info, 10)
        self.create_subscription(Image, "depth_image", self.on_depth_image, 10)
        self.create_subscription(String, "detections", self.on_detection, 10)
        self.point_pub = self.create_publisher(PointStamped, "target_point_camera", 10)
        self.debug_pub = self.create_publisher(String, "target_debug", 10)

    def on_camera_info(self, msg: CameraInfo):
        self.camera_info = msg

    def on_depth_image(self, msg: Image):
        self.depth_image = msg

    def on_detection(self, msg: String):
        detection = json.loads(msg.data)
        if float(detection.get("confidence", 0.0)) < float(self.get_parameter("min_confidence").value):
            return

        bbox = detection.get("bbox")
        if not bbox or len(bbox) != 4:
            self.get_logger().warning("Detection does not contain bbox=[left, top, right, bottom].")
            return

        u = 0.5 * (float(bbox[0]) + float(bbox[2]))
        v = 0.5 * (float(bbox[1]) + float(bbox[3]))
        depth_m = self.estimate_depth(u, v)
        point = self.project_pixel(u, v, depth_m)
        if point is None:
            return

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter("camera_frame").value
        out.point.x, out.point.y, out.point.z = point
        self.point_pub.publish(out)

        debug = String()
        debug.data = json.dumps(
            {
                "class_name": detection.get("class_name", "target"),
                "pixel": [u, v],
                "depth_m": depth_m,
                "point_camera": [out.point.x, out.point.y, out.point.z],
            },
            ensure_ascii=False,
        )
        self.debug_pub.publish(debug)

    def estimate_depth(self, u: float, v: float) -> float:
        if self.depth_image is None:
            return float(self.get_parameter("default_depth_m").value)

        if self.depth_image.encoding not in ("16UC1", "32FC1"):
            return float(self.get_parameter("default_depth_m").value)

        width = self.depth_image.width
        height = self.depth_image.height
        x = min(max(int(round(u)), 0), width - 1)
        y = min(max(int(round(v)), 0), height - 1)

        if self.depth_image.encoding == "16UC1":
            index = y * self.depth_image.step + x * 2
            raw = int.from_bytes(self.depth_image.data[index : index + 2], byteorder="little", signed=False)
            depth = raw * float(self.get_parameter("depth_scale").value)
        else:
            import struct

            index = y * self.depth_image.step + x * 4
            depth = struct.unpack_from("<f", bytes(self.depth_image.data), index)[0]

        if not math.isfinite(depth) or depth <= 0.0:
            return float(self.get_parameter("default_depth_m").value)
        return float(depth)

    def project_pixel(self, u: float, v: float, depth_m: float):
        if self.camera_info is None:
            fx = fy = 600.0
            cx = 320.0
            cy = 240.0
        else:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

        if fx == 0.0 or fy == 0.0:
            self.get_logger().error("Invalid camera intrinsics: fx/fy is zero.")
            return None

        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return x, y, z


def main():
    rclpy.init()
    node = DepthProjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
