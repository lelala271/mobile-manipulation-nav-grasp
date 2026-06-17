import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockDetector(Node):
    """Publishes a deterministic 2D detection for pipeline integration tests."""

    def __init__(self):
        super().__init__("mock_detector")
        self.declare_parameter("class_name", "target")
        self.declare_parameter("confidence", 0.95)
        self.declare_parameter("bbox", [300, 210, 380, 290])
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.publisher = self.create_publisher(String, "detections", 10)
        self.timer = self.create_timer(0.5, self.publish_detection)

    def publish_detection(self):
        bbox = [int(v) for v in self.get_parameter("bbox").value]
        msg = String()
        msg.data = json.dumps(
            {
                "stamp_sec": self.get_clock().now().seconds_nanoseconds()[0],
                "frame_id": self.get_parameter("frame_id").value,
                "class_name": self.get_parameter("class_name").value,
                "confidence": float(self.get_parameter("confidence").value),
                "bbox": bbox,
            },
            ensure_ascii=False,
        )
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = MockDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
