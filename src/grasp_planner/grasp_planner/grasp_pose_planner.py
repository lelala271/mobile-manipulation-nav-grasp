import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class GraspPosePlanner(Node):
    """Converts a target point into a simple pre-grasp and grasp pose."""

    def __init__(self):
        super().__init__("grasp_pose_planner")
        self.declare_parameter("arm_base_frame", "arm_base_link")
        self.declare_parameter("pregrasp_offset_x", -0.08)
        self.declare_parameter("grasp_offset_z", 0.0)
        self.declare_parameter("workspace_x", [0.10, 0.65])
        self.declare_parameter("workspace_y", [-0.35, 0.35])
        self.declare_parameter("workspace_z", [0.02, 0.55])

        self.create_subscription(PointStamped, "target_point_arm_base", self.on_target, 10)
        self.pregrasp_pub = self.create_publisher(PoseStamped, "pregrasp_pose", 10)
        self.grasp_pub = self.create_publisher(PoseStamped, "grasp_pose", 10)
        self.status_pub = self.create_publisher(String, "grasp_plan_status", 10)

    def on_target(self, msg: PointStamped):
        if not self.in_workspace(msg.point.x, msg.point.y, msg.point.z):
            self.publish_status("target_out_of_workspace")
            return

        grasp = self.make_pose(msg.point.x, msg.point.y, msg.point.z + float(self.get_parameter("grasp_offset_z").value))
        pre = self.make_pose(
            msg.point.x + float(self.get_parameter("pregrasp_offset_x").value),
            msg.point.y,
            msg.point.z + float(self.get_parameter("grasp_offset_z").value),
        )
        self.pregrasp_pub.publish(pre)
        self.grasp_pub.publish(grasp)
        self.publish_status("grasp_plan_ready")

    def make_pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter("arm_base_frame").value
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = math.sqrt(0.5)
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = math.sqrt(0.5)
        return pose

    def in_workspace(self, x: float, y: float, z: float) -> bool:
        x_min, x_max = [float(v) for v in self.get_parameter("workspace_x").value]
        y_min, y_max = [float(v) for v in self.get_parameter("workspace_y").value]
        z_min, z_max = [float(v) for v in self.get_parameter("workspace_z").value]
        return x_min <= x <= x_max and y_min <= y <= y_max and z_min <= z <= z_max

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = GraspPosePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
