from enum import Enum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String


class TaskState(str, Enum):
    IDLE = "idle"
    NAVIGATE_TO_PICK = "navigate_to_pick"
    WAIT_FOR_GRASP_PLAN = "wait_for_grasp_plan"
    EXECUTE_GRASP = "execute_grasp"
    NAVIGATE_TO_PLACE = "navigate_to_place"
    EXECUTE_PLACE = "execute_place"
    DONE = "done"
    ERROR = "error"


class PickPlaceStateMachine(Node):
    """Minimal state machine showing the expected mobile manipulation flow."""

    def __init__(self):
        super().__init__("pick_place_state_machine")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("state_period_s", 1.0)
        self.state = TaskState.IDLE
        self.has_grasp_plan = False
        self.tick_count = 0

        self.create_subscription(PoseStamped, "grasp_pose", self.on_grasp_pose, 10)
        self.create_subscription(String, "grasp_plan_status", self.on_grasp_status, 10)
        self.state_pub = self.create_publisher(String, "task_state", 10)
        self.command_pub = self.create_publisher(String, "task_command", 10)
        period = float(self.get_parameter("state_period_s").value)
        self.timer = self.create_timer(period, self.tick)

    def on_grasp_pose(self, _msg: PoseStamped):
        self.has_grasp_plan = True

    def on_grasp_status(self, msg: String):
        if msg.data == "target_out_of_workspace":
            self.state = TaskState.ERROR

    def tick(self):
        self.tick_count += 1
        if self.state == TaskState.IDLE and bool(self.get_parameter("auto_start").value):
            self.transition(TaskState.NAVIGATE_TO_PICK, "navigate_to_pick_region")
        elif self.state == TaskState.NAVIGATE_TO_PICK:
            self.transition(TaskState.WAIT_FOR_GRASP_PLAN, "start_perception")
        elif self.state == TaskState.WAIT_FOR_GRASP_PLAN and self.has_grasp_plan:
            self.transition(TaskState.EXECUTE_GRASP, "execute_grasp")
        elif self.state == TaskState.EXECUTE_GRASP:
            self.transition(TaskState.NAVIGATE_TO_PLACE, "navigate_to_place_region")
        elif self.state == TaskState.NAVIGATE_TO_PLACE:
            self.transition(TaskState.EXECUTE_PLACE, "execute_place")
        elif self.state == TaskState.EXECUTE_PLACE:
            self.transition(TaskState.DONE, "task_done")

        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

    def transition(self, next_state: TaskState, command: str):
        self.state = next_state
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"state={self.state.value}, command={command}")


def main():
    rclpy.init()
    node = PickPlaceStateMachine()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
