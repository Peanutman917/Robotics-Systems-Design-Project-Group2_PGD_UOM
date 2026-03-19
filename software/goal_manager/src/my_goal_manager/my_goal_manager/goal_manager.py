#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray
from rclpy.qos import qos_profile_sensor_data

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        # Subscribe to detected objects
        self.object_sub = self.create_subscription(
            PoseStamped,       # Object position as PoseStamped
            '/detected_object',
            self.object_callback,
            qos_profile_sensor_data
        )

        # Monitor navigation goal status
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/move_base/status',
            self.status_callback,
            qos_profile_sensor_data
        )

        # Publish goals to move_base
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)

        # Publish to arm controller
        self.arm_pub = self.create_publisher(Bool, '/arm_pick_trigger', 10)

        # State management
        self.state = 'EXPLORING'   # EXPLORING or OBJECT_NAVIGATION
        self.current_goal = None
        self.object_goal = None

    def object_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Object detected at x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        if self.state == 'EXPLORING':
            self.object_goal = msg
            self.send_object_goal()

    def send_object_goal(self):
        if self.object_goal:
            self.state = 'OBJECT_NAVIGATION'
            self.current_goal = self.object_goal
            self.goal_pub.publish(self.current_goal)
            self.get_logger().info("Navigating to detected object...")
            self.object_goal = None

    def status_callback(self, msg: GoalStatusArray):
        if self.state == 'OBJECT_NAVIGATION' and msg.status_list:
            # Check if current goal succeeded
            status = msg.status_list[-1].status
            if status == 3:  # SUCCEEDED
                self.get_logger().info("Reached object, triggering arm...")
                # Trigger arm node
                self.arm_pub.publish(Bool(data=True))
                # Resume exploration
                self.state = 'EXPLORING'
                self.current_goal = None

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()