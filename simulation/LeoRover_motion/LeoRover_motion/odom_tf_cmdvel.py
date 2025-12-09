import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster

class FakeOdomFromCmdVel(Node):
    def __init__(self):
        super().__init__('fake_odom_tf_cmdvel')
        self.br = TransformBroadcaster(self)

        self.declare_parameter("update_rate", 50.0)
        rate = float(self.get_parameter("update_rate").value)
        self.dt = 1.0 / rate

        # 位姿
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 速度（来自 cmd_vel）
        self.v = 0.0
        self.w = 0.0

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info("Fake motion (cmd_vel) started. Listening on /cmd_vel")

    def on_cmd(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def on_timer(self):
        # 简单差速运动学积分
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        qz = math.sin(self.yaw/2)
        qw = math.cos(self.yaw/2)
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.br.sendTransform(tf)

def main():
    rclpy.init()
    node = FakeOdomFromCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
