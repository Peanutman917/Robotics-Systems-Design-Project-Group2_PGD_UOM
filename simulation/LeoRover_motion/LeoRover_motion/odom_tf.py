import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class FakeOdomTF(Node):
    def __init__(self):
        super().__init__('fake_odom_tf')
        self.br = TransformBroadcaster(self)
        self.t = 0.0
        self.timer = self.create_timer(0.05, self.on_timer)  # 20 Hz

    def on_timer(self):
        self.t += 0.05
        x = 0.5 * math.cos(self.t * 0.2)   # 让车沿圆走
        y = 0.5 * math.sin(self.t * 0.2)
        yaw = self.t * 0.2

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.br.sendTransform(tf)

def main():
    rclpy.init()
    node = FakeOdomTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
