#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class HSVInspector(Node):
    """
    简单的 HSV 调试节点：
    - 订阅相机彩色图像
    - 显示原始画面
    - 在窗口里左键点击一个点，在终端打印该点的 HSV 值
    """

    def __init__(self):
        super().__init__('hsv_inspector')

        self.bridge = CvBridge()
        self.current_bgr = None
        self.current_hsv = None

        # ★ 把下面这个话题改成你自己的彩色图像话题 ★
        # 你现在用的是 /camera/camera/color/image_raw
        image_topic = '/camera/camera/color/image_raw'

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'Subscribed to image topic: {image_topic}')

        # OpenCV 窗口
        self.window_name = 'hsv_debug'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        # 定时器：刷新窗口
        self.timer = self.create_timer(0.03, self.gui_timer_callback)

    # ROS 图像回调
    def image_callback(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge conversion failed: {e}')
            return

        self.current_bgr = bgr
        self.current_hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # 定时器：刷新窗口 & 处理事件
    def gui_timer_callback(self):
        if self.current_bgr is not None:
            cv2.imshow(self.window_name, self.current_bgr)
        cv2.waitKey(1)

    # 鼠标点击回调：打印 HSV
    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.current_hsv is not None:
            h, s, v = self.current_hsv[y, x]
            self.get_logger().info(f'Click at ({x}, {y}): H={int(h)}, S={int(s)}, V={int(v)}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HSVInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

