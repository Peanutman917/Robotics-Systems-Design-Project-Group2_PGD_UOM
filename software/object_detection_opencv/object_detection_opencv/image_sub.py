#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import cv2
import numpy as np
import tensorflow as tf

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# 面积过滤阈值（根据你块的大小大概调一下）
MIN_AREA = 2000      # 太小的点状/细条都不要
MAX_AREA = 20000     # 特别大的一整片背景也不要


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_sub')

        self.bridge = CvBridge()
        self.current_bgr = None
        self.current_result = None

        # ============ 1. 加载 CNN 模型 ============
        model_path = '/home/student17/triangle_bar_cnn/triangle_bar_cnn.keras'
        if not os.path.exists(model_path):
            self.get_logger().error(f'CNN 模型文件不存在: {model_path}')
        else:
            self.get_logger().info(f'加载 CNN 模型: {model_path}')

        self.cnn_model = tf.keras.models.load_model(model_path)
        self.img_size = 64  # 训练时的输入尺寸
        # =========================================

        # ============ 2. 订阅彩色图像 ============
        # 注意：这里已经改成你现在实际的 topic
        image_topic = '/camera/camera/color/image_raw'
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'订阅图像话题: {image_topic}')

        # ============ 3. OpenCV 窗口 + 定时器 ============
        self.window_name = 'object_detect_cnn'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # 每 0.03s 刷新窗口 & 处理事件
        self.timer = self.create_timer(0.03, self.gui_timer_callback)

    # ----------------- 颜色分割：红 + 绿 + 黄 -----------------
    def get_color_mask(self, hsv):
        # 更严格一点：提高 S、V 的下界，避免低饱和的桌面/皮肤被吃进来


        # 红色：对暗面宽容一点，S / V 下界调低
        lower_red1 = np.array([0,   80, 40])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 80, 40])
        upper_red2 = np.array([179, 255, 255])


        # ---- 绿色：Hue 收到 45~85，S/V 也拉高一点 ----
        # 如果绿块还是漏检，可以把 S/V 再往下调一点点
        lower_green = np.array([45, 90, 70])
        upper_green = np.array([85, 255, 255])

        # ---- 黄色：尽量别吃到桌面/皮肤 ----
        lower_yellow = np.array([18, 150, 140])
        upper_yellow = np.array([35, 255, 255])
        


        


        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 合并三种颜色
        mask = cv2.bitwise_or(mask_red, mask_green)
        mask = cv2.bitwise_or(mask, mask_yellow)

        # 形态学去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        return mask

    # ----------------- 用 CNN 做 triangle/bar 分类 -----------------
    def classify_shape(self, contour):
        if self.current_bgr is None:
            return 'unknown', 0.0

        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return 'unknown', 0.0

        pad = 10
        h_img, w_img, _ = self.current_bgr.shape
        x1 = max(x - pad, 0)
        y1 = max(y - pad, 0)
        x2 = min(x + w + pad, w_img)
        y2 = min(y + h + pad, h_img)

        roi_bgr = self.current_bgr[y1:y2, x1:x2]
        if roi_bgr.size == 0:
            return 'unknown', 0.0

        roi_rgb = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2RGB)
        roi_rgb = cv2.resize(roi_rgb, (self.img_size, self.img_size))
        roi_norm = roi_rgb.astype(np.float32) / 255.0

        roi_input = np.expand_dims(roi_norm, axis=0)

        prob = float(self.cnn_model.predict(roi_input, verbose=0)[0][0])
        # 训练时: 1 = triangle, 0 = bar
        if prob > 0.5:
            label = 'triangle'
            conf = prob
        else:
            label = 'bar'
            conf = 1.0 - prob

        return label, conf

    # ----------------- ROS 图像回调 -----------------
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge 转换失败: {e}')
            return

        self.current_bgr = cv_image.copy()

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = self.get_color_mask(hsv)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        result = cv_image.copy()

        for contour in contours:
            area = cv2.contourArea(contour)

            # 面积太小/太大的全部忽略
            if area < MIN_AREA or area > MAX_AREA:
                continue

            label, conf = self.classify_shape(contour)

            # 置信度不够高，当作 unknown（这样背景即使被卷进来，也不会当成有效识别）
            if conf < 0.7:
                label = 'unknown'

            x, y, w, h = cv2.boundingRect(contour)

            if label == 'triangle':
                color = (0, 255, 0)      # 绿色框
            elif label == 'bar':
                color = (0, 0, 255)      # 红色框
            else:
                color = (0, 255, 255)    # 黄色框 unknown

            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            text = f'{label}'
            if label != 'unknown':
                text += f' ({conf:.2f})'

            cv2.putText(
                result,
                text,
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        self.current_result = result

    # ----------------- 定时器：刷新窗口 -----------------
    def gui_timer_callback(self):
        if self.current_result is not None:
            cv2.imshow(self.window_name, self.current_result)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

