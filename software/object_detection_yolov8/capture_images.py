#!/usr/bin/env python3
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def main():
    # ========= 1. 设置保存路径 =========
    # 图片会保存到：~/detectdata/dataset/images/all
    save_dir = os.path.expanduser("~/detectdata/dataset/new")
    os.makedirs(save_dir, exist_ok=True)
    print(f"[INFO] Images will be saved to: {save_dir}")

    # ========= 2. 配置 RealSense =========
    pipeline = rs.pipeline()
    config = rs.config()

    # 只开彩色流就够了，分辨率 640x480，30 FPS
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[INFO] Starting RealSense pipeline...")
    pipeline.start(config)

    print("[INFO] Press 's' to save image, 'q' to quit.")

    saved_count = 0

    try:
        while True:
            # 读取一帧
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # 转成 numpy BGR 图像
            color_image = np.asanyarray(color_frame.get_data())

            # 显示画面
            cv2.imshow("RealSense Color", color_image)
            key = cv2.waitKey(1) & 0xFF

            # 按 's' 保存图片
            if key == ord('s'):
                timestamp_ms = int(time.time() * 1000)  # 毫秒级时间戳
                filename = f"img_{timestamp_ms}.jpg"
                save_path = os.path.join(save_dir, filename)

                cv2.imwrite(save_path, color_image)
                saved_count += 1
                print(f"[INFO] Saved #{saved_count}: {save_path}")

            # 按 'q' 或 ESC 退出
            if key == ord('q') or key == 27:
                break

    finally:
        # 收尾工作
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] Pipeline stopped, window closed.")


if __name__ == "__main__":
    main()

