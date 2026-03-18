import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

# ======= 1. 配置你的模型路径 =======
# 把这个路径改成你自己 best.pt 的实际位置
MODEL_PATH = "/home/student17/Desktop/modeltest/best.pt"


def main():
    # ---------- 1) 加载 YOLOv8 模型 ----------
    print("[INFO] Loading YOLOv8 model...")
    model = YOLO(MODEL_PATH)
    print("[INFO] Model loaded. Classes:", model.names)

    # ---------- 2) 配置 RealSense 流 ----------
    pipeline = rs.pipeline()
    config = rs.config()

    # 同时开 depth + color，分辨率要一致，方便对齐
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    print("[INFO] Starting RealSense pipeline...")
    profile = pipeline.start(config)

    # 深度对齐到彩色图像
    align_to = rs.stream.color
    align = rs.align(align_to)

    # 深度尺度（一个单位对应多少米）
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[INFO] Depth scale: {depth_scale} meters per unit")

    print("[INFO] Press 'q' to quit.")

    try:
        while True:
            # ---------- 3) 读取并对齐帧 ----------
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 转成 numpy
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # ---------- 4) YOLOv8 推理 ----------
            # 直接把 numpy BGR 图像给 YOLO
            results = model.predict(
                source=color_image,
                conf=0.5,        # 置信度阈值
                verbose=False
            )

            # 拷贝一份用于画图
            annotated = color_image.copy()

            # ---------- 5) 处理检测结果 ----------
            for r in results:
                boxes = r.boxes
                if boxes is None:
                    continue

                for box in boxes:
                    # xyxy: [x1, y1, x2, y2]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())

                    # 框中心点（用于取深度）
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # ---------- 6) 计算距离 ----------
                    # 为了减少噪声，不只取一个点，取中心附近一个小窗口平均
                    win = 5  # 5x5 小窗口
                    x_start = max(cx - win // 2, 0)
                    x_end = min(cx + win // 2, depth_image.shape[1] - 1)
                    y_start = max(cy - win // 2, 0)
                    y_end = min(cy + win // 2, depth_image.shape[0] - 1)

                    depth_patch = depth_image[y_start:y_end + 1, x_start:x_end + 1].astype(float)

                    # 去掉 0（无效深度）
                    valid = depth_patch[depth_patch > 0]
                    if valid.size > 0:
                        depth_m = valid.mean() * depth_scale  # 转成米
                        dist_text = f"{depth_m:.2f} m"
                    else:
                        depth_m = None
                        dist_text = "no depth"

                    # ---------- 7) 画框 + 文本 ----------
                    # 画矩形框
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # 类别名（颜色 + 形状）
                    class_name = model.names.get(cls_id, str(cls_id))

                    # 标签内容：类别 + 置信度 + 距离
                    label = f"{class_name} {conf:.2f} {dist_text}"

                    # 文字位置
                    text_x, text_y = x1, max(y1 - 10, 0)
                    cv2.putText(
                        annotated,
                        label,
                        (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA
                    )

                    # 你要是想在终端打印，也可以：
                    print(f"Detected {class_name} @ ({cx},{cy}), dist = {dist_text}")

            # ---------- 8) 显示结果 ----------
            cv2.imshow("RealSense + YOLOv8 (color+shape+distance)", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' 或 ESC 退出
                break

    finally:
        # ---------- 9) 善后 ----------
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[INFO] Stopped RealSense pipeline and closed window.")


if __name__ == "__main__":
    main()

