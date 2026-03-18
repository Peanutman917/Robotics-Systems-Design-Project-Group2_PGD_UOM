import traceback
import socket
import json
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


CONFIG = {
    # ===== robot arm_server =====
    "robot_ip": "10.3.14.59",
    "robot_port": 5000,

    # ===== YOLO 模型参数 =====
    "model_path": "/home/group2/best.pt",
    "conf_thres": 0.55,
    "imgsz": 640,

    # ===== 相机参数 =====
    "camera_width": 640,
    "camera_height": 480,
    "camera_fps": 30,
    "depth_patch": 5,

    # ===== 类别名映射 =====
    # 左边是模型输出类别名，右边是程序内部统一名称
    "CLASS_MAP": {
        "blue cube": "blue cube",
        "green cube": "green cube",
        "purple cube": "purple cube",
        "yellow rectangle": "yellow rectangle",
    },

    # ===== 任务类别定义 =====
    "target_block_classes": ["blue cube", "green cube"],
    "target_bin_classes": ["yellow rectangle", "purple cube"],

    # 这里集中定义 block -> bin 的对应关系。
    # 如果你们改成同色放同色，只要改这里。
    # 注意：右边必须是你的 YOLO 模型实际会输出的 bin 类别名。
    "bin_class_for_block": {
        "blue cube": "yellow rectangle",
        "green cube": "purple cube",
        # 例如如果模型真的输出 blue bin / green bin，可改成：
        # "blue cube": "blue bin",
        # "green cube": "green bin",
    },

    # ===== 相机外参近似（单位 mm） =====
    "camera_height_mm": 400.0,
    "camera_offset_x_mm": 0.0,
    "camera_offset_y_mm": 0.0,

    # ===== 相机与机械臂平面旋转/轴向补偿 =====
    "camera_yaw_deg": 0.0,
    "camera_sign_x": 1.0,
    "camera_sign_y": 1.0,

    # ===== 调试 =====
    "show_window": True,
}


@dataclass
class Detection3D:
    cls_name: str
    conf: float
    bbox: Tuple[int, int, int, int]
    center_uv: Tuple[int, int]
    point_cam_mm: np.ndarray
    point_base_mm: np.ndarray


class RealSenseCamera:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.pipeline = rs.pipeline()
        self.align = rs.align(rs.stream.color)

        config = rs.config()
        config.enable_stream(rs.stream.color, cfg["camera_width"], cfg["camera_height"], rs.format.bgr8, cfg["camera_fps"])
        config.enable_stream(rs.stream.depth, cfg["camera_width"], cfg["camera_height"], rs.format.z16, cfg["camera_fps"])

        self.profile = self.pipeline.start(config)
        for _ in range(10):
            self.pipeline.wait_for_frames()

    def get_aligned_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return None, None, None, None

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        intr = color_frame.profile.as_video_stream_profile().intrinsics
        return color_image, depth_image, depth_frame, intr

    def get_robust_depth_m(self, depth_frame, u: int, v: int) -> Optional[float]:
        patch = self.cfg["depth_patch"]
        half = patch // 2
        values = []

        for yy in range(v - half, v + half + 1):
            for xx in range(u - half, u + half + 1):
                try:
                    d = depth_frame.get_distance(xx, yy)
                    if d > 0.05:
                        values.append(d)
                except Exception:
                    pass

        if not values:
            return None
        return float(np.median(values))

    def pixel_to_cam_xyz_mm(self, intr, u: int, v: int, depth_m: float) -> np.ndarray:
        point = rs.rs2_deproject_pixel_to_point(intr, [float(u), float(v)], float(depth_m))
        return np.array(point, dtype=np.float64) * 1000.0

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


class YoloDetector:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.model = YOLO(cfg["model_path"])
        self.class_map = cfg["CLASS_MAP"]

    def infer(self, image: np.ndarray):
        results = self.model.predict(
            source=image,
            conf=self.cfg["conf_thres"],
            imgsz=self.cfg["imgsz"],
            verbose=False,
        )
        return results[0]

    def parse_detections(self, result) -> List[Dict]:
        out = []
        if result.boxes is None:
            return out

        names = result.names
        for box in result.boxes:
            cls_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int).tolist()
            raw_name = str(names[cls_id])
            std_name = self.class_map.get(raw_name, raw_name)
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)
            out.append({
                "raw_name": raw_name,
                "cls_name": std_name,
                "conf": conf,
                "bbox": (x1, y1, x2, y2),
                "center_uv": (u, v),
            })
        return out


# =========================
# 坐标与工具
# =========================

def cam_to_base(cfg: dict, p_cam_mm: np.ndarray) -> np.ndarray:
    x_cam, y_cam, z_cam = p_cam_mm

    x_aligned = float(cfg.get("camera_sign_x", 1.0)) * float(x_cam)
    y_aligned = float(cfg.get("camera_sign_y", 1.0)) * float(y_cam)

    yaw_deg = float(cfg.get("camera_yaw_deg", 0.0))
    yaw_rad = np.deg2rad(yaw_deg)
    cos_yaw = np.cos(yaw_rad)
    sin_yaw = np.sin(yaw_rad)

    x_rot = cos_yaw * x_aligned - sin_yaw * y_aligned
    y_rot = sin_yaw * x_aligned + cos_yaw * y_aligned

    x_base = x_rot + float(cfg["camera_offset_x_mm"])
    y_base = y_rot + float(cfg["camera_offset_y_mm"])
    z_base = float(cfg["camera_height_mm"]) - float(z_cam)
    return np.array([x_base, y_base, z_base], dtype=np.float64)


def choose_best_detection(dets: List[Detection3D], allowed_classes: List[str]) -> Optional[Detection3D]:
    candidates = [d for d in dets if d.cls_name in allowed_classes]
    if not candidates:
        return None
    candidates.sort(key=lambda x: x.conf, reverse=True)
    return candidates[0]


def expected_bin_class_for_block(cfg: dict, block_cls: str) -> Optional[str]:
    return cfg.get("bin_class_for_block", {}).get(block_cls)


def recv_json_line(sock: socket.socket, max_bytes: int = 65536) -> Dict:
    chunks = []
    total = 0

    while True:
        part = sock.recv(4096)
        if not part:
            break
        chunks.append(part)
        total += len(part)
        if total > max_bytes:
            raise RuntimeError("Robot response too large")
        if b"\n" in part:
            break

    if not chunks:
        raise RuntimeError("No response from robot server")

    raw = b"".join(chunks).split(b"\n", 1)[0].strip()
    if not raw:
        raise RuntimeError("Empty response from robot server")
    return json.loads(raw.decode("utf-8"))


def send_robot_command(robot_ip: str, robot_port: int, msg: Dict) -> Dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(60)

    try:
        sock.connect((robot_ip, robot_port))
        sock.sendall((json.dumps(msg) + "\n").encode("utf-8"))
        sock.shutdown(socket.SHUT_WR)
        resp_obj = recv_json_line(sock)
        if not resp_obj.get("ok", False):
            raise RuntimeError(f"Robot failed: {resp_obj}")
        return resp_obj
    finally:
        sock.close()


def send_pick_request(robot_ip: str, robot_port: int, block: Detection3D) -> Dict:
    msg = {
        "cmd": "pick_only",
        "object_label": block.cls_name,
        "block": {
            "x": float(block.point_base_mm[0]),
            "y": float(block.point_base_mm[1]),
            "z": float(block.point_base_mm[2]),
        },
    }
    return send_robot_command(robot_ip, robot_port, msg)


def send_place_request(robot_ip: str, robot_port: int, target_bin: Detection3D) -> Dict:
    msg = {
        "cmd": "place_only",
        "bin": {
            "x": float(target_bin.point_base_mm[0]),
            "y": float(target_bin.point_base_mm[1]),
            "z": float(target_bin.point_base_mm[2]),
        },
    }
    return send_robot_command(robot_ip, robot_port, msg)


# =========================
# 从检测恢复 3D
# =========================

def build_3d_detections(cfg: dict, detector: YoloDetector, rs_cam: RealSenseCamera,
                        color_img: np.ndarray, depth_frame, intr) -> Tuple[List[Detection3D], np.ndarray]:
    result = detector.infer(color_img)
    parsed = detector.parse_detections(result)

    dets_3d = []
    vis = color_img.copy()

    for item in parsed:
        cls_name = item["cls_name"]
        conf = item["conf"]
        x1, y1, x2, y2 = item["bbox"]
        u, v = item["center_uv"]

        depth_m = rs_cam.get_robust_depth_m(depth_frame, u, v)
        if depth_m is None:
            label = f"{cls_name} {conf:.2f} depth=NONE"
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.circle(vis, (u, v), 4, (0, 0, 255), -1)
            cv2.putText(vis, label, (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
            continue

        p_cam_mm = rs_cam.pixel_to_cam_xyz_mm(intr, u, v, depth_m)
        p_base_mm = cam_to_base(cfg, p_cam_mm)

        det = Detection3D(
            cls_name=cls_name,
            conf=conf,
            bbox=(x1, y1, x2, y2),
            center_uv=(u, v),
            point_cam_mm=p_cam_mm,
            point_base_mm=p_base_mm,
        )
        dets_3d.append(det)

        text = f"{cls_name} {conf:.2f} xyz=({p_base_mm[0]:.0f},{p_base_mm[1]:.0f},{p_base_mm[2]:.0f})"
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(vis, (u, v), 4, (255, 0, 0), -1)
        cv2.putText(vis, text, (x1, max(25, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2)

    return dets_3d, vis


# =========================
# 主逻辑
# =========================

def main():
    cfg = CONFIG
    holding_object = False
    held_block_class = None

    print("========== 系统启动（两阶段版） ==========")
    print("正在启动 RealSense...")
    rs_cam = RealSenseCamera(cfg)

    print("正在加载 YOLO 模型...")
    detector = YoloDetector(cfg)

    print("系统启动完成。")
    print("按 q 退出")
    print("按 s：只抓取当前看到的物块（pick_only）")
    print("按 p：只放置到当前看到的目标 bin（place_only）")
    print("按 c：清除本地 held 状态（仅 UI 状态，不会控制手臂）")

    try:
        while True:
            color_img, depth_img, depth_frame, intr = rs_cam.get_aligned_frames()
            if color_img is None:
                print("未获取到相机图像")
                continue

            dets_3d, vis = build_3d_detections(cfg, detector, rs_cam, color_img, depth_frame, intr)

            block = choose_best_detection(dets_3d, cfg["target_block_classes"])

            expected_bin_cls = expected_bin_class_for_block(cfg, held_block_class) if held_block_class else None
            if expected_bin_cls:
                bin_candidates = [d for d in dets_3d if d.cls_name == expected_bin_cls]
                target_bin = choose_best_detection(bin_candidates, [expected_bin_cls])
            else:
                target_bin = None

            status_parts = []
            status_parts.append(f"HOLDING={holding_object}")
            if held_block_class:
                status_parts.append(f"HELD={held_block_class}")
            if block is not None:
                status_parts.append(f"BLOCK={block.cls_name}")
            if expected_bin_cls:
                status_parts.append(f"EXPECT_BIN={expected_bin_cls}")
            if target_bin is not None:
                status_parts.append(f"BIN={target_bin.cls_name}")
            status = " | ".join(status_parts) if status_parts else "WAITING"

            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
            cv2.putText(vis, "s=pick_only  p=place_only  q=quit", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2)

            if cfg["show_window"]:
                cv2.imshow("pick_place_two_stage_debug", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

            if key == ord("c"):
                holding_object = False
                held_block_class = None
                print("本地 held 状态已清除。注意：这不会改变手臂当前真实夹持状态。")

            if key == ord("s"):
                if holding_object:
                    print("本地状态显示：手臂已经在夹东西。请先移动到 bin 区并按 p 放置。")
                    continue

                if block is None:
                    print("没有识别到目标物块，无法执行 pick_only")
                    continue

                print("\n========== 发送 pick_only ==========")
                print(f"物块类别: {block.cls_name}, 置信度: {block.conf:.2f}")
                print(f"物块 camera 坐标(mm): {block.point_cam_mm}")
                print(f"物块 base 坐标(mm): {block.point_base_mm}")

                try:
                    resp = send_pick_request(cfg["robot_ip"], cfg["robot_port"], block)
                    print("Robot response:", resp)
                    holding_object = True
                    held_block_class = block.cls_name
                except Exception as e:
                    print("發送 pick_only 失敗：", e)
                    traceback.print_exc()

            if key == ord("p"):
                if not holding_object:
                    print("本地状态显示：目前沒有夾著物體，無法執行 place_only")
                    continue

                if held_block_class is None:
                    print("缺少 held_block_class，无法决定目标 bin")
                    continue

                if expected_bin_cls is None:
                    print(f"沒有設定 {held_block_class} 對應的 bin 類別，請修改 CONFIG['bin_class_for_block']")
                    continue

                if target_bin is None:
                    print(f"目前沒有識別到對應 bin: {expected_bin_cls}，無法執行 place_only")
                    continue

                print("\n========== 发送 place_only ==========")
                print(f"手上物塊類別: {held_block_class}")
                print(f"目標桶類別: {target_bin.cls_name}, 置信度: {target_bin.conf:.2f}")
                print(f"桶 base 座標(mm): {target_bin.point_base_mm}")

                try:
                    resp = send_place_request(cfg["robot_ip"], cfg["robot_port"], target_bin)
                    print("Robot response:", resp)
                    holding_object = False
                    held_block_class = None
                except Exception as e:
                    print("發送 place_only 失敗：", e)
                    traceback.print_exc()

    finally:
        print("系统退出，释放资源...")
        rs_cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
