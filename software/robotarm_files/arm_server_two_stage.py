import socket
import json
import time
from pymycobot import MyCobot

# ===== Robot serial =====
PORT = "/dev/ttyAMA0"
BAUD = 1000000

# ===== motion config =====
FIXED_RPY = [180, 0, -90]
SAFE_HOME = [180, 0, 220]
Z_PRE_GRASP = 60
Z_GRASP = -5
Z_SAFE = 220
SPEED_MOVE = 40
SPEED_GRIP = 60
XY_LIMIT = 260
Z_LIMIT = 300


# ===============================
# workspace protection
# ===============================

def within_workspace(x, y, z):
    if abs(x) > XY_LIMIT:
        return False
    if abs(y) > XY_LIMIT:
        return False
    if z < -50 or z > Z_LIMIT:
        return False
    return True


def check_pose(x, y, z):
    if not within_workspace(x, y, z):
        raise ValueError(f"pose out of workspace: {(x, y, z)}")


# ===============================
# robot wrapper
# ===============================

class RobotArm:
    def __init__(self):
        self.mc = MyCobot(PORT, BAUD)
        time.sleep(1)
        print("Robot connected")

    def open_gripper(self):
        self.mc.set_gripper_state(0, SPEED_GRIP)
        time.sleep(0.6)

    def close_gripper(self):
        self.mc.set_gripper_state(1, SPEED_GRIP)
        time.sleep(0.6)

    def move(self, coords):
        self.mc.sync_send_coords(coords, SPEED_MOVE, 1, timeout=15)
        time.sleep(0.2)

    def safe_home(self):
        rx, ry, rz = FIXED_RPY
        self.move([SAFE_HOME[0], SAFE_HOME[1], SAFE_HOME[2], rx, ry, rz])


# ===============================
# robot motions
# ===============================

def pick_object(robot, x, y, z):
    rx, ry, rz = FIXED_RPY
    z_pre = z + Z_PRE_GRASP
    z_grasp = z + Z_GRASP

    for zz in [Z_SAFE, z_pre, z_grasp]:
        check_pose(x, y, zz)

    robot.open_gripper()
    robot.move([x, y, Z_SAFE, rx, ry, rz])
    robot.move([x, y, z_pre, rx, ry, rz])
    robot.move([x, y, z_grasp, rx, ry, rz])
    robot.close_gripper()
    robot.move([x, y, Z_SAFE, rx, ry, rz])


def place_object(robot, x, y, z):
    rx, ry, rz = FIXED_RPY
    z_pre = z + 60
    z_place = z + 20

    for zz in [Z_SAFE, z_pre, z_place]:
        check_pose(x, y, zz)

    robot.move([x, y, Z_SAFE, rx, ry, rz])
    robot.move([x, y, z_pre, rx, ry, rz])
    robot.move([x, y, z_place, rx, ry, rz])
    robot.open_gripper()
    robot.move([x, y, Z_SAFE, rx, ry, rz])


# ===============================
# socket helpers
# ===============================

def recv_json_message(conn, max_bytes=65536):
    chunks = []
    total = 0

    while True:
        part = conn.recv(4096)
        if not part:
            break

        chunks.append(part)
        total += len(part)

        if total > max_bytes:
            raise ValueError("message too large")

        if b"\n" in part:
            break

    if not chunks:
        raise ValueError("empty request")

    raw = b"".join(chunks).split(b"\n", 1)[0].strip()
    if not raw:
        raise ValueError("empty request")

    return json.loads(raw.decode("utf-8"))


def send_json_message(conn, payload):
    conn.sendall((json.dumps(payload) + "\n").encode("utf-8"))


# ===============================
# command handler
# ===============================

def handle_command(robot, state, msg):
    cmd = msg.get("cmd", "pick_and_place")
    object_label = msg.get("object_label")

    if cmd == "pick_only":
        if state["holding_object"]:
            raise RuntimeError("robot is already holding an object; place it before picking again")

        block = msg["block"]
        bx, by, bz = float(block["x"]), float(block["y"]), float(block["z"])

        print("CMD: pick_only")
        print("Pick:", bx, by, bz)
        robot.safe_home()
        pick_object(robot, bx, by, bz)
        robot.safe_home()

        state["holding_object"] = True
        state["held_label"] = object_label
        return {"ok": True, "cmd": cmd, "holding_object": True, "held_label": state["held_label"]}

    if cmd == "place_only":
        if not state["holding_object"]:
            raise RuntimeError("robot is not holding any object; cannot place")

        target = msg["bin"]
        px, py, pz = float(target["x"]), float(target["y"]), float(target["z"])

        print("CMD: place_only")
        print("Place:", px, py, pz)
        robot.safe_home()
        place_object(robot, px, py, pz)
        robot.safe_home()

        placed_label = state["held_label"]
        state["holding_object"] = False
        state["held_label"] = None
        return {"ok": True, "cmd": cmd, "holding_object": False, "placed_label": placed_label}

    if cmd == "pick_and_place":
        block = msg["block"]
        target = msg["bin"]
        bx, by, bz = float(block["x"]), float(block["y"]), float(block["z"])
        px, py, pz = float(target["x"]), float(target["y"]), float(target["z"])

        print("CMD: pick_and_place")
        print("Pick:", bx, by, bz)
        print("Place:", px, py, pz)
        robot.safe_home()
        pick_object(robot, bx, by, bz)
        place_object(robot, px, py, pz)
        robot.safe_home()

        state["holding_object"] = False
        state["held_label"] = None
        return {"ok": True, "cmd": cmd, "holding_object": False}

    if cmd == "home":
        print("CMD: home")
        robot.safe_home()
        return {
            "ok": True,
            "cmd": cmd,
            "holding_object": state["holding_object"],
            "held_label": state["held_label"],
        }

    raise ValueError(f"unknown cmd: {cmd}")


# ===============================
# main server
# ===============================

def main():
    robot = RobotArm()
    robot.safe_home()

    state = {
        "holding_object": False,
        "held_label": None,
    }

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 5000))
    s.listen(5)

    print("arm_server_two_stage listening on port 5000")

    while True:
        conn, addr = s.accept()

        try:
            conn.settimeout(10)
            msg = recv_json_message(conn)
            conn.settimeout(None)
            resp = handle_command(robot, state, msg)
            send_json_message(conn, resp)

        except Exception as e:
            print("Error:", e)
            send_json_message(conn, {
                "ok": False,
                "err": str(e),
                "holding_object": state["holding_object"],
                "held_label": state["held_label"],
            })

        finally:
            conn.close()


if __name__ == "__main__":
    main()
