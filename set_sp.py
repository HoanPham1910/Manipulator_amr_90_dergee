#!/usr/bin/env python3
"""
Tool đọc vị trí real-time của robot:
  - /joint_states  → góc j1..j4
  - TF world→end_effector_link → x y z  qx qy qz qw
Chạy:
    python3 arm_monitor.py
Nhấn  [p]  để lưu điểm hiện tại vào points.json
Nhấn  [q]  để thoát.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
import threading
import sys
import tty
import termios
import json
import os

JOINT_NAMES  = ['joint1', 'joint2', 'joint3', 'joint4']
POINTS_FILE  = os.path.join(os.path.dirname(__file__), 'points_joint.json')

# ─── đọc phím không blocking ───
def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


class ArmMonitor(Node):
    def __init__(self):
        super().__init__('arm_monitor')
        self.joints = {}
        self.pose   = None          # (x, y, z, qx, qy, qz, qw)

        self.sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10
        )
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"Monitor READY  |  [p] lưu điểm → {POINTS_FILE}  |  [q] thoát"
        )

        # thread đọc bàn phím
        t = threading.Thread(target=self._key_loop, daemon=True)
        t.start()

    # ── joint states ──
    def _joint_cb(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    # ── cập nhật TF mỗi 100 ms ──
    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'world', 'end_effector_link', rclpy.time.Time()
            )
            p = tf.transform.translation
            q = tf.transform.rotation
            self.pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            self._print_live()
        except Exception as e:
            self.get_logger().warn(f"TF chưa sẵn sàng: {e}", throttle_duration_sec=3.0)

    # ── in live lên terminal ──
    def _print_live(self):
        if self.pose is None:
            return
        x, y, z, qx, qy, qz, qw = self.pose
        j_str = "  ".join(
            f"j{i+1}={self.joints.get(JOINT_NAMES[i], 0.0):+.4f}"
            for i in range(4)
        )
        line = (
            f"\r  EEF → x={x:+.4f}  y={y:+.4f}  z={z:+.4f}"
            f"  |  q=({qx:+.3f},{qy:+.3f},{qz:+.3f},{qw:+.3f})"
            f"  ||  {j_str}   "
        )
        sys.stdout.write(line)
        sys.stdout.flush()

    # ── vòng đọc phím ──
    def _key_loop(self):
        while True:
            k = read_key()
            if k == 'p':
                self._save_point()
            elif k == 'q':
                print("\nThoát.")
                rclpy.shutdown()
                break

    # ── lưu điểm vào JSON ──
    def _save_point(self):
        if self.pose is None:
            print("\n  [!] Chưa có TF")
            return

        x, y, z, qx, qy, qz, qw = self.pose
        new_pt = {
            "x":  float(x),
            "y":  float(y),
            "z":  float(z),
            "qx": float(qx),
            "qy": float(qy),
            "qz": float(qz),
            "qw": float(qw),
            # THÊM: lưu luôn góc khớp
            "joints": {
                name: float(self.joints.get(name, 0.0))
                for name in JOINT_NAMES
            }
        }

        # đọc danh sách cũ (nếu có)
        points = []
        if os.path.exists(POINTS_FILE):
            try:
                with open(POINTS_FILE, 'r') as f:
                    points = json.load(f)
            except Exception:
                points = []

        points.append(new_pt)

        with open(POINTS_FILE, 'w') as f:
            json.dump(points, f, indent=2)

        idx = len(points)
        print(f"\n  ✓ Đã lưu điểm #{idx}: "
              f"pos=({x:.4f}, {y:.4f}, {z:.4f})  →  {POINTS_FILE}")


def main():
    rclpy.init()
    node = ArmMonitor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()