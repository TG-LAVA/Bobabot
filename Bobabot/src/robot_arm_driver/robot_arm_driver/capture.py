#!/usr/bin/env python3
"""
ROS 2 node: dh_capture  

Keyboard commands:
    r —— save when detected checkerboard and show corners in 0.5 s
    w —— write all samples in .npz
    q —— exit node
    Esc—— exit node in OPENCV window
"""
import rclpy, cv2, threading, os
import numpy as np
from rclpy.node         import Node
from sensor_msgs.msg    import Image          # ← UncompressedImage
from base_interfaces.msg import JointAngles
from cv_bridge          import CvBridge
from datetime           import datetime
from sensor_msgs.msg    import CompressedImage

# ── Camera and Checkerboard human measured parameters ───────────────────────────────────────────
BOARD_SIZE  = (8, 6)
SQUARE      = 0.025  # m
K = np.array([[475.680676, 0, 319.348050],
              [0, 474.219595, 240.180089],
              [0, 0, 1]])
DIST = np.array([-0.036038, -0.270808, 0.003515,
                 -0.002552,  0.574804])

def detect_pose(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    ok, c = cv2.findChessboardCorners(
        gray, BOARD_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH |
        cv2.CALIB_CB_FAST_CHECK |
        cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok: return None
    obj = np.zeros((BOARD_SIZE[0]*BOARD_SIZE[1],3), np.float32)
    obj[:,:2] = np.mgrid[0:BOARD_SIZE[0],0:BOARD_SIZE[1]].T.reshape(-1,2)
    obj *= SQUARE
    cv2.cornerSubPix(gray, c, (11,11), (-1,-1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    ok, rvec, tvec = cv2.solvePnP(obj, c, K, DIST)
    if not ok: return None
    return c, rvec.flatten(), tvec.flatten()

# ── node ─────────────────────────────────────────────────────
class CaptureNode(Node):
    def __init__(self):
        super().__init__("dh_capture")
        self.bridge = CvBridge()

        self.create_subscription(CompressedImage, "/camera_arm/color/image_raw/compressed",
                                 self.img_cb, 10)
        self.create_subscription(JointAngles, "/joint_angles",
                                 self.joint_cb, 10)

        self.lock = threading.Lock()
        self.curr_img = None
        self.curr_q4  = None    # 4-DOF angle (deg)

        self.samples_q, self.samples_r, self.samples_t = [], [], []

        # highlight corners
        self.highlight_corners = None
        self.highlight_time    = 0.0     # seconds stamp

        threading.Thread(target=self.viewer,   daemon=True).start()
        threading.Thread(target=self.keyboard, daemon=True).start()
        self.get_logger().info("dh_capture ready  –  r:record  w:write  q/Esc:quit")

    # —— ROS callback —— #
    def img_cb(self, msg:Image):
        bgr = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with self.lock:
            self.curr_img = bgr

    def joint_cb(self, msg:JointAngles):
        with self.lock:
            self.curr_q4 = np.array(msg.angles[:4], float)

    # —— OpenCV real time window —— #
    def viewer(self):
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                frame = None if self.curr_img is None else self.curr_img.copy()
                corners = self.highlight_corners
                start   = self.highlight_time
            if frame is not None:
                if corners is not None and (cv2.getTickCount()/cv2.getTickFrequency() - start) < 0.5:
                    cv2.drawChessboardCorners(frame, BOARD_SIZE, corners, True)
                cv2.imshow("camera", frame)
            if cv2.waitKey(30) & 0xFF == 27:  # Esc
                rclpy.shutdown(); break

    # —— keyboard input —— #
    def keyboard(self):
        while rclpy.ok():
            k = input().strip().lower()
            if k == 'r':
                self.record_once()
            elif k == 'w':
                self.write_npz()
            elif k == 'q':
                rclpy.shutdown(); break

    # —— acquire —— #
    def record_once(self):
        with self.lock:
            img = None if self.curr_img is None else self.curr_img.copy()
            q   = None if self.curr_q4  is None else self.curr_q4.copy()
        if img is None or q is None:
            print("⚠ no image or joint data"); return
        res = detect_pose(img)
        if res is None:
            print("✖ checkerboard NOT found"); return
        corners, rvec, tvec = res

        self.samples_q.append(q)
        self.samples_r.append(rvec)
        self.samples_t.append(tvec)
        print(f"✓ sample #{len(self.samples_q)} recorded")

        with self.lock:
            self.highlight_corners = corners
            self.highlight_time    = cv2.getTickCount()/cv2.getTickFrequency()

    # —— 保存文件 —— #
    def write_npz(self):
        if not self.samples_q:
            print("⚠ nothing to save"); return
        default = datetime.now().strftime("calib_%Y%m%d_%H%M%S.npz")
        fn = input(f"filename [{default}]: ").strip() or default
        np.savez(fn,
                 angles=np.vstack(self.samples_q),
                 rvecs =np.vstack(self.samples_r),
                 tvecs =np.vstack(self.samples_t))
        print(f"✔ saved {len(self.samples_q)} samples to {os.path.abspath(fn)}")

# ── main ───────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
