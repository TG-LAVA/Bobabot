import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class QRCodePnPNode(Node):
    def __init__(self):
        super().__init__('qrcode_pnp_node')
        self.bridge = CvBridge()

        # Camera intrinsics for cam1
        self.camera_matrix_cam1 = np.array([
            [475.680676, 0.0, 319.348050],
            [0.0, 474.219595, 240.180089],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs_cam1 = np.array([-0.036038, -0.270808, 0.003515, -0.002552, 0.574804], dtype=np.float32)

        # Camera intrinsics for cam2
        self.camera_matrix_cam2 = np.array([
            [648.831594, 0.0, 168.883804],
            [0.0, 648.685496, 128.315864],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs_cam2 = np.array([0.056893, 1.620623, -0.003688, 0.011280, -5.393870], dtype=np.float32)

        self.qrcode_size = 0.03  # unit: meters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscription for camera 1
        self.cam1_sub = self.create_subscription(
            CompressedImage,
            '/camera_arm/color/image_raw/compressed',
            lambda msg: self.image_callback(msg, 'camera_arm_link', 'qrcode_cam1', 'target_point_cam1', self.camera_matrix_cam1, self.dist_coeffs_cam1),
            10
        )

        # Subscription for camera 2
        self.cam2_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            lambda msg: self.image_callback(msg, 'camera_link', 'qrcode_cam2', 'target_point_cam2', self.camera_matrix_cam2, self.dist_coeffs_cam2),
            10
        )

        self.detector = cv2.QRCodeDetector()

    def image_callback(self, msg, camera_frame, qrcode_frame, target_frame, camera_matrix, dist_coeffs):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().error("Image decoding failed")
            return

        retval, decoded_info, points, straight_qrcode = self.detector.detectAndDecodeMulti(img)
        if points is None or len(points) == 0:
            self.get_logger().info("QR code not detected")
            return

        corners = points[0].reshape(-1, 2)
        if corners.shape[0] != 4:
            self.get_logger().warn("Incorrect number of corners, cannot solve PnP")
            return

        half_size = self.qrcode_size / 2
        object_points = np.array([
            [-half_size,  half_size, 0],  # top-left
            [ half_size,  half_size, 0],  # top-right
            [ half_size, -half_size, 0],  # bottom-right
            [-half_size, -half_size, 0]   # bottom-left
        ], dtype=np.float32)

        image_points = np.array(corners, dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if not success:
            self.get_logger().error("PnP solving failed")
            return

        # ------------------- QR code frame --------------------
        R_c2m, _ = cv2.Rodrigues(rvec)
        R_fix = R.from_euler('y', 90, degrees=True).as_matrix()
        R_final = R_c2m @ R_fix
        quat = R.from_matrix(R_final).as_quat()

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = camera_frame
        t.child_frame_id = qrcode_frame
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        t.transform.rotation.x = float(quat[0])
        t.transform.rotation.y = float(quat[1])
        t.transform.rotation.z = float(quat[2])
        t.transform.rotation.w = float(quat[3])
        self.tf_broadcaster.sendTransform(t)

        # ------------------- Parse QR content as dx, dy, dz --------------------
        info = decoded_info[0] if decoded_info and len(decoded_info) > 0 else ""
        if info.strip() == "":
            self.get_logger().warn(f"[{camera_frame}] QR code detected but content is empty.")
            return

        try:
            dx, dy, dz = [float(x.strip()) for x in info.split(",")]
        except Exception as e:
            self.get_logger().error(f"QR content format error, expected format like 0.2,0,0. Got: '{info}', error: {e}")
            return

        # ------------------- Target point frame --------------------
        t_point = TransformStamped()
        t_point.header.stamp = self.get_clock().now().to_msg()
        t_point.header.frame_id = qrcode_frame
        t_point.child_frame_id = target_frame
        t_point.transform.translation.x = float(dx)
        t_point.transform.translation.y = float(dy)
        t_point.transform.translation.z = float(dz) * 0.01
        t_point.transform.rotation.x = 0.0
        t_point.transform.rotation.y = 0.0
        t_point.transform.rotation.z = 0.0
        t_point.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_point)

        # ------------------- Log --------------------
        self.get_logger().info(f"[{camera_frame}] QR code content: '{info}', published {target_frame}: ({dx}, {dy}, {dz})")

def main(args=None):
    rclpy.init(args=args)
    node = QRCodePnPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
