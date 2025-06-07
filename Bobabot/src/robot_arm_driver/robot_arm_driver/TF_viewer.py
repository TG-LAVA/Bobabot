import rclpy
from rclpy.node import Node
from base_interfaces.msg import JointAngles
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time
from scipy.spatial.transform import Rotation as R
from robot_arm_driver.RobotArmFK import *
from tf_transformations import quaternion_from_matrix

        # ── hand–eye solution from calibration ──────────────
tvec = [-0.050332, -0.036715, -0.032146]          # metres
rvec = [ 1.251116,  1.137865,  1.147833]          # Rodrigues (rad)

class RobotArmTFPublisher(Node):
    def __init__(self):
        super().__init__('robot_arm_tf_publisher')
        self.robot_arm = RobotArmFK()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            JointAngles,
            'joint_angles',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_angles = []

    def listener_callback(self, msg:JointAngles):
        self.latest_angles = msg.angles[0:4]
        self.end_effector_angle = msg.angles[4]
        self.robot_arm.EditAngle([float(x) for x in self.latest_angles])


    def timer_callback(self):
        self.send_static_transform()
        matrixes = self.robot_arm.FKanalysis()
        for i in range(1,len(matrixes)):
            self.homogeneous_to_TF(matrixes[i],f'dh_base',f'link{i}')

    def send_static_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'dh_base'

        t.transform.translation.x = -0.125  # joint_1 is behind camera
        t.transform.translation.y = -0.02
        t.transform.translation.z = 0.100  # joint_1 is lower than camera

        # No rotation (identity quaternion)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'link4'
        static_tf.child_frame_id = 'flange_link'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        # Send it once — StaticTransformBroadcaster latches internally
        self.tf_broadcaster.sendTransform(static_tf)


        # ── publish TF  camera_arm_link  ↔  link4 ──────────
        c = TransformStamped()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id    = "link4"          # base (flange) frame
        c.child_frame_id     = "camera_arm_link"

        # translation
        c.transform.translation.x, \
        c.transform.translation.y, \
        c.transform.translation.z = (float(t) for t in tvec)

        # rotation  (Rodrigues → quaternion)
        qx, qy, qz, qw = R.from_rotvec(rvec).as_quat()     # (x, y, z, w)

        c.transform.rotation.x = float(qx)
        c.transform.rotation.y = float(qy)
        c.transform.rotation.z = float(qz)
        c.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(c)

        

    def homogeneous_to_TF(self,matrix: np.ndarray,header_name:str,child_name:str):
        if matrix.shape != (4, 4):
            raise ValueError(f"Expected a 4x4 matrix, got shape {matrix.shape}")
        quat = quaternion_from_matrix(matrix)
        translation = matrix[:3, 3]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = header_name
        t.child_frame_id = child_name
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
