from base_interfaces.msg import JointAngles, ServoStatus
from rclpy.node import Node
import rclpy

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.create_subscription(JointAngles, 'joint_angles', self.angle_callback, 10)
        self.create_subscription(ServoStatus, 'servo_status', self.status_callback, 10)

    def angle_callback(self, msg):
        angle_str = ', '.join([f"{a:.2f}" for a in msg.angles])
        self.get_logger().info(f"[Angles] {angle_str}")

    def status_callback(self, msg):
        status_str = ', '.join(msg.status_bits)
        self.get_logger().info(f"[Status] {status_str}")

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
