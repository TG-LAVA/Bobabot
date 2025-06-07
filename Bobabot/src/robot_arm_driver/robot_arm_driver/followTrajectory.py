#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from base_interfaces.msg import JointAngles

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # === Define your arm joint names here ===
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        self.general_joint_states = {}  # {name: (position, velocity, effort)}
        self.last_stamp = self.get_clock().now().to_msg()

        # Subscribe to original joint_states (e.g., from TurtleBot3)
        self.create_subscription(JointState, '/joint_states', self.base_joint_cb, 10)

        # Subscribe to your arm's joint_angles (custom message)
        self.create_subscription(JointAngles, '/joint_angles', self.arm_joint_cb, 10)

        # Publisher for the merged topic
        self.pub = self.create_publisher(JointState, '/general/joint_states', 10)

        # Publish at 50Hz
        self.create_timer(0.02, self.publish_general_joint_states)

    def base_joint_cb(self, msg):
        # Update all joints except those that are in self.arm_joint_names
        for i, name in enumerate(msg.name):
            if name not in self.arm_joint_names:
                pos = msg.position[i] if len(msg.position) > i else 0.0
                vel = msg.velocity[i] if len(msg.velocity) > i else 0.0
                eff = msg.effort[i] if len(msg.effort) > i else 0.0
                self.general_joint_states[name] = (pos, vel, eff)
        self.last_stamp = msg.header.stamp

    def arm_joint_cb(self, msg):
        # Update arm joints
        for i, name in enumerate(self.arm_joint_names):
            pos = msg.angles[i] if len(msg.angles) > i else 0.0
            # velocity and effort are left as 0.0 by default
            self.general_joint_states[name] = (pos, 0.0, 0.0)
        # Stamp is updated on publish below

    def publish_general_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.general_joint_states.keys())
        msg.position = [self.general_joint_states[n][0] for n in msg.name]
        msg.velocity = [self.general_joint_states[n][1] for n in msg.name]
        msg.effort  = [self.general_joint_states[n][2] for n in msg.name]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
