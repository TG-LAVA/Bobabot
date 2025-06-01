#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from base_interfaces.msg import JointAngles, ServoStatus
from base_interfaces.srv import SendAngles
import serial
import threading

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.last_values = [0.0] * 5

        # Serial connection
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
            self.get_logger().info('Serial connected at /dev/ttyACM0')
        except Exception as e:
            self.get_logger().error(f'Serial port error: {e}')
            exit(1)

        self.pub_angle = self.create_publisher(JointAngles, 'joint_angles', 10)
        self.pub_status = self.create_publisher(ServoStatus, 'servo_status', 10)

        self.create_service(SendAngles, 'send_servo_command', self.handle_send_request)

        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()

    def read_loop(self):
        while rclpy.ok():
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode().strip()
                    if line.startswith("A:"):
                        parts = line[2:].split(',')
                        if len(parts) != 10:
                            continue
                        angles = list(map(float, parts[:5]))
                        statuses = parts[5:]
                        self.publish_angles(angles)
                        self.publish_status(statuses)
            except Exception as e:
                self.get_logger().error(f"[Serial Read Error] {e}")

    def publish_angles(self, angles):
        msg = JointAngles()
        msg.angles = angles
        self.pub_angle.publish(msg)

    def publish_status(self, status_bits):
        msg = ServoStatus()
        msg.status_bits = status_bits
        self.pub_status.publish(msg)

    def handle_send_request(self, request, response):
        if self.serial_port is None:
            self.get_logger().error("Serial port not available.")
            response.success = False
            return response

        try:
            parts = request.command.split(',')
            if len(parts) != 5:
                self.get_logger().error("Invalid command format (must have 5 values).")
                response.success = False
                return response

            values = [float(p.strip()) for p in parts]
            if values[4] == -100:
                values[4] = self.last_values[4]

            command_str = ','.join([f"{v:.2f}" for v in values])
            self.serial_port.write((command_str + '\n').encode())
            self.get_logger().info(f"Sent to Arduino: {command_str}")

            self.last_values = values
            response.success = True

        except Exception as e:
            self.get_logger().error(f"Send error: {e}")
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
