

# #!/usr/bin/env python3
import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces.action import PathPlanner
from geometry_msgs.msg import Point
from base_interfaces.msg import JointAngles
from base_interfaces.msg import ServoStatus
from rclpy.duration import Duration
from rclpy.action import GoalResponse, CancelResponse
from unique_identifier_msgs.msg import UUID
from geometry_msgs.msg import Point
from base_interfaces.srv import SendAngles
from tf2_ros import Buffer, TransformListener
from robot_arm_driver.ik_solver import *
import tf2_geometry_msgs
import asyncio



class PathPlannerServer(Node):

    def __init__(self):
        super().__init__('path_planner_server')
        self.iksolver = FiveDOFIKSolver()
        self.goal: Point = None
        self.goal_handle = None
        self.p_base = None
        self.joint_angles: np.ndarray = None
        self.allfinished = False

        self.cost_error = 0.02

        # client to send servo commands
        self.cli = self.create_client(SendAngles, 'send_servo_command')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service "send_servo_command" not available!')
            raise RuntimeError('Service not available')

        # subscription to joint angless
        self.joint_sub = self.create_subscription(
            JointAngles, 'joint_angles', self.joint_angles_callback, 10
        )

        # subscription to servo status
        self.servo_sub = self.create_subscription(
            ServoStatus, 'servo_status', self.servo_status_callback, 10
        )

        # TF listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # action server
        self._action_server = ActionServer(
            self,
            PathPlanner,
            'path_planner',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info('PathPlanner action server ready')

        # flag & constants
        self.reached_target = False
        self.keep_current_state = -100  # sentinel for "no gripper change"
        


    def joint_angles_callback(self, msg: JointAngles):
        self.joint_angles = list(msg.angles)

    def servo_status_callback(self, msg: ServoStatus):
        # allfinished == 1 only when all servos report LSB '1'
        flags = [(s[-1] == '1') for s in msg.status_bits if s]
        self.allfinished = 1 if all(flags) and flags else 0

    def goal_callback(self, goal_request) -> GoalResponse:
        # transform incoming PointStamped to dh_base frame
        p_stamped = goal_request.goal
        self.endposition = goal_request.gripper
        p_base_stamped = self._tf_buffer.transform(
            p_stamped, "dh_base", timeout=Duration(seconds=1.0)
        )
        self.p_base: Point = p_base_stamped.point

        # reachability check
        if not self.iksolver.check_reachability(
            self.p_base.x, self.p_base.y, self.p_base.z,
            self.joint_angles[:4] if self.joint_angles else [0,0,0,0],self.cost_error
        ):
            self.cost_error += 0.02  # increase error threshold for next time
            self.get_logger().info('Goal is unreachable/invalid')
            return GoalResponse.REJECT
        self.cost_error = 0.02 
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # publish initial feedback
        self._publish_progress(goal_handle, 0)
        self.goal = self.p_base
        self.goal_handle = goal_handle

        self.get_logger().info(
            f'Received goal → x: {self.goal.x:.2f}, y: {self.goal.y:.2f}, z: {self.goal.z:.2f}'
        )
        self._publish_progress(goal_handle, 1)

        # compute full path
        path_stamped = self.iksolver.solve(
            self.p_base.x, self.p_base.y, self.p_base.z,
            self.joint_angles[:4] if self.joint_angles else [0,0,0,0]
        )

        # iterate through intermediate poses
        self.reached_target = False
        last_stamped = path_stamped[-1]
        while not self.reached_target:
            target = np.append(path_stamped[0], self.keep_current_state)
            await self.plan_path(target)
            # ensure we have a numpy array so we can do 2-D slicing
            if isinstance(path_stamped, list):
                path_stamped = np.array(path_stamped)
            path_stamped = path_stamped[1:, :]

            if path_stamped.size == 0:
                self.reached_target = True

        # final gripper move
        self.get_logger().info('Reached target position, now operating gripper')
        final_target = np.append(last_stamped, self.get_angle(self.endposition))
        await self.plan_path(final_target)

        # finish
        result = PathPlanner.Result()
        result.success = True
        result.error_code = 0
        self.get_logger().info('Path planning succeeded')
        goal_handle.succeed()
        return result

    async def plan_path(self, target: np.ndarray):
        is_gripper_move = (target[-1] != self.keep_current_state)
        self.get_logger().info(f'Planning path to target: {target.tolist()}')

        # send command
        if not await self.UART_SEND(target):
            self.get_logger().error('Failed to send angles to robot arm')
            return

        # wait appropriately
        if is_gripper_move:
            await self.wait_for_reached()
        else:
            joint_targets = target[:-1].tolist()
            await self.wait_for_joint_threshold(joint_targets, threshold_deg=10.0)

    async def wait_for_joint_threshold(self, joint_targets, threshold_deg=10.0):
        self.get_logger().info(
            f'Waiting until within {threshold_deg}° of targets {joint_targets}…'
        )
        await self.sleep(0.1)
        while True:
            if not self.joint_angles:
                await self.sleep(0.1)
                continue

            current = self.joint_angles[:len(joint_targets)]
            errors = [abs(c - t) for c, t in zip(current, joint_targets)]
            if all(err <= threshold_deg for err in errors):
                self.get_logger().info('Within threshold, moving on.')
                break
            await self.sleep(0.1)

    async def wait_for_reached(self):
        await self.sleep(0.5)
        while self.allfinished == 0:
            await self.sleep(0.2)

    async def UART_SEND(self, angle: np.ndarray) -> bool:
        req = SendAngles.Request()
        req.angles = angle.astype(np.float64).tolist()
        future = self.cli.call_async(req)
        result = await future
        if result is not None:
            return result.success
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return False

    async def sleep(self, duration_sec):
        future = rclpy.task.Future()

        def timer_callback():
            future.set_result(None)

        timer = self.create_timer(duration_sec, timer_callback)
        try:
            await future
        finally:
            timer.cancel()

    def _publish_progress(self, goal_handle, progress: int):
        feedback = PathPlanner.Feedback()
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)
        self.get_logger().debug(f"Feedback published: progress={progress}")

    angle_map = {0: 0.0, 1: 70.0, 2: -100.0}
    def get_angle(self, code: int) -> float:
        return self.angle_map.get(code, 0.0)


def main():
    rclpy.init()
    server = PathPlannerServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
