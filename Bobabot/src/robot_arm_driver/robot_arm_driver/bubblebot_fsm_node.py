#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto
import numpy as np
from rclpy.duration import Duration
from base_interfaces.msg import JointAngles
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from base_interfaces.action import PathPlanner
from base_interfaces.action import MapCreator  
from base_interfaces.srv import WorkFlow
from builtin_interfaces.msg import Time
import asyncio, math
from tf_transformations import quaternion_multiply, quaternion_from_euler
import tf2_ros                       # for the exception classes
from base_interfaces.srv import SendAngles
from dataclasses import dataclass
from base_interfaces.action import COMPlanner
from action_msgs.msg import GoalStatus
from tf2_geometry_msgs import do_transform_point
from base_interfaces.srv import BaseCommand  # Adjust to your srv module path


from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from builtin_interfaces.msg import Time


@dataclass  
class RequestEntry:
    status: int
    pose: PoseStamped
    point: PointStamped



class RobotFSMNode(Node):

    class State(Enum):
        INITIALIZE               = auto()
        COMMUNICATE_TO_SERVER    = auto()
        NAVIGATING_TO_POSITION   = auto()
        ROBOT_ARM_OPERATING      = auto()
        LIQUID_FILL              = auto()
        MAP                      = auto()
        ROBOT_ARM_INITIALIZING   = auto()
        POUR                     = auto()
        TARGETING                = auto()

    def __init__(self):
        super().__init__('robot_fsm')
        #─── Entry setup ───────────────────────────────────────────
        self.entries = []
        self.joint_angles = []
        # ─── FSM setup ───────────────────────────────────────────
        self.state = self.State.INITIALIZE
        self._handlers = {
            self.State.INITIALIZE:            self._handle_initialize,
            self.State.MAP:                   self._handle_map, 
            self.State.COMMUNICATE_TO_SERVER: self._handle_communicate,
            self.State.NAVIGATING_TO_POSITION:self._handle_navigate,
            self.State.ROBOT_ARM_OPERATING:   self._handle_arm_operation,
            self.State.ROBOT_ARM_INITIALIZING:self._handle_arm_Initialize,
            self.State.LIQUID_FILL:           self._handle_liquid_fill,
            self.State.POUR:                  self._handle_Pour,
            self.State.TARGETING:             self._handle_target,  # same as operating
        }
        # tick FSM at 10 Hz
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"[FSM] Starting in {self.state.name}")

        # ─── Nav2 Action Client ───────────────────────────────────
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav2_goal_handle = None
        self._nav2_goal_sent = False
        self._nav2_done = False
        self.next_state = None
        self.IsArmInitialized = False  # Flag to check if arm is initialized
        self.logged_joint_angles = None  # Store joint angles for pouring

        self.cli_base = self.create_client(BaseCommand, 'db_base_command')
        while not self.cli_base.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for db_base_command service...')
        self.req_base = BaseCommand.Request()

        # client to send servo commands
        self.cli_angle = self.create_client(SendAngles, 'send_servo_command')
        if not self.cli_angle.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service "send_servo_command" not available!')
            raise RuntimeError('Service not available')
        
                # subscription to joint angles
        self.joint_sub = self.create_subscription(
            JointAngles, 'joint_angles', self.joint_angles_callback, 10
        )

        # ─── PathPlanner (robot-arm) Client ───────────────────────
        self._arm_client = ActionClient(self, PathPlanner, 'path_planner')
        self.grabber_status:int = 1  # status: 0: no_action 1. grab 2. release 3. dump 4. fill with liquid
        self.grabber_position:PointStamped = PointStamped()   # position: 0: no_action 1. grab 2. release 3. dump

        self.grabber_position.header.stamp = Time()            # 0-initialised
        self.grabber_position.header.frame_id = "dh_base"    # or "world", etc.

        self.grabber_position.point.x = 0.20   # forward 20 cm
        self.grabber_position.point.y = 0.00   # centred
        self.grabber_position.point.z = 0.30   # 15 cm above the base

        
        self.grabber_next_state = None  # next state after arm operation
        # ─── MapServer Client ───────────────────────
        self._map_server = ActionClient(self, MapCreator , 'navigate_by_fsm')
        # ─── OrderServerServer Client ───────────────────────
        self.srv = self.create_service(
            WorkFlow,           # service type
            'workflow',        # service name
            self._handle_workorder_request  # callback
        )
        self._next_orfer = False
        self._map_goal_sent = False
        self._map_done = False
        self._nav2_target_pose:PoseStamped = PoseStamped()

        self._next_map_action_id = None
        self._next_map_filename = None

        # ─── COM Planner ───────────────────────────────────
        self._com_client = ActionClient(self, COMPlanner, 'send_images')
        self._com_goal_sent    = False
        self._com_done         = False
        self._com_result      = None


        # ─── Hard Positions ───────────────────────────────────
        self.HomePosition = PoseStamped()  #TBD


        self.ArmInitialPosition = PointStamped()  
        self.ArmInitialPosition.header.stamp = Time()            # 0-initialised
        self.ArmInitialPosition.header.frame_id = "dh_base"    # or "world", etc.
        self.isDumpStation = False  # Flag to check if dump station is reached
        self.DumpPosition = PointStamped()

        self.DumpPosition.point.x = 0.00   # forward 20 cm
        self.DumpPosition.point.y = 20.0   
        self.DumpPosition.point.z = 0.15   # 15 cm above the base

        # ── Position (metres in the chosen frame) ───────────────
        self.ArmInitialPosition.point.x = 0.15   # forward 20 cm
        self.ArmInitialPosition.point.y = 0.00   # centred
        self.ArmInitialPosition.point.z = 0.15   # 15 cm above the base


        self.grabber_nonmoving_position = -100
        # ─── Grapper Status ───────────────────────────────────
        self.arm_sent_goal = False  # Flag to check if gripper goal is sent
        self.arm_reached = False
        # ─── Drink Station ───────────────────────────────────
        self.injection = False  # Flag to check if injection is needed
        self._liquid_fill_timer = None
        # ─── Pour station ───────────────────────────────────
        self.Pour = False
        self._pour_timer = None
        self.Pouring = False

        # ───QR Code Handlers ──────────────────────────────
        self.is_timer_running_qr = False
        self._qr_timer = None  # Timer for QR code scanning

        # ─── TF Buffer ─────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        

    def _tick(self):
        self._handlers[self.state]()

    def transition(self, new_state):
        self.get_logger().info(f"[FSM] → {self.state.name} → {new_state.name}")
        self.state = new_state


    def joint_angles_callback(self, msg: JointAngles):
        self.joint_angles = list(msg.angles)

    # ─── Workorder Handlers ────────────────────────────────────────
    def _handle_workorder_request(self, request, response):
        if self.state != self.State.COMMUNICATE_TO_SERVER:
            response.success = False
            self._next_orfer = True  #have next order, cant relax
            return response

        # your logic here; for demo we just prepend “Echo: ”
        for status, pose, point in zip(
                request.grabberstatus,
                request.turtlebotgoal,
                request.armgoal):
            entry = RequestEntry(
                status=status,
                pose=pose,
                point=point
            )
            self.entries.append(entry)
        self._next_orfer = False
        self._set_goal()
        response.success = True
        return response

    def _set_goal(self):
        if self.entries:
            first_entry = self.entries.pop(0)
            # if robot arm position given, first navigate then move robot arm
            self.next_state = self.State.ROBOT_ARM_OPERATING
            self.grabber_status = first_entry.status
            self._nav2_target_pose = first_entry.pose
            self.grabber_position = first_entry.point
            self.transition(self.State.ROBOT_ARM_INITIALIZING) #INITIALIZE then go to target position
        elif self._next_orfer:
            self.transition(self.State.COMMUNICATE_TO_SERVER)
        else:
            self.get_logger().info("No more work orders, going to IDLE")
            self._nav2_target_pose = self.HomePosition
            self.next_state = self.State.COMMUNICATE_TO_SERVER
            self.transition(self.State.ROBOT_ARM_INITIALIZING)

    # ─── State Handlers ────────────────────────────────────────

    def _handle_initialize(self):
        self.get_logger().info("INITIALIZE: doing setup…")
        # … your setup logic here …

        options = {
            '0': "create map",
            '1': "communicate to server",
        }

        # display the menu
        print("\nSelect next state:")
        for key, desc in options.items():
            print(f"  {key}: {desc}")
        choice = input("Enter number and press <Enter>: ").strip()

        if choice in options:
            # parse the action ID
            action_id = int(choice)

            # prompt for filename
            filename = input("Enter map filename (e.g. my_map.yaml): ").strip()
            if not filename:
                self.get_logger().warn("No filename entered; staying in INITIALIZE")
                return

            # store both for use in _handle_map
            self._next_map_action_id = action_id
            self._next_map_filename = filename

            # transition to MAP state
            self.transition(self.State.MAP)
        else:
            self.get_logger().warn(f"Invalid choice '{choice}', staying in INITIALIZE")




    def send_command(self, command_str):
        req = BaseCommand.Request()
        req.command = command_str
        future = self.cli_base.call_async(req)
        future.add_done_callback(self._handle_base_response)



    def _handle_base_response(self, future):
        resp = future.result()
        if resp is not None:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = resp.loc_x
            pose.pose.position.y = resp.loc_y
            pose.pose.position.z = resp.loc_z
            pose.pose.orientation.x = resp.loc_ox
            pose.pose.orientation.y = resp.loc_oy
            pose.pose.orientation.z = resp.loc_oz
            pose.pose.orientation.w = resp.loc_ow
            self.get_logger().info(
                f"Got pose at x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, z={pose.pose.position.z:.3f} "
                f"and orientation (ox,oy,oz,ow)=({pose.pose.orientation.x:.3f}, {pose.pose.orientation.y:.3f}, "
                f"{pose.pose.orientation.z:.3f}, {pose.pose.orientation.w:.3f})"
            )
            self.HomePosition = pose
        else:
            self.get_logger().error("Service call failed")




    def _handle_communicate(self):
        self.get_logger().info("COMMUNICATE_TO_SERVER: Waiting for work order…")


    def _handle_navigate(self):
        if not self._nav2_goal_sent:
            self._send_nav2_goal()
        elif self._nav2_done:
            # reset for next time and move on
            self._nav2_goal_sent = False
            self._nav2_done = False
            self.transition(self.next_state)

    def _handle_map(self):
        # 1) never sent yet → send it
        if not self._map_goal_sent:
            self.map_server_send_goal(self._next_map_filename, self._next_map_action_id)

        # 3) it finished successfully → move on
        elif self._map_done:
            # reset for next time
            self._map_goal_sent = False
            self._map_done = False
            self.get_logger().info("SENDING CMD")
            self.send_command("get_home")
            self.transition(self.State.COMMUNICATE_TO_SERVER)


    def _handle_arm_operation(self):
        self.get_logger().info("Moving gripper…")
        if not self.arm_sent_goal:
            # send the initial position to the robot arm
            grabber_goal = -100
            self.grabber_next_state = None  # no action, just communicate
            if self.grabber_status == 1:
                self.grabber_next_state = self.State.TARGETING  # next state is targeting
            elif self.grabber_status == 2:
                self.grabber_next_state = self.State.TARGETING  # next state is targeting
            elif self.grabber_status == 3:
                self.grabber_next_state = self.State.TARGETING  # next state is targeting
            elif self.grabber_status == 4:
                self.grabber_next_state = self.State.TARGETING  # next state is targeting
                

            elif self.grabber_status == 5:    #returned to do its own things
                grabber_goal = 1  # grab
            elif self.grabber_status == 6:    #returned to do its own things
                grabber_goal = 0  # release
            elif self.grabber_status == 7:    #returned to do its own things
                grabber_goal = 1
                self.injection = True  # injection is needed
            elif self.grabber_status == 8:    #returned to do its own things
                grabber_goal = 1
                self.Pour = True  # Pour is needed

            self.arm_sent_goal = True
            self.robot_arm_send_goal(self.grabber_position, grabber_goal)
        elif self.arm_reached:
            # reset for next time
            self.arm_sent_goal = False
            self.arm_reached = False
            self.get_logger().info("Gripper initialized successfully")
            if self.injection:
                self.get_logger().info("Reaching completed, moving to Injection")
                self.injection = False
                self.transition(self.State.LIQUID_FILL)
                return
            if self.Pour:
                self.get_logger().info("Reaching completed, moving to next state")
                self.Pour = False
                self.transition(self.State.POUR)
            if self.grabber_next_state is not None:
                self.get_logger().info(f"Reaching completed, moving to {self.grabber_next_state.name}")
                self.transition(self.grabber_next_state)
            else:
                self._set_goal()


    #apriltag / segmentation
    def _handle_target(self):
        if self.grabber_status == 1:
            self._handle_compute_com()
        else: 
            # Entering the CHECK_QR state?
            if not self.is_timer_running_qr:
                self.get_logger().info("CHECK_QR: starting 1 s timer to poll for qr_code_arm TF")
                # repeat every 1 s
                self._qr_check_timer = self.create_timer(
                    1.0,
                    self._on_qr_check_timer,
                    oneshot=False
                )
                self.is_timer_running_qr = True

    def _on_qr_check_timer(self):
        try:
            # non-blocking check for “map ← qr_code_arm” at latest time
            if self.tf_buffer.can_transform(
                target_frame="dh_base",
                source_frame="qr_code_arm",
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            ):
                t: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame="dh_base",
                source_frame="qr_code_arm",
                time=rclpy.time.Time())
                
                                # 2) Build your PointStamped in dh_base
                qr_pt = PointStamped()
                qr_pt.header.stamp    = t.header.stamp
                qr_pt.header.frame_id = "dh_base"
                qr_pt.point.x         = t.transform.translation.x
                qr_pt.point.y         = t.transform.translation.y
                qr_pt.point.z         = t.transform.translation.z
                self.get_logger().info("Found TF 'dh_base' ← 'qr_code_arm'!")
                self.grabber_position = qr_pt
                self.grabber_status   = self.grabber_status+4
                self.transition(self.State.ROBOT_ARM_OPERATING)

                # stop polling
                self._qr_check_timer.cancel()
                self._qr_check_timer = None
                self._checking_for_qr = False

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            # TF not ready yet—just log at debug level
                self.get_logger().debug(f"QR TF not yet available: {e}")


    def _handle_compute_com(self):
        if not self._com_goal_sent:
            goal_msg = COMPlanner.Goal()
            goal_msg.dummy = True  # or False, depending on your needs

            self._com_client.wait_for_server()
            self._send_goal_future = self._com_client.send_goal_async(
                goal_msg,
                feedback_callback=self._com_feedback
            )
            self._send_goal_future.add_done_callback(self._com_response)
            self._com_goal_sent = True

        elif self._com_done:
            self._com_goal_sent = False
            self._com_done      = False

            # snapshot the raw list
            com = list(self._com_result.com)
            # schedule the TF + transition
            asyncio.run(self._on_com_and_project(com))

    async def _on_com_and_project(self, com: list[float]):
        x, y, z = com[0], com[1], com[2]
        base_pt = await self.project_cup_to_base(x, y, z, 'camera_rgb_frame')
        if base_pt is None:
            self.get_logger().error("TF projection failed")
            return

        self.grabber_position = base_pt
        self.grabber_status   = 5
        self.get_logger().info("COM + TF succeeded")
        self.transition(self.State.ROBOT_ARM_OPERATING)



    async def project_cup_to_base(self, x: float, y: float, z: float,pointid ) -> PointStamped:
        # 1) Build the point in the camera frame
        cam_pt = PointStamped()
        cam_pt.header.stamp    = self.get_clock().now().to_msg()
        cam_pt.header.frame_id = pointid
        cam_pt.point.x = x*0.001
        cam_pt.point.y = y*0.001
        cam_pt.point.z = z*0.001

        # 2) Lookup transform: dh_base ← camera_rgb_frame
        try:
            t = await self.tf_buffer.lookup_transform_async(
                target_frame='dh_base',
                source_frame=pointid,
                time=rclpy.time.Time())    # “latest”
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

        # 3) Apply it to get the point in dh_base
        base_pt: PointStamped = do_transform_point(cam_pt, t)

        # now base_pt.header.frame_id == 'dh_base'
        # and base_pt.point is the cup location in base coords
        return base_pt


    def _com_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("COMPlanner goal rejected")
            self._com_goal_sent = False
            return

        self.get_logger().info("COMPlanner goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._com_result_cb)
        result_future.add_done_callback(self._com_aborted_cb)


    def _com_feedback(self, feedback_msg):
        # feedback_msg.feedback.dummy is a bool
        self._com_feedback_val = feedback_msg.feedback.dummy
        self.get_logger().debug(f"COMPlanner feedback: dummy={self._com_feedback_val}")

    def _com_result_cb(self, future):
        """Called when the action completes (any terminal status)."""
        response = future.result()           # GetResult.Response
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            # normal success path
            self._com_result = response.result
            self._com_done   = True
        # else: let _com_aborted_cb deal with other statuses

    def _com_aborted_cb(self, future):
        """Also called on terminal status—reset if it wasn’t a success."""
        response = future.result()
        if response.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("COMPlanner action aborted → resetting goal flag")
            self._com_goal_sent = False
        elif response.status not in (GoalStatus.STATUS_SUCCEEDED,):
            # you can catch other statuses here too (CANCELED, etc.)
            self.get_logger().warn(
                f"COMPlanner ended with status {response.status} → resetting goal flag"
            )
            self._com_goal_sent = False



            
    def _handle_Pour(self):
        if self.Pouring:
            return

        self.get_logger().info("POURING")
        self.Pouring = True

        # snapshot once → safe against concurrent updates
        self.logged_joint_angles = np.array(self.joint_angles, dtype=np.float64)

        # 5-element vector: j0 j1 j2 (j3-90) j4
        pour_angles = np.concatenate((
            self.logged_joint_angles[:3],
            [self.logged_joint_angles[3] - 90.0,
            self.logged_joint_angles[4]]
        ))

        asyncio.create_task(self._send_and_report(pour_angles))


    # ------------------------------------------------------------
    async def _send_and_report(self, pour_angles: np.ndarray):
        try:
            # 1) Send pour command
            if await self.UART_SEND(pour_angles):
                self.get_logger().info("Pour command accepted")
            else:
                self.get_logger().error("Pour command failed")

            # 2) Let the liquid flow 5 s
            await asyncio.sleep(5)

            # 3) Return to original posture
            if await self.UART_SEND(self.logged_joint_angles):
                self.get_logger().info("Return command accepted")
            else:
                self.get_logger().error("Return command failed")

            # 4) Wait 2 s for the move to finish
            await asyncio.sleep(2)

            # 5) Advance FSM
            self._set_goal()

        except Exception as e:
            self.get_logger().exception(f"Pour coroutine crashed: {e}")

        finally:
            # Always clear flag so another pour can be triggered
            self.Pouring = False




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




    # first initialize the arm, then move to target position
    def _handle_arm_Initialize(self):
        self.get_logger().info("initializing gripper…")
        if not self.arm_sent_goal:
            # send the initial position to the robot arm
            self.robot_arm_send_goal(self.ArmInitialPosition, self.grabber_nonmoving_position)
        elif self.arm_reached:
            # reset for next time
            self.arm_sent_goal = False
            self.arm_reached = False
            self.get_logger().info("Gripper initialized successfully")
            self.transition(self.State.NAVIGATING_TO_POSITION)

    def _handle_liquid_fill(self):
        self.get_logger().info("LIQUID_FILL: running pump…")
        if self._liquid_fill_timer is None:
            self.get_logger().info("LIQUID_FILL: running pump…")
            # when this timer fires once, it calls _on_liquid_fill_done
            self._liquid_fill_timer = self.create_timer(
                3.0,
                self._on_liquid_fill_done
            )


    def _on_liquid_fill_done(self):
        # cancel and clear the timer so it doesn’t repeat
        self._liquid_fill_timer.cancel()
        self._liquid_fill_timer = None

        self.get_logger().info("LIQUID_FILL: pump finished, dumping cup")
        self._set_goal()



    # ─── Nav2 Helpers & Callbacks ──────────────────────────────

    def _send_nav2_goal(self):
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("Nav2 server not ready, will retry soon")
            # try again next tick
            return

        goal = NavigateToPose.Goal()
        goal.pose = self._nav2_target_pose  #need to have target pose to start

        self.get_logger().info(
            f"NAVIGATING_TO_POSITION: sending Nav2 goal → "
            f"x={self._nav2_target_pose.pose.position.x:.2f}, "
            f"y={self._nav2_target_pose.pose.position.y:.2f}"
        )
        send_goal = self._nav2_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback
        )
        send_goal.add_done_callback(self._nav2_response)
        self._nav2_goal_sent = True

    def _nav2_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected, resending…")
            # let _handle_navigate clear and resend next tick
            self._nav2_goal_sent = False
            return

        self.get_logger().info("Nav2 goal accepted")
        self._nav2_goal_handle = goal_handle
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self._nav2_result)

    def _nav2_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Nav2 feedback: remaining {dist:.2f} m")

    def _nav2_result(self, future):
        res = future.result()
        status = getattr(res, 'status', None)
        result_msg = getattr(res, 'result', None)
        # Defensive: Show for debug
        print(f"[DEBUG] status: {status}, result_msg: {result_msg}")

        if status == 4:  # SUCCEEDED
            self.get_logger().info("Nav2 succeeded")
            self._nav2_done = True
        elif status == 5:  # CANCELED
            self.get_logger().warn("Nav2 was canceled")
            self._nav2_goal_sent = False
        elif status == 6:  # ABORTED
            self.get_logger().error("Nav2 aborted")
            self._nav2_goal_sent = False
        else:
            self.get_logger().error(f"Nav2 failed (status={status}), retrying…")
            self._nav2_goal_sent = False



    # ─── Robot‐Arm Helper (PathPlanner) ────────────────────────

    def robot_arm_send_goal(self, point, gripper):
        """Call this from any state to drive your PathPlanner action."""
        if not self._arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("PathPlanner server not ready")
            return

        goal = PathPlanner.Goal()
        goal.goal = point
        goal.gripper = gripper
        send = self._arm_client.send_goal_async(
            goal, feedback_callback=self._arm_feedback
        )
        self.arm_sent_goal = True
        self.arm_reached = False
        send.add_done_callback(self._arm_response)

    def _arm_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn("Arm goal rejected")
            self.arm_sent_goal = False
            return
        # Register for result!
        gh.get_result_async().add_done_callback(self._arm_result)

        

    def _arm_feedback(self, feedback_msg):
        self.get_logger().debug(f"Arm feedback: {feedback_msg.feedback.progress}%")

    def _arm_result(self, future):
        res = future.result().result
        self.arm_reached = True
        self.get_logger().info(f"Arm result: success={res.success}, code={res.error_code}")

    # ─── Map Planner Helper ────────────────────────

    def map_server_send_goal(self, mapname: str, action_id: int):
        """Call this from any state to drive your MapServer action."""
        if not self._map_server.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("MapServer server not ready")
            return

        goal = MapCreator.Goal()
        goal.mapname = mapname
        goal.action_id = action_id

        send_goal_future = self._map_server.send_goal_async(
            goal,
            feedback_callback=self._map_feedback
        )
        self._map_goal_sent = True
        send_goal_future.add_done_callback(self._map_response)

    def _map_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("MapServer goal rejected")
            self._map_goal_sent = False
            self.transition(self.State.INITIALIZE)
            return
        goal_handle.get_result_async().add_done_callback(self._map_result)

    def _map_feedback(self, feedback_msg):
        # feedback_msg.feedback.success is a bool
        self.get_logger().info(
            f"[MapServer] feedback - success={feedback_msg.feedback.success}"
        )

    def _map_result(self, future):
        result = future.result().result
        self._map_done = True
        # result.progress is a string
        self.get_logger().info(f"[MapServer] result - progress: '{result.progress}'")



def main():
    rclpy.init()
    node = RobotFSMNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
