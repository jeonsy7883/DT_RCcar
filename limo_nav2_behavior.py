import asyncio

import carb
import carb.input
import numpy as np
import omni.appwindow
import omni.kit.app
import omni.replicator.core as rep
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist, TransformStamped
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from nav_msgs.msg import Odometry
from omni.kit.scripting import BehaviorScript
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster

_PHYSICS_CALLBACK_NAME = "limo_nav2_physics_step"
_LIDAR_RELATIVE_PATH = "/lidar"

# limo_ROS.usd 내장 OmniGraph(/limo/drive)에서 쓰던 값과 동일하게 맞춤
_WHEEL_RADIUS = 0.025
_WHEEL_BASE = 0.16
_LEFT_JOINTS = ["front_left_wheel", "rear_left_wheel"]
_RIGHT_JOINTS = ["front_right_wheel", "rear_right_wheel"]

_MAX_LINEAR_CMD = 1.0
_MAX_ANGULAR_CMD = 2.5

# WASD 수동 조작(limo_wasd_teleop_behavior.py와 동일한 매핑). 키를 하나라도 누르고 있으면
# Nav2의 cmd_vel보다 우선해서 직접 조작하고, 모두 떼면 다시 Nav2 명령을 따른다.
_INPUT_MAPPING = {
    "W": [0.6, 0.0],
    "S": [-0.6, 0.0],
    "A": [0.0, 1.2],
    "D": [0.0, -1.2],
}


class LimoNav2Bridge(BehaviorScript):
    """Play를 누르면 자동 실행되는 Limo Nav2 브릿지.

    /cmd_vel(geometry_msgs/Twist)을 구독해 바퀴 관절을 구동하고, ground-truth pose를
    /odom(nav_msgs/Odometry) + TF(odom->base_link)로 publish한다.
    RTX Lidar(/limo/lidar, prepare_nav2_assets_limo.py에서 생성됨)를 ROS2 LaserScan(/scan)으로 publish한다.
    WASD 키보드 수동 조작도 지원한다 — 키를 누르고 있는 동안은 Nav2의 cmd_vel보다 우선한다.

    World/BehaviorScript 초기화 순서는 limo_wasd_teleop_behavior.py에서 검증된 패턴을 그대로 재사용한다.
    """

    def on_init(self):
        self._world = None
        self._robot = None
        self._controller = None
        self._joint_indices = None
        self._setup_task = None
        self._setting_up = False
        self._node = None
        self._cmd_vel_sub = None
        self._odom_pub = None
        self._clock_pub = None
        self._tf_broadcaster = None
        self._lidar_hydra_texture = None
        self._elapsed_sim_time = 0.0
        self._sub_keyboard = None
        self._base_command = np.array([0.0, 0.0])
        self._wasd_command = np.array([0.0, 0.0])
        self._keys_held = set()

    def on_play(self):
        if self._setting_up:
            return
        self._setting_up = True
        self._base_command = np.array([0.0, 0.0])
        # elapsed_sim_time을 여기서 0으로 리셋하면 안 됨: Nav2는 Isaac Sim과 별도 프로세스로 계속
        # 떠있어서, Stop 후 재생 시각이 0으로 되돌아가면 Nav2의 tf2 버퍼가 이미 더 큰 시각을
        # 기억하고 있어 "TF_OLD_DATA ignoring data from the past"로 새 TF/오도메트리를 전부 버린다.
        # on_init에서 한 번만 0으로 초기화하고, 이후 Stop/Play를 반복해도 계속 단조 증가하도록 유지한다.
        self._wasd_command = np.array([0.0, 0.0])
        self._keys_held = set()

        appwindow = omni.appwindow.get_default_app_window()
        keyboard = appwindow.get_keyboard()
        self._sub_keyboard = self.input.subscribe_to_keyboard_events(keyboard, self._on_keyboard_event)

        self._setup_task = asyncio.ensure_future(self._setup())

    async def _setup(self):
        try:
            await self._setup_impl()
            carb.log_warn("[limo_nav2] setup complete, physics callback registered")
        except Exception:
            import traceback

            carb.log_error(f"[limo_nav2] setup FAILED:\n{traceback.format_exc()}")
        finally:
            self._setting_up = False

    async def _setup_impl(self):
        # on_play()가 타임라인 전환 도중 호출되므로, World.reset_async() 등 타임라인을 건드리는
        # API는 무한 재귀 + 렌더러 크래시를 유발할 수 있어 쓰지 않는다. 저수준 초기화만 사용.
        world = World()
        self._world = world

        if world._physics_context is None:
            world._init_stage()
        world.initialize_physics()

        app = omni.kit.app.get_app()
        await app.next_update_async()

        self._robot = SingleArticulation(prim_path=str(self.prim_path), name="Limo")
        self._robot.initialize()
        self._joint_indices = [self._robot.get_dof_index(j) for j in (_LEFT_JOINTS + _RIGHT_JOINTS)]
        self._controller = DifferentialController(
            name="limo_diff_controller",
            wheel_radius=_WHEEL_RADIUS,
            wheel_base=_WHEEL_BASE,
            max_linear_speed=_MAX_LINEAR_CMD,
            max_angular_speed=_MAX_ANGULAR_CMD,
        )

        self._setup_ros()
        self._setup_lidar_publisher()

        if world.physics_callback_exists(_PHYSICS_CALLBACK_NAME):
            world.remove_physics_callback(_PHYSICS_CALLBACK_NAME)
        world.add_physics_callback(_PHYSICS_CALLBACK_NAME, callback_fn=self._on_physics_step)

    def _setup_ros(self):
        # RTX Lidar -> ROS2 LaserScan(/scan)은 Isaac Sim 내부 시뮬레이션 시각을 타임스탬프로 쓴다.
        # 여기서 odom/TF를 wall clock으로 찍으면 두 시각 도메인이 어긋나서 Nav2가
        # "timestamp earlier than all the data in the transform cache"로 TF를 버린다.
        # 그래서 /clock을 시뮬레이션 시각으로 직접 publish하고, odom/TF도 같은 시각을 쓴다.
        # Nav2 쪽 파라미터(nav2_params_limo.yaml)도 use_sim_time: true 로 맞춰야 한다.
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("limo_nav2_bridge")
        self._cmd_vel_sub = self._node.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self._odom_pub = self._node.create_publisher(Odometry, "/odom", 10)
        self._clock_pub = self._node.create_publisher(Clock, "/clock", 10)
        self._tf_broadcaster = TransformBroadcaster(self._node)

    def _sim_time_msg(self) -> TimeMsg:
        # omni.timeline.get_current_time()은 우리 저수준 초기화 경로(timeline.play()를 절대
        # 부르지 않음)에서는 절대 진행되지 않아 항상 0을 반환한다. 그래서 물리 콜백에 들어오는
        # step_size를 직접 누적해서 시각을 만든다 — RTX Lidar도 같은 물리 스텝 루프에 종속되어
        # 있어서 이 값이 라이다 타임스탬프와 근접하게 맞는다.
        sec = int(self._elapsed_sim_time)
        nanosec = int(round((self._elapsed_sim_time - sec) * 1e9))
        return TimeMsg(sec=sec, nanosec=nanosec)

    def _setup_lidar_publisher(self):
        lidar_path = str(self.prim_path) + _LIDAR_RELATIVE_PATH
        hydra_texture = rep.create.render_product(lidar_path, [1, 1], name="Isaac")
        self._lidar_hydra_texture = hydra_texture

        writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        writer.initialize(topicName="scan", frameId="lidar_frame")
        writer.attach([hydra_texture])

    def _on_cmd_vel(self, msg: Twist):
        vx = float(np.clip(msg.linear.x, -_MAX_LINEAR_CMD, _MAX_LINEAR_CMD))
        wz = float(np.clip(msg.angular.z, -_MAX_ANGULAR_CMD, _MAX_ANGULAR_CMD))
        self._base_command = np.array([vx, wz])

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in _INPUT_MAPPING:
                self._keys_held.add(event.input.name)
                self._wasd_command = self._wasd_command + np.array(_INPUT_MAPPING[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in _INPUT_MAPPING:
                self._keys_held.discard(event.input.name)
                self._wasd_command = self._wasd_command - np.array(_INPUT_MAPPING[event.input.name])
        return True

    def on_pause(self):
        pass

    def on_stop(self):
        self._setting_up = False
        if self._setup_task is not None and not self._setup_task.done():
            self._setup_task.cancel()
        self._setup_task = None

        if self._sub_keyboard is not None:
            appwindow = omni.appwindow.get_default_app_window()
            self.input.unsubscribe_to_keyboard_events(appwindow.get_keyboard(), self._sub_keyboard)
            self._sub_keyboard = None
        self._wasd_command = np.array([0.0, 0.0])
        self._keys_held = set()

        if self._world is not None and self._world.physics_callback_exists(_PHYSICS_CALLBACK_NAME):
            self._world.remove_physics_callback(_PHYSICS_CALLBACK_NAME)

        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

        self._cmd_vel_sub = None
        self._odom_pub = None
        self._clock_pub = None
        self._tf_broadcaster = None
        self._lidar_hydra_texture = None
        self._robot = None
        self._controller = None
        self._joint_indices = None
        self._world = None

    def _on_physics_step(self, step_size):
        if self._robot is None or self._controller is None:
            return
        # 키를 하나라도 누르고 있으면 수동 조작이 Nav2의 cmd_vel보다 우선한다.
        command = self._wasd_command if self._keys_held else self._base_command
        action = self._controller.forward(command=command)
        left_vel, right_vel = action.joint_velocities[0], action.joint_velocities[1]
        joint_velocities = [left_vel, left_vel, right_vel, right_vel]
        self._robot.apply_action(
            ArticulationAction(joint_velocities=joint_velocities, joint_indices=self._joint_indices)
        )
        self._elapsed_sim_time += step_size

        if self._node is not None:
            rclpy.spin_once(self._node, timeout_sec=0.0)
            stamp = self._sim_time_msg()
            self._clock_pub.publish(Clock(clock=stamp))
            self._publish_odom(stamp)

    def _publish_odom(self, stamp: TimeMsg):
        position, orientation = self._robot.get_world_pose()  # orientation: (w, x, y, z)
        lin_vel = self._robot.get_linear_velocity()
        ang_vel = self._robot.get_angular_velocity()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation.w = float(orientation[0])
        odom.pose.pose.orientation.x = float(orientation[1])
        odom.pose.pose.orientation.y = float(orientation[2])
        odom.pose.pose.orientation.z = float(orientation[3])
        odom.twist.twist.linear.x = float(lin_vel[0])
        odom.twist.twist.linear.y = float(lin_vel[1])
        odom.twist.twist.linear.z = float(lin_vel[2])
        odom.twist.twist.angular.x = float(ang_vel[0])
        odom.twist.twist.angular.y = float(ang_vel[1])
        odom.twist.twist.angular.z = float(ang_vel[2])
        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = float(position[0])
        tf_msg.transform.translation.y = float(position[1])
        tf_msg.transform.translation.z = float(position[2])
        tf_msg.transform.rotation.w = float(orientation[0])
        tf_msg.transform.rotation.x = float(orientation[1])
        tf_msg.transform.rotation.y = float(orientation[2])
        tf_msg.transform.rotation.z = float(orientation[3])
        self._tf_broadcaster.sendTransform(tf_msg)
