import asyncio

import carb
import carb.input
import numpy as np
import omni.appwindow
import omni.kit.app
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.kit.scripting import BehaviorScript

# WASD 매핑: W/S = 전진/후진 선속도(m/s), A/D = 좌/우 회전 각속도(rad/s)
_INPUT_MAPPING = {
    "W": [0.6, 0.0],
    "S": [-0.6, 0.0],
    "A": [0.0, 1.2],
    "D": [0.0, -1.2],
}

# limo_ROS.usd 내장 OmniGraph(/limo/drive)에서 쓰던 값과 동일하게 맞춤
_WHEEL_RADIUS = 0.025
_WHEEL_BASE = 0.16

_LEFT_JOINTS = ["front_left_wheel", "rear_left_wheel"]
_RIGHT_JOINTS = ["front_right_wheel", "rear_right_wheel"]

_PHYSICS_CALLBACK_NAME = "limo_wasd_physics_step"


class LimoWasdTeleop(BehaviorScript):
    """Play를 누르면 자동으로 실행되는 Limo WASD 텔레오퍼레이션.

    이 스크립트가 붙은 프림(예: /limo)을 대상 articulation의 prim_path로 사용한다.
    limo_ROS.usd에 내장된 ROS2 cmd_vel 구동 그래프(/limo/drive)와 동시에 켜두면
    같은 바퀴 조인트를 두 시스템이 서로 다투게 되므로, 그 그래프는 비활성화된 상태여야 한다.
    """

    def on_init(self):
        self._world = None
        self._robot = None
        self._controller = None
        self._joint_indices = None
        self._sub_keyboard = None
        self._base_command = np.array([0.0, 0.0])
        self._setup_task = None
        self._setting_up = False

    def on_play(self):
        if self._setting_up:
            return
        self._setting_up = True

        self._base_command = np.array([0.0, 0.0])

        appwindow = omni.appwindow.get_default_app_window()
        keyboard = appwindow.get_keyboard()
        self._sub_keyboard = self.input.subscribe_to_keyboard_events(keyboard, self._on_keyboard_event)

        self._setup_task = asyncio.ensure_future(self._setup())

    async def _setup(self):
        try:
            await self._setup_impl()
            carb.log_warn("[limo_wasd] setup complete, physics callback registered")
        except Exception:
            import traceback
            carb.log_error(f"[limo_wasd] setup FAILED:\n{traceback.format_exc()}")
        finally:
            self._setting_up = False

    async def _setup_impl(self):
        # on_play()는 타임라인이 이미 Play 상태로 전환되는 도중에 호출되므로,
        # world.reset_async() 등 타임라인을 건드리는 API는 재귀 호출을 유발할 수 있어 쓰지 않는다.
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
            max_linear_speed=1.0,
            max_angular_speed=2.5,
        )

        if world.physics_callback_exists(_PHYSICS_CALLBACK_NAME):
            world.remove_physics_callback(_PHYSICS_CALLBACK_NAME)
        world.add_physics_callback(_PHYSICS_CALLBACK_NAME, callback_fn=self._on_physics_step)

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

        if self._world is not None and self._world.physics_callback_exists(_PHYSICS_CALLBACK_NAME):
            self._world.remove_physics_callback(_PHYSICS_CALLBACK_NAME)

        self._robot = None
        self._controller = None
        self._joint_indices = None
        self._world = None

    def _on_keyboard_event(self, event, *args, **kwargs):
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in _INPUT_MAPPING:
                self._base_command = self._base_command + np.array(_INPUT_MAPPING[event.input.name])
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            if event.input.name in _INPUT_MAPPING:
                self._base_command = self._base_command - np.array(_INPUT_MAPPING[event.input.name])
        return True

    def _on_physics_step(self, step_size):
        if self._robot is None or self._controller is None:
            return
        action = self._controller.forward(command=self._base_command)
        left_vel, right_vel = action.joint_velocities[0], action.joint_velocities[1]
        joint_velocities = [left_vel, left_vel, right_vel, right_vel]
        self._robot.apply_action(
            ArticulationAction(joint_velocities=joint_velocities, joint_indices=self._joint_indices)
        )
