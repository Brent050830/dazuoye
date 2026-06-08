import math
import os
import sys
from threading import Lock

import carla


def _prepare_conda_dll_path():
    """让直接调用 Conda 环境中的 python.exe 也能找到 numpy 依赖 DLL。"""
    if os.name != "nt":
        return
    library_bin = os.path.join(sys.prefix, "Library", "bin")
    if not os.path.isdir(library_bin):
        return
    path_entries = os.environ.get("PATH", "").split(os.pathsep)
    if library_bin.lower() not in [entry.lower() for entry in path_entries]:
        os.environ["PATH"] = library_bin + os.pathsep + os.environ.get("PATH", "")


_prepare_conda_dll_path()

pygame_import_error = None
try:
    import pygame
except ImportError as exc:
    pygame = None
    pygame_import_error = exc

numpy_import_error = None
try:
    import numpy as np
except ImportError as exc:
    np = None
    numpy_import_error = exc


# ===================== 碰撞监测传感器 =====================

class CollisionMonitor:
    """CARLA 内置碰撞传感器封装，实时记录自车碰撞事件"""

    def __init__(self, world, vehicle, actor_list):
        self.history = []  # 碰撞事件历史列表
        blueprint = world.get_blueprint_library().find("sensor.other.collision") # 从 CARLA 蓝图库中找到碰撞传感器的蓝图
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self._on_collision) # 注册碰撞事件回调函数，当发生碰撞时会调用 _on_collision 方法记录事件信息
        actor_list.append(self.sensor)

    def _on_collision(self, event):
        """碰撞事件回调：记录并打印碰撞对象信息"""
        self.history.append(event)
        role_name = event.other_actor.attributes.get("role_name", "--")
        print(
            "检测到碰撞，对象 actor id：{}，role={}，type={}".format(
                event.other_actor.id,
                role_name,
                event.other_actor.type_id,
            )
        )


class DemoCamera:
    """基于 CARLA RGB 相机传感器的图像获取与转换封装，提供 get_surface() 方法返回 Pygame Surface 供显示使用。"""

    def __init__(self, world, vehicle, actor_list, width, height):
        self.surface = None
        self.latest_image = None
        self.latest_size = None
        self.lock = Lock()

        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(width))
        blueprint.set_attribute("image_size_y", str(height))
        blueprint.set_attribute("fov", "90")

        camera_transform = carla.Transform(
            carla.Location(x=-7.0, z=3.2),
            carla.Rotation(pitch=-14.0),
        )
        self.sensor = world.spawn_actor(blueprint, camera_transform, attach_to=vehicle)
        self.sensor.listen(self._on_image)
        actor_list.append(self.sensor)

    def _on_image(self, image):
        with self.lock:
            self.latest_image = bytes(image.raw_data)
            self.latest_size = (image.width, image.height)

    def get_surface(self):
        with self.lock:
            image_bytes = self.latest_image
            image_size = self.latest_size

        if image_bytes is None or image_size is None:
            return self.surface

        image_array = np.frombuffer(image_bytes, dtype=np.uint8)
        image_array = np.reshape(image_array, (image_size[1], image_size[0], 4))
        image_array = image_array[:, :, :3][:, :, ::-1]
        image_array = np.ascontiguousarray(image_array.swapaxes(0, 1))
        self.surface = pygame.surfarray.make_surface(image_array)
        return self.surface


class DemoHUD:
    """基于 Pygame 的 HUD 显示封装，提供 draw() 方法在屏幕上叠加显示仿真状态和车辆信息。
    通过 _format_number() 方法格式化数值显示，处理 None 和无穷大情况。
    """

    def __init__(self, width):
        self.width = width
        self.font = pygame.font.SysFont("consolas", 18)

    @staticmethod
    def _format_number(value, precision=2, fallback="--"):
        if value is None:
            return fallback
        if isinstance(value, float) and not math.isfinite(value):
            return fallback
        return ("{:.%df}" % precision).format(value)

    def draw(self, display, telemetry):
        sim_time = telemetry.get("sim_time")
        state = telemetry.get("state", "--")
        scenario = telemetry.get("scenario", "--")
        ego_speed = telemetry.get("ego_speed")
        lead_speed = telemetry.get("lead_speed")
        front_distance = telemetry.get("front_distance")
        front_ttc = telemetry.get("front_ttc")
        front_actor_role = telemetry.get("front_actor_role", "--")
        front_risk_level = telemetry.get("front_risk_level", 0)
        right_distance = telemetry.get("right_object_distance")
        right_ttc = telemetry.get("right_object_ttc")
        right_object_type = telemetry.get("right_object_type", "--")
        right_risk_level = telemetry.get("right_risk_level", 0)
        steer = telemetry.get("steer")
        throttle = telemetry.get("throttle")
        brake = telemetry.get("brake")
        collision_count = telemetry.get("collision_count", 0)
        lap_distance = telemetry.get("lap_distance")

        risk_labels = {0: "安全", 1: "注意", 2: "警告", 3: "危险"}
        risk_colors = {0: (0, 180, 0), 1: (200, 200, 0), 2: (255, 140, 0), 3: (255, 50, 50)}
        front_role_short = front_actor_role if len(front_actor_role) <= 24 else front_actor_role[:21] + "..."
        right_type_short = right_object_type if right_object_type else "--"
        front_risk_color = risk_colors.get(front_risk_level, (200, 200, 200))
        right_risk_color = risk_colors.get(right_risk_level, (200, 200, 200))

        lines = [
            "t={}s  state={}  scenario={}  collisions={}".format(
                self._format_number(sim_time),
                state,
                scenario,
                collision_count,
            ),
            "ego={}m/s  lead={}m/s  dist={}m  TTC={}s  steer={}  throttle={}  brake={}".format(
                self._format_number(ego_speed, 1),
                self._format_number(lead_speed, 1),
                self._format_number(front_distance, 1),
                self._format_number(front_ttc, 2),
                self._format_number(steer, 2),
                self._format_number(throttle, 2),
                self._format_number(brake, 2),
            ),
            "FRONT: {} | dist={}m  TTC={}s".format(
                front_role_short,
                self._format_number(front_distance, 1),
                self._format_number(front_ttc, 2),
            ),
            "RIGHT: {} | dist={}m  TTC={}s".format(
                right_type_short,
                self._format_number(right_distance, 1),
                self._format_number(right_ttc, 2),
            ),
            "lap_distance={}m / target={}m".format(
                self._format_number(lap_distance, 1),
                self._format_number(telemetry.get("lap_target_distance"), 1),
            ),
        ]

        panel_height = 128
        background = pygame.Surface((self.width, panel_height))
        background.set_alpha(165)
        background.fill((0, 0, 0))
        display.blit(background, (0, 0))

        for index, line in enumerate(lines):
            text_surface = self.font.render(line, True, (255, 255, 255))
            display.blit(text_surface, (12, 8 + index * 24))

        # 用彩色文字叠加风险等级（覆盖在 FRONT/RIGHT 行末尾）
        risk_overlays = [
            (2, risk_labels.get(front_risk_level, "?"), front_risk_color),
            (3, risk_labels.get(right_risk_level, "?"), right_risk_color),
        ]
        for line_idx, label, color in risk_overlays:
            risk_text = self.font.render(label, True, color)
            x_pos = self.width - 72
            y_pos = 8 + line_idx * 24
            display.blit(risk_text, (x_pos, y_pos))


class PygameDemoDisplay:
    """综合封装了相机图像获取和 HUD 显示功能，提供 process_events() 和 render() 方法供主循环调用。"""

    def __init__(self, world, vehicle, actor_list, width=1280, height=720):
        self.enabled = pygame is not None and np is not None
        self.width = width
        self.height = height
        self.display = None
        self.clock = None
        self.camera = None
        self.hud = None

        if not self.enabled:
            print(
                "Animation window disabled: pygame_error={!r}, numpy_error={!r}.".format(
                    pygame_import_error,
                    numpy_import_error,
                )
            )
            return

        pygame.init()
        pygame.font.init()
        self.display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
        pygame.display.set_caption("CARLA Emergency Avoidance Demo")
        self.clock = pygame.time.Clock()
        self.camera = DemoCamera(world, vehicle, actor_list, width, height)
        self.hud = DemoHUD(width)

    def process_events(self):
        if not self.enabled:
            return True

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYUP and event.key in (pygame.K_ESCAPE, pygame.K_q):
                return False
        return True

    def render(self, telemetry):
        if not self.enabled:
            return

        surface = self.camera.get_surface()
        if surface is not None:
            self.display.blit(surface, (0, 0))
        else:
            self.display.fill((0, 0, 0))

        self.hud.draw(self.display, telemetry)
        pygame.display.flip()
        self.clock.tick(0)

    def close(self):
        if self.enabled:
            pygame.quit()


