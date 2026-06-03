import math
import random
import time

import carla

<<<<<<< Updated upstream
from actors import (
    spawn_background_r344_bicycles,
    spawn_background_route_vehicles,
    spawn_right_side_bicycle_crossing,
    spawn_scenario,
)
from config import (
    CLIENT_TIMEOUT,
    EGO_TARGET_SPEED,
    FIXED_DELTA_SECONDS,
    HOST,
    LANE_CHANGE_LENGTH,
    LEAD_BRAKE_TIME,
    LEAD_TARGET_SPEED,
    MAP_NAME,
    PORT,
    RIGHT_OBJECT_DETECT_DISTANCE,
    RIGHT_OBJECT_STOP_DISTANCE,
    RIGHT_OBJECT_TTC_THRESHOLD,
    RIGHT_OBJECT_YIELD_SPEED,
    ROUTE_COMPLETION_HOLD_SECONDS,
    SAFE_DISTANCE,
    SIM_SECONDS,
    TRAFFIC_RANDOM_SEED,
    TTC_AVOID_THRESHOLD,
    TTC_BRAKE_THRESHOLD,
)
from control import QuinticLaneChangeTrajectory, SamplingMPCTracker
from display import CollisionMonitor, PygameDemoDisplay
from perception import VirtualGroundTruthSensor
from route import LoopRoute
from utils import get_speed, speed_control, waypoint_steer
=======
try:
    import pygame
except ImportError:
    pygame = None

try:
    import numpy as np
except ImportError:
    np = None


# ===================== 场景全局配置 =====================
HOST = "localhost"          # CARLA 服务器地址
PORT = 2000                 # CARLA 服务器端口
MAP_NAME = "Town10HD_Opt"   # 使用 Town10HD_Opt 城市地图，便于后续加入人行横道/非机动车场景
CLIENT_TIMEOUT = 120.0      # 客户端连接超时时间（秒）
FIXED_DELTA_SECONDS = 0.05  # 同步仿真步长（20Hz）
SIM_SECONDS = 90.0          # 最大仿真时长（秒），Town10 一圈比 Town04 直道场景更长

INITIAL_GAP = 48.0          # 自车与前车的初始纵向间距（米）
LEAD_BRAKE_TIME = 6.0       # 前车开始紧急制动的仿真时刻（秒）
EGO_TARGET_SPEED = 15.5     # 自车期望行驶速度（m/s）
LEAD_TARGET_SPEED = 13.0    # 前车期望行驶速度（m/s）

TTC_BRAKE_THRESHOLD = 4.5   # TTC低于此值时触发辅助制动（秒）
TTC_AVOID_THRESHOLD = 3.6   # TTC低于此值时触发紧急换道（秒）
SAFE_DISTANCE = 34.0        # 换道触发所需的最小安全距离（米）
LANE_CLEAR_FRONT = 45.0     # 判断邻道净空：前方检测距离（米）
LANE_CLEAR_REAR = 18.0      # 判断邻道净空：后方检测距离（米）

LANE_CHANGE_LENGTH = 28.0   # 换道轨迹的纵向长度（米）
MPC_HORIZON_STEPS = 18      # MPC 预测时域步数
MPC_DT = 0.10               # MPC 每步时间间隔（秒）
WHEEL_BASE = 2.85           # 车辆轴距（米，用于自行车模型）

TOWN10_START_SPAWN_INDEX = 141 # Town10HD_Opt 的固定起始点在排序后的生成点列表中的索引位置，确保每次运行都从同一位置开始，便于结果对比和调试
TOWN10_ROUTE_STEP = 4.0
TOWN10_ROUTE_CLOSE_RADIUS = 8.0
TOWN10_ROUTE_MIN_POINTS_BEFORE_CLOSE = 80
TOWN10_ROUTE_SELF_CLOSE_MIN_SEPARATION = 55
TOWN10_ROUTE_CLOSE_HEADING_DEGREES = 25.0
TOWN10_SHORT_LOOP_BRANCH_OVERRIDES = {
    (5, -1): 795,  # 走 Town10 中间连接路，避免继续绕外侧大圈。
    (13, -2): 934, # 第二个路口转弯后继续直行，跳过转弯后的第一个小路口。
    (20, -2): 875, # 再次转弯后跳过第一个路口，到第二个路口再右转。
}
TOWN10_RIGHT_TURN_PREPARE_LANE_CHANGES = {
    (1, 1),  # 第一个大弯结束后的直道上，为后续十字路口右转提前进入同向右侧车道。
}
TOWN10_RIGHT_TURN_PREPARE_MAX_X = 56.0
TOWN10_RIGHT_TURN_PREPARE_HEADING_DEGREES = 180.0
TOWN10_RIGHT_TURN_PREPARE_HEADING_TOLERANCE = 15.0
ROUTE_COMPLETION_HOLD_SECONDS = 4.0

RIGHT_OBJECT_TTC_THRESHOLD = 5.0
RIGHT_OBJECT_DETECT_DISTANCE = 34.0
RIGHT_OBJECT_STOP_DISTANCE = 13.0
RIGHT_OBJECT_YIELD_SPEED = 3.0
RIGHT_OBJECT_TRIGGER_ROAD = 344
RIGHT_OBJECT_EXIT_ROAD = 20
RIGHT_OBJECT_TRIGGER_ROUTE_STEPS = 13
RIGHT_OBJECT_CLEAR_ROUTE_STEPS = 14
RIGHT_OBJECT_CROSSING_SPEED = 3.8
RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS = 6
RIGHT_OBJECT_R344_RIGHT_OFFSET = 3.0
RIGHT_OBJECT_R344_START_FORWARD_OFFSET = -8.0
RIGHT_OBJECT_R344_END_FORWARD_OFFSET = 22.0

TRAFFIC_RANDOM_SEED = 20260602
SLOW_RIGHT_LANE_VEHICLE_DISTANCE = 85.0
SLOW_RIGHT_LANE_VEHICLE_SPEED = 5.0
SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME = "slow_right_lane_vehicle"
NONMOTOR_TRIGGER_DISTANCE = 5.0
NONMOTOR_WAIT_AT_INTERSECTION = True
NONMOTOR_START_CLUSTER_NEAR_JUNCTION = True
NONMOTOR_VISUAL_PHYSICS_EXPERIMENT = False
BACKGROUND_VEHICLE_ROUTE_INDICES = (32, 54, 76, 96, 108)
BACKGROUND_VEHICLE_SPEED_MIN = 7.0
BACKGROUND_VEHICLE_SPEED_MAX = 8.8
BACKGROUND_BICYCLE_FORWARD_OFFSETS = (-42.0, -30.0, -18.0)
BACKGROUND_BICYCLE_RIGHT_OFFSETS = (4.4, 1.8, 5.6)
BACKGROUND_BICYCLE_END_FORWARD_OFFSET = 36.0
BACKGROUND_BICYCLE_SPEED_MIN = 2.6
BACKGROUND_BICYCLE_SPEED_MAX = 4.3
BACKGROUND_NONMOTOR_TYPES = ("bicycle", "pedestrian", "pedestrian")
VISUAL_TRAFFIC_ROUTE_INDICES = (44, 66, 86, 116, 136)
VISUAL_TRAFFIC_SPEED_MIN = 6.0
VISUAL_TRAFFIC_SPEED_MAX = 9.5
VISUAL_TRAFFIC_ROLE_PREFIX = "visual_opposite_vehicle"


# ===================== 通用工具函数 =====================

def clamp(value, low, high):
    """将 value 限制在 [low, high] 范围内"""
    return max(low, min(high, value))


def vector_length(vector):
    """计算三维向量的欧几里得长度"""
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def dot_2d(a, b):
    """计算两个向量在水平面（XY）上的点积"""
    return a.x * b.x + a.y * b.y


def normalize_angle(angle):
    """将任意角度归一化到 (-π, π] 区间"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_rad(rotation):
    """将 CARLA Rotation 的偏航角（度）转换为弧度"""
    return math.radians(rotation.yaw)


def get_speed(vehicle):
    """获取车辆当前速度的标量值（m/s）"""
    return vector_length(vehicle.get_velocity())


def speed_control(current_speed, target_speed):
    """简单比例速度控制器，返回 (油门, 制动) 元组"""
    error = target_speed - current_speed
    if error >= 0.0:
        return clamp(0.18 + 0.06 * error, 0.0, 0.75), 0.0
    return 0.0, clamp(-0.12 * error, 0.0, 0.75)


def waypoint_steer(vehicle, carla_map, lookahead=12.0):
    """基于前视路点的纯追踪转向控制，返回归一化转向量"""
    waypoint = carla_map.get_waypoint(
        vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
    )
    next_waypoints = waypoint.next(lookahead)
    if not next_waypoints:
        return 0.0

    target = next_waypoints[0].transform.location
    transform = vehicle.get_transform()
    dx = target.x - transform.location.x
    dy = target.y - transform.location.y
    target_yaw = math.atan2(dy, dx)
    heading_error = normalize_angle(target_yaw - yaw_to_rad(transform.rotation))
    return clamp(1.8 * heading_error, -0.45, 0.45)


def vehicle_transform_from_waypoint(waypoint):
    """从路点生成车辆生成位置，Z轴抬高0.45m避免穿地"""
    transform = waypoint.transform
    transform.location.z += 0.45
    return transform


def same_direction_lane(source_wp, target_wp):
    """判断目标路点是否为与源路点同向的行驶车道"""
    if target_wp is None or target_wp.lane_type != carla.LaneType.Driving:
        return False
    yaw_error = abs(normalize_angle(yaw_to_rad(source_wp.transform.rotation) - yaw_to_rad(target_wp.transform.rotation)))
    return yaw_error < math.radians(30.0)


def set_actor_target_velocity_safe(actor, velocity):
    try:
        actor.set_target_velocity(velocity)
    except (AttributeError, RuntimeError):
        pass


def get_rightmost_same_direction_lane(waypoint):
    """寻找给定路点右侧最外层的同向 Driving 车道。"""
    current = waypoint
    while True:
        right_lane = current.get_right_lane()
        if not same_direction_lane(current, right_lane):
            return current
        current = right_lane


def get_town10_start_waypoint(carla_map):
    """获取 Town10 固定起点，确保每次仿真从同一位置开始。"""
    spawn_points = sorted(
        carla_map.get_spawn_points(),
        key=lambda t: (round(t.location.x, 1), round(t.location.y, 1), round(t.rotation.yaw, 1)),
    )

    if TOWN10_START_SPAWN_INDEX >= len(spawn_points):
        raise RuntimeError("Town10 fixed spawn index is out of range.")

    transform = spawn_points[TOWN10_START_SPAWN_INDEX]
    waypoint = carla_map.get_waypoint(
        transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    print(
        "Town10 fixed loop start: sorted_spawn_index={}, location=({:.1f}, {:.1f}), road={}, lane={}".format(
            TOWN10_START_SPAWN_INDEX,
            waypoint.transform.location.x,
            waypoint.transform.location.y,
            waypoint.road_id,
            waypoint.lane_id,
        )
    )
    return waypoint


# ===================== 传感器模块 =====================

@dataclass
class FrontVehicleReading: # 定义类
    """前车感知数据结构"""
    distance: float       # 纵向间距（米）
    closing_speed: float  # 接近速度（m/s，正值表示靠近）
    ttc: float            # 碰撞时间（秒）
    lateral_offset: float # 横向偏移（米）
    is_front_vehicle: bool  # 是否确认为正前方车辆


@dataclass
class RightSideObjectReading:
    """右侧非机动车/行人目标的虚拟感知数据。"""
    distance: float
    ttc: float
    is_conflict_object: bool


class VirtualGroundTruthSensor:
    """虚拟真值传感器：直接从仿真引擎读取精确状态，供决策与控制使用。
    注意：此传感器无噪声和延迟，仅用于算法验证阶段，后续可替换为雷达/激光雷达感知。
    """

    def __init__(
        self,
        world,
        carla_map,
        ego_vehicle,
        lead_vehicle,
        right_object_scenario=None,
        front_extra_vehicles=None,
        right_object_scenarios=None,
    ):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle
        self.front_extra_vehicles = front_extra_vehicles or []
        self.right_object_scenarios = []
        if right_object_scenario is not None:
            self.right_object_scenarios.append(right_object_scenario)
        self.right_object_scenarios.extend(right_object_scenarios or [])

    def front_vehicle(self):
        """计算同车道前方最近车辆的纵向距离、横向偏移、接近速度和TTC。"""
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        lane_width = self.carla_map.get_waypoint(ego_loc).lane_width
        ego_speed_along = dot_2d(self.ego.get_velocity(), forward)

        closest = FrontVehicleReading(float("inf"), 0.0, float("inf"), 0.0, False)
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue

            relative = vehicle.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)
            lateral = dot_2d(relative, right)
            if longitudinal <= 0.0 or abs(lateral) >= lane_width * 0.65:
                continue

            target_speed_along = dot_2d(vehicle.get_velocity(), forward)
            closing_speed = ego_speed_along - target_speed_along
            ttc = longitudinal / closing_speed if closing_speed > 0.1 else float("inf")
            if longitudinal < closest.distance:
                closest = FrontVehicleReading(longitudinal, closing_speed, ttc, lateral, True)

        return closest

    def lane_clear(self, side):
        """检测指定侧邻道在前后安全范围内是否无车"""
        ego_wp = self.carla_map.get_waypoint(
            self.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        target_wp = ego_wp.get_left_lane() if side == "left" else ego_wp.get_right_lane()
        if not same_direction_lane(ego_wp, target_wp):
            return False  # 邻道不存在或方向不同

        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()

        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.id == self.ego.id:
                continue  # 跳过自车本身

            actor_wp = self.carla_map.get_waypoint(
                actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
            )
            if actor_wp.road_id != target_wp.road_id or actor_wp.lane_id != target_wp.lane_id:
                continue  # 不在目标车道上，跳过

            relative = actor.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)
            if -LANE_CLEAR_REAR <= longitudinal <= LANE_CLEAR_FRONT:
                return False  # 邻道安全范围内有车，不可换道

        return True

    def right_side_object(self, route_index):
        """读取右侧非机动车目标，并判断其是否位于当前右转冲突窗口。"""
        best_conflict = None
        best_nearby = None
        for scenario in self.right_object_scenarios:
            reading = self._right_side_object_reading(scenario, route_index)
            if reading is None:
                continue
            if best_nearby is None or reading.distance < best_nearby.distance:
                best_nearby = reading
            if reading.is_conflict_object and (
                best_conflict is None
                or reading.ttc < best_conflict.ttc
                or reading.distance < best_conflict.distance
            ):
                best_conflict = reading

        if best_conflict is not None:
            return best_conflict
        if best_nearby is not None:
            return best_nearby
        return RightSideObjectReading(float("inf"), float("inf"), False)

    def _right_side_object_reading(self, scenario, route_index):
        if scenario is None or scenario.actor is None:
            return None

        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        actor_loc = scenario.actor.get_location()
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        relative = actor_loc - ego_loc

        longitudinal = dot_2d(relative, forward)
        lateral = dot_2d(relative, right)
        distance = vector_length(relative)

        to_object_length = max(distance, 0.1)
        to_object = carla.Vector3D(relative.x / to_object_length, relative.y / to_object_length, 0.0)
        ego_velocity = self.ego.get_velocity()
        relative_speed = carla.Vector3D(
            ego_velocity.x - scenario.velocity.x,
            ego_velocity.y - scenario.velocity.y,
            ego_velocity.z - scenario.velocity.z,
        )
        closing_speed = dot_2d(relative_speed, to_object)
        ttc = distance / closing_speed if closing_speed > 0.1 else float("inf")

        in_geometry_gate = -8.0 <= longitudinal <= 34.0 and -14.0 <= lateral <= 18.0
        is_conflict = (
            scenario.is_active
            and scenario.is_conflict_window(route_index)
            and in_geometry_gate
        )
        return RightSideObjectReading(distance, ttc, is_conflict)


# ===================== 换道轨迹规划 =====================

class QuinticLaneChangeTrajectory:
    """五次多项式换道轨迹：d(s) = D * (10t³ - 15t⁴ + 6t⁵)，t=s/L。
    保证起止点的位移、速度、加速度均为零，轨迹平滑。
    """

    def __init__(self, start_transform, lateral_offset, length):
        """初始化换道轨迹
        参数：
            start_transform: 换道起点的车辆坐标变换
            lateral_offset:  目标侧向偏移量（负值为左换道）
            length:          换道纵向总长度（米）
        """
        self.origin = start_transform.location
        self.start_yaw = yaw_to_rad(start_transform.rotation)
        self.forward = start_transform.get_forward_vector()
        self.right = start_transform.get_right_vector()
        self.lateral_offset = lateral_offset
        self.length = length

    def to_local(self, location):
        """将全局坐标转换为以起点为原点的局部纵横坐标 (s, d)"""
        relative = location - self.origin
        return dot_2d(relative, self.forward), dot_2d(relative, self.right)

    def lateral_at(self, s):
        """计算纵向位置 s 处的目标横向偏移量"""
        if s <= 0.0:
            return 0.0
        if s >= self.length:
            return self.lateral_offset
        tau = s / self.length
        blend = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        return self.lateral_offset * blend

    def lateral_slope_at(self, s):
        """计算纵向位置 s 处的轨迹横向斜率（用于计算参考航向角）"""
        if s <= 0.0 or s >= self.length:
            return 0.0
        tau = s / self.length
        blend_dot = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        return self.lateral_offset * blend_dot / self.length


# ===================== MPC 轨迹跟踪控制器 =====================

class SamplingMPCTracker:
    """基于采样的后退时域MPC控制器，使用运动学自行车模型进行滚动优化。
    通过枚举转向角和加速度候选值，选取预测代价最小的控制动作。
    """

    def __init__(self):
        self.previous_steer = 0.0  # 上一帧的转向量，用于连续性惩罚

    def control(self, ego_vehicle, trajectory, target_speed):
        """计算当前帧的最优控制指令
        返回：carla.VehicleControl（油门、制动、转向）
        """
        transform = ego_vehicle.get_transform()
        s0, d0 = trajectory.to_local(transform.location)  # 当前局部纵横坐标
        yaw0 = normalize_angle(yaw_to_rad(transform.rotation) - trajectory.start_yaw)  # 相对航向角
        v0 = get_speed(ego_vehicle)  # 当前速度

        # 候选转向角集合（共9个离散值）
        steer_candidates = [
            clamp(self.previous_steer + delta, -0.65, 0.65)
            for delta in (-0.45, -0.32, -0.20, -0.10, 0.0, 0.10, 0.20, 0.32, 0.45)
        ]
        # 候选加速度集合（共5个离散值，单位 m/s²）
        accel_candidates = (-4.0, -2.0, -1.0, 0.0, 1.0)

        best_cost = float("inf")
        best_action = (0.0, -3.0)  # 默认保守制动动作

        for steer in steer_candidates:
            for accel in accel_candidates:
                s = s0
                d = d0
                yaw = yaw0
                speed = v0
                cost = 0.0

                # 沿预测时域逐步积分代价
                for step in range(MPC_HORIZON_STEPS):
                    speed = max(0.0, speed + accel * MPC_DT)
                    s += speed * math.cos(yaw) * MPC_DT
                    d += speed * math.sin(yaw) * MPC_DT
                    yaw = normalize_angle(yaw + speed / WHEEL_BASE * math.tan(steer) * MPC_DT)

                    ref_d = trajectory.lateral_at(s)          # 参考横向偏移
                    ref_yaw = math.atan(trajectory.lateral_slope_at(s))  # 参考航向角
                    lateral_error = d - ref_d                  # 横向跟踪误差
                    yaw_error = normalize_angle(yaw - ref_yaw) # 航向误差
                    speed_error = speed - target_speed         # 速度误差

                    cost += 6.0 * lateral_error**2    # 横向误差惩罚
                    cost += 1.7 * yaw_error**2        # 航向误差惩罚
                    cost += 0.07 * speed_error**2     # 速度误差惩罚
                    cost += 0.08 * steer**2           # 转向幅度惩罚
                    cost += 0.01 * accel**2           # 加速度幅度惩罚
                    cost += 0.02 * step * abs(steer - self.previous_steer)  # 转向连续性惩罚

                if cost < best_cost:
                    best_cost = cost
                    best_action = (steer, accel)

        steer, accel = best_action
        self.previous_steer = steer  # 保存本帧转向量供下帧使用

        # 将加速度映射为油门/制动量
        if accel >= 0.0:
            throttle = clamp(0.25 + 0.18 * accel, 0.0, 0.65)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp(-accel / 7.5, 0.0, 1.0)

        return carla.VehicleControl(throttle=throttle, brake=brake, steer=steer)


# ===================== 碰撞监测传感器 =====================

class CollisionMonitor:
    """CARLA 内置碰撞传感器封装，实时记录自车碰撞事件"""

    def __init__(self, world, vehicle, actor_list):
        self.history = []  # 碰撞事件历史列表
        blueprint = world.get_blueprint_library().find("sensor.other.collision")
        self.sensor = world.spawn_actor(blueprint, carla.Transform(), attach_to=vehicle)
        self.sensor.listen(self._on_collision)
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
        right_distance = telemetry.get("right_object_distance")
        right_ttc = telemetry.get("right_object_ttc")
        steer = telemetry.get("steer")
        throttle = telemetry.get("throttle")
        brake = telemetry.get("brake")
        collision_count = telemetry.get("collision_count", 0)
        lap_distance = telemetry.get("lap_distance")

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
            "right_object_dist={}m  right_object_TTC={}s".format(
                self._format_number(right_distance, 1),
                self._format_number(right_ttc, 2),
            ),
            "lap_distance={}m / target={}m".format(
                self._format_number(lap_distance, 1),
                self._format_number(telemetry.get("lap_target_distance"), 1),
            ),
        ]

        panel_height = 104
        background = pygame.Surface((self.width, panel_height))
        background.set_alpha(165)
        background.fill((0, 0, 0))
        display.blit(background, (0, 0))

        for index, line in enumerate(lines):
            text_surface = self.font.render(line, True, (255, 255, 255))
            display.blit(text_surface, (12, 8 + index * 24))


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
            print("pygame or numpy is not installed; running without animation window.")
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

>>>>>>> Stashed changes

# ===================== 仿真世界初始化 =====================

def setup_world(client):
    """加载指定地图并启用同步模式，返回配置好的世界对象"""
    world = client.get_world()
    current_map = world.get_map().name
    if MAP_NAME not in current_map:
        print("正在加载地图 {}...原地图: {}".format(MAP_NAME, current_map))
        for attempt in range(3):
            try:
                world = client.load_world(MAP_NAME)
                break
            except RuntimeError as exc:
                if attempt == 2:
                    raise
                print("地图加载失败（第{}次）: {}，正在重试...".format(attempt + 1, exc))
                time.sleep(2.0)
    else:
        print("使用已加载地图 {}。".format(current_map))

    # 启用同步模式并设置固定步长
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)
    return world


def restore_world(world, original_settings):
    """仿真结束后恢复世界为异步模式，不影响其他程序使用CARLA"""
    world.apply_settings(original_settings)


def set_spectator(world, ego_vehicle):
    """将观察者视角定位到自车正上方，仿真中提供上帝视角"""
    ego_tf = ego_vehicle.get_transform()
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            ego_tf.location + carla.Location(z=45.0),
            carla.Rotation(pitch=-75.0, yaw=ego_tf.rotation.yaw),
        )
    )


def choose_avoidance_side(sensor):
    """根据邻道净空状况选择换道方向：优先左转，其次右转，无道则返回 None"""
    if sensor.lane_clear("left"):
        return "left"
    if sensor.lane_clear("right"):
        return "right"
    return None


<<<<<<< Updated upstream
=======
class RightSideBicycleCrossing:
    """R344 -> R20 右转路口的右侧非机动车横穿目标。"""

    def __init__(
        self,
        actor,
        start_location,
        end_location,
        trigger_index,
        clear_index,
        speed,
        trigger_location=None,
        trigger_distance=None,
    ):
        self.actor = actor
        self.name = actor.attributes.get("role_name", "right_side_bicycle")
        self.start_location = start_location
        self.end_location = end_location
        self.trigger_index = trigger_index
        self.clear_index = clear_index
        self.speed = speed
        self.trigger_location = trigger_location
        self.trigger_distance = trigger_distance
        self.progress = 0.0
        self.is_active = False
        self.is_finished = False

        dx = end_location.x - start_location.x
        dy = end_location.y - start_location.y
        self.length = max(math.sqrt(dx * dx + dy * dy), 0.1)
        self.yaw = math.degrees(math.atan2(dy, dx))
        self.velocity = carla.Vector3D(dx / self.length * speed, dy / self.length * speed, 0.0)
        self._set_location(start_location)

    def _set_location(self, location):
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=self.yaw)))

    def _should_start(self, route_index, ego_vehicle):
        if self.trigger_location is not None and self.trigger_distance is not None and ego_vehicle is not None:
            return ego_vehicle.get_location().distance(self.trigger_location) <= self.trigger_distance
        return route_index >= self.trigger_index

    def update(self, route_index, dt, ego_vehicle=None):
        if self.is_finished:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            set_actor_target_velocity_safe(self.actor, self.velocity)
            return

        if not self.is_active and self._should_start(route_index, ego_vehicle):
            self.is_active = True
            print(
                "{} started: trigger_index={}, clear_index={}, trigger_distance={}.".format(
                    self.name,
                    self.trigger_index,
                    self.clear_index,
                    self.trigger_distance,
                )
            )

        if not self.is_active:
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            set_actor_target_velocity_safe(self.actor, self.velocity)
            self._set_location(self.start_location)
            return

        self.progress = min(self.length, self.progress + self.speed * dt)
        ratio = self.progress / self.length
        current_location = carla.Location(
            x=self.start_location.x + (self.end_location.x - self.start_location.x) * ratio,
            y=self.start_location.y + (self.end_location.y - self.start_location.y) * ratio,
            z=self.start_location.z + (self.end_location.z - self.start_location.z) * ratio,
        )
        self.velocity = carla.Vector3D(
            (self.end_location.x - self.start_location.x) / self.length * self.speed,
            (self.end_location.y - self.start_location.y) / self.length * self.speed,
            0.0,
        )
        self._set_location(current_location)
        set_actor_target_velocity_safe(self.actor, self.velocity)

        if self.progress >= self.length:
            self.is_finished = True
            self.is_active = False
            self.velocity = carla.Vector3D(0.0, 0.0, 0.0)
            set_actor_target_velocity_safe(self.actor, self.velocity)
            print("{} finished crossing.".format(self.name))

    def is_conflict_window(self, route_index):
        return self.trigger_index <= route_index <= self.clear_index and not self.is_finished


class BackgroundRouteVehicle:
    """沿自车同一条固定路线行驶的背景车辆，仅改变初始位置和速度。"""

    def __init__(self, actor, target_speed, start_index, loop_route):
        self.actor = actor
        self.target_speed = target_speed
        self.loop_route = loop_route
        self.route_span = max(loop_route.step_distance, (len(loop_route.points) - 1) * loop_route.step_distance)
        self.progress = clamp(start_index, 0, len(loop_route.points) - 2) * loop_route.step_distance
        self.z_offset = actor.get_location().z - loop_route.points[int(self.progress / loop_route.step_distance)].z
        self.update(0.0)

    def update(self, dt):
        self.progress = (self.progress + self.target_speed * dt) % self.route_span
        route_position = self.progress / self.loop_route.step_distance
        low_index = min(int(route_position), len(self.loop_route.points) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.loop_route.points[low_index]
        end = self.loop_route.points[high_index]
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z

        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


class RightLaneSlowVehicleController:
    """沿最右侧同向车道慢速行驶的交通车。"""

    def __init__(self, actor, lane_waypoints, target_speed, step_distance):
        if len(lane_waypoints) < 2:
            raise RuntimeError("RightLaneSlowVehicleController requires at least two waypoints.")
        self.actor = actor
        self.lane_waypoints = lane_waypoints
        self.target_speed = target_speed
        self.step_distance = step_distance
        self.route_span = max(step_distance, (len(lane_waypoints) - 1) * step_distance)
        self.progress = 0.0
        self.z_offset = actor.get_location().z - lane_waypoints[0].transform.location.z
        self.update(0.0)

    def update(self, dt):
        self.progress = min(self.route_span, self.progress + self.target_speed * dt)
        route_position = self.progress / self.step_distance
        low_index = min(int(route_position), len(self.lane_waypoints) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.lane_waypoints[low_index].transform.location
        end = self.lane_waypoints[high_index].transform.location
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z

        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


class VisualLaneVehicleController:
    """沿非主路线车道行驶的纯视觉交通车。"""

    def __init__(self, actor, lane_waypoints, target_speed, step_distance):
        self.actor = actor
        self.lane_waypoints = lane_waypoints
        self.target_speed = target_speed
        self.step_distance = step_distance
        self.route_span = max(step_distance, (len(lane_waypoints) - 1) * step_distance)
        self.progress = 0.0
        self.z_offset = actor.get_location().z - lane_waypoints[0].transform.location.z
        self.update(0.0)

    def update(self, dt):
        self.progress = (self.progress + self.target_speed * dt) % self.route_span
        route_position = self.progress / self.step_distance
        low_index = min(int(route_position), len(self.lane_waypoints) - 2)
        high_index = low_index + 1
        ratio = route_position - low_index

        start = self.lane_waypoints[low_index].transform.location
        end = self.lane_waypoints[high_index].transform.location
        dx = end.x - start.x
        dy = end.y - start.y
        dz = end.z - start.z

        location = carla.Location(
            x=start.x + dx * ratio,
            y=start.y + dy * ratio,
            z=start.z + dz * ratio + self.z_offset,
        )
        yaw = math.degrees(math.atan2(dy, dx))
        self.actor.set_transform(carla.Transform(location, carla.Rotation(yaw=yaw)))

        segment_length = math.sqrt(dx * dx + dy * dy)
        if segment_length > 0.001:
            velocity = carla.Vector3D(
                x=dx / segment_length * self.target_speed,
                y=dy / segment_length * self.target_speed,
                z=0.0,
            )
            self.actor.set_target_velocity(velocity)


def build_right_lane_waypoint_path(start_waypoint, step_distance, max_steps=80):
    """沿同向车道向前构建慢车行驶路点序列。"""
    waypoints = [start_waypoint]
    waypoint = start_waypoint
    for _ in range(max_steps):
        next_waypoints = waypoint.next(step_distance)
        if not next_waypoints:
            break
        same_direction_next = [
            candidate for candidate in next_waypoints
            if same_direction_lane(waypoint, candidate)
        ]
        if not same_direction_next:
            break
        waypoint = same_direction_next[0]
        waypoints.append(waypoint)
    return waypoints


class LoopRoute:
    """生成 Town10 固定短路线，供纯追踪控制使用。

    当前路线保留起点和直道急停避障段，在左侧路口走中间连接路，
    避免继续绕 Town10 外侧大圈；末段先进入右侧车道，再完成右转并闭合到已经过的北向道路。
    """

    def __init__(
        self,
        start_waypoint,
        step_distance=TOWN10_ROUTE_STEP,
        close_radius=TOWN10_ROUTE_CLOSE_RADIUS,
    ):
        self.step_distance = step_distance
        self.close_radius = close_radius
        self.waypoints = []
        self.points = []
        self.turn_events = []
        self.close_to_index = None
        self.right_lane_prepare_index = None
        self.completed = False
        self.max_index = 0
        self.last_index = 0

        self._build_short_town10_route(start_waypoint)

        if len(self.points) < 60:
            raise RuntimeError("Failed to build a usable Town10 short loop route.")

        self.length = (len(self.points) - 1) * step_distance
        self.turn_events = self._detect_turn_events()
        self.right_turn_count = len([event for event in self.turn_events if event["direction"] == "right"])
        self.right_lane_before_turn = self.right_lane_prepare_index is not None

    def _select_next_waypoint(self, waypoint, next_waypoints):
        preferred_road = TOWN10_SHORT_LOOP_BRANCH_OVERRIDES.get((waypoint.road_id, waypoint.lane_id))
        if preferred_road is not None:
            for candidate in next_waypoints:
                if candidate.road_id == preferred_road:
                    return candidate
        return next_waypoints[0]

    def _try_prepare_right_lane(self, waypoint):
        if self.right_lane_prepare_index is not None:
            return waypoint
        if (waypoint.road_id, waypoint.lane_id) not in TOWN10_RIGHT_TURN_PREPARE_LANE_CHANGES:
            return waypoint

        location = waypoint.transform.location
        heading_error = abs(
            normalize_angle(
                math.radians(waypoint.transform.rotation.yaw - TOWN10_RIGHT_TURN_PREPARE_HEADING_DEGREES)
            )
        )
        if (
            location.x > TOWN10_RIGHT_TURN_PREPARE_MAX_X
            or heading_error > math.radians(TOWN10_RIGHT_TURN_PREPARE_HEADING_TOLERANCE)
        ):
            return waypoint

        right_waypoint = waypoint.get_right_lane()
        if not same_direction_lane(waypoint, right_waypoint):
            return waypoint

        self.right_lane_prepare_index = len(self.points)
        self.waypoints.append(right_waypoint)
        self.points.append(right_waypoint.transform.location)
        return right_waypoint

    def _find_self_close_index(self, waypoint):
        if len(self.points) <= TOWN10_ROUTE_MIN_POINTS_BEFORE_CLOSE:
            return None

        location = waypoint.transform.location
        yaw = waypoint.transform.rotation.yaw
        search_end = len(self.points) - TOWN10_ROUTE_SELF_CLOSE_MIN_SEPARATION
        for index in range(max(0, search_end)):
            candidate = self.waypoints[index]
            yaw_error = abs(normalize_angle(math.radians(yaw - candidate.transform.rotation.yaw)))
            if (
                location.distance(candidate.transform.location) <= self.close_radius
                and yaw_error <= math.radians(TOWN10_ROUTE_CLOSE_HEADING_DEGREES)
            ):
                return index
        return None

    def _build_short_town10_route(self, start_waypoint):
        waypoint = start_waypoint
        self.waypoints.append(start_waypoint)
        self.points.append(start_waypoint.transform.location)

        for _ in range(800):
            waypoint = self._try_prepare_right_lane(waypoint)
            next_waypoints = waypoint.next(self.step_distance)
            if not next_waypoints:
                break

            waypoint = self._select_next_waypoint(waypoint, next_waypoints)
            location = waypoint.transform.location
            self.waypoints.append(waypoint)
            self.points.append(location)

            self.close_to_index = self._find_self_close_index(waypoint)
            if self.close_to_index is not None:
                break

    def _detect_turn_events(self, min_total_degrees=50.0):
        events = []
        current_direction = None
        current_total = 0.0
        start_index = 0
        last_turn_index = 0
        straight_steps = 0

        for index in range(1, len(self.waypoints)):
            previous_yaw = self.waypoints[index - 1].transform.rotation.yaw
            current_yaw = self.waypoints[index].transform.rotation.yaw
            delta = math.degrees(normalize_angle(math.radians(current_yaw - previous_yaw)))
            if abs(delta) < 2.0:
                if current_direction is not None:
                    straight_steps += 1
                    if straight_steps >= 5:
                        if abs(current_total) >= min_total_degrees:
                            events.append(
                                self._make_turn_event(
                                    current_direction, current_total, start_index, last_turn_index
                                )
                            )
                        current_direction = None
                        current_total = 0.0
                continue

            straight_steps = 0
            last_turn_index = index
            # CARLA/Unreal yaw increases clockwise in the XY plane, so positive yaw change is a right turn.
            direction = "right" if delta > 0.0 else "left"
            if direction != current_direction:
                if current_direction is not None and abs(current_total) >= min_total_degrees:
                    events.append(self._make_turn_event(current_direction, current_total, start_index, index - 1))
                current_direction = direction
                current_total = delta
                start_index = index - 1
            else:
                current_total += delta

        if current_direction is not None and abs(current_total) >= min_total_degrees:
            events.append(self._make_turn_event(current_direction, current_total, start_index, len(self.waypoints) - 1))

        return events

    def _make_turn_event(self, direction, total_degrees, start_index, end_index):
        return {
            "direction": direction,
            "degrees": total_degrees,
            "start_index": start_index,
            "end_index": end_index,
        }

    def _nearest_index(self, location, anchor_index=None, search_back=5, search_ahead=45):
        if anchor_index is not None:
            start_index = max(0, anchor_index - search_back)
            end_index = min(len(self.points), anchor_index + search_ahead)
            if end_index > start_index:
                return min(
                    range(start_index, end_index),
                    key=lambda index: self.points[index].distance(location),
                )

        return min(
            range(len(self.points)),
            key=lambda index: self.points[index].distance(location),
        )

    def steer(self, vehicle, lookahead=14.0):
        location = vehicle.get_location()
        nearest = self._nearest_index(location, self.last_index)
        lookahead_steps = max(2, int(lookahead / self.step_distance))
        target_index = min(nearest + lookahead_steps, len(self.points) - 1)
        target = self.points[target_index]

        transform = vehicle.get_transform()
        dx = target.x - transform.location.x
        dy = target.y - transform.location.y
        target_yaw = math.atan2(dy, dx)
        heading_error = normalize_angle(target_yaw - yaw_to_rad(transform.rotation))
        return clamp(1.8 * heading_error, -0.45, 0.45)

    def update(self, vehicle):
        location = vehicle.get_location()
        nearest = self._nearest_index(location, self.last_index)
        self.last_index = nearest
        self.max_index = max(self.max_index, nearest)

        if self.max_index >= len(self.points) - 8:
            self.completed = True

        return self.completed

    @property
    def progress_distance(self):
        return min(self.length, self.max_index * self.step_distance)


def find_route_transition_index(loop_route, from_road, to_road):
    """在路线中查找从 from_road 进入 to_road 的第一个路点索引。"""
    for index in range(1, len(loop_route.waypoints)):
        if (
            loop_route.waypoints[index - 1].road_id == from_road
            and loop_route.waypoints[index].road_id == to_road
        ):
            return index
    for index, waypoint in enumerate(loop_route.waypoints):
        if waypoint.road_id == to_road:
            return index
    return None


def find_nonmotor_blueprint(blueprint_library):
    """优先选择自行车蓝图；不可用时使用摩托车或普通车辆替代。"""
    preferred_ids = (
        "vehicle.bh.crossbike",
        "vehicle.diamondback.century",
        "vehicle.gazelle.omafiets",
        "vehicle.yamaha.yzf",
        "vehicle.kawasaki.ninja",
    )
    for blueprint_id in preferred_ids:
        try:
            return blueprint_library.find(blueprint_id)
        except (IndexError, RuntimeError):
            continue

    vehicles = blueprint_library.filter("vehicle.*")
    if not vehicles:
        raise RuntimeError("No vehicle blueprint is available for the right-side object scenario.")
    return vehicles[0]


def find_pedestrian_blueprint(blueprint_library):
    walkers = blueprint_library.filter("walker.pedestrian.*")
    if not walkers:
        raise RuntimeError("No pedestrian blueprint is available for the walking crossing scenario.")
    return walkers[0]


def opposite_direction_lane(source_wp):
    for lane_getter in (source_wp.get_left_lane, source_wp.get_right_lane):
        candidate = lane_getter()
        if candidate is None or candidate.lane_type != carla.LaneType.Driving:
            continue
        yaw_error = abs(math.radians(candidate.transform.rotation.yaw - source_wp.transform.rotation.yaw))
        yaw_error = abs((yaw_error + math.pi) % (2.0 * math.pi) - math.pi)
        if yaw_error > math.radians(150.0):
            return candidate
    return None


def get_r344_nonmotor_anchor(loop_route, transition_index):
    """获取 R344 右转入口附近的非机动车直行锚点。"""
    anchor_index = max(0, transition_index - RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS)
    anchor_wp = loop_route.waypoints[anchor_index]
    if anchor_wp.road_id != RIGHT_OBJECT_TRIGGER_ROAD:
        return None, anchor_index
    return anchor_wp, anchor_index


def r344_nonmotor_location(anchor_wp, forward_offset, right_offset):
    """在 R344 连接段右侧按前向/右向偏移生成非机动车位置。"""
    anchor_location = anchor_wp.transform.location
    anchor_forward = anchor_wp.transform.get_forward_vector()
    anchor_right = anchor_wp.transform.get_right_vector()
    return carla.Location(
        x=anchor_location.x + anchor_forward.x * forward_offset + anchor_right.x * right_offset,
        y=anchor_location.y + anchor_forward.y * forward_offset + anchor_right.y * right_offset,
        z=anchor_location.z + anchor_forward.z * forward_offset + anchor_right.z * right_offset + 0.65,
    )


def spawn_actor_with_z_retry(world, blueprint, transform, z_retry=0.5):
    """先按原始位置生成 actor，失败后抬高一次重试。"""
    actor = world.try_spawn_actor(blueprint, transform)
    if actor is not None:
        return actor
    retry_transform = carla.Transform(
        carla.Location(
            x=transform.location.x,
            y=transform.location.y,
            z=transform.location.z + z_retry,
        ),
        transform.rotation,
    )
    return world.try_spawn_actor(blueprint, retry_transform)


def spawn_right_side_bicycle_crossing(world, loop_route, actor_list):
    """在 R344 -> R20 右转处生成从右侧通过的非机动车目标。"""
    transition_index = find_route_transition_index(
        loop_route, RIGHT_OBJECT_TRIGGER_ROAD, RIGHT_OBJECT_EXIT_ROAD
    )
    if transition_index is None:
        print("Right-side bicycle skipped: R344 -> R20 transition not found on route.")
        return None

    anchor_wp, anchor_index = get_r344_nonmotor_anchor(loop_route, transition_index)
    if anchor_wp is None:
        print(
            "Right-side bicycle skipped: R344 crossing anchor mismatch, index={}, road={}.".format(
                anchor_index, loop_route.waypoints[anchor_index].road_id
            )
        )
        return None

    start_forward_offset = -2.0 if NONMOTOR_START_CLUSTER_NEAR_JUNCTION else RIGHT_OBJECT_R344_START_FORWARD_OFFSET
    start_location = r344_nonmotor_location(
        anchor_wp, start_forward_offset, RIGHT_OBJECT_R344_RIGHT_OFFSET
    )
    end_location = r344_nonmotor_location(
        anchor_wp, RIGHT_OBJECT_R344_END_FORWARD_OFFSET, RIGHT_OBJECT_R344_RIGHT_OFFSET
    )
    yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
    start_transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

    blueprint = find_nonmotor_blueprint(world.get_blueprint_library())
    try:
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "right_side_bicycle")
    except AttributeError:
        pass
    actor = spawn_actor_with_z_retry(world, blueprint, start_transform)
    if actor is None:
        print("Right-side bicycle skipped: failed to spawn actor near R344 -> R20.")
        return None

    actor.set_simulate_physics(NONMOTOR_VISUAL_PHYSICS_EXPERIMENT)
    actor_list.append(actor)
    print(
        "Right-side bicycle note: CARLA bicycle blueprints may have limited pedal/wheel animation; "
        "kinematic mode is kept by default for stable scenario playback."
    )

    scenario = RightSideBicycleCrossing(
        actor,
        start_transform.location,
        end_location,
        max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS),
        min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS),
        RIGHT_OBJECT_CROSSING_SPEED,
        trigger_location=anchor_wp.transform.location if NONMOTOR_WAIT_AT_INTERSECTION else None,
        trigger_distance=NONMOTOR_TRIGGER_DISTANCE if NONMOTOR_WAIT_AT_INTERSECTION else None,
    )
    print(
        "Right-side bicycle ready: blueprint={}, transition_index={}, anchor_index={}, anchor_road={}, path=straight_along_r344, start=({:.1f}, {:.1f}), end=({:.1f}, {:.1f}).".format(
            blueprint.id,
            transition_index,
            anchor_index,
            anchor_wp.road_id,
            start_location.x,
            start_location.y,
            end_location.x,
            end_location.y,
        )
    )
    return scenario


def spawn_slow_right_lane_vehicle(world, carla_map, loop_route, actor_list):
    """在自车既定路线前方约 75m 生成一辆最右侧同向慢车。"""
    route_index = min(
        int(SLOW_RIGHT_LANE_VEHICLE_DISTANCE / loop_route.step_distance),
        len(loop_route.waypoints) - 1,
    )
    route_waypoint = loop_route.waypoints[route_index]
    lane_waypoint = get_rightmost_same_direction_lane(route_waypoint)
    lane_path = build_right_lane_waypoint_path(lane_waypoint, loop_route.step_distance)
    if len(lane_path) < 2:
        print("Slow right-lane vehicle skipped: no usable lane path at route_index={}.".format(route_index))
        return None

    blueprint_library = world.get_blueprint_library()
    blueprint = blueprint_library.find("vehicle.audi.tt")
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME)

    transform = vehicle_transform_from_waypoint(lane_path[0])
    actor = spawn_actor_with_z_retry(world, blueprint, transform)
    if actor is None:
        print("Slow right-lane vehicle skipped: failed to spawn actor.")
        return None

    actor.set_simulate_physics(False)
    actor_list.append(actor)
    controller = RightLaneSlowVehicleController(
        actor,
        lane_path,
        SLOW_RIGHT_LANE_VEHICLE_SPEED,
        loop_route.step_distance,
    )
    print(
        "Slow right-lane vehicle ready: role={}, route_index={}, road={}, lane={}, speed={:.1f}m/s.".format(
            SLOW_RIGHT_LANE_VEHICLE_ROLE_NAME,
            route_index,
            lane_waypoint.road_id,
            lane_waypoint.lane_id,
            SLOW_RIGHT_LANE_VEHICLE_SPEED,
        )
    )
    return controller


def spawn_background_route_vehicles(world, loop_route, actor_list, rng):
    """在自车固定路线不同路段生成 5 辆较慢的背景车辆。"""
    blueprint_library = world.get_blueprint_library()
    preferred_ids = (
        "vehicle.audi.tt",
        "vehicle.dodge.charger_2020",
        "vehicle.mercedes.coupe",
        "vehicle.mini.cooper_s",
        "vehicle.nissan.patrol",
    )
    vehicles = []
    for index, blueprint_id in zip(BACKGROUND_VEHICLE_ROUTE_INDICES, preferred_ids):
        blueprint = blueprint_library.find(blueprint_id)
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "background_vehicle_{}".format(len(vehicles) + 1))

        waypoint_index = min(index, len(loop_route.waypoints) - 1)
        transform = vehicle_transform_from_waypoint(loop_route.waypoints[waypoint_index])
        actor = spawn_actor_with_z_retry(world, blueprint, transform)
        if actor is None:
            print("Background vehicle skipped: index={}, blueprint={}.".format(waypoint_index, blueprint_id))
            continue

        actor.set_simulate_physics(False)
        target_speed = rng.uniform(BACKGROUND_VEHICLE_SPEED_MIN, BACKGROUND_VEHICLE_SPEED_MAX)
        actor_list.append(actor)
        vehicles.append(BackgroundRouteVehicle(actor, target_speed, waypoint_index, loop_route))
        print(
            "Background vehicle ready: role={}, route_index={}, target_speed={:.1f}m/s.".format(
                actor.attributes.get("role_name", "--"),
                waypoint_index,
                target_speed,
            )
        )
    return vehicles


def spawn_visual_traffic_vehicles(world, loop_route, actor_list, rng):
    """生成不接入感知的对向/旁路视觉交通流。"""
    blueprint_library = world.get_blueprint_library()
    preferred_ids = (
        "vehicle.lincoln.mkz_2020",
        "vehicle.bmw.grandtourer",
        "vehicle.chevrolet.impala",
        "vehicle.toyota.prius",
        "vehicle.carlamotors.carlacola",
    )
    vehicles = []
    for idx, route_index in enumerate(VISUAL_TRAFFIC_ROUTE_INDICES, 1):
        route_index = min(route_index, len(loop_route.waypoints) - 1)
        lane_wp = opposite_direction_lane(loop_route.waypoints[route_index])
        if lane_wp is None:
            print("Visual traffic skipped: no opposite lane at route_index={}.".format(route_index))
            continue

        lane_path = build_right_lane_waypoint_path(lane_wp, loop_route.step_distance, max_steps=90)
        if len(lane_path) < 2:
            print("Visual traffic skipped: short lane path at route_index={}.".format(route_index))
            continue

        blueprint = blueprint_library.find(preferred_ids[(idx - 1) % len(preferred_ids)])
        if blueprint.has_attribute("role_name"):
            blueprint.set_attribute("role_name", "{}_{}".format(VISUAL_TRAFFIC_ROLE_PREFIX, idx))

        transform = vehicle_transform_from_waypoint(lane_path[0])
        actor = spawn_actor_with_z_retry(world, blueprint, transform)
        if actor is None:
            print("Visual traffic skipped: failed to spawn at route_index={}.".format(route_index))
            continue

        actor.set_simulate_physics(False)
        actor_list.append(actor)
        target_speed = rng.uniform(VISUAL_TRAFFIC_SPEED_MIN, VISUAL_TRAFFIC_SPEED_MAX)
        vehicles.append(VisualLaneVehicleController(actor, lane_path, target_speed, loop_route.step_distance))
        print(
            "Visual traffic ready: role={}, route_index={}, road={}, lane={}, speed={:.1f}m/s.".format(
                actor.attributes.get("role_name", "--"),
                route_index,
                lane_wp.road_id,
                lane_wp.lane_id,
                target_speed,
            )
        )
    return vehicles


def spawn_background_r344_bicycles(world, loop_route, actor_list, rng):
    """在 R344 右侧非机动车区域生成额外背景自行车。"""
    transition_index = find_route_transition_index(
        loop_route, RIGHT_OBJECT_TRIGGER_ROAD, RIGHT_OBJECT_EXIT_ROAD
    )
    if transition_index is None:
        print("Background bicycles skipped: R344 -> R20 transition not found on route.")
        return []

    anchor_wp, anchor_index = get_r344_nonmotor_anchor(loop_route, transition_index)
    if anchor_wp is None:
        print("Background bicycles skipped: R344 nonmotor anchor not found.")
        return []

    blueprint_library = world.get_blueprint_library()
    bicycle_blueprint = find_nonmotor_blueprint(blueprint_library)
    pedestrian_blueprint = find_pedestrian_blueprint(blueprint_library)
    bicycles = []
    for idx, (forward_offset, right_offset) in enumerate(
        zip(BACKGROUND_BICYCLE_FORWARD_OFFSETS, BACKGROUND_BICYCLE_RIGHT_OFFSETS),
        1,
    ):
        nonmotor_type = BACKGROUND_NONMOTOR_TYPES[min(idx - 1, len(BACKGROUND_NONMOTOR_TYPES) - 1)]
        if NONMOTOR_START_CLUSTER_NEAR_JUNCTION:
            forward_offset = -4.0 - 2.0 * (idx - 1)
        start_location = r344_nonmotor_location(anchor_wp, forward_offset, right_offset)
        end_location = r344_nonmotor_location(anchor_wp, BACKGROUND_BICYCLE_END_FORWARD_OFFSET, right_offset)
        if nonmotor_type == "pedestrian":
            start_location.z -= 0.55
            end_location.z -= 0.55
        yaw = math.degrees(math.atan2(end_location.y - start_location.y, end_location.x - start_location.x))
        transform = carla.Transform(start_location, carla.Rotation(yaw=yaw))

        bicycle_bp = pedestrian_blueprint if nonmotor_type == "pedestrian" else bicycle_blueprint
        try:
            if bicycle_bp.has_attribute("role_name"):
                bicycle_bp.set_attribute("role_name", "background_{}_{}".format(nonmotor_type, idx))
        except AttributeError:
            pass

        actor = spawn_actor_with_z_retry(world, bicycle_bp, transform)
        if actor is None:
            print("Background {} skipped: index={}, start_offset={}.".format(nonmotor_type, idx, forward_offset))
            continue

        actor.set_simulate_physics(NONMOTOR_VISUAL_PHYSICS_EXPERIMENT)
        actor_list.append(actor)
        speed = rng.uniform(BACKGROUND_BICYCLE_SPEED_MIN, BACKGROUND_BICYCLE_SPEED_MAX)
        scenario = RightSideBicycleCrossing(
            actor,
            transform.location,
            end_location,
            max(0, transition_index - RIGHT_OBJECT_TRIGGER_ROUTE_STEPS - 10 + idx * 3),
            min(len(loop_route.waypoints) - 1, transition_index + RIGHT_OBJECT_CLEAR_ROUTE_STEPS + 8),
            speed,
            trigger_location=anchor_wp.transform.location if NONMOTOR_WAIT_AT_INTERSECTION else None,
            trigger_distance=NONMOTOR_TRIGGER_DISTANCE if NONMOTOR_WAIT_AT_INTERSECTION else None,
        )
        bicycles.append(scenario)
        print(
            "Background {} ready: role={}, anchor_index={}, right_offset={:.1f}, speed={:.1f}m/s.".format(
                nonmotor_type,
                actor.attributes.get("role_name", "--"),
                anchor_index,
                right_offset,
                speed,
            )
        )
    return bicycles


>>>>>>> Stashed changes
def main():
    actor_list = []
    camera_display = None
    world = None
    original_settings = None

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(CLIENT_TIMEOUT)

        world = client.get_world()
        original_settings = world.get_settings()
        world = setup_world(client)
        carla_map = world.get_map()

        ego_vehicle, lead_vehicle, ego_start_wp = spawn_scenario(world)
        actor_list.extend([ego_vehicle, lead_vehicle])
        collision_monitor = CollisionMonitor(world, ego_vehicle, actor_list)
        mpc = SamplingMPCTracker()
        camera_display = PygameDemoDisplay(world, ego_vehicle, actor_list)

        world.tick()
        set_spectator(world, ego_vehicle)
        loop_route = LoopRoute(ego_start_wp)
        traffic_rng = random.Random(TRAFFIC_RANDOM_SEED)
        slow_right_lane_vehicle = spawn_slow_right_lane_vehicle(world, carla_map, loop_route, actor_list)
        background_vehicles = spawn_background_route_vehicles(world, loop_route, actor_list, traffic_rng)
        visual_traffic_vehicles = spawn_visual_traffic_vehicles(world, loop_route, actor_list, traffic_rng)
        background_bicycles = spawn_background_r344_bicycles(world, loop_route, actor_list, traffic_rng)
        right_object_scenario = spawn_right_side_bicycle_crossing(world, loop_route, actor_list)
<<<<<<< Updated upstream
        right_object_scenarios = [scenario for scenario in [right_object_scenario] + background_bicycles if scenario]
=======
        front_extra_vehicles = []
        if slow_right_lane_vehicle is not None:
            front_extra_vehicles.append(slow_right_lane_vehicle.actor)
        front_extra_vehicles.extend([controller.actor for controller in background_vehicles])
>>>>>>> Stashed changes
        sensor = VirtualGroundTruthSensor(
            world,
            carla_map,
            ego_vehicle,
            lead_vehicle,
<<<<<<< Updated upstream
            front_extra_vehicles=[controller.actor for controller in background_vehicles],
            right_object_scenarios=right_object_scenarios,
=======
            right_object_scenario,
            front_extra_vehicles=front_extra_vehicles,
            right_object_scenarios=background_bicycles,
>>>>>>> Stashed changes
        )

        state = "FOLLOW"
        trajectory = None
        start_time = time.time()
        frame = 0
        route_completion_time = None
        right_object_yield_done = False

        print("Scenario started: map={}, ego=Tesla Model3, lead=Lincoln MKZ 2020".format(MAP_NAME))
        print("Lead car will brake hard at {:.1f}s.".format(LEAD_BRAKE_TIME))
        print(
            "Background traffic: slow_right_lane_vehicle={}, route_vehicles={}, visual_traffic={}, nonmotor_background={}.".format(
                slow_right_lane_vehicle is not None,
                len(background_vehicles),
                len(visual_traffic_vehicles),
                len(background_bicycles),
            )
        )
        print(
            "Loop route: {:.1f}m, {} waypoints.".format(
                loop_route.length, len(loop_route.points)
            )
        )
        print(
            "Route turn check: right_turn_count={}, turn_events={}".format(
                loop_route.right_turn_count,
                [
                    "{}:{:.1f}deg@{}-{}".format(
                        event["direction"],
                        event["degrees"],
                        event["start_index"],
                        event["end_index"],
                    )
                    for event in loop_route.turn_events
                ],
            )
        )
        print(
            "Route lane check: right_lane_before_turn={}, prepare_index={}, close_to_index={}.".format(
                loop_route.right_lane_before_turn,
                loop_route.right_lane_prepare_index,
                loop_route.close_to_index,
            )
        )

        while frame * FIXED_DELTA_SECONDS < SIM_SECONDS: # 主循环，持续运行直到达到最大仿真时间
            if camera_display is not None and not camera_display.process_events():
                print("Animation window closed by user.")
                break

            sim_time = frame * FIXED_DELTA_SECONDS

            if sim_time < LEAD_BRAKE_TIME:
                lead_throttle, lead_brake = speed_control(get_speed(lead_vehicle), LEAD_TARGET_SPEED)
                lead_steer = waypoint_steer(lead_vehicle, carla_map)
                lead_vehicle.apply_control(
                    carla.VehicleControl(throttle=lead_throttle, brake=lead_brake, steer=lead_steer)
                )
            else:
                lead_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))

            if slow_right_lane_vehicle is not None:
                slow_right_lane_vehicle.update(FIXED_DELTA_SECONDS)

            for background_vehicle in background_vehicles:
                background_vehicle.update(FIXED_DELTA_SECONDS)

<<<<<<< Updated upstream
            for right_object in right_object_scenarios:
                right_object.update(loop_route.last_index, FIXED_DELTA_SECONDS)
=======
            for visual_vehicle in visual_traffic_vehicles:
                visual_vehicle.update(FIXED_DELTA_SECONDS)

            if right_object_scenario is not None:
                right_object_scenario.update(loop_route.last_index, FIXED_DELTA_SECONDS, ego_vehicle)
            for background_bicycle in background_bicycles:
                background_bicycle.update(loop_route.last_index, FIXED_DELTA_SECONDS, ego_vehicle)
>>>>>>> Stashed changes

            front = sensor.front_vehicle()
            right_object = sensor.right_side_object(loop_route.last_index)
            ego_speed = get_speed(ego_vehicle)

            emergency_needed = (
                front.is_front_vehicle
                and front.distance < SAFE_DISTANCE
                and front.ttc < TTC_AVOID_THRESHOLD
            )
            brake_needed = (
                front.is_front_vehicle
                and front.ttc < TTC_BRAKE_THRESHOLD
            )
            right_object_risk = (
                right_object.is_conflict_object
                and (
                    right_object.ttc < RIGHT_OBJECT_TTC_THRESHOLD
                    or right_object.distance < RIGHT_OBJECT_DETECT_DISTANCE
                )
            )

            if state == "FOLLOW" and emergency_needed:
                avoidance_side = choose_avoidance_side(sensor)
                if avoidance_side is not None:
                    lane_width = carla_map.get_waypoint(ego_vehicle.get_location()).lane_width
                    lateral_offset = -lane_width if avoidance_side == "left" else lane_width
                    trajectory = QuinticLaneChangeTrajectory(
                        ego_vehicle.get_transform(), lateral_offset, LANE_CHANGE_LENGTH
                    )
                    state = "AVOID"
                    print(
                        "Avoidance started at {:.2f}s: side={}, distance={:.1f}m, TTC={:.2f}s".format(
                            sim_time, avoidance_side, front.distance, front.ttc
                        )
                    )
                else:
                    state = "EMERGENCY_BRAKE"
                    print(
                        "Emergency brake only at {:.2f}s: no adjacent clear lane, TTC={:.2f}s".format(
                            sim_time, front.ttc
                        )
                    )

            if state in ("FOLLOW", "LANE_KEEP") and right_object_risk and not right_object_yield_done:
                state = "RIGHT_OBJECT_YIELD"
                print(
                    "Right object yield started at {:.2f}s: distance={:.1f}m, TTC={:.2f}s, route_index={}.".format(
                        sim_time,
                        right_object.distance,
                        right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        loop_route.last_index,
                    )
                )

            if route_completion_time is not None:
                state = "ROUTE_HOLD"
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "AVOID" and trajectory is not None:
                target_speed = min(EGO_TARGET_SPEED, max(8.0, ego_speed))
                ego_control = mpc.control(ego_vehicle, trajectory, target_speed)
                if brake_needed:
                    ego_control.brake = max(ego_control.brake, 0.20)
                    ego_control.throttle = 0.0

                progress, lateral = trajectory.to_local(ego_vehicle.get_location())
                if progress > LANE_CHANGE_LENGTH + 8.0 and abs(lateral - trajectory.lateral_offset) < 0.65:
                    state = "LANE_KEEP"
                    print("Avoidance completed at {:.2f}s.".format(sim_time))

            elif state == "EMERGENCY_BRAKE":
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "RIGHT_OBJECT_YIELD":
                target_speed = RIGHT_OBJECT_YIELD_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                if right_object.distance < RIGHT_OBJECT_STOP_DISTANCE:
                    throttle = 0.0
                    brake = max(brake, 0.85)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=loop_route.steer(ego_vehicle),
                )
                if not right_object_risk:
                    state = "LANE_KEEP"
                    right_object_yield_done = True
                    print("Right object yield completed at {:.2f}s.".format(sim_time))

            else:
                if brake_needed:
                    target_speed = min(EGO_TARGET_SPEED, max(0.0, ego_speed - 5.0))
                else:
                    target_speed = EGO_TARGET_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=loop_route.steer(ego_vehicle),
                )

            ego_vehicle.apply_control(ego_control)

            if frame % int(1.0 / FIXED_DELTA_SECONDS) == 0:
                print(
                    "t={:05.2f}s state={:<18} dist={:05.1f}m ttc={:05.2f}s "
                    "right={:05.1f}m r_ttc={:05.2f}s ego={:04.1f}m/s lead={:04.1f}m/s steer={:+.2f} brake={:.2f}".format(
                        sim_time,
                        state,
                        front.distance,
                        front.ttc if math.isfinite(front.ttc) else 99.99,
                        right_object.distance if math.isfinite(right_object.distance) else 99.9,
                        right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        ego_speed,
                        get_speed(lead_vehicle),
                        ego_control.steer,
                        ego_control.brake,
                    )
                )

            world.tick()
            set_spectator(world, ego_vehicle)
            lap_completed = loop_route.update(ego_vehicle)
            if camera_display is not None:
                camera_display.render({
                    "sim_time": sim_time,
                    "state": state,
                    "scenario": "front_brake_and_right_object",
                    "ego_speed": ego_speed,
                    "lead_speed": get_speed(lead_vehicle),
                    "front_distance": front.distance,
                    "front_ttc": front.ttc,
                    "right_object_distance": right_object.distance,
                    "right_object_ttc": right_object.ttc,
                    "steer": ego_control.steer,
                    "throttle": ego_control.throttle,
                    "brake": ego_control.brake,
                    "collision_count": len(collision_monitor.history),
                    "lap_distance": loop_route.progress_distance,
                    "lap_target_distance": loop_route.length,
                })
            frame += 1

            # 碰撞发生后立即提前终止仿真
            if collision_monitor.history:
                print("检测到碰撞，提前终止仿真。")
                break

            if lap_completed and route_completion_time is None:
                route_completion_time = sim_time
                print(
                    "完成 Town10 固定路线一圈，行驶距离 {:.1f}m，继续运行 {:.1f}s 后结束。".format(
                        loop_route.progress_distance,
                        ROUTE_COMPLETION_HOLD_SECONDS,
                    )
                )

            if (
                route_completion_time is not None
                and sim_time - route_completion_time >= ROUTE_COMPLETION_HOLD_SECONDS
            ):
                print("到达路线终点后已继续运行 {:.1f}s，结束仿真。".format(ROUTE_COMPLETION_HOLD_SECONDS))
                break

        elapsed = time.time() - start_time
        print(
            "Scenario finished in {:.1f}s wall time. Collisions: {}".format(
                elapsed, len(collision_monitor.history)
            )
        )

    finally:
        if world is not None and original_settings is not None:
            restore_world(world, original_settings)
        for actor in reversed(actor_list):
            if actor is not None:
                try:
                    actor.destroy()
                except RuntimeError as exc:
                    print("Cleanup warning: failed to destroy actor {}: {}".format(actor.id, exc))
        if camera_display is not None:
            try:
                camera_display.close()
            except RuntimeError as exc:
                print("Cleanup warning: failed to close pygame display: {}".format(exc))
        print("Cleanup finished.")


if __name__ == "__main__":
    main()
