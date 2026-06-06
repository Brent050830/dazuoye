"""虚拟感知与风险评估模块。

维护约定：
- FrontVehicleReading / RightSideObjectReading / RiskAssessment 数据类
  的字段变更必须同步更新 PROGRAM_FRAMEWORK.md 第 8、9 节。
- 风险阈值参数统一维护在 config.py 中。
"""

from dataclasses import dataclass, field
import math
import random

import carla

from config import (
    DISTANCE_STD,
    FRONT_DETECTION_RANGE,
    FRONT_FOV_HALF_ANGLE_DEG,
    FRONT_LANE_ADJACENT_THRESHOLD,
    FRONT_LANE_SAME_THRESHOLD,
    FRONT_TOP_K,
    LANE_CLEAR_FRONT,
    LANE_CLEAR_REAR,
    MISS_DETECTION_PROB,
    RIGHT_CONFIRM_FRAMES,
    RIGHT_CONFLICT_FRONT_ANGLE_DEG,
    RIGHT_CONFLICT_MAX_DISTANCE,
    RIGHT_CONFLICT_MIN_LATERAL,
    RIGHT_CONFLICT_MAX_LATERAL,
    RIGHT_OBJECT_STOP_DISTANCE,
    RIGHT_PREDICTION_SECONDS,
    SAFE_DISTANCE,
    SENSOR_NOISE_ENABLED,
    SIDE_DETECTION_RANGE,
    SIDE_FOV_HALF_ANGLE_DEG,
    SPEED_STD,
    TTC_AVOID_THRESHOLD,
    TTC_BRAKE_THRESHOLD,
)
from utils import dot_2d, same_direction_lane, vector_length


# ===================== 感知数据结构 =====================

@dataclass
class FrontVehicleReading:
    """前车感知数据结构。
    维护约定：新增字段时必须同步更新 PROGRAM_FRAMEWORK.md 第 8.1 节。
    """
    distance: float          # 纵向间距（米），正值表示在前方
    closing_speed: float     # 接近速度（m/s），正值表示靠近
    ttc: float               # 碰撞时间（秒）
    lateral_offset: float    # 横向偏移（米），正值表示在右侧
    lane_relative_lateral: float = 0.0   # 横向偏移占车道宽度的比例，范围约 (-1, 1)
    is_front_vehicle: bool = False       # 是否确认为正前方车辆（同车道或邻车道前方）
    is_same_lane: bool = False           # 是否明确在同一车道
    risk_level: int = 0                  # 0=无风险, 1=注意, 2=警告, 3=危险
    actor_id: int = None
    actor_role: str = ""


@dataclass
class RightSideObjectReading:
    """右侧非机动车/行人目标的虚拟感知数据。
    维护约定：新增字段时必须同步更新 PROGRAM_FRAMEWORK.md 第 8.2 节。
    """
    distance: float            # 欧氏距离（米）
    ttc: float                 # 基于径向接近速度的 TTC（秒）
    relative_longitudinal: float = 0.0  # 自车坐标系下的纵向相对位置（米）
    relative_lateral: float = 0.0       # 自车坐标系下的横向相对位置（米），正值表示右侧
    is_conflict_object: bool = False    # 是否处于冲突窗口
    risk_level: int = 0                 # 0=无风险, 1=注意, 2=警告, 3=危险
    is_moving_toward_conflict: bool = False  # 是否朝冲突区域移动
    predicted_ttc: float = float("inf")     # 基于运动预测的 TTC（秒）
    object_type: str = ""                   # "bicycle" / "pedestrian"


@dataclass
class RiskAssessment:
    """统一的风险评估结果，供行为决策层使用。
    维护约定：新增字段时必须同步更新 PROGRAM_FRAMEWORK.md 第 9 节。
    """
    primary_risk_type: str = "none"   # "front" / "right_object" / "none"
    primary_risk_level: int = 0       # 0~3
    front_brake_needed: bool = False          # 前车需要制动
    front_emergency_needed: bool = False      # 前车需要紧急避障
    front_emergency_recovered: bool = False   # 紧急制动是否已恢复
    right_object_yield_needed: bool = False   # 右侧目标需要减速让行
    right_object_stop_needed: bool = False    # 右侧目标需要停车
    front_reading: FrontVehicleReading = field(default_factory=FrontVehicleReading)
    right_reading: RightSideObjectReading = field(default_factory=RightSideObjectReading)


# ===================== 虚拟传感器模块 =====================

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
        front_extra_vehicles=None,
        right_object_scenarios=None,
    ):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle
        self.front_extra_vehicles = front_extra_vehicles or []
        self.right_object_scenarios = right_object_scenarios or []

                # 右侧目标连续确认计数器
        self._right_confirm_count = 0
        self._right_confirm_frames = RIGHT_CONFIRM_FRAMES

        # 噪声随机数生成器（固定种子确保可重复）
        self._noise_rng = random.Random(20260606)

    # ========= 传感器模拟辅助方法 =========

    def _add_noise(self, value, std):
        """给测量值叠加高斯噪声。
        
        用 Box-Muller 方法生成高斯随机数，避免依赖 numpy。
        """
        if not SENSOR_NOISE_ENABLED or std <= 0.0:
            return value
        u1 = self._noise_rng.random()
        u2 = self._noise_rng.random()
        # 避免 log(0)
        if u1 <= 0.0:
            u1 = 1e-10
        z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
        return value + z * std

    def _check_front_fov(self, longitudinal, lateral):
        """检查目标是否在前向传感器 FOV 内。
        
        返回 True 表示可检测。
        """
        if not SENSOR_NOISE_ENABLED:
            return True
        if longitudinal <= 0.0:
            return False
        if longitudinal > FRONT_DETECTION_RANGE:
            return False
        angle = abs(math.degrees(math.atan2(abs(lateral), longitudinal)))
        return angle <= FRONT_FOV_HALF_ANGLE_DEG

    def _check_side_fov(self, longitudinal, lateral):
        """检查目标是否在侧向传感器 FOV 内。
        
        侧向 FOV 定义：目标在自车右侧（lateral > 0）且在角度范围内。
        """
        if not SENSOR_NOISE_ENABLED:
            return True
        distance = math.sqrt(longitudinal**2 + lateral**2)
        if distance > SIDE_DETECTION_RANGE:
            return False
        if lateral <= 0.0:
            return False
        angle = abs(math.degrees(math.atan2(longitudinal, lateral)))
        return angle <= SIDE_FOV_HALF_ANGLE_DEG

    def _should_miss_detect(self, distance):
        """根据距离决定是否模拟漏检。
        
        仅对超出 50m 的目标有概率漏检。
        """
        if not SENSOR_NOISE_ENABLED:
            return False
        if distance <= 50.0:
            return False
        return self._noise_rng.random() < MISS_DETECTION_PROB

    # ==========================================

    def front_vehicles(self):
        """返回前方同车道和邻车道的前 FRONT_TOP_K 个最近车辆。
        增强说明：
        - 新增 lane_relative_lateral 归一化横向偏移
        - 区分 is_same_lane / is_adjacent_lane
        - 计算 risk_level：距离、TTC、横向位置综合
        - 返回按距离排序的前 Top-K 车辆
        """
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        ego_wp = self.carla_map.get_waypoint(ego_loc)
        lane_width = max(ego_wp.lane_width, 2.5)
        ego_speed_along = dot_2d(self.ego.get_velocity(), forward)

        candidates = []
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue

            relative = vehicle.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)
            lateral = dot_2d(relative, right)
            lane_relative = lateral / lane_width

                        # 只考虑前方且在邻车道范围内的车辆
            if longitudinal <= 0.0 or abs(lane_relative) >= FRONT_LANE_ADJACENT_THRESHOLD:
                continue

            # === 传感器模拟：前向 FOV 和漏检 ===
            if not self._check_front_fov(longitudinal, lateral):
                continue
            raw_distance = math.sqrt(longitudinal**2 + lateral**2)
            if self._should_miss_detect(raw_distance):
                continue

            target_speed_along = dot_2d(vehicle.get_velocity(), forward)
            raw_closing_speed = ego_speed_along - target_speed_along

            # 叠加噪声
            noisy_longitudinal = max(0.1, self._add_noise(longitudinal, DISTANCE_STD))
            noisy_closing_speed = self._add_noise(raw_closing_speed, SPEED_STD)

            ttc = noisy_longitudinal / noisy_closing_speed if noisy_closing_speed > 0.1 else float("inf")

            # 判断车道归属（用去噪后的 longitudinal 但保留 noisy 距离用于风险评估）
            is_same_lane = abs(lane_relative) < FRONT_LANE_SAME_THRESHOLD

            # 风险等级（基于噪声后的距离和 TTC）
            risk_level = 0
            if is_same_lane and noisy_longitudinal < SAFE_DISTANCE:
                if ttc < TTC_AVOID_THRESHOLD:
                    risk_level = 3  # 危险
                elif ttc < TTC_BRAKE_THRESHOLD:
                    risk_level = 2  # 警告
                elif noisy_longitudinal < SAFE_DISTANCE * 0.6:
                    risk_level = 1  # 注意

            candidates.append(FrontVehicleReading(
                distance=noisy_longitudinal,
                closing_speed=noisy_closing_speed,
                ttc=ttc,
                lateral_offset=lateral,
                lane_relative_lateral=lane_relative,
                is_front_vehicle=True,
                is_same_lane=is_same_lane,
                risk_level=risk_level,
                actor_id=vehicle.id,
                actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
            ))

        # 按距离排序，取前 FRONT_TOP_K 个
        candidates.sort(key=lambda r: r.distance)
        return candidates[:FRONT_TOP_K]

    def front_vehicle(self):
        """兼容旧接口：返回前方最近车辆。
        实际调用 front_vehicles() 取第一个。
        """
        results = self.front_vehicles()
        if results:
            return results[0]
        return FrontVehicleReading(float("inf"), 0.0, float("inf"), 0.0, False)

    def lane_clear(self, side):
        """检测指定侧邻道在前后安全范围内是否无车。
        增强说明：
        - 复用前车感知数据避免重复遍历
        - 在路口附近放宽 lane_id 精确匹配，改用方向一致性
        - 检查邻道车辆相对速度，高速车辆即使距离近也可能不阻塞
        """
        ego_wp = self.carla_map.get_waypoint(
            self.ego.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )
        target_wp = ego_wp.get_left_lane() if side == "left" else ego_wp.get_right_lane()
        if not same_direction_lane(ego_wp, target_wp):
            return False

        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()

        for actor in self.world.get_actors().filter("vehicle.*"):
            if actor.id == self.ego.id:
                continue

            actor_wp = self.carla_map.get_waypoint(
                actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
            )

            # 放宽路口附近的车道匹配：如果 road_id 相同且方向一致，视为同车道
            same_road = actor_wp.road_id == target_wp.road_id
            same_lane = actor_wp.lane_id == target_wp.lane_id
            if not same_road:
                continue
            if not same_lane and not same_direction_lane(target_wp, actor_wp):
                continue

            relative = actor.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)

            if -LANE_CLEAR_REAR > longitudinal or longitudinal > LANE_CLEAR_FRONT:
                continue

            # 检查邻道车辆相对速度：如果邻道车辆速度明显快于自车，说明它在拉开距离
            actor_speed_along = dot_2d(actor.get_velocity(), forward)
            relative_speed = dot_2d(self.ego.get_velocity(), forward) - actor_speed_along
            relative_speed_ahead = relative_speed  # 正值表示前车在远离

            if relative_speed_ahead > 2.0 and longitudinal > 0:
                continue  # 前车在加速远离，不阻塞

            return False

        return True

    def right_side_object(self, route_index):
        """读取右侧非机动车目标，并判断其是否位于当前右转冲突窗口。
        增强说明：
        - 新增 relative_longitudinal / relative_lateral 输出
        - 使用连续帧确认避免单帧误判
        - 预测目标未来位置，判断是否趋向冲突区域
        - 输出 object_type 区分自行车和行人
        - 计算 risk_level：综合距离、TTC、是否进入冲突窗口
        """
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
                or reading.risk_level > best_conflict.risk_level
                or (reading.risk_level == best_conflict.risk_level and reading.ttc < best_conflict.ttc)
            ):
                best_conflict = reading

        # 连续帧确认
        if best_conflict is not None and best_conflict.is_conflict_object:
            self._right_confirm_count += 1
        else:
            self._right_confirm_count = 0

        result = best_conflict if best_conflict is not None else best_nearby
        if result is None:
            return RightSideObjectReading(float("inf"), float("inf"))

        # 只有连续确认帧数达到阈值才标记为真正的冲突
        if result.is_conflict_object and self._right_confirm_count < self._right_confirm_frames:
            # 返回数据但标记为非冲突，让调用方预览
            return RightSideObjectReading(
                distance=result.distance,
                ttc=result.ttc,
                relative_longitudinal=result.relative_longitudinal,
                relative_lateral=result.relative_lateral,
                is_conflict_object=False,
                risk_level=min(result.risk_level, 1),
                is_moving_toward_conflict=result.is_moving_toward_conflict,
                predicted_ttc=result.predicted_ttc,
                object_type=result.object_type,
            )
        return result

    def _right_side_object_reading(self, scenario, route_index):
        """计算单个右侧目标的感知读数。"""
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
        raw_distance = vector_length(relative)

        # === 传感器模拟：侧向 FOV 和漏检 ===
        if not self._check_side_fov(longitudinal, lateral):
            return None
        if self._should_miss_detect(raw_distance):
            return None

        # 叠加噪声
        noisy_distance = max(0.1, self._add_noise(raw_distance, DISTANCE_STD))
        noisy_longitudinal = self._add_noise(longitudinal, DISTANCE_STD)
        noisy_lateral = self._add_noise(lateral, DISTANCE_STD)

        # 径向接近速度与 TTC（基于噪声距离）
        to_object_len = max(noisy_distance, 0.1)
        to_object = carla.Vector3D(
            relative.x / to_object_len, relative.y / to_object_len, 0.0
        )
        ego_velocity = self.ego.get_velocity()
        raw_relative_vel = carla.Vector3D(
            ego_velocity.x - scenario.velocity.x,
            ego_velocity.y - scenario.velocity.y,
            ego_velocity.z - scenario.velocity.z,
        )
        raw_closing_speed = dot_2d(raw_relative_vel, to_object)
        noisy_closing_speed = self._add_noise(raw_closing_speed, SPEED_STD)
        ttc = noisy_distance / noisy_closing_speed if noisy_closing_speed > 0.1 else float("inf")

        # 判断目标类型
        obj_type = ""
        if scenario.is_walker:
            obj_type = "pedestrian"
        elif hasattr(scenario, "name") and "bicycle" in scenario.name:
            obj_type = "bicycle"
        elif scenario.actor.type_id.startswith("vehicle."):
            obj_type = "bicycle"

                # 基础几何门限（使用去噪的原始位置判断，不受噪声影响）
        in_geometry_gate = (
            -8.0 <= longitudinal <= RIGHT_CONFLICT_MAX_DISTANCE
            and RIGHT_CONFLICT_MIN_LATERAL <= lateral <= RIGHT_CONFLICT_MAX_LATERAL
        )

        # 自车速度自适应调整检测距离
        ego_speed = vector_length(ego_velocity)
        dynamic_max_dist = RIGHT_CONFLICT_MAX_DISTANCE + max(0.0, ego_speed * 0.5)
        in_dynamic_gate = (
            -8.0 <= longitudinal <= dynamic_max_dist
            and RIGHT_CONFLICT_MIN_LATERAL <= lateral <= RIGHT_CONFLICT_MAX_LATERAL
        )

        # 冲突窗口 + 几何门限
        in_window = scenario.is_active and scenario.is_conflict_window(route_index)
        is_conflict = in_window and in_dynamic_gate

        # 预测判断：目标是否朝冲突区域移动（使用去噪原始位置和速度）
        is_moving_toward = False
        predicted_ttc = float("inf")
        if is_conflict:
            obj_forward_speed = dot_2d(scenario.velocity, forward)
            relative_long_speed = (ego_speed * 1.0) - obj_forward_speed
            if relative_long_speed > 0.5 and longitudinal > 0:
                is_moving_toward = True
                closing_long = relative_long_speed
                # predicted_ttc 叠加噪声
                raw_predicted_ttc = longitudinal / closing_long if closing_long > 0.1 else float("inf")
                predicted_ttc = self._add_noise(raw_predicted_ttc, 0.3) if raw_predicted_ttc != float("inf") else float("inf")
            elif lateral > RIGHT_CONFLICT_MIN_LATERAL and lateral < 10.0:
                is_moving_toward = True

        # 风险等级（基于噪声后的距离和 TTC）
        risk_level = 0
        if is_conflict:
            if ttc < TTC_AVOID_THRESHOLD or (predicted_ttc < TTC_AVOID_THRESHOLD and is_moving_toward):
                risk_level = 3  # 危险
            elif ttc < TTC_BRAKE_THRESHOLD:
                risk_level = 2  # 警告
            elif noisy_distance < SAFE_DISTANCE:
                risk_level = 1  # 注意

        return RightSideObjectReading(
            distance=noisy_distance,
            ttc=ttc,
            relative_longitudinal=noisy_longitudinal,
            relative_lateral=noisy_lateral,
            is_conflict_object=is_conflict,
            risk_level=risk_level,
            is_moving_toward_conflict=is_moving_toward,
            predicted_ttc=predicted_ttc,
            object_type=obj_type,
        )

    def assess_risk(self, route_index):
        """统一风险评估：综合前车和右侧目标信息，输出 RiskAssessment。
        维护约定：此方法封装了第 9 节中的风险判断逻辑。
        如果修改风险判断条件，必须同步更新 PROGRAM_FRAMEWORK.md。
        """
        front_readings = self.front_vehicles()
        right_reading = self.right_side_object(route_index)

        # 主前车（最近同车道车辆）
        front = front_readings[0] if front_readings else FrontVehicleReading(
            float("inf"), 0.0, float("inf"), 0.0, False
        )

        # === 前车风险 ===
        brake_needed = (
            front.is_front_vehicle
            and front.is_same_lane
            and front.ttc < TTC_BRAKE_THRESHOLD
        )
        emergency_needed = (
            front.is_front_vehicle
            and front.is_same_lane
            and front.distance < SAFE_DISTANCE
            and front.ttc < TTC_AVOID_THRESHOLD
        )
        emergency_recovered = (
            not front.is_front_vehicle
            or not front.is_same_lane
            or front.distance > SAFE_DISTANCE + 8.0
            or front.ttc > TTC_BRAKE_THRESHOLD + 1.0
        )

                # === 右侧目标风险 ===
        # 放宽条件：恢复到与原来等效的"TTC < 阈值 或 距离 < 阈值"逻辑
        # 同时保留 risk_level >= 3（危险）的硬触发
        right_yield_needed = (
            right_reading.is_conflict_object
            and (
                right_reading.risk_level >= 3
                or (right_reading.risk_level >= 1 and right_reading.distance < 34.0)
                or right_reading.ttc < 5.0
            )
        )
        right_stop_needed = (
            right_reading.is_conflict_object
            and right_reading.risk_level >= 2
            and right_reading.distance < RIGHT_OBJECT_STOP_DISTANCE
        )

        # === 优先级仲裁 ===
        primary_type = "none"
        primary_level = 0
        if emergency_needed:
            primary_type = "front"
            primary_level = 3
        elif right_yield_needed:
            primary_type = "right_object"
            primary_level = 2
        elif brake_needed:
            primary_type = "front"
            primary_level = 2

        return RiskAssessment(
            primary_risk_type=primary_type,
            primary_risk_level=primary_level,
            front_brake_needed=brake_needed,
            front_emergency_needed=emergency_needed,
            front_emergency_recovered=emergency_recovered,
            right_object_yield_needed=right_yield_needed,
            right_object_stop_needed=right_stop_needed,
            front_reading=front,
            right_reading=right_reading,
        )


