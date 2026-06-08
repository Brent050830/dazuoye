import math
import random
from dataclasses import dataclass

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
    RADAR_CLUSTER_RADIUS,
    RADAR_ENABLED,
    RADAR_MIN_POINTS_PER_CLUSTER,
    RIGHT_CONFIRM_FRAMES,
    RIGHT_OBJECT_DETECT_DISTANCE,
    RIGHT_OBJECT_LATERAL_MAX,
    RIGHT_OBJECT_LATERAL_MIN,
    RIGHT_OBJECT_LONGITUDINAL_MAX,
    RIGHT_OBJECT_LONGITUDINAL_MIN,
    RIGHT_OBJECT_STOP_DISTANCE,
    RIGHT_OBJECT_TTC_THRESHOLD,
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
    """前车感知数据结构。"""

    distance: float
    closing_speed: float
    ttc: float
    lateral_offset: float
    is_front_vehicle: bool
    actor_id: int = None
    actor_role: str = ""
    target_speed_along: float = 0.0
    lane_relative_lateral: float = 0.0
    is_same_lane: bool = True
    risk_level: int = 0


@dataclass
class RightSideObjectReading:
    """右侧非机动车/行人目标的虚拟感知数据。"""

    distance: float
    ttc: float
    is_conflict_object: bool = False
    actor_id: int = None
    actor_role: str = ""
    longitudinal: float = 0.0
    lateral: float = 0.0
    risk_level: int = 0
    is_moving_toward_conflict: bool = False
    predicted_ttc: float = float("inf")
    object_type: str = ""


@dataclass
class RiskAssessment:
    """统一风险评估结果，供后续逐步收敛状态机使用。"""

    primary_risk_type: str = "none"
    primary_risk_level: int = 0
    front_brake_needed: bool = False
    front_emergency_needed: bool = False
    front_emergency_recovered: bool = True
    right_object_yield_needed: bool = False
    right_object_stop_needed: bool = False
    front_reading: FrontVehicleReading = None
    right_reading: RightSideObjectReading = None


# ===================== 虚拟/雷达融合感知模块 =====================

class VirtualGroundTruthSensor:
    """虚拟真值传感器，可选叠加噪声/FOV/漏检和 CARLA 前向雷达点云。"""

    def __init__(
        self,
        world,
        carla_map,
        ego_vehicle,
        lead_vehicle,
        front_extra_vehicles=None,
        right_object_scenarios=None,
        loop_route=None,
    ):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle
        self.front_extra_vehicles = front_extra_vehicles or []
        self.right_object_scenarios = right_object_scenarios or []
        self.loop_route = loop_route
        self._right_confirm_count = 0
        self._right_confirm_frames = RIGHT_CONFIRM_FRAMES
        self._noise_rng = random.Random(20260606)
        self._radar_detections = []

    def _empty_front(self):
        return FrontVehicleReading(float("inf"), 0.0, float("inf"), 0.0, False)

    def _add_noise(self, value, std):
        """给测量值叠加固定随机种子的高斯噪声。"""
        if not SENSOR_NOISE_ENABLED or std <= 0.0:
            return value
        u1 = max(self._noise_rng.random(), 1e-10)
        u2 = self._noise_rng.random()
        z = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
        return value + z * std

    def _should_miss_detect(self, distance):
        """模拟远距离目标漏检。"""
        if not SENSOR_NOISE_ENABLED or distance <= 50.0:
            return False
        return self._noise_rng.random() < MISS_DETECTION_PROB

    def _check_front_fov(self, longitudinal, lateral):
        """检查前向目标是否处于前视 FOV。"""
        if not SENSOR_NOISE_ENABLED:
            return True
        if longitudinal <= 0.0 or longitudinal > FRONT_DETECTION_RANGE:
            return False
        angle = abs(math.degrees(math.atan2(abs(lateral), longitudinal)))
        return angle <= FRONT_FOV_HALF_ANGLE_DEG

    def _check_side_fov(self, longitudinal, lateral):
        """检查右侧目标是否处于侧向 FOV。"""
        if not SENSOR_NOISE_ENABLED:
            return True
        distance = math.sqrt(longitudinal * longitudinal + lateral * lateral)
        if distance > SIDE_DETECTION_RANGE or lateral <= 0.0:
            return False
        angle = abs(math.degrees(math.atan2(longitudinal, lateral)))
        return angle <= SIDE_FOV_HALF_ANGLE_DEG

    def set_radar_detections(self, detections):
        """接收 CARLA radar callback 的原始点云。"""
        self._radar_detections = list(detections)

    def front_vehicle(self, use_route_reference=True):
        """兼容旧接口：返回前方最近同车道车辆。"""
        readings = self.front_vehicles(use_route_reference=use_route_reference)
        for reading in readings:
            if reading.is_front_vehicle and reading.is_same_lane:
                return reading
        return self._empty_front()

    def front_vehicles(self, use_route_reference=True):
        """返回前方候选车辆列表，默认仍保留当前弧线参考逻辑。"""
        if RADAR_ENABLED and self._radar_detections:
            return self._radar_front_vehicles()
        if use_route_reference and self.loop_route is not None:
            return self._route_front_vehicles()
        return self._ego_frame_front_vehicles()

    def _ego_frame_front_vehicles(self):
        """用自车当前直角坐标系读取前方车辆，作为无路线参考时的兜底方案。"""
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        lane_width = max(self.carla_map.get_waypoint(ego_loc).lane_width, 2.5)
        ego_speed_along = dot_2d(self.ego.get_velocity(), forward)

        readings = []
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue

            relative = vehicle.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward)
            lateral = dot_2d(relative, right)
            lane_relative = lateral / lane_width
            if longitudinal <= 0.0 or abs(lane_relative) >= FRONT_LANE_ADJACENT_THRESHOLD:
                continue
            if not self._check_front_fov(longitudinal, lateral):
                continue
            if self._should_miss_detect(math.sqrt(longitudinal * longitudinal + lateral * lateral)):
                continue

            target_speed_along = dot_2d(vehicle.get_velocity(), forward)
            reading = self._make_front_reading(
                distance=longitudinal,
                closing_speed=ego_speed_along - target_speed_along,
                lateral=lateral,
                lane_relative=lane_relative,
                is_same_lane=abs(lane_relative) < 0.65,
                actor_id=vehicle.id,
                actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
                target_speed_along=target_speed_along,
            )
            readings.append(reading)

        readings.sort(key=lambda reading: reading.distance)
        return readings[:FRONT_TOP_K]

    def _route_front_vehicles(self):
        """将车辆投影到当前路线局部弧线，用弧长 s 和横向 d 判断弯道前车。"""
        ego_loc = self.ego.get_location()
        ego_projection = self._project_to_route(ego_loc, self.loop_route.last_index, search_back=6, search_ahead=18)
        lane_width = max(self.carla_map.get_waypoint(ego_loc).lane_width, 2.5)
        ego_speed_along = self._speed_along_route(self.ego, ego_projection)
        search_ahead = int((LANE_CLEAR_FRONT + 20.0) / self.loop_route.step_distance) + 8

        readings = []
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue

            target_projection = self._project_to_route(
                vehicle.get_location(),
                int(ego_projection["raw_index"]),
                search_back=3,
                search_ahead=search_ahead,
            )
            longitudinal = (target_projection["raw_index"] - ego_projection["raw_index"]) * self.loop_route.step_distance
            lateral = target_projection["lateral"] - ego_projection["lateral"]
            lane_relative = lateral / lane_width
            if longitudinal <= 0.0 or abs(lane_relative) >= FRONT_LANE_ADJACENT_THRESHOLD:
                continue
            if not self._check_front_fov(longitudinal, lateral):
                continue
            if self._should_miss_detect(math.sqrt(longitudinal * longitudinal + lateral * lateral)):
                continue

            target_speed_along = self._speed_along_route(vehicle, target_projection)
            reading = self._make_front_reading(
                distance=longitudinal,
                closing_speed=ego_speed_along - target_speed_along,
                lateral=lateral,
                lane_relative=lane_relative,
                is_same_lane=abs(lateral) < lane_width * 0.45,
                actor_id=vehicle.id,
                actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
                target_speed_along=target_speed_along,
            )
            readings.append(reading)

        readings.sort(key=lambda reading: reading.distance)
        return readings[:FRONT_TOP_K]

    def _make_front_reading(
        self,
        distance,
        closing_speed,
        lateral,
        lane_relative,
        is_same_lane,
        actor_id=None,
        actor_role="",
        target_speed_along=0.0,
    ):
        noisy_distance = max(0.1, self._add_noise(distance, DISTANCE_STD))
        noisy_closing_speed = self._add_noise(closing_speed, SPEED_STD)
        ttc = noisy_distance / noisy_closing_speed if noisy_closing_speed > 0.1 else float("inf")
        risk_level = 0
        if is_same_lane and noisy_distance < SAFE_DISTANCE:
            if ttc < TTC_AVOID_THRESHOLD:
                risk_level = 3
            elif ttc < TTC_BRAKE_THRESHOLD:
                risk_level = 2
            elif noisy_distance < SAFE_DISTANCE * 0.6:
                risk_level = 1
        return FrontVehicleReading(
            distance=noisy_distance,
            closing_speed=noisy_closing_speed,
            ttc=ttc,
            lateral_offset=lateral,
            is_front_vehicle=is_same_lane,
            actor_id=actor_id,
            actor_role=actor_role,
            target_speed_along=target_speed_along,
            lane_relative_lateral=lane_relative,
            is_same_lane=is_same_lane,
            risk_level=risk_level,
        )

    def _radar_front_vehicles(self):
        """将 CARLA 雷达原始点云聚类为前方候选目标。"""
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        right = ego_tf.get_right_vector()
        lane_width = max(self.carla_map.get_waypoint(ego_loc).lane_width, 2.5)
        ego_speed_along = dot_2d(self.ego.get_velocity(), ego_tf.get_forward_vector())

        clusters = []
        for detection in self._radar_detections:
            longitudinal = detection.depth * math.cos(detection.azimuth)
            lateral = detection.depth * math.sin(detection.azimuth)
            if longitudinal <= 0.0:
                continue
            point = {"longitudinal": longitudinal, "lateral": lateral, "velocity": detection.velocity, "count": 1}
            matched = None
            for cluster in clusters:
                dx = cluster["longitudinal"] - longitudinal
                dy = cluster["lateral"] - lateral
                if math.sqrt(dx * dx + dy * dy) < RADAR_CLUSTER_RADIUS:
                    matched = cluster
                    break
            if matched is None:
                clusters.append(point)
            else:
                count = matched["count"] + 1
                matched["longitudinal"] = (matched["longitudinal"] * matched["count"] + longitudinal) / count
                matched["lateral"] = (matched["lateral"] * matched["count"] + lateral) / count
                matched["velocity"] = (matched["velocity"] * matched["count"] + detection.velocity) / count
                matched["count"] = count

        readings = []
        for cluster in clusters:
            if cluster["count"] < RADAR_MIN_POINTS_PER_CLUSTER:
                continue
            lane_relative = cluster["lateral"] / lane_width
            if abs(lane_relative) >= FRONT_LANE_ADJACENT_THRESHOLD:
                continue
            closing_speed = max(0.0, -cluster["velocity"])
            target_speed_along = max(0.0, ego_speed_along - closing_speed)
            readings.append(
                self._make_front_reading(
                    distance=cluster["longitudinal"],
                    closing_speed=closing_speed,
                    lateral=cluster["lateral"],
                    lane_relative=lane_relative,
                    is_same_lane=abs(lane_relative) < FRONT_LANE_SAME_THRESHOLD,
                    actor_role="radar_cluster",
                    target_speed_along=target_speed_along,
                )
            )

        readings.sort(key=lambda reading: reading.distance)
        return readings[:FRONT_TOP_K]

    def _project_to_route(self, location, anchor_index, search_back=5, search_ahead=24):
        """把位置投影到路线局部窗口，返回浮点路线索引和相对路线的横向偏移。"""
        last_segment = max(0, len(self.loop_route.points) - 2)
        start = max(0, int(anchor_index) - search_back)
        end = min(last_segment, int(anchor_index) + search_ahead)
        best = None
        for index in range(start, end + 1):
            p0 = self.loop_route.points[index]
            p1 = self.loop_route.points[index + 1]
            segment = p1 - p0
            segment_len_sq = max(dot_2d(segment, segment), 0.001)
            to_location = location - p0
            blend = max(0.0, min(dot_2d(to_location, segment) / segment_len_sq, 1.0))
            projected = carla.Location(
                x=p0.x + segment.x * blend,
                y=p0.y + segment.y * blend,
                z=p0.z + segment.z * blend,
            )
            error = projected.distance(location)
            if best is None or error < best["error"]:
                raw_index = index + blend
                right = self._route_right_at(raw_index)
                best = {
                    "raw_index": raw_index,
                    "location": projected,
                    "right": right,
                    "lateral": dot_2d(location - projected, right),
                    "error": error,
                }
        if best is not None:
            return best

        right = self._route_right_at(float(anchor_index))
        return {
            "raw_index": float(anchor_index),
            "location": self.loop_route.points[int(anchor_index)],
            "right": right,
            "lateral": dot_2d(location - self.loop_route.points[int(anchor_index)], right),
            "error": 0.0,
        }

    def _route_location_at_raw_index(self, raw_index):
        lower = max(0, min(int(raw_index), len(self.loop_route.points) - 1))
        upper = max(0, min(lower + 1, len(self.loop_route.points) - 1))
        blend = max(0.0, min(raw_index - lower, 1.0))
        p0 = self.loop_route.points[lower]
        p1 = self.loop_route.points[upper]
        return carla.Location(
            x=p0.x + (p1.x - p0.x) * blend,
            y=p0.y + (p1.y - p0.y) * blend,
            z=p0.z + (p1.z - p0.z) * blend,
        )

    def _route_right_at(self, raw_index):
        """由路线中心线前后差分得到平滑道路右向量。"""
        look_index = 1.25
        before = self._route_location_at_raw_index(raw_index - look_index)
        after = self._route_location_at_raw_index(raw_index + look_index)
        tangent = carla.Vector3D(x=after.x - before.x, y=after.y - before.y, z=0.0)
        tangent_length = max(vector_length(tangent), 0.001)
        tangent.x /= tangent_length
        tangent.y /= tangent_length
        return carla.Vector3D(x=-tangent.y, y=tangent.x, z=0.0)

    def _speed_along_route(self, vehicle, projection):
        right = projection["right"]
        tangent = carla.Vector3D(x=right.y, y=-right.x, z=0.0)
        return dot_2d(vehicle.get_velocity(), tangent)

    def lane_clear(self, side):
        """检测指定侧邻道在前后安全范围内是否无车。"""
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
            if actor_wp.road_id != target_wp.road_id or actor_wp.lane_id != target_wp.lane_id:
                continue
            longitudinal = dot_2d(actor.get_location() - ego_loc, forward)
            if -LANE_CLEAR_REAR <= longitudinal <= LANE_CLEAR_FRONT:
                return False
        return True

    def right_side_object(self, route_index):
        """读取右侧非机动车/行人目标，并判断其是否位于当前右转冲突窗口。"""
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

        if best_conflict is not None and best_conflict.is_conflict_object:
            self._right_confirm_count += 1
        else:
            self._right_confirm_count = 0

        result = best_conflict if best_conflict is not None else best_nearby
        if result is None:
            return RightSideObjectReading(float("inf"), float("inf"))
        if result.is_conflict_object and self._right_confirm_count < self._right_confirm_frames:
            result.is_conflict_object = False
            result.risk_level = min(result.risk_level, 1)
        return result

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
        if not self._check_side_fov(longitudinal, lateral) or self._should_miss_detect(distance):
            return None

        noisy_distance = max(0.1, self._add_noise(distance, DISTANCE_STD))
        noisy_longitudinal = self._add_noise(longitudinal, DISTANCE_STD)
        noisy_lateral = self._add_noise(lateral, DISTANCE_STD)
        to_object_length = max(distance, 0.1)
        to_object = carla.Vector3D(relative.x / to_object_length, relative.y / to_object_length, 0.0)
        ego_velocity = self.ego.get_velocity()
        relative_speed = carla.Vector3D(
            ego_velocity.x - scenario.velocity.x,
            ego_velocity.y - scenario.velocity.y,
            ego_velocity.z - scenario.velocity.z,
        )
        closing_speed = dot_2d(relative_speed, to_object)
        noisy_closing_speed = self._add_noise(closing_speed, SPEED_STD)
        ttc = noisy_distance / noisy_closing_speed if noisy_closing_speed > 0.1 else float("inf")

        object_type = ""
        if getattr(scenario, "is_walker", False):
            object_type = "pedestrian"
        elif "bicycle" in getattr(scenario, "name", "") or scenario.actor.type_id.startswith("vehicle."):
            object_type = "bicycle"

        in_geometry_gate = (
            RIGHT_OBJECT_LONGITUDINAL_MIN <= longitudinal <= RIGHT_OBJECT_LONGITUDINAL_MAX
            and RIGHT_OBJECT_LATERAL_MIN <= lateral <= RIGHT_OBJECT_LATERAL_MAX
        )
        is_conflict = scenario.is_active and scenario.is_conflict_window(route_index) and in_geometry_gate
        is_moving_toward = False
        predicted_ttc = float("inf")
        if is_conflict:
            ego_speed = vector_length(ego_velocity)
            object_forward_speed = dot_2d(scenario.velocity, forward)
            relative_long_speed = ego_speed - object_forward_speed
            if relative_long_speed > 0.5 and longitudinal > 0.0:
                is_moving_toward = True
                predicted_ttc = longitudinal / relative_long_speed
                predicted_ttc = self._add_noise(predicted_ttc, 0.3)
            elif RIGHT_OBJECT_LATERAL_MIN < lateral < 10.0:
                is_moving_toward = True

        risk_level = 0
        if is_conflict:
            if ttc < RIGHT_OBJECT_TTC_THRESHOLD or predicted_ttc < TTC_AVOID_THRESHOLD:
                risk_level = 3
            elif noisy_distance < RIGHT_OBJECT_STOP_DISTANCE:
                risk_level = 2
            elif noisy_distance < RIGHT_OBJECT_DETECT_DISTANCE:
                risk_level = 1

        return RightSideObjectReading(
            distance=noisy_distance,
            ttc=ttc,
            is_conflict_object=is_conflict,
            actor_id=scenario.actor.id,
            actor_role=getattr(scenario, "name", scenario.actor.type_id),
            longitudinal=noisy_longitudinal,
            lateral=noisy_lateral,
            risk_level=risk_level,
            is_moving_toward_conflict=is_moving_toward,
            predicted_ttc=predicted_ttc,
            object_type=object_type,
        )

    def assess_risk(self, route_index, use_route_reference=True):
        """统一风险评估接口，当前主循环仍可继续使用旧的显式状态条件。"""
        front = self.front_vehicle(use_route_reference=use_route_reference)
        right = self.right_side_object(route_index)
        close_slow_front_vehicle = (
            front.is_front_vehicle
            and front.distance < 34.0
            and front.closing_speed > 2.0
        )
        brake_needed = front.is_front_vehicle and front.ttc < TTC_BRAKE_THRESHOLD
        emergency_needed = (
            front.is_front_vehicle
            and front.distance < SAFE_DISTANCE
            and (front.ttc < TTC_AVOID_THRESHOLD or close_slow_front_vehicle)
        )
        emergency_recovered = (
            not front.is_front_vehicle
            or front.distance > SAFE_DISTANCE + 8.0
            or front.ttc > TTC_BRAKE_THRESHOLD + 1.0
        )
        right_yield_needed = (
            right.is_conflict_object
            and (
                right.risk_level >= 2
                or right.ttc < RIGHT_OBJECT_TTC_THRESHOLD
                or right.distance < RIGHT_OBJECT_DETECT_DISTANCE
            )
        )
        right_stop_needed = right.is_conflict_object and right.distance < RIGHT_OBJECT_STOP_DISTANCE
        primary_type = "none"
        primary_level = 0
        if emergency_needed:
            primary_type = "front"
            primary_level = 3
        elif right_yield_needed:
            primary_type = "right_object"
            primary_level = max(2, right.risk_level)
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
            right_reading=right,
        )
