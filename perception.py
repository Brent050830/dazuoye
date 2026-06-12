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
from utils import SmoothRouteReference, dot_2d, smooth_reference_for, vector_length


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
class ReplacementSegment:
    """当前跟踪路线中替换基础路线的一段。"""

    start_s: float
    end_s: float
    points: list
    end_offset: float = 0.0


class TrackingRoute:
    """当前用于前车投影筛选的跟踪路线。"""

    def __init__(
        self,
        points,
        reference=None,
        loop_route=None,
        is_temporary=False,
        ego_search_back=20.0,
        ego_search_ahead=90.0,
        target_search_back=12.0,
        target_search_ahead=LANE_CLEAR_FRONT + 35.0,
    ):
        self.points = list(points)
        self.reference = reference
        self.loop_route = loop_route
        self.is_temporary = is_temporary
        self.ego_search_back = ego_search_back
        self.ego_search_ahead = ego_search_ahead
        self.target_search_back = target_search_back
        self.target_search_ahead = target_search_ahead
        self.step_distance = loop_route.step_distance if loop_route is not None else 2.0
        self.cumulative = [0.0]
        for before, after in zip(self.points, self.points[1:]):
            self.cumulative.append(self.cumulative[-1] + before.distance(after))
        if self.reference is None and len(self.points) >= 2:
            self.reference = SmoothRouteReference(self)
        self.length = self.reference.max_s if self.reference is not None else 0.0
        self.is_route_relative = True

    @classmethod
    def from_loop_route(cls, loop_route):
        if loop_route is None:
            return None
        step = max(loop_route.step_distance, 0.001)
        return cls(
            loop_route.points,
            reference=smooth_reference_for(loop_route),
            loop_route=loop_route,
            ego_search_back=6.0 * step,
            ego_search_ahead=18.0 * step,
            target_search_back=3.0 * step,
            target_search_ahead=(int((LANE_CLEAR_FRONT + 20.0) / step) + 8) * step,
        )

    @classmethod
    def from_base_with_replacements(cls, base_route, segments):
        if base_route is None:
            return None
        points = _compose_tracking_points(base_route, segments)
        return cls(
            points,
            loop_route=base_route.loop_route,
            is_temporary=bool(segments),
            ego_search_back=base_route.ego_search_back,
            ego_search_ahead=base_route.ego_search_ahead,
            target_search_back=base_route.target_search_back,
            target_search_ahead=base_route.target_search_ahead,
        )

    @property
    def is_valid(self):
        return self.reference is not None and len(self.points) >= 2 and self.cumulative[-1] > 0.1

    @property
    def center_s(self):
        if self.loop_route is None:
            return 0.0
        return self.loop_route.last_index * self.loop_route.step_distance

    def location_at(self, s):
        return self.reference.location_at_route_s(s)

    def reference_yaw_at(self, s):
        return self.reference.yaw_at_route_s(s)

    def to_local(self, location):
        projection = self.reference.project(
            location,
            self.center_s,
            search_back=self.ego_search_back,
            search_ahead=self.ego_search_ahead,
        )
        return projection["route_s"], projection["lateral"]


def _compose_tracking_points(base_route, segments):
    step = max(1.0, getattr(base_route, "step_distance", 2.0))
    max_s = base_route.reference.max_s
    sorted_segments = sorted(segments, key=lambda segment: segment.start_s)
    points = []
    cursor = 0.0
    current_offset = 0.0

    for segment in sorted_segments:
        start_s = max(0.0, min(segment.start_s, max_s))
        end_s = max(start_s, min(segment.end_s, max_s))
        _append_base_samples(points, base_route, cursor, start_s, step, current_offset)
        _append_points(points, segment.points)
        cursor = end_s
        current_offset = segment.end_offset

    _append_base_samples(points, base_route, cursor, max_s, step, current_offset)
    return points


def _append_base_samples(points, route, start_s, end_s, step, lateral_offset=0.0):
    if end_s < start_s:
        return
    sample_count = max(0, int(math.ceil((end_s - start_s) / step)))
    for index in range(sample_count + 1):
        s = min(end_s, start_s + index * step)
        _append_point(points, _offset_route_location(route, s, lateral_offset))


def _offset_route_location(route, route_s, lateral_offset):
    location = route.reference.location_at_route_s(route_s)
    if abs(lateral_offset) < 0.01:
        return location
    right = route.reference.right_at_route_s(route_s)
    return location + carla.Location(x=right.x * lateral_offset, y=right.y * lateral_offset, z=0.0)


def _append_points(points, new_points):
    for point in new_points:
        _append_point(points, point)


def _append_point(points, point):
    if not points or points[-1].distance(point) > 0.05:
        points.append(point)


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
        self._right_confirm_count = 0
        self._right_confirm_frames = RIGHT_CONFIRM_FRAMES
        self._noise_rng = random.Random(20260606)
        self._radar_detections = []
        self._base_tracking_route = TrackingRoute.from_loop_route(loop_route)
        self._replacement_segments = []
        self._tracking_route = self._base_tracking_route
        self._current_offset = 0.0

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

    def tracking_route(self):
        return self._tracking_route

    def replacement_segments(self):
        return list(self._replacement_segments)

    def current_offset(self):
        return self._current_offset

    def apply_replacement_segment(self, start_s, end_s, points, end_offset=0.0):
        """用规划出的路径点替换基础路线上的一个 s 区间。"""
        if self._base_tracking_route is None or end_s <= start_s:
            return
        segment = ReplacementSegment(start_s, end_s, list(points), end_offset)
        self._replacement_segments = [
            existing
            for existing in self._replacement_segments
            if existing.end_s <= segment.start_s or existing.start_s >= segment.end_s
        ]
        self._replacement_segments.append(segment)
        self._current_offset = segment.end_offset
        self._tracking_route = TrackingRoute.from_base_with_replacements(
            self._base_tracking_route, self._replacement_segments
        )

    def front_vehicle(self):
        """兼容旧接口：返回前方最近同车道车辆。"""
        readings = self.front_vehicles()
        for reading in readings:
            if reading.is_front_vehicle and reading.is_same_lane:
                return reading
        return self._empty_front()

    def front_vehicles(self):
        """返回前方候选车辆列表。"""
        if self._tracking_route is not None:
            return self._tracking_route_front_vehicles()
        if RADAR_ENABLED and self._radar_detections:
            return self._radar_front_vehicles()
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

    def _tracking_route_front_vehicles(self):
        """把车辆投影到当前跟踪路线，用弧长 s 和横向 d 判断前车。"""
        route = self._tracking_route
        if route is None or not route.is_valid:
            return self._ego_frame_front_vehicles()

        ego_loc = self.ego.get_location()
        ego_projection = self._project_to_tracking_route(
            ego_loc,
            route,
            center_s=route.center_s,
            search_back=route.ego_search_back,
            search_ahead=route.ego_search_ahead,
        )
        lane_width = max(self.carla_map.get_waypoint(ego_loc).lane_width, 2.5)
        ego_speed_along = self._speed_along_projection(self.ego, ego_projection)

        readings = []
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive:
                continue

            target_projection = self._project_to_tracking_route(
                vehicle.get_location(),
                route,
                center_s=ego_projection["route_s"],
                search_back=route.target_search_back,
                search_ahead=route.target_search_ahead,
            )
            longitudinal = target_projection["route_s"] - ego_projection["route_s"]
            lateral = target_projection["lateral"] - ego_projection["lateral"]
            lane_relative = lateral / lane_width
            if longitudinal <= 0.0 or abs(lane_relative) >= FRONT_LANE_ADJACENT_THRESHOLD:
                continue
            if not self._check_front_fov(longitudinal, lateral):
                continue
            if self._should_miss_detect(math.sqrt(longitudinal * longitudinal + lateral * lateral)):
                continue

            target_speed_along = self._speed_along_projection(vehicle, target_projection)
            readings.append(
                self._make_front_reading(
                    distance=longitudinal,
                    closing_speed=ego_speed_along - target_speed_along,
                    lateral=lateral,
                    lane_relative=lane_relative,
                    is_same_lane=abs(lateral) < lane_width * 0.45,
                    actor_id=vehicle.id,
                    actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
                    target_speed_along=target_speed_along,
                )
            )

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

    def _project_to_tracking_route(self, location, route, center_s, search_back, search_ahead):
        """把位置投影到当前跟踪路线。"""
        return route.reference.project(
            location,
            center_s,
            search_back=search_back,
            search_ahead=search_ahead,
        )

    def _speed_along_projection(self, vehicle, projection):
        """计算车辆在投影参考线切线方向上的速度分量。"""
        right = projection["right"]
        tangent = carla.Vector3D(x=right.y, y=-right.x, z=0.0)
        return dot_2d(vehicle.get_velocity(), tangent)

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
