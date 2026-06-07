from dataclasses import dataclass

import carla

from config import (
    LANE_CLEAR_FRONT,
    LANE_CLEAR_REAR,
    RIGHT_OBJECT_LATERAL_MAX,
    RIGHT_OBJECT_LATERAL_MIN,
    RIGHT_OBJECT_LONGITUDINAL_MAX,
    RIGHT_OBJECT_LONGITUDINAL_MIN,
)
from utils import dot_2d, same_direction_lane, vector_length


# ===================== 传感器模块 =====================

@dataclass
class FrontVehicleReading:
    """前车感知数据结构"""
    distance: float       # 纵向间距（米）
    closing_speed: float  # 接近速度（m/s，正值表示靠近）
    ttc: float            # 碰撞时间（秒）
    lateral_offset: float # 横向偏移（米）
    is_front_vehicle: bool  # 是否确认为正前方车辆
    actor_id: int = None
    actor_role: str = ""
    target_speed_along: float = 0.0


@dataclass
class RightSideObjectReading:
    """右侧非机动车/行人目标的虚拟感知数据。"""
    distance: float
    ttc: float
    is_conflict_object: bool
    actor_id: int = None
    actor_role: str = ""
    longitudinal: float = 0.0
    lateral: float = 0.0


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
        loop_route=None,
    ):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle
        self.front_extra_vehicles = front_extra_vehicles or []
        self.right_object_scenarios = right_object_scenarios or []
        self.loop_route = loop_route

    def front_vehicle(self, use_route_reference=True):
        """计算同车道前方最近车辆的纵向距离、横向偏移、接近速度和TTC。"""
        if use_route_reference and self.loop_route is not None:
            return self._route_front_vehicle()
        return self._ego_frame_front_vehicle()

    def _ego_frame_front_vehicle(self):
        """用自车当前直角坐标系读取前方车辆，作为无路线参考线时的兜底方案。"""
        ego_tf = self.ego.get_transform()
        ego_loc = ego_tf.location
        forward = ego_tf.get_forward_vector()
        right = ego_tf.get_right_vector()
        lane_width = self.carla_map.get_waypoint(ego_loc).lane_width
        ego_speed_along = dot_2d(self.ego.get_velocity(), forward)

        closest = FrontVehicleReading(float("inf"), 0.0, float("inf"), 0.0, False)
        for vehicle in [self.lead] + self.front_extra_vehicles:
            if vehicle is None or not vehicle.is_alive: # 如果车辆不存在或已销毁，跳过该车辆的计算
                continue

            relative = vehicle.get_location() - ego_loc
            longitudinal = dot_2d(relative, forward) # 计算该车辆相对于自车在前后方向上的距离，正值表示在前方，负值表示在后方
            lateral = dot_2d(relative, right) # 计算该车辆相对于自车在左右方向上的距离，正值表示在右侧，负值表示在左侧
            if longitudinal <= 0.0 or abs(lateral) >= lane_width * 0.65: # 只考虑前方同车道车辆，纵向必须在前方，横向必须在车道范围内（这里取0.65倍车道宽作为安全边界）
                continue

            target_speed_along = dot_2d(vehicle.get_velocity(), forward)
            closing_speed = ego_speed_along - target_speed_along
            ttc = longitudinal / closing_speed if closing_speed > 0.1 else float("inf")
            if longitudinal < closest.distance: # 如果该车辆比当前最近的车辆更近，则更新最近车辆的信息
                closest = FrontVehicleReading(
                    distance=longitudinal,
                    closing_speed=closing_speed,
                    ttc=ttc,
                    lateral_offset=lateral,
                    is_front_vehicle=True,
                    actor_id=vehicle.id,
                    actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
                    target_speed_along=target_speed_along,
                )

        return closest # 返回最近的前方同车道车辆的感知信息，包括距离、接近速度、TTC、横向偏移和是否确认为正前方车辆（没有正前方的话就是初始值）

    def _route_front_vehicle(self):
        """将车辆位置投影到当前路线局部弧线，用弧长 s 和横向 d 判断弯道前车。"""
        ego_loc = self.ego.get_location()
        ego_projection = self._project_to_route(ego_loc, self.loop_route.last_index, search_back=6, search_ahead=18)
        lane_width = self.carla_map.get_waypoint(ego_loc).lane_width
        ego_speed_along = self._speed_along_route(self.ego, ego_projection)
        closest = FrontVehicleReading(float("inf"), 0.0, float("inf"), 0.0, False)

        search_ahead = int((LANE_CLEAR_FRONT + 20.0) / self.loop_route.step_distance) + 8
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
            if longitudinal <= 0.0 or abs(lateral) >= lane_width * 0.45:
                continue

            target_speed_along = self._speed_along_route(vehicle, target_projection)
            closing_speed = ego_speed_along - target_speed_along
            ttc = longitudinal / closing_speed if closing_speed > 0.1 else float("inf")
            if longitudinal < closest.distance:
                closest = FrontVehicleReading(
                    distance=longitudinal,
                    closing_speed=closing_speed,
                    ttc=ttc,
                    lateral_offset=lateral,
                    is_front_vehicle=True,
                    actor_id=vehicle.id,
                    actor_role=vehicle.attributes.get("role_name", vehicle.type_id),
                    target_speed_along=target_speed_along,
                )

        return closest

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
                lateral = dot_2d(location - projected, right)
                best = {
                    "raw_index": raw_index,
                    "location": projected,
                    "right": right,
                    "lateral": lateral,
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

        in_geometry_gate = (
            RIGHT_OBJECT_LONGITUDINAL_MIN <= longitudinal <= RIGHT_OBJECT_LONGITUDINAL_MAX
            and RIGHT_OBJECT_LATERAL_MIN <= lateral <= RIGHT_OBJECT_LATERAL_MAX
        )
        is_conflict = (
            scenario.is_active
            and scenario.is_conflict_window(route_index)
            and in_geometry_gate
        )
        return RightSideObjectReading(
            distance,
            ttc,
            is_conflict,
            scenario.actor.id,
            scenario.name,
            longitudinal,
            lateral,
        )


