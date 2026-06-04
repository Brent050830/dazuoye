from dataclasses import dataclass

import carla

from config import LANE_CLEAR_FRONT, LANE_CLEAR_REAR
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
        front_extra_vehicles=None,
        right_object_scenarios=None,
    ):
        self.world = world
        self.carla_map = carla_map
        self.ego = ego_vehicle
        self.lead = lead_vehicle
        self.front_extra_vehicles = front_extra_vehicles or []
        self.right_object_scenarios = right_object_scenarios or []

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
                closest = FrontVehicleReading(longitudinal, closing_speed, ttc, lateral, True)

        return closest # 返回最近的前方同车道车辆的感知信息，包括距离、接近速度、TTC、横向偏移和是否确认为正前方车辆（没有正前方的话就是初始值）

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


