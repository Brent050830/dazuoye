"""通用数学、车辆状态和基础道路辅助函数。"""

import math
import os
import sys

import carla

from config import TOWN10_START_SPAWN_INDEX

if os.name == "nt":
    _conda_dll_dir = os.path.join(os.path.dirname(sys.executable), "Library", "bin")
    if os.path.isdir(_conda_dll_dir) and _conda_dll_dir not in os.environ.get("PATH", ""):
        os.environ["PATH"] = _conda_dll_dir + os.pathsep + os.environ.get("PATH", "")

try:
    from scipy.interpolate import CubicSpline
except Exception:
    CubicSpline = None


def clamp(value, low, high):
    """将 value 限制在 [low, high] 范围内。"""
    return max(low, min(high, value))


def vector_length(vector):
    """计算三维向量的欧几里得长度。"""
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)


def dot_2d(a, b):
    """计算两个向量在水平面（XY）上的点积。"""
    return a.x * b.x + a.y * b.y


class SmoothRouteReference:
    """基于累计弧长和样条的平滑路线参考线。"""

    def __init__(self, loop_route):
        self.loop_route = loop_route
        self.s_values = []
        self.points = []
        cumulative = 0.0
        previous = None

        for point in loop_route.points:
            if previous is not None:
                distance = point.distance(previous)
                if distance < 0.01:
                    continue
                cumulative += distance
            self.points.append(point)
            self.s_values.append(cumulative)
            previous = point

        self.max_s = self.s_values[-1] if self.s_values else 0.0
        self._use_spline = CubicSpline is not None and len(self.points) >= 4 and self.max_s > 1.0
        if self._use_spline:
            self._spline_x = CubicSpline(self.s_values, [point.x for point in self.points], bc_type="natural")
            self._spline_y = CubicSpline(self.s_values, [point.y for point in self.points], bc_type="natural")
            self._spline_z = CubicSpline(self.s_values, [point.z for point in self.points], bc_type="natural")
        else:
            self._spline_x = None
            self._spline_y = None
            self._spline_z = None

    def clamp_s(self, route_s):
        return clamp(route_s, 0.0, self.max_s)

    def location_at_route_s(self, route_s):
        route_s = self.clamp_s(route_s)
        if self._use_spline:
            return carla.Location(
                x=float(self._spline_x(route_s)),
                y=float(self._spline_y(route_s)),
                z=float(self._spline_z(route_s)),
            )
        return self._linear_location_at(route_s)

    def tangent_at_route_s(self, route_s):
        route_s = self.clamp_s(route_s)
        if self._use_spline:
            dx = float(self._spline_x(route_s, 1))
            dy = float(self._spline_y(route_s, 1))
        else:
            before = self.location_at_route_s(route_s - 0.75)
            after = self.location_at_route_s(route_s + 0.75)
            dx = after.x - before.x
            dy = after.y - before.y

        length = math.sqrt(dx * dx + dy * dy)
        if length < 0.001:
            return carla.Vector3D(x=1.0, y=0.0, z=0.0)
        return carla.Vector3D(x=dx / length, y=dy / length, z=0.0)

    def right_at_route_s(self, route_s):
        tangent = self.tangent_at_route_s(route_s)
        return carla.Vector3D(x=-tangent.y, y=tangent.x, z=0.0)

    def yaw_at_route_s(self, route_s):
        tangent = self.tangent_at_route_s(route_s)
        return math.atan2(tangent.y, tangent.x)

    def project(self, location, center_route_s, search_back=12.0, search_ahead=80.0):
        start_s = self.clamp_s(center_route_s - search_back)
        end_s = self.clamp_s(center_route_s + search_ahead)
        if end_s <= start_s:
            end_s = min(self.max_s, start_s + max(search_ahead, 1.0))

        best_s = self._nearest_sample_s(location, start_s, end_s, 1.0)
        best_s = self._nearest_sample_s(location, max(start_s, best_s - 1.5), min(end_s, best_s + 1.5), 0.20)

        route_location = self.location_at_route_s(best_s)
        route_right = self.right_at_route_s(best_s)
        return {
            "route_s": best_s,
            "location": route_location,
            "right": route_right,
            "lateral": dot_2d(location - route_location, route_right),
            "error": route_location.distance(location),
        }

    def _nearest_sample_s(self, location, start_s, end_s, step):
        best_s = start_s
        best_distance = float("inf")
        count = max(1, int((end_s - start_s) / step))
        for index in range(count + 1):
            sample_s = min(end_s, start_s + index * step)
            sample_location = self.location_at_route_s(sample_s)
            distance = sample_location.distance(location)
            if distance < best_distance:
                best_distance = distance
                best_s = sample_s
        return best_s

    def _linear_location_at(self, route_s):
        if not self.points:
            return carla.Location()
        if route_s <= 0.0:
            return self.points[0]
        if route_s >= self.max_s:
            return self.points[-1]
        for index in range(len(self.s_values) - 1):
            s0 = self.s_values[index]
            s1 = self.s_values[index + 1]
            if s0 <= route_s <= s1:
                blend = (route_s - s0) / max(s1 - s0, 0.001)
                p0 = self.points[index]
                p1 = self.points[index + 1]
                return carla.Location(
                    x=p0.x + (p1.x - p0.x) * blend,
                    y=p0.y + (p1.y - p0.y) * blend,
                    z=p0.z + (p1.z - p0.z) * blend,
                )
        return self.points[-1]


def smooth_reference_for(loop_route):
    """缓存并返回路线对应的平滑参考线。"""
    reference = getattr(loop_route, "_smooth_reference", None)
    if reference is None:
        reference = SmoothRouteReference(loop_route)
        setattr(loop_route, "_smooth_reference", reference)
    return reference


def normalize_angle(angle):
    """将任意角度归一化到 (-π, π] 区间。"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_rad(rotation):
    """将 CARLA Rotation 的偏航角（度）转换为弧度。"""
    return math.radians(rotation.yaw)


def get_speed(vehicle):
    """获取车辆当前速度的标量值（m/s）。"""
    return vector_length(vehicle.get_velocity())


def speed_control(current_speed, target_speed):
    """简单比例速度控制器，返回 (油门, 制动) 元组。"""
    error = target_speed - current_speed
    if error >= 0.0:
        return clamp(0.18 + 0.06 * error, 0.0, 0.75), 0.0
    return 0.0, clamp(-0.12 * error, 0.0, 0.75)


def waypoint_steer(vehicle, carla_map, lookahead=12.0):
    """基于前视路点的纯追踪转向控制，返回归一化转向量。"""
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
    """从路点生成车辆生成位置，Z 轴抬高 0.45m 避免穿地。"""
    transform = waypoint.transform
    transform.location.z += 0.45
    return transform


def same_direction_lane(source_wp, target_wp):
    """判断目标路点是否为与源路点同向的行驶车道。"""
    if target_wp is None or target_wp.lane_type != carla.LaneType.Driving:
        return False
    yaw_error = abs(
        normalize_angle(yaw_to_rad(source_wp.transform.rotation) - yaw_to_rad(target_wp.transform.rotation))
    )
    return yaw_error < math.radians(30.0)


def get_town10_start_waypoint(carla_map):
    """获取 Town10 固定起点，确保每次仿真从同一位置开始。"""
    spawn_points = sorted(
        carla_map.get_spawn_points(),
        key=lambda transform: (
            round(transform.location.x, 1),
            round(transform.location.y, 1),
            round(transform.rotation.yaw, 1),
        ),
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
