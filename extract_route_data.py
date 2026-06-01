"""
Step 1: 从 CARLA 提取路线和路网数据，导出为 JSON
用 carla_env (Python 3.7) 运行
"""

import json
import math
import sys

# ===================== CARLA 连接 =====================
CARLA_EGG = r"D:\17871\CARLA_0.9.15\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.15-py3.7-win-amd64.egg"
if CARLA_EGG not in sys.path:
    sys.path.insert(0, CARLA_EGG)
import carla

# ===================== 从 guiji.py 复制的配置 & 工具函数 =====================
TOWN10_START_SPAWN_INDEX = 141
TOWN10_ROUTE_STEP = 4.0
TOWN10_ROUTE_CLOSE_RADIUS = 8.0
TOWN10_ROUTE_MIN_POINTS_BEFORE_CLOSE = 80
TOWN10_ROUTE_SELF_CLOSE_MIN_SEPARATION = 55
TOWN10_ROUTE_CLOSE_HEADING_DEGREES = 25.0
TOWN10_SHORT_LOOP_BRANCH_OVERRIDES = {
    (5, -1): 795,
    (13, -2): 934,
    (20, -2): 875,
}
TOWN10_RIGHT_TURN_PREPARE_LANE_CHANGES = {(1, 1)}
TOWN10_RIGHT_TURN_PREPARE_MAX_X = 56.0
TOWN10_RIGHT_TURN_PREPARE_HEADING_DEGREES = 180.0
TOWN10_RIGHT_TURN_PREPARE_HEADING_TOLERANCE = 15.0


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def same_direction_lane(source_wp, target_wp):
    if target_wp is None or target_wp.lane_type != carla.LaneType.Driving:
        return False
    yaw_error = abs(
        normalize_angle(
            math.radians(source_wp.transform.rotation.yaw)
            - math.radians(target_wp.transform.rotation.yaw)
        )
    )
    return yaw_error < math.radians(30.0)


def get_town10_start_waypoint(carla_map):
    spawn_points = sorted(
        carla_map.get_spawn_points(),
        key=lambda t: (round(t.location.x, 1), round(t.location.y, 1), round(t.rotation.yaw, 1)),
    )
    if TOWN10_START_SPAWN_INDEX >= len(spawn_points):
        raise RuntimeError("Town10 fixed spawn index is out of range.")
    transform = spawn_points[TOWN10_START_SPAWN_INDEX]
    return carla_map.get_waypoint(
        transform.location, project_to_road=True, lane_type=carla.LaneType.Driving
    )


# ===================== LoopRoute =====================

class LoopRouteForPlot:
    def __init__(self, start_waypoint):
        self.step_distance = TOWN10_ROUTE_STEP
        self.close_radius = TOWN10_ROUTE_CLOSE_RADIUS
        self.waypoints = []
        self.points = []
        self.close_to_index = None
        self.right_lane_prepare_index = None
        self._build_short_town10_route(start_waypoint)

    def _select_next_waypoint(self, waypoint, next_waypoints):
        preferred_road = TOWN10_SHORT_LOOP_BRANCH_OVERRIDES.get(
            (waypoint.road_id, waypoint.lane_id)
        )
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
            self.waypoints.append(waypoint)
            self.points.append(waypoint.transform.location)
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
                            events.append(dict(
                                direction=current_direction, degrees=current_total,
                                start_index=start_index, end_index=last_turn_index,
                            ))
                        current_direction = None
                        current_total = 0.0
                continue
            straight_steps = 0
            last_turn_index = index
            direction = "right" if delta > 0.0 else "left"
            if direction != current_direction:
                if current_direction is not None and abs(current_total) >= min_total_degrees:
                    events.append(dict(
                        direction=current_direction, degrees=current_total,
                        start_index=start_index, end_index=index - 1,
                    ))
                current_direction = direction
                current_total = delta
                start_index = index - 1
            else:
                current_total += delta
        if current_direction is not None and abs(current_total) >= min_total_degrees:
            events.append(dict(
                direction=current_direction, degrees=current_total,
                start_index=start_index, end_index=len(self.waypoints) - 1,
            ))
        return events


def build_road_network(carla_map):
    """从 CARLA topology 构建路网 — 去重道路中心线"""
    topology = carla_map.get_topology()
    road_segments = {}  # road_id -> list of dicts with x, y, lane_id

    for w1, w2 in topology:
        rid = w1.road_id
        lid = w1.lane_id
        if rid not in road_segments:
            road_segments[rid] = {}
        if lid not in road_segments[rid]:
            road_segments[rid][lid] = []
        x1, y1 = w1.transform.location.x, w1.transform.location.y
        x2, y2 = w2.transform.location.x, w2.transform.location.y
        road_segments[rid][lid].append({"x1": x1, "y1": y1, "x2": x2, "y2": y2})

    # 转换为可用于绘图的格式：每条道路取一条有代表性的 lane 的中心线
    road_lines = []
    for rid, lanes in road_segments.items():
        # 取点数最多的 lane
        best_lane = max(lanes.values(), key=lambda segs: len(segs))
        # 从 segs 提取有序点
        pts_x, pts_y = [], []
        if best_lane:
            pts_x.append(best_lane[0]["x1"])
            pts_y.append(best_lane[0]["y1"])
            for seg in best_lane:
                pts_x.append(seg["x2"])
                pts_y.append(seg["y2"])
        if pts_x:
            road_lines.append({"road_id": rid, "x": pts_x, "y": pts_y})

    return road_lines


# ===================== 主逻辑 =====================

def main():
    print("Step 1: 连接 CARLA ...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    carla_map = world.get_map()
    print(f"地图: {carla_map.name}")

    # 1. 构建自车路线
    print("构建 LoopRoute ...")
    start_wp = get_town10_start_waypoint(carla_map)
    route = LoopRouteForPlot(start_wp)
    turn_events = route._detect_turn_events()
    route_len = route.step_distance * (len(route.points) - 1)
    print(f"路点: {len(route.waypoints)}, 长度≈{route_len:.0f}m, 转弯: {len(turn_events)}处")

    # 序列化路线数据
    route_data = {
        "waypoints": [
            {
                "x": wp.transform.location.x,
                "y": wp.transform.location.y,
                "z": wp.transform.location.z,
                "yaw": wp.transform.rotation.yaw,
                "road_id": wp.road_id,
                "lane_id": wp.lane_id,
                "section_id": wp.section_id,
            }
            for wp in route.waypoints
        ],
        "close_to_index": route.close_to_index,
        "right_lane_prepare_index": route.right_lane_prepare_index,
        "step_distance": route.step_distance,
        "route_length": route_len,
        "turn_events": turn_events,
        "start_point": {
            "x": route.waypoints[0].transform.location.x,
            "y": route.waypoints[0].transform.location.y,
        },
    }

    # 2. 构建路网
    print("构建路网 topology ...")
    road_lines = build_road_network(carla_map)
    print(f"路网: {len(road_lines)} 条道路")

    # 3. 导出 JSON
    output = {"route": route_data, "road_network": road_lines, "map_name": carla_map.name}
    output_path = "trajectory_data.json"
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(output, f, ensure_ascii=False)
    print(f"✅ 数据已导出: {output_path} ({len(json.dumps(output))} bytes)")


if __name__ == "__main__":
    main()
