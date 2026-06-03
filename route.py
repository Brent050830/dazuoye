import math

from config import (
    TOWN10_RIGHT_TURN_PREPARE_HEADING_DEGREES,
    TOWN10_RIGHT_TURN_PREPARE_HEADING_TOLERANCE,
    TOWN10_RIGHT_TURN_PREPARE_LANE_CHANGES,
    TOWN10_RIGHT_TURN_PREPARE_MAX_X,
    TOWN10_ROUTE_CLOSE_HEADING_DEGREES,
    TOWN10_ROUTE_CLOSE_RADIUS,
    TOWN10_ROUTE_MIN_POINTS_BEFORE_CLOSE,
    TOWN10_ROUTE_SELF_CLOSE_MIN_SEPARATION,
    TOWN10_ROUTE_STEP,
    TOWN10_SHORT_LOOP_BRANCH_OVERRIDES,
)
from utils import clamp, normalize_angle, same_direction_lane, yaw_to_rad


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


