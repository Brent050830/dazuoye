"""
Town10 车辆行驶轨迹可视化脚本
- 绘制完整的地图路网（从 CARLA topology）
- 绘制自车规划路线（LoopRoute）
- 标注道路编号（road_id）
- 标注关键事件点（起点、转弯、闭合点、右侧车道预换点）
"""

import math
import sys

# ===================== CARLA 连接 =====================
CARLA_EGG = r"D:\17871\CARLA_0.9.15\WindowsNoEditor\PythonAPI\carla\dist\carla-0.9.15-py3.7-win-amd64.egg"
if CARLA_EGG not in sys.path:
    sys.path.insert(0, CARLA_EGG)
import carla

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

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


# ===================== LoopRoute（与 guiji.py 相同逻辑） =====================

class LoopRouteForPlot:
    """复制 guiji.py 中 LoopRoute 的路线构建逻辑，用于获取路点序列"""

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
                math.radians(
                    waypoint.transform.rotation.yaw
                    - TOWN10_RIGHT_TURN_PREPARE_HEADING_DEGREES
                )
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
            yaw_error = abs(
                normalize_angle(math.radians(yaw - candidate.transform.rotation.yaw))
            )
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
            delta = math.degrees(
                normalize_angle(math.radians(current_yaw - previous_yaw))
            )
            if abs(delta) < 2.0:
                if current_direction is not None:
                    straight_steps += 1
                    if straight_steps >= 5:
                        if abs(current_total) >= min_total_degrees:
                            events.append(
                                dict(
                                    direction=current_direction,
                                    degrees=current_total,
                                    start_index=start_index,
                                    end_index=last_turn_index,
                                )
                            )
                        current_direction = None
                        current_total = 0.0
                continue

            straight_steps = 0
            last_turn_index = index
            direction = "right" if delta > 0.0 else "left"
            if direction != current_direction:
                if current_direction is not None and abs(current_total) >= min_total_degrees:
                    events.append(
                        dict(
                            direction=current_direction,
                            degrees=current_total,
                            start_index=start_index,
                            end_index=index - 1,
                        )
                    )
                current_direction = direction
                current_total = delta
                start_index = index - 1
            else:
                current_total += delta

        if current_direction is not None and abs(current_total) >= min_total_degrees:
            events.append(
                dict(
                    direction=current_direction,
                    degrees=current_total,
                    start_index=start_index,
                    end_index=len(self.waypoints) - 1,
                )
            )
        return events


# ===================== 主绘图逻辑 =====================

def build_road_network(carla_map):
    """从 CARLA topology 构建路网：去重后的道路中心线片段列表"""
    topology = carla_map.get_topology()
    segments = {}  # (road_id, lane_id) -> list of (x, y)

    for w1, w2 in topology:
        # 只保留单向道的主侧或双向道的正方向来避免重复
        key = (w1.road_id, w1.lane_id)
        x1, y1 = w1.transform.location.x, w1.transform.location.y
        x2, y2 = w2.transform.location.x, w2.transform.location.y

        if key not in segments:
            segments[key] = [(x1, y1)]
        segments[key].append((x2, y2))

    # 合并相同 road_id 的连续片段
    road_lines = {}  # road_id -> [(x1,y1), (x2,y2), ...]
    for (road_id, lane_id), pts in segments.items():
        if road_id not in road_lines:
            road_lines[road_id] = pts
        else:
            road_lines[road_id].extend(pts)

    return road_lines


def plot_all(ax, road_lines, route, turn_events, close_to_index, right_lane_prepare_index):
    """主绘制函数"""

    # ---- 1. 绘制背景路网（浅灰色） ----
    for road_id, pts in road_lines.items():
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color="#E0E0E0", linewidth=0.8, zorder=1)

    # ---- 2. 绘制自车路线（渐变色，粗线） ----
    route_x = [wp.transform.location.x for wp in route.waypoints]
    route_y = [wp.transform.location.y for wp in route.waypoints]

    n_pts = len(route.waypoints)
    colors = plt.cm.plasma(np.linspace(0, 1, n_pts))

    for i in range(n_pts - 1):
        ax.plot(
            route_x[i : i + 2],
            route_y[i : i + 2],
            color=colors[i],
            linewidth=3.0,
            solid_capstyle="round",
            zorder=5,
        )

    # ---- 3. 标注道路编号 (road_id) ----
    last_road_id = None
    road_label_positions = []

    for i, wp in enumerate(route.waypoints):
        rid = wp.road_id
        if rid != last_road_id:
            # 在新 road_id 的起点标号
            road_label_positions.append(
                (wp.transform.location.x, wp.transform.location.y, rid, i)
            )
            last_road_id = rid
        # 每隔 15 个路点在长路段上重复标注
        elif i % 15 == 0 and i > 0:
            if i - road_label_positions[-1][3] >= 12 if road_label_positions else True:
                road_label_positions.append(
                    (wp.transform.location.x, wp.transform.location.y, rid, i)
                )

    # 去重：相近位置的标注只保留一个
    filtered_labels = []
    for x, y, rid, idx in road_label_positions:
        too_close = False
        for fx, fy, _, _ in filtered_labels:
            if math.sqrt((x - fx) ** 2 + (y - fy) ** 2) < 12.0:
                too_close = True
                break
        if not too_close:
            filtered_labels.append((x, y, rid, idx))

    for x, y, rid, _ in filtered_labels:
        # 文字加白底，增加可读性
        ax.annotate(
            f"R{rid}",
            xy=(x, y),
            fontsize=7,
            fontweight="bold",
            color="#1A237E",
            bbox=dict(
                boxstyle="round,pad=0.25",
                facecolor="white",
                edgecolor="#1A237E",
                alpha=0.85,
                linewidth=0.6,
            ),
            zorder=10,
        )

    # ---- 4. 标记关键点 ----
    # 起点（绿色圆形）
    start_loc = route.waypoints[0].transform.location
    ax.scatter(
        start_loc.x, start_loc.y, c="#2E7D32", s=180, marker="o",
        edgecolors="white", linewidth=1.5, zorder=15, label="起点 (Start)"
    )

    # 终点/闭合点（如果存在）
    if close_to_index is not None and close_to_index < len(route.waypoints):
        close_wp = route.waypoints[close_to_index]
        ax.scatter(
            close_wp.transform.location.x,
            close_wp.transform.location.y,
            c="#C62828", s=140, marker="D",
            edgecolors="white", linewidth=1.2, zorder=15, label="闭合点 (Close)"
        )

    # 右侧车道预换点（如果存在）
    if right_lane_prepare_index is not None and right_lane_prepare_index < len(route.waypoints):
        rl_wp = route.waypoints[right_lane_prepare_index]
        ax.scatter(
            rl_wp.transform.location.x,
            rl_wp.transform.location.y,
            c="#E65100", s=120, marker="s",
            edgecolors="white", linewidth=1.2, zorder=15, label="右道预换点 (R-Lane prep.)"
        )

    # ---- 5. 标注转弯事件 ----
    turn_colors = {"left": "#1565C0", "right": "#C62828"}
    for evt in turn_events:
        si, ei = evt["start_index"], evt["end_index"]
        # 在转弯起止点的弧线范围内标注
        mid_i = (si + ei) // 2
        if 0 <= mid_i < len(route.waypoints):
            wp = route.waypoints[mid_i]
            color = turn_colors.get(evt["direction"], "#333333")
            ax.annotate(
                f"{evt['direction']}\n{evt['degrees']:.0f}°",
                xy=(wp.transform.location.x, wp.transform.location.y),
                fontsize=6,
                color="white",
                fontweight="bold",
                ha="center",
                va="center",
                bbox=dict(
                    boxstyle="round,pad=0.3",
                    facecolor=color,
                    edgecolor="none",
                    alpha=0.8,
                ),
                zorder=12,
            )

    # ---- 6. 图例与装饰 ----
    legend_elements = [
        mpatches.Patch(color="#2E7D32", label="起点 (Start)"),
        mpatches.Patch(color="#C62828", label="闭合点 (Close)"),
        mpatches.Patch(color="#E65100", label="右道预换点"),
        mpatches.Patch(color="#1565C0", label="左转 (Left Turn)"),
        mpatches.Patch(color="#C62828", label="右转 (Right Turn)"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=8, framealpha=0.9)

    # ---- 7. 坐标轴与标题 ----
    ax.set_xlabel("X (m)", fontsize=11)
    ax.set_ylabel("Y (m)", fontsize=11)

    # 自动缩放：聚焦在路线周围，留 10% 边距
    all_x = route_x
    all_y = route_y
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    x_margin = (x_max - x_min) * 0.12 + 20
    y_margin = (y_max - y_min) * 0.12 + 20
    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25, linestyle="--", linewidth=0.5)

    title = (
        f"Town10HD_Opt 行驶轨迹图\n"
        f"路线长度 ≈ {route_len:.0f}m  |  "
        f"路点数: {len(route.waypoints)}  |  "
        f"转弯: {len(turn_events)} 处  |  "
        f"涉及 {len(set(wp.road_id for wp in route.waypoints))} 条道路"
    )
    ax.set_title(title, fontsize=13, fontweight="bold", pad=14)


# ===================== 入口 =====================

if __name__ == "__main__":
    print("连接 CARLA ...")
    client = carla.Client("localhost", 2000)
    client.set_timeout(30.0)
    world = client.get_world()
    carla_map = world.get_map()
    print(f"地图: {carla_map.name}")

    # 1. 构建自车路线
    print("构建路线 ...")
    start_wp = get_town10_start_waypoint(carla_map)
    route = LoopRouteForPlot(start_wp)
    turn_events = route._detect_turn_events()
    route_len = route.step_distance * (len(route.points) - 1)
    print(
        f"路线: {len(route.waypoints)} 路点, "
        f"长度 ≈ {route_len:.0f}m, "
        f"转弯: {len(turn_events)} 处, "
        f"涉及道路: {len(set(wp.road_id for wp in route.waypoints))} 条"
    )

    # 2. 构建路网
    print("构建路网 ...")
    road_lines = build_road_network(carla_map)
    print(f"路网: {len(road_lines)} 条道路")

    # 3. 绘图
    print("生成轨迹图 ...")
    fig, ax = plt.subplots(figsize=(22, 18))
    plot_all(
        ax,
        road_lines,
        route,
        turn_events,
        route.close_to_index,
        route.right_lane_prepare_index,
    )

    output_path = "trajectory_map.png"
    fig.savefig(output_path, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"✅ 轨迹图已保存: {output_path}")
