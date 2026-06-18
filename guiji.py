import argparse
import math
import random
import time
from dataclasses import dataclass

import carla

from actors import ( # 场景中涉及的各种演员生成函数，包括自车、前车、背景车辆、背景自行车和右侧过街自行车
    spawn_background_r344_bicycles,
    spawn_background_route_vehicles,
    spawn_right_side_bicycle_crossing,
    spawn_right_side_pedestrians, # 生成右侧过街行人，增加右侧物体避让的复杂性
    spawn_scenario, # 生成自车和前车，并返回自车的起始路点
    spawn_slow_right_lane_vehicle, # 生成右侧慢速车辆，增加右侧物体避让的复杂性
)

from config import ( # 仿真参数配置，包括服务器连接、仿真时间、车辆目标速度、避障触发和右侧让行阈值等
    CLIENT_TIMEOUT,
    DEBUG_DRAW_INTERVAL_FRAMES, # 调试绘制的帧间隔，控制仿真中轨迹和目标点的绘制频率，避免过于密集导致画面混乱
    DEBUG_DRAW_LIFETIME,
    DEBUG_DRAW_LOOKAHEAD_DISTANCE,
    DEBUG_DRAW_MAX_ALTERNATIVE_TRAJECTORIES,
    DEBUG_DRAW_SENSOR_OVERLAY,
    DEBUG_DRAW_SELECTED_TRAJECTORY_ONLY,
    DEBUG_DRAW_TRAJECTORY,
    DEBUG_DRAW_TRAJECTORY_DURATION,
    DEBUG_DRAW_TRAJECTORY_STEP,
    EGO_TARGET_SPEED,
    FIXED_DELTA_SECONDS,
    FRONT_BRAKE_MAX_DECEL,
    FRONT_BRAKE_REACTION_TIME,
    FRONT_BRAKE_RELEASE_MARGIN,
    FRONT_BRAKE_SAFE_DISTANCE,
    FRONT_CONFLICT_LATERAL_MARGIN,
    FRONT_PLANNING_RETRY_SPEED_DROP,
    FRONT_STEER_MAX_LATERAL_ACCEL,
    FRONT_STEER_SAFE_DISTANCE,
    HOST,
    LANE_CHANGE_LENGTH,
    LEAD_BRAKE_TIME,
    LEAD_TARGET_SPEED,
    MAP_NAME,
    PORT,
    RADAR_ENABLED,
    RADAR_FOV_HORIZONTAL_DEG,
    RADAR_FOV_VERTICAL_DEG,
    RADAR_POINTS_PER_SECOND,
    RADAR_RANGE,
    RIGHT_OBJECT_CROSS_X_MARGIN,
    RIGHT_OBJECT_EGO_TIME_MIN_SPEED,
    RIGHT_OBJECT_PATH_LOOKAHEAD,
    RIGHT_OBJECT_PATH_SAMPLE_STEP,
    RIGHT_OBJECT_PASS_MARGIN,
    RIGHT_OBJECT_TTC_THRESHOLD,
    ROUTE_COMPLETION_HOLD_SECONDS,
    SIM_SECONDS,
    TRAFFIC_RANDOM_SEED,
)
from control import LTVMPCTracker # MPC 控制器实现
from control import select_best_route_offset_trajectory, select_return_to_base_trajectory
from display import CollisionMonitor, PygameDemoDisplay # 碰撞监测和仿真显示实现
from perception import VirtualGroundTruthSensor # 虚拟传感器实现，提供前车和右侧过街物体的距离、TTC 等信息
from route import LoopRoute # 固定路线实现，提供路线点、航向和转弯信息
from utils import clamp, get_speed, normalize_angle, smooth_reference_for, speed_control, waypoint_steer # 工具函数，包括获取速度、速度控制和航向控制等

RIGHT_OBJECT_CLEAR_HOLD_SECONDS = 2.0
PLAN_RETRY_LOG_INTERVAL = 1.0
RETURN_TO_BASE_CLEARANCE = 2.0
CONTROL_DEBUG_STEER_JUMP = 0.22
CONTROL_DEBUG_MIN_INTERVAL = 0.25
CONTROL_MPC_EXIT_BLEND_SECONDS = 0.25
CONTROL_MPC_EXIT_BLEND_MIN_DELTA = 0.08


@dataclass
class FrontLimitDecision:
    front_conflict: bool = False
    planning_needed: bool = False
    emergency_brake_needed: bool = False
    s_clear: float = float("inf")
    d_steer: float = 0.0
    d_brake: float = 0.0
    delta_d_min: float = 0.0


# ===================== 仿真世界初始化 =====================

def parse_args():
    """读取演示运行参数。默认按 1x 实时速度播放，便于观察 pygame 动画。"""
    parser = argparse.ArgumentParser(description="CARLA Town10 emergency avoidance demo")
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="仿真播放倍率，1.0 表示按真实时间 1x 播放。",
    )
    parser.add_argument(
        "--free-run",
        action="store_true",
        help="不按真实时间等待，尽可能快地推进仿真。",
    )
    return parser.parse_args()


def pace_realtime_playback(start_wall_time, sim_elapsed, playback_speed, free_run):
    """将同步仿真节奏限制为固定播放倍率，避免 pygame 画面忽快忽慢。"""
    if free_run or playback_speed <= 0.0:
        return
    target_wall_elapsed = sim_elapsed / playback_speed
    wait_time = start_wall_time + target_wall_elapsed - time.time()
    if wait_time > 0.0:
        time.sleep(wait_time)

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


def tracking_route_lateral_slope(loop_route, ego_vehicle, tracking_route, base_route_s):
    """计算当前跟踪路线在自车位置的航向相对于基础路线的横向坡度，用于辅助判断转向时机和评估转向安全性。"""
    if tracking_route is None or not tracking_route.is_valid:
        return 0.0
    try:
        active_route_s, _ = tracking_route.to_local(ego_vehicle.get_location())
        active_yaw = tracking_route.reference_yaw_at(active_route_s)
        base_yaw = smooth_reference_for(loop_route).yaw_at_route_s(base_route_s)
        return clamp(math.tan(normalize_angle(active_yaw - base_yaw)), -0.60, 0.60)
    except Exception:
        return 0.0


def plan_route_replacement(loop_route, ego_vehicle, front, obstacle_actors, sim_time, tracking_route=None):
    """生成一个可替换当前基础路线 s 区间的避障路径段。"""
    target_speed = 0.0 if front.actor_role == "lead" else max(0.0, getattr(front, "target_speed_along", 0.0))
    predicted_motion = target_speed * min(3.0, max(1.0, front.distance / max(front.closing_speed, 0.1)))
    d_clear = front.distance + predicted_motion
    if not math.isfinite(d_clear):
        d_clear = LANE_CHANGE_LENGTH + predicted_motion
    base_length = clamp(d_clear, 6.0, 52.0)
    base_projection = project_actor_to_base_route(loop_route, ego_vehicle)
    start_lateral_slope = tracking_route_lateral_slope(
        loop_route, ego_vehicle, tracking_route, base_projection["route_s"]
    )
    start_time = time.perf_counter()
    best_candidate, candidates, diagnostics = select_best_route_offset_trajectory(
        loop_route,
        ego_vehicle,
        front,
        base_length,
        obstacle_actors=obstacle_actors,
        start_route_s=base_projection["route_s"],
        start_offset=base_projection["lateral"],
        start_lateral_slope=start_lateral_slope,
    )
    diagnostics["elapsed_ms"] = (time.perf_counter() - start_time) * 1000.0
    print_avoidance_candidate_summary(best_candidate, candidates, show_rejection_samples=True) # 打印候选路径数量、筛选结果和最终选中的关键参数，便于分析避障轨迹的生成和选择过程
    print_planning_diagnostics("avoidance", diagnostics)
    selected_trajectory = best_candidate.trajectory if best_candidate is not None else None
    if selected_trajectory is not None:
        print(
            "Route replacement planned at {:.2f}s: s={:.1f}-{:.1f}, peak_offset={:.2f}m, start_slope={:.3f}, transition_ratio={:.2f}, target={}, distance={:.1f}m, TTC={:.2f}s".format(
                sim_time,
                best_candidate.start_route_s,
                best_candidate.end_route_s,
                best_candidate.target_offset,
                best_candidate.start_lateral_slope,
                best_candidate.transition_ratio,
                right_object_label(front),
                front.distance,
                front.ttc if math.isfinite(front.ttc) else 99.99,
            )
        )
    return best_candidate, candidates


def plan_return_to_base(loop_route, ego_vehicle, obstacle_actors, sim_time, tracking_route=None):
    """生成从当前偏移回到基础路线的候选段。"""
    start_time = time.perf_counter()
    base_projection = project_actor_to_base_route(loop_route, ego_vehicle)
    active_route_s = None
    active_lateral = None
    if tracking_route is not None and tracking_route.is_valid:
        active_route_s, active_lateral = tracking_route.to_local(ego_vehicle.get_location())
    start_lateral_slope = tracking_route_lateral_slope(
        loop_route, ego_vehicle, tracking_route, base_projection["route_s"]
    )
    best_candidate, candidates, diagnostics = select_return_to_base_trajectory(
        loop_route,
        ego_vehicle,
        LANE_CHANGE_LENGTH,
        obstacle_actors=obstacle_actors,
        start_route_s=base_projection["route_s"],
        start_offset=base_projection["lateral"],
        start_lateral_slope=start_lateral_slope,
        active_route_s=active_route_s,
        active_lateral=active_lateral,
    )
    diagnostics["elapsed_ms"] = (time.perf_counter() - start_time) * 1000.0
    print_avoidance_candidate_summary(best_candidate, candidates, show_rejection_samples=False)
    print_planning_diagnostics("return", diagnostics)
    if best_candidate is not None:
        print(
            "Return-to-base planned at {:.2f}s: s={:.1f}-{:.1f}, start_offset={:.2f}m, start_slope={:.3f}, active_s={}, active_d={}, length={:.1f}m, transition_ratio={:.2f}.".format(
                sim_time,
                best_candidate.start_route_s,
                best_candidate.end_route_s,
                best_candidate.start_offset,
                best_candidate.start_lateral_slope,
                "{:.1f}".format(active_route_s) if active_route_s is not None else "n/a",
                "{:.2f}".format(active_lateral) if active_lateral is not None else "n/a",
                best_candidate.length,
                best_candidate.transition_ratio,
            )
        )
    return best_candidate, candidates


def print_planning_diagnostics(label, diagnostics):
    """打印规划耗时和候选筛选规模，定位卡顿来源。"""
    print(
        "{} planning cost: {:.1f}ms, total={}, valid={}, coarse={}, refine={}, broad={}, return={}, cheap_rejects={}, collision_checks={}, actor_proj={}.".format(
            label,
            diagnostics.get("elapsed_ms", 0.0),
            diagnostics.get("total_candidates", 0),
            diagnostics.get("valid_candidates", 0),
            diagnostics.get("coarse_candidates", 0),
            diagnostics.get("refine_candidates", 0),
            diagnostics.get("broad_candidates", 0),
            diagnostics.get("return_candidates", 0),
            diagnostics.get("cheap_rejects", 0),
            diagnostics.get("collision_checks", 0),
            diagnostics.get("actor_projections", 0),
        )
    )


def print_avoidance_candidate_summary(best_candidate, candidates, show_rejection_samples=False):
    """打印候选路径数量、筛选结果和最终选中的关键参数。"""
    valid_count = sum(1 for candidate in candidates if candidate.is_valid)
    if best_candidate is None:
        reasons = {}
        samples = []
        for candidate in candidates:
            reason_key = candidate.reject_reason.split(":", 1)[0] if candidate.reject_reason else "unknown"
            reasons[reason_key] = reasons.get(reason_key, 0) + 1
            if show_rejection_samples and candidate.reject_reason and len(samples) < 5:
                samples.append(
                    "L={:.1f}, target={:.2f}, ratio={:.2f}, reason={}".format(
                        candidate.length,
                        candidate.target_offset,
                        candidate.transition_ratio,
                        candidate.reject_reason,
                    )
                )
        print("Avoidance candidates rejected: total={}, reasons={}.".format(len(candidates), reasons))
        for sample in samples:
            print("  rejected sample: {}".format(sample))
        return

    print(
        "Best avoidance candidate: valid={}/{}, length={:.1f}m, transition_ratio={:.2f}, start_offset={:.2f}m, target_offset={:.2f}m, ay={:.2f}m/s^2, cost={:.2f}.".format(
            valid_count,
            len(candidates),
            best_candidate.length,
            best_candidate.transition_ratio,
            best_candidate.start_offset,
            best_candidate.target_offset,
            best_candidate.lateral_accel,
            best_candidate.total_cost,
        )
    )
    diagnostic_samples = [
        candidate for candidate in candidates
        if (
            not candidate.is_valid
            and candidate.reject_reason
            and candidate.target_offset > candidate.start_offset + 0.05
            and 2.5 <= abs(candidate.target_offset) <= 4.0
        )
    ]
    if not diagnostic_samples:
        diagnostic_samples = [
            candidate for candidate in candidates
            if (
                not candidate.is_valid
                and candidate.reject_reason
                and candidate.target_offset > candidate.start_offset + 0.05
                and abs(candidate.target_offset) < abs(best_candidate.target_offset) - 0.05
            )
        ]
    if show_rejection_samples:
        for candidate in diagnostic_samples[:5]:
            print(
                "  right-offset rejected: L={:.1f}, target={:.2f}, ratio={:.2f}, ay={:.2f}, reason={}".format(
                    candidate.length,
                    candidate.target_offset,
                    candidate.transition_ratio,
                    candidate.lateral_accel,
                    candidate.reject_reason,
                )
            )


def front_limit_decision(front, ego_vehicle, obstacle_actors):
    """基于极限转向/制动距离判断当前前车冲突是否需要规划或急刹。"""
    if not front.is_front_vehicle or not math.isfinite(front.distance):
        return FrontLimitDecision()

    front_actor = find_actor_by_id(obstacle_actors, front.actor_id)
    ego_half_length, ego_half_width = vehicle_half_extents(ego_vehicle)
    front_half_length, front_half_width = vehicle_half_extents(front_actor)
    s_clear = max(0.0, front.distance - ego_half_length - front_half_length)

    v_ego = max(0.0, get_speed(ego_vehicle))
    v_front = max(0.0, getattr(front, "target_speed_along", 0.0))
    v_rel = max(0.0, v_ego - v_front)

    delta_d_min = max(
        0.0,
        ego_half_width + front_half_width + FRONT_CONFLICT_LATERAL_MARGIN - abs(front.lateral_offset),
    )
    if delta_d_min > 0.0:
        t_steer_min = math.sqrt(
            10.0 * math.sqrt(3.0) * delta_d_min / (3.0 * max(FRONT_STEER_MAX_LATERAL_ACCEL, 0.1))
        )
    else:
        t_steer_min = 0.0
    d_steer = v_rel * t_steer_min + FRONT_STEER_SAFE_DISTANCE

    brake_energy = max(0.0, v_ego * v_ego - v_front * v_front)
    d_brake = (
        v_rel * FRONT_BRAKE_REACTION_TIME
        + brake_energy / (2.0 * max(FRONT_BRAKE_MAX_DECEL, 0.1))
        + FRONT_BRAKE_SAFE_DISTANCE
    )

    return FrontLimitDecision(
        front_conflict=True,
        planning_needed=s_clear <= d_steer,
        emergency_brake_needed=s_clear <= d_brake,
        s_clear=s_clear,
        d_steer=d_steer,
        d_brake=d_brake,
        delta_d_min=delta_d_min,
    )


def front_planning_needed(front_decision):
    return front_decision.front_conflict and front_decision.planning_needed


def front_emergency_brake_needed(front_decision):
    return front_decision.front_conflict and front_decision.emergency_brake_needed


def apply_route_replacement(sensor, candidate):
    """将选中的避障路径段应用到当前基础路线的替换区间，更新传感器的路线参考。"""
    if candidate is None or candidate.trajectory is None:
        return None
    trajectory = candidate.trajectory
    sensor.apply_replacement_segment( # 将选中的避障路径段应用到当前基础路线的替换区间，更新传感器的路线参考。通过调用传感器的 apply_replacement_segment 方法，将选中的避障路径段应用到当前基础路线的指定 s 区间，并设置相应的目标横向偏移，从而更新传感器的路线参考，使其能够在后续的控制和规划过程中考虑新的避障路径段。最后，返回应用了避障路径段后的轨迹对象，供后续使用。
        candidate.start_route_s, # 将选中的避障路径段应用到当前基础路线的替换区间，更新传感器的路线参考。通过调用传感器的 apply_replacement_segment 方法，将选中的避障路径段应用到当前基础路线的指定 s 区间，并设置相应的目标横向偏移，从而更新传感器的路线参考，使其能够在后续的控制和规划过程中考虑新的避障路径段。最后，返回应用了避障路径段后的轨迹对象，供后续使用。
        candidate.end_route_s, # 将选中的避障路径段应用到当前基础路线的替换区间，更新传感器的路线参考。通过调用传感器的 apply_replacement_segment 方法，将选中的避障路径段应用到当前基础路线的指定 s 区间，并设置相应的目标横向偏移，从而更新传感器的路线参考，使其能够在后续的控制和规划过程中考虑新的避障路径段。最后，返回应用了避障路径段后的轨迹对象，供后续使用。
        trajectory.replacement_points(DEBUG_DRAW_TRAJECTORY_STEP), # 将选中的避障路径段应用到当前基础路线的替换区间，更新传感器的路线参考。通过调用传感器的 apply_replacement_segment 方法，将选中的避障路径段应用到当前基础路线的指定 s 区间，并设置相应的目标横向偏移，从而更新传感器的路线参考，使其能够在后续的控制和规划过程中考虑新的避障路径段。最后，返回应用了避障路径段后的轨迹对象，供后续使用。
        end_offset=candidate.target_offset,
    )
    return trajectory


def reset_mpc_plan_after_route_change(mpc, last_steer=None, sim_time=None):
    reset_plan = getattr(mpc, "reset_plan", None)
    if callable(reset_plan):
        reset_plan(last_steer=last_steer)
        reset_debug = getattr(mpc, "last_reset_debug", None)
        if reset_debug:
            prefix = "MPC soft reset"
            if sim_time is not None:
                prefix += " at {:.2f}s".format(sim_time)
            print(
                "{}: source={}, last_steer={}, delta_init={}rad, steer_init={}, kept_steps={}, guess_uses={}, hold_frames={}.".format(
                    prefix,
                    reset_debug.get("source"),
                    _format_float(last_steer),
                    _format_float(reset_debug.get("delta_init"), 3),
                    _format_float(reset_debug.get("steer_init")),
                    reset_debug.get("kept_solution_steps"),
                    reset_debug.get("guess_uses"),
                    reset_debug.get("hold_frames"),
                )
            )


def make_avoidance_target(front, obstacle_actors, target_offset):
    return {
        "actor_id": front.actor_id,
        "actor_role": front.actor_role,
        "actor": find_actor_by_id(obstacle_actors, front.actor_id),
        "target_offset": target_offset,
    }


def active_avoidance_covers_front(front, active_avoidance_target, route_offset_active):
    if active_avoidance_target is None or not route_offset_active:
        return False
    return front.actor_id is not None and front.actor_id == active_avoidance_target.get("actor_id")


def find_actor_by_id(actors, actor_id):
    if actor_id is None:
        return None
    for actor in actors:
        if actor is not None and getattr(actor, "id", None) == actor_id:
            return actor
    return None


def vehicle_half_extents(actor, default_length=2.4, default_width=0.95):
    bbox = getattr(actor, "bounding_box", None)
    extent = getattr(bbox, "extent", None)
    if extent is None:
        return default_length, default_width
    return max(0.1, float(extent.x)), max(0.1, float(extent.y))


def vehicle_half_length(actor, default_length=2.4):
    return vehicle_half_extents(actor, default_length=default_length)[0]


def avoidance_target_passed(loop_route, ego_vehicle, target_actor, clearance=RETURN_TO_BASE_CLEARANCE):
    """判断是否满足“自车车尾超过障碍物车头”。"""
    if target_actor is None or not target_actor.is_alive:
        return True

    reference = smooth_reference_for(loop_route)
    ego_projection = project_actor_to_base_route(loop_route, ego_vehicle)
    target_projection = reference.project(
        target_actor.get_location(),
        ego_projection["route_s"],
        search_back=60.0,
        search_ahead=80.0,
    )
    ego_tail_s = ego_projection["route_s"] - vehicle_half_length(ego_vehicle)
    target_head_s = target_projection["route_s"] + vehicle_half_length(target_actor)
    return ego_tail_s > target_head_s + clearance


def project_actor_to_base_route(loop_route, actor):
    reference = smooth_reference_for(loop_route)
    center_s = loop_route.last_index * loop_route.step_distance
    return reference.project(
        actor.get_location(),
        center_s,
        search_back=20.0,
        search_ahead=40.0,
    )


def right_object_label(reading):
    """格式化右侧避让目标，便于判断是否发生目标切换。"""
    if reading.actor_id is None:
        return "none"
    return "{}#{}".format(reading.actor_role or "actor", reading.actor_id)


def _format_float(value, digits=2):
    if value is None:
        return "none"
    return "{:.{}f}".format(value, digits) if math.isfinite(value) else "inf"


def print_right_object_decision(action, sim_time, right_object, route_index):
    print(
        "Right object decision {} at {:.2f}s: decision={}, target={}, TTC={}s, cross_s={}m, "
        "t_ego={}s, x_path={}m, y_path={}m, y_rel={}m, Y_safe={}m, distance={}m, actor_id={}, route_index={}.".format(
            action,
            sim_time,
            right_object.decision,
            right_object_label(right_object),
            _format_float(right_object.line_ttc),
            _format_float(right_object.cross_s),
            _format_float(right_object.t_ego),
            _format_float(right_object.x_path),
            _format_float(right_object.y_path),
            _format_float(right_object.y_rel),
            _format_float(right_object.y_safe),
            _format_float(right_object.distance),
            right_object.actor_id,
            route_index,
        )
    )


def print_control_switch_debug(
    sim_time,
    reason,
    ego_vehicle,
    tracking_route,
    base_route_s,
    mpc_tracking_until_s,
    current_offset,
    use_mpc_tracking,
    route_changed_this_frame,
    target_speed,
    ego_control,
    steer_delta,
    right_object_decision,
    raw_steer=None,
    blend_alpha=None,
    blend_source_steer=None,
):
    progress = float("inf")
    lateral = float("inf")
    if tracking_route is not None and getattr(tracking_route, "is_valid", False):
        try:
            progress, lateral = tracking_route.to_local(ego_vehicle.get_location())
        except Exception:
            pass
    print(
        "Control tracking debug at {:.2f}s [{}]: route_changed={}, use_mpc={}, base_s={}m, "
        "mpc_until={}m, track_s={}m, track_d={}m, current_offset={}m, ego_speed={}m/s, "
        "target_speed={}m/s, steer={:+.2f}, raw_steer={}, blend_alpha={}, blend_source={}, "
        "steer_delta={:+.2f}, brake={:.2f}, right_decision={}.".format(
            sim_time,
            reason,
            route_changed_this_frame,
            use_mpc_tracking,
            _format_float(base_route_s),
            _format_float(mpc_tracking_until_s),
            _format_float(progress),
            _format_float(lateral),
            _format_float(current_offset),
            _format_float(get_speed(ego_vehicle)),
            _format_float(target_speed),
            ego_control.steer,
            _format_float(raw_steer),
            _format_float(blend_alpha),
            _format_float(blend_source_steer),
            steer_delta,
            ego_control.brake,
            right_object_decision,
        )
    )


def find_right_object_scenario(right_object_scenarios, actor_id):
    if actor_id is None:
        return None
    for scenario in right_object_scenarios:
        actor = getattr(scenario, "actor", None)
        if actor is not None and actor.is_alive and actor.id == actor_id:
            return scenario
    return None


def _right_object_default_decision(right_object, decision="brake", y_safe=0.0):
    right_object.decision = decision
    right_object.cross_s = float("inf")
    right_object.t_ego = float("inf")
    right_object.x_path = float("inf")
    right_object.y_path = float("inf")
    right_object.y_rel = float("inf")
    right_object.y_safe = y_safe
    right_object.risk_level = 2 if decision == "pass" else (3 if decision == "brake" else 0)
    return decision


def _right_object_axes(scenario):
    actor = getattr(scenario, "actor", None)
    if actor is None or not actor.is_alive:
        return None
    velocity = getattr(scenario, "velocity", actor.get_velocity())
    speed = math.hypot(velocity.x, velocity.y)
    if speed > 0.2:
        ey_x = velocity.x / speed
        ey_y = velocity.y / speed
    else:
        forward = actor.get_transform().get_forward_vector()
        forward_length = math.hypot(forward.x, forward.y)
        if forward_length <= 0.01:
            return None
        ey_x = forward.x / forward_length
        ey_y = forward.y / forward_length
    ex_x = -ey_y
    ex_y = ey_x
    object_speed_y = velocity.x * ey_x + velocity.y * ey_y
    return ex_x, ex_y, ey_x, ey_y, object_speed_y


def _route_sample_context(ego_vehicle, loop_route, tracking_route):
    ego_location = ego_vehicle.get_location()
    if tracking_route is not None and getattr(tracking_route, "is_valid", False):
        try:
            start_s, _ = tracking_route.to_local(ego_location)
            end_s = min(tracking_route.length, start_s + RIGHT_OBJECT_PATH_LOOKAHEAD)
            return start_s, end_s, tracking_route.location_at
        except Exception:
            pass

    reference = smooth_reference_for(loop_route)
    center_s = loop_route.last_index * loop_route.step_distance
    projection = reference.project(
        ego_location,
        center_s,
        search_back=20.0,
        search_ahead=60.0,
    )
    start_s = reference.clamp_s(projection["route_s"])
    end_s = min(reference.max_s, start_s + RIGHT_OBJECT_PATH_LOOKAHEAD)
    return start_s, end_s, reference.location_at_route_s


def decide_right_object_by_route(right_object, right_object_scenarios, ego_vehicle, loop_route, tracking_route):
    scenario = find_right_object_scenario(right_object_scenarios, right_object.actor_id)
    actor = getattr(scenario, "actor", None) if scenario is not None else None
    if actor is None or not actor.is_alive:
        return _right_object_default_decision(right_object, "brake")

    axes = _right_object_axes(scenario)
    ego_half_length, ego_half_width = vehicle_half_extents(ego_vehicle)
    object_half_length, object_half_width = vehicle_half_extents(actor)
    x_safe = ego_half_width + object_half_width + RIGHT_OBJECT_CROSS_X_MARGIN
    y_safe = ego_half_length + object_half_length + RIGHT_OBJECT_PASS_MARGIN
    if axes is None:
        return _right_object_default_decision(right_object, "brake", y_safe)

    ex_x, ex_y, ey_x, ey_y, object_speed_y = axes
    actor_location = actor.get_location()
    start_s, end_s, location_at = _route_sample_context(ego_vehicle, loop_route, tracking_route)
    if end_s <= start_s:
        return _right_object_default_decision(right_object, "brake", y_safe)

    ego_time_speed = max(get_speed(ego_vehicle), RIGHT_OBJECT_EGO_TIME_MIN_SPEED)
    sample_count = max(1, int(math.ceil((end_s - start_s) / max(RIGHT_OBJECT_PATH_SAMPLE_STEP, 0.2))))
    for index in range(sample_count + 1):
        route_s = min(end_s, start_s + index * (end_s - start_s) / sample_count)
        route_distance = max(0.0, route_s - start_s)
        route_location = location_at(route_s)
        relative = route_location - actor_location
        x_path = relative.x * ex_x + relative.y * ex_y
        if abs(x_path) >= x_safe:
            continue
        y_path = relative.x * ey_x + relative.y * ey_y
        t_ego = route_distance / ego_time_speed
        y_rel = y_path - object_speed_y * t_ego
        decision = "pass" if y_rel > y_safe else "brake"
        right_object.decision = decision
        right_object.cross_s = route_s
        right_object.t_ego = t_ego
        right_object.x_path = x_path
        right_object.y_path = y_path
        right_object.y_rel = y_rel
        right_object.y_safe = y_safe
        right_object.risk_level = 2 if decision == "pass" else 3
        return decision

    return _right_object_default_decision(right_object, "brake", y_safe)


def draw_debug_marker(world, location, color, life_time=DEBUG_DRAW_LIFETIME):
    """在 CARLA 世界中画一个醒目的短生命周期目标点。"""
    marker_base = carla.Location(location.x, location.y, location.z + 0.35)
    marker_top = carla.Location(location.x, location.y, location.z + 2.0)
    world.debug.draw_point(marker_base, size=0.18, color=color, life_time=life_time)
    world.debug.draw_line(marker_base, marker_top, thickness=0.08, color=color, life_time=life_time)


def draw_tracking_route_lookahead_marker(world, ego_vehicle, tracking_route, lookahead_distance=DEBUG_DRAW_LOOKAHEAD_DISTANCE):
    """在当前合成跟踪路线前方画红色目标标记。"""
    if tracking_route is None or not tracking_route.is_valid:
        return
    progress, _ = tracking_route.to_local(ego_vehicle.get_location())
    target_s = min(tracking_route.length, progress + lookahead_distance)
    draw_debug_marker(world, tracking_route.location_at(target_s), carla.Color(255, 0, 0))


def draw_avoidance_candidates(world, candidates, selected_trajectory):
    """Draw a small diagnostic subset of avoidance trajectories."""
    if not selected_trajectory and not candidates:
        return

    trajectories = []
    if selected_trajectory is not None:
        trajectories.append((selected_trajectory, True))

    if not DEBUG_DRAW_SELECTED_TRAJECTORY_ONLY and DEBUG_DRAW_MAX_ALTERNATIVE_TRAJECTORIES > 0:
        alternative_count = 0
        for candidate in candidates:
            if not candidate.is_valid or candidate.trajectory is selected_trajectory:
                continue
            trajectories.append((candidate.trajectory, False))
            alternative_count += 1
            if alternative_count >= DEBUG_DRAW_MAX_ALTERNATIVE_TRAJECTORIES:
                break

    for trajectory, is_selected in trajectories:
        if trajectory is None:
            continue
        color = carla.Color(0, 255, 40) if is_selected else carla.Color(0, 170, 0)
        thickness = 0.08 if is_selected else 0.04
        s = 0.0
        previous = trajectory.location_at(s) + carla.Location(z=0.18)
        while s < trajectory.length:
            s = min(trajectory.length, s + DEBUG_DRAW_TRAJECTORY_STEP)
            current = trajectory.location_at(s) + carla.Location(z=0.18)
            world.debug.draw_line(previous, current, thickness=thickness, color=color, life_time=DEBUG_DRAW_LIFETIME)
            previous = current


def draw_trajectory_debug(world, ego_vehicle, tracking_route, selected_trajectory, candidates, frame, draw_plan_debug):
    """统一绘制当前跟踪路线前视点和最近一次候选轨迹。"""
    if not DEBUG_DRAW_TRAJECTORY:
        return
    if frame % max(1, DEBUG_DRAW_INTERVAL_FRAMES) != 0:
        return
    if draw_plan_debug:
        draw_avoidance_candidates(world, candidates, selected_trajectory)
    draw_tracking_route_lookahead_marker(world, ego_vehicle, tracking_route)


def main(args=None):
    """主函数：设置仿真环境，生成演员，执行主循环进行控制，并在结束后清理资源"""
    args = args or parse_args()
    actor_list = [] # 用于跟踪所有生成的演员，以便在仿真结束后统一销毁
    camera_display = None # 用于显示仿真画面，如果创建失败则保持为 None
    world = None # 仿真世界对象，初始化为 None，在 setup_world 成功后赋值，最后在 finally 块中恢复设置
    original_settings = None # 用于保存仿真世界的原始设置，以便在仿真结束后恢复，避免影响其他程序使用 CARLA

    try:
        client = carla.Client(HOST, PORT)
        client.set_timeout(CLIENT_TIMEOUT)

        world = client.get_world() # 获取当前世界对象
        original_settings = world.get_settings()
        world = setup_world(client) # 加载指定地图并启用同步模式，返回配置好的世界对象
        carla_map = world.get_map()

        ego_vehicle, lead_vehicle, ego_start_wp = spawn_scenario(world) # 生成自车和前车，并返回自车的起始路点
        actor_list.extend([ego_vehicle, lead_vehicle]) # 将生成的自车和前车添加到演员列表中，以便后续管理和清理
        collision_monitor = CollisionMonitor(world, ego_vehicle, actor_list)
        mpc = LTVMPCTracker() # 创建 MPC 控制器实例，用于后续的轨迹跟踪控制
        camera_display = PygameDemoDisplay(world, ego_vehicle, actor_list) # 创建仿真显示实例，提供实时画面显示和信息渲染，如果创建失败则保持为 None

        world.tick() # 仿真世界进行一次更新，确保所有演员都已生成并准备就绪
        set_spectator(world, ego_vehicle) # 将观察者视角定位到自车正上方，提供上帝视角观察仿真过程
        loop_route = LoopRoute(ego_start_wp) # 创建固定路线实例，基于自车的起始路点生成一圈环形路线，提供路线点、航向和转弯信息
        traffic_rng = random.Random(TRAFFIC_RANDOM_SEED) # 创建随机数生成器实例，使用固定种子确保背景交通的可重复性
        background_vehicles = spawn_background_route_vehicles(world, loop_route, actor_list, traffic_rng) # 生成背景交通车辆，基于固定路线和随机数生成器创建多个车辆，并添加到演员列表中
        slow_vehicle = spawn_slow_right_lane_vehicle(world, loop_route, actor_list)
        if slow_vehicle:
            background_vehicles.append(slow_vehicle)  # 复用现有前车感知列表，不改感知逻辑
        background_bicycles = spawn_background_r344_bicycles(world, loop_route, actor_list, traffic_rng)
        right_object_scenario = spawn_right_side_bicycle_crossing(world, loop_route, actor_list)
        right_pedestrians = spawn_right_side_pedestrians(world, loop_route, actor_list)
        right_object_scenarios = [
            scenario for scenario in [right_object_scenario] + background_bicycles + right_pedestrians if scenario
        ] # 将右侧过街自行车、背景自行车和右侧行人场景合并成一个列表，供后续统一处理
        sensor = VirtualGroundTruthSensor( # 创建虚拟传感器实例，提供前车和右侧过街物体的距离、TTC 等信息，供控制决策使用
            world,
            carla_map,
            ego_vehicle,
            lead_vehicle,
            front_extra_vehicles=[controller.actor for controller in background_vehicles],
            right_object_scenarios=right_object_scenarios,
            loop_route=loop_route,
        )
        if RADAR_ENABLED: # 如果启用雷达传感器，则创建雷达传感器实例，设置其属性并安装在自车前方，提供额外的前车感知信息，供控制决策使用
            radar_bp = world.get_blueprint_library().find("sensor.other.radar")
            radar_bp.set_attribute("horizontal_fov", str(RADAR_FOV_HORIZONTAL_DEG))
            radar_bp.set_attribute("vertical_fov", str(RADAR_FOV_VERTICAL_DEG))
            radar_bp.set_attribute("range", str(RADAR_RANGE))
            radar_bp.set_attribute("points_per_second", str(RADAR_POINTS_PER_SECOND))
            radar_tf = carla.Transform(carla.Location(x=2.2, z=1.0), carla.Rotation(pitch=0.0))
            front_radar = world.spawn_actor(radar_bp, radar_tf, attach_to=ego_vehicle)
            actor_list.append(front_radar)

            def radar_callback(data):
                sensor.set_radar_detections(data)

            front_radar.listen(radar_callback)
            print(
                "Front radar sensor mounted: range={:.0f}m, horizontal_fov={:.0f}deg.".format(
                    RADAR_RANGE, RADAR_FOV_HORIZONTAL_DEG
                )
            )

        state = "ROUTE_FOLLOW"
        selected_plan_trajectory = None
        start_time = time.time()
        frame = 0
        route_completion_time = None
        right_object_stop_active = False
        last_right_object_actor_id = None
        last_right_object_decision = "normal"
        last_right_object_debug_time = -float("inf")
        right_object_clear_since = None
        last_plan_failure_log_time = -999.0
        avoidance_candidates = []
        debug_plan_draw_until_time = -999.0
        active_avoidance_target = None
        mpc_tracking_until_s = None
        last_use_mpc_tracking = False
        last_control_steer = 0.0
        last_control_debug_time = -float("inf")
        mpc_exit_blend_start_time = None
        mpc_exit_blend_start_steer = 0.0
        obstacle_actors = [lead_vehicle] + [controller.actor for controller in background_vehicles]

        if args.free_run:
            print("Playback mode: free-run.")
        else:
            print("Playback mode: {:.1f}x realtime.".format(args.playback_speed))
        print("Scenario started: map={}, ego=Tesla Model3, lead=Lincoln MKZ 2020".format(MAP_NAME))
        print("Lead car will brake hard at {:.1f}s.".format(LEAD_BRAKE_TIME))
        print(
            "Background traffic: route_vehicles={}, r344_bicycles={}.".format(
                len(background_vehicles),
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
            if camera_display is not None and not camera_display.process_events(): # 处理显示事件，如果用户关闭了显示窗口，则提前终止仿真
                print("用户停止了仿真，正在退出...")
                break

            sim_time = frame * FIXED_DELTA_SECONDS # 计算当前仿真时间，基于帧计数器和固定步长计算得到，供控制逻辑使用

            if sim_time < LEAD_BRAKE_TIME:
                """前车在指定时间前保持目标速度，使用 speed_control 函数计算所需的油门和制动值，并应用控制命令（也就是控制前车）"""
                lead_throttle, lead_brake = speed_control(get_speed(lead_vehicle), LEAD_TARGET_SPEED) # 前车在指定时间前保持目标速度，使用 speed_control 函数计算所需的油门和制动值
                lead_steer = waypoint_steer(lead_vehicle, carla_map) # 计算前车的航向控制值，基于当前车辆位置和地图信息进行路径跟踪
                lead_vehicle.apply_control(  # 应用控制命令，控制前车的油门、制动和转向，保持在目标速度上行驶，并根据路径进行航向调整
                    carla.VehicleControl(throttle=lead_throttle, brake=lead_brake, steer=lead_steer)
                )
            else: # 前车在指定时间后紧急制动，直接应用全制动控制命令，油门为0，制动为1，方向盘不转动
                lead_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))

            for background_vehicle in background_vehicles: # 更新背景车辆的状态，调用每个背景车辆控制器的 update 方法，传入当前仿真时间和固定步长，执行预设的背景交通行为
                background_vehicle.update(FIXED_DELTA_SECONDS, ego_vehicle)

            for right_object in right_object_scenarios: # 更新右侧过街物体的状态，调用每个右侧过街物体场景的 update 方法，传入当前路线索引和固定步长，执行预设的过街行为
                right_object.update(loop_route.last_index, FIXED_DELTA_SECONDS)

            front = sensor.front_vehicle()
            right_object = sensor.right_side_object(loop_route.last_index) # 获取右侧过街物体的感知信息，调用传感器的 right_side_object 方法获取当前右侧过街物体的距离、TTC 等信息，供控制决策使用
            left_side = sensor.side_vehicle("left")
            ego_speed = get_speed(ego_vehicle)

            base_route_s_for_decision = project_actor_to_base_route(loop_route, ego_vehicle)["route_s"]
            if mpc_tracking_until_s is not None and base_route_s_for_decision > mpc_tracking_until_s:
                mpc_tracking_until_s = None
            route_offset_active_for_planning = abs(sensor.current_offset()) > 0.05 or mpc_tracking_until_s is not None
            front_limit = front_limit_decision(front, ego_vehicle, obstacle_actors)
            planning_needed = front_planning_needed(front_limit)
            if (
                active_avoidance_covers_front(front, active_avoidance_target, route_offset_active_for_planning)
                and not front_emergency_brake_needed(front_limit)
            ):
                planning_needed = False
            front_slowdown_needed = False
            emergency_recovered = (
                not front_limit.front_conflict
                or front_limit.s_clear > front_limit.d_brake + FRONT_BRAKE_RELEASE_MARGIN
            )
            right_object_risk = (
                right_object.is_conflict_object
                and right_object.ttc < RIGHT_OBJECT_TTC_THRESHOLD
            )
            right_object_decision = "normal"

            route_changed_this_frame = False

            if state == "EMERGENCY_BRAKE": # 如果当前状态是紧急制动，首先判断是否满足恢复条件，如果满足则切换回正常跟随状态，并打印恢复日志；否则如果仍然需要规划避障，则尝试生成替换路线，如果成功生成则切换回正常跟随状态并应用替换路线，否则如果仍然需要紧急制动且没有替换路线可用，则保持在紧急制动状态，并根据设定的日志间隔打印当前状态和风险信息
                if emergency_recovered:
                    state = "ROUTE_FOLLOW"
                    print(
                        "Emergency brake recovered at {:.2f}s: distance={:.1f}m, TTC={:.2f}s".format(
                            sim_time,
                            front.distance,
                            front.ttc if math.isfinite(front.ttc) else 99.99,
                        )
                    )
                elif planning_needed: # 如果仍然需要规划避障，则尝试生成替换路线，如果成功生成则切换回正常跟随状态并应用替换路线，否则如果仍然需要紧急制动且没有替换路线可用，则保持在紧急制动状态，并根据设定的日志间隔打印当前状态和风险信息
                    best_candidate, avoidance_candidates = plan_route_replacement(
                        loop_route, ego_vehicle, front, obstacle_actors, sim_time, tracking_route=sensor.tracking_route()
                    )
                    selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                    if selected_plan_trajectory is not None:
                        reset_mpc_plan_after_route_change(mpc, last_control_steer, sim_time)
                        active_avoidance_target = make_avoidance_target(
                            front, obstacle_actors, best_candidate.target_offset
                        )
                        mpc_tracking_until_s = best_candidate.end_route_s + 2.0
                        route_changed_this_frame = True
                        state = "ROUTE_FOLLOW"
                        print(
                            "Emergency brake switched back to route follow with replacement at {:.2f}s: target={}, distance={:.1f}m, TTC={:.2f}s".format(
                                sim_time,
                                right_object_label(front),
                                front.distance,
                                front.ttc if math.isfinite(front.ttc) else 99.99,
                            )
                        )

            if (
                state == "ROUTE_FOLLOW"
                and active_avoidance_target is not None
                and not route_changed_this_frame
                and avoidance_target_passed(loop_route, ego_vehicle, active_avoidance_target.get("actor"))
            ):
                active_target_label = "{}#{}".format(
                    active_avoidance_target.get("actor_role") or "actor",
                    active_avoidance_target.get("actor_id"),
                )
                best_candidate, avoidance_candidates = plan_return_to_base(
                    loop_route, ego_vehicle, obstacle_actors, sim_time, tracking_route=sensor.tracking_route()
                )
                selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                if selected_plan_trajectory is not None: # 如果成功生成返回基础路线的替换路径段，则应用该路径段作为新的跟踪路线，同时清除当前的避让目标，更新 MPC 跟踪的截止点，并打印相关日志；如果需要规划但没有成功生成替换路线，并且满足日志间隔条件，则打印当前规划失败的状态和相关信息
                    reset_mpc_plan_after_route_change(mpc, last_control_steer, sim_time)
                    mpc_tracking_until_s = best_candidate.end_route_s + 2.0
                    print(
                        "Avoidance target passed, returning to base route at {:.2f}s: target={}, offset={:.2f}m.".format(
                            sim_time,
                            active_target_label,
                            sensor.current_offset(),
                        )
                    )
                    active_avoidance_target = None
                    route_changed_this_frame = True
                elif sim_time - last_plan_failure_log_time >= PLAN_RETRY_LOG_INTERVAL: # 如果需要规划但没有成功生成替换路线，并且满足日志间隔条件，则打印当前规划失败的状态和相关信息
                    print(
                        "Return planning blocked at {:.2f}s: keep offset={:.2f}m, target={}.".format(
                            sim_time,
                            sensor.current_offset(),
                            active_target_label,
                        )
                    )
                    last_plan_failure_log_time = sim_time

            if state == "ROUTE_FOLLOW" and planning_needed and not route_changed_this_frame: # 如果当前状态是正常跟随，并且需要进行前车避障规划，并且当前帧还没有进行路线切换，则尝试生成替换路线，如果成功生成则应用替换路线并更新相关状态和日志，否则如果仍然需要紧急制动且没有替换路线可用，则保持在紧急制动状态，并根据设定的日志间隔打印当前状态和风险信息
                best_candidate, avoidance_candidates = plan_route_replacement(
                    loop_route, ego_vehicle, front, obstacle_actors, sim_time, tracking_route=sensor.tracking_route()
                )
                selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                if selected_plan_trajectory is not None: # 如果成功生成替换路线，则应用替换路线并更新相关状态和日志；否则如果仍然需要紧急制动且没有替换路线可用，则保持在紧急制动状态，并根据设定的日志间隔打印当前状态和风险信息
                    reset_mpc_plan_after_route_change(mpc, last_control_steer, sim_time)
                    active_avoidance_target = make_avoidance_target(
                        front, obstacle_actors, best_candidate.target_offset
                    )
                    mpc_tracking_until_s = best_candidate.end_route_s + 2.0
                    route_changed_this_frame = True
                else: # 如果需要规划但没有成功生成替换路线，并且满足日志间隔条件，则打印当前规划失败的状态和相关信息
                    if front_emergency_brake_needed(front_limit):
                        state = "EMERGENCY_BRAKE"
                        print(
                            "Emergency brake at {:.2f}s: no collision-free replacement, target={}, S_clear={:.1f}m, D_brake={:.1f}m.".format(
                                sim_time,
                                right_object_label(front),
                                front_limit.s_clear,
                                front_limit.d_brake,
                            )
                        )
                    else:
                        front_slowdown_needed = True
                        if sim_time - last_plan_failure_log_time >= PLAN_RETRY_LOG_INTERVAL:
                            print(
                                "Replacement planning failed at {:.2f}s: slow down and retry, target={}, S_clear={:.1f}m, D_steer={:.1f}m, D_brake={:.1f}m.".format(
                                    sim_time,
                                    right_object_label(front),
                                    front_limit.s_clear,
                                    front_limit.d_steer,
                                    front_limit.d_brake,
                                )
                            )
                            last_plan_failure_log_time = sim_time

            if right_object_risk:
                right_object_decision = decide_right_object_by_route(
                    right_object,
                    right_object_scenarios,
                    ego_vehicle,
                    loop_route,
                    sensor.tracking_route(),
                )
            else:
                right_object.decision = "normal"
                right_object.risk_level = 0

            if right_object_risk: # 如果右侧过街物体构成风险，记录最危险目标的 pass/brake 决策；如果风险解除，则启动清除保持计时器，避免刹车抖动
                target_changed = last_right_object_actor_id != right_object.actor_id
                decision_changed = last_right_object_decision != right_object_decision
                if target_changed or decision_changed or sim_time - last_right_object_debug_time >= 1.0:
                    if target_changed:
                        action = "started" if last_right_object_actor_id is None else "target changed"
                    elif decision_changed:
                        action = "changed"
                    else:
                        action = "debug"
                    print_right_object_decision(action, sim_time, right_object, loop_route.last_index)
                    last_right_object_actor_id = right_object.actor_id
                    last_right_object_decision = right_object_decision
                    last_right_object_debug_time = sim_time
                right_object_clear_since = None
            elif last_right_object_actor_id is not None:
                if right_object_clear_since is None:
                    right_object_clear_since = sim_time
                elif sim_time - right_object_clear_since >= RIGHT_OBJECT_CLEAR_HOLD_SECONDS:
                    right_object_stop_active = False
                    last_right_object_actor_id = None
                    last_right_object_decision = "normal"
                    right_object_clear_since = None
                    print("Right object decision completed at {:.2f}s.".format(sim_time))

            target_speed = 0.0
            tracking_route_for_control = sensor.tracking_route()
            base_route_s = base_route_s_for_decision
            use_mpc_tracking = False
            raw_control_steer = 0.0
            mpc_exit_blend_alpha = None
            mpc_exit_blend_started = False
            mpc_exit_blend_active = False
            mpc_exit_blend_source_steer = None

            if route_completion_time is not None: # 如果已经完成了一圈路线，则保持在原地等待，应用全制动控制命令，油门为0，制动为1，方向盘不转动
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "EMERGENCY_BRAKE":
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            else:
                target_speed = EGO_TARGET_SPEED
                if front_slowdown_needed:
                    target_speed = min(target_speed, max(0.0, ego_speed - FRONT_PLANNING_RETRY_SPEED_DROP))

                tracking_route_for_control = sensor.tracking_route()
                base_route_s = project_actor_to_base_route(loop_route, ego_vehicle)["route_s"]
                if mpc_tracking_until_s is not None and base_route_s > mpc_tracking_until_s:
                    mpc_tracking_until_s = None
                replacement_segment_active = mpc_tracking_until_s is not None
                offset_route_active = abs(sensor.current_offset()) > 0.05
                use_mpc_tracking = (
                    tracking_route_for_control is not None
                    and tracking_route_for_control.is_valid
                    and (replacement_segment_active or offset_route_active)
                )

                if use_mpc_tracking:
                    ego_control = mpc.control(ego_vehicle, tracking_route_for_control, target_speed)
                else:
                    throttle, brake = speed_control(ego_speed, target_speed)
                    ego_control = carla.VehicleControl(
                        throttle=throttle,
                        brake=brake,
                        steer=loop_route.steer(ego_vehicle),
                    )
                raw_control_steer = ego_control.steer

                if front_slowdown_needed: # 如果需要前车慢行但不需要紧急制动，则确保至少施加一定的制动值，并且油门为0，以实现减速效果
                    ego_control.brake = max(ego_control.brake, 0.20)
                    ego_control.throttle = 0.0

                if right_object_decision == "brake" and not right_object_stop_active:
                    right_object_stop_active = True
                    print(
                        "Right object hard stop engaged at {:.2f}s: target={}, decision={}, TTC={}s, cross_s={}m, t_ego={}s.".format(
                            sim_time,
                            right_object_label(right_object),
                            right_object_decision,
                            _format_float(right_object.line_ttc),
                            _format_float(right_object.cross_s),
                            _format_float(right_object.t_ego),
                        )
                    )

                clear_waiting = (
                    right_object_clear_since is not None
                    and sim_time - right_object_clear_since < RIGHT_OBJECT_CLEAR_HOLD_SECONDS
                )
                if right_object_stop_active or clear_waiting:
                    ego_control.throttle = 0.0
                    ego_control.brake = max(ego_control.brake, 0.85)

            if route_completion_time is not None or state == "EMERGENCY_BRAKE":
                mpc_exit_blend_start_time = None
            elif use_mpc_tracking:
                mpc_exit_blend_start_time = None
            elif last_use_mpc_tracking:
                steer_gap = abs(raw_control_steer - last_control_steer)
                if steer_gap >= CONTROL_MPC_EXIT_BLEND_MIN_DELTA:
                    mpc_exit_blend_start_time = sim_time
                    mpc_exit_blend_start_steer = last_control_steer
                    mpc_exit_blend_started = True
                else:
                    mpc_exit_blend_start_time = None

            if (
                mpc_exit_blend_start_time is not None
                and not use_mpc_tracking
            ):
                if ego_control.brake >= 0.80:
                    mpc_exit_blend_start_time = None
                else:
                    blend_elapsed = max(0.0, sim_time - mpc_exit_blend_start_time)
                    if blend_elapsed <= CONTROL_MPC_EXIT_BLEND_SECONDS:
                        mpc_exit_blend_alpha = clamp(
                            blend_elapsed / CONTROL_MPC_EXIT_BLEND_SECONDS,
                            0.0,
                            1.0,
                        )
                        mpc_exit_blend_source_steer = mpc_exit_blend_start_steer
                        if (
                            blend_elapsed > 0.0
                            and tracking_route_for_control is not None
                            and getattr(tracking_route_for_control, "is_valid", False)
                        ):
                            try:
                                mpc_exit_control = mpc.control(
                                    ego_vehicle,
                                    tracking_route_for_control,
                                    target_speed,
                                )
                                mpc_exit_blend_source_steer = mpc_exit_control.steer
                            except Exception:
                                mpc_exit_blend_source_steer = mpc_exit_blend_start_steer
                        ego_control.steer = clamp(
                            (1.0 - mpc_exit_blend_alpha) * mpc_exit_blend_source_steer
                            + mpc_exit_blend_alpha * raw_control_steer,
                            -1.0,
                            1.0,
                        )
                        mpc_exit_blend_active = True
                    else:
                        mpc_exit_blend_start_time = None

            steer_delta = ego_control.steer - last_control_steer
            debug_reason = None
            if route_changed_this_frame:
                debug_reason = "route-change"
            elif mpc_exit_blend_started:
                debug_reason = "mpc-exit-blend-start"
            elif mpc_exit_blend_active and sim_time - last_control_debug_time >= CONTROL_DEBUG_MIN_INTERVAL:
                debug_reason = "mpc-exit-blend"
            elif use_mpc_tracking != last_use_mpc_tracking:
                debug_reason = "mpc-start" if use_mpc_tracking else "mpc-stop"
            elif (
                abs(steer_delta) >= CONTROL_DEBUG_STEER_JUMP
                and sim_time - last_control_debug_time >= CONTROL_DEBUG_MIN_INTERVAL
            ):
                debug_reason = "mpc-steer-jump" if use_mpc_tracking else "steer-jump"
            if debug_reason is not None:
                print_control_switch_debug(
                    sim_time,
                    debug_reason,
                    ego_vehicle,
                    tracking_route_for_control,
                    base_route_s,
                    mpc_tracking_until_s,
                    sensor.current_offset(),
                    use_mpc_tracking,
                    route_changed_this_frame,
                    target_speed,
                    ego_control,
                    steer_delta,
                    right_object_decision,
                    raw_steer=raw_control_steer,
                    blend_alpha=mpc_exit_blend_alpha,
                    blend_source_steer=mpc_exit_blend_source_steer,
                )
                last_control_debug_time = sim_time
            last_use_mpc_tracking = use_mpc_tracking
            last_control_steer = ego_control.steer
            ego_vehicle.apply_control(ego_control) # 应用控制命令，控制自车的油门、制动和转向，根据当前状态和感知信息计算得到的控制命令进行应用，实现跟车、避障、右侧物体避让等行为

            if route_changed_this_frame and selected_plan_trajectory is not None:
                debug_plan_draw_until_time = sim_time + DEBUG_DRAW_TRAJECTORY_DURATION
            draw_trajectory_debug(
                world,
                ego_vehicle,
                sensor.tracking_route(),
                selected_plan_trajectory,
                avoidance_candidates,
                frame,
                sim_time <= debug_plan_draw_until_time,
            )

            if frame % int(1.0 / FIXED_DELTA_SECONDS) == 0:
                """每秒输出一次当前状态和关键信息，包括仿真时间、当前状态、前车距离和TTC、右侧物体距离和TTC、自车速度、前车速度、控制命令等，供调试和分析使用"""
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

            world.tick() # 仿真世界进行一次更新，推进仿真时间，并确保所有演员状态更新到最新
            set_spectator(world, ego_vehicle) # 更新观察者视角，保持在自车正上方，提供持续的上帝视角观察仿真过程
            lap_completed = loop_route.update(ego_vehicle) # 更新固定路线状态，基于自车当前的位置更新路线的进度和索引信息，并判断是否完成一圈
            if camera_display is not None:
                """渲染仿真画面和信息，调用显示实例的 render 方法，传入当前仿真时间、状态、前车和右侧物体的感知信息、自车和前车的速度、控制命令、碰撞次数、路线进度等信息，在显示窗口中进行实时渲染，供观察和分析使用"""
                camera_display.render({
                    "sim_time": sim_time,
                    "state": state,
                    "scenario": "front_brake_and_right_object",
                    "ego_speed": ego_speed,
                    "lead_speed": get_speed(lead_vehicle),
                    "front_distance": front.distance,
                    "front_ttc": front.ttc,
                    "front_actor_role": right_object_label(front),
                    "front_risk_level": 3 if state == "EMERGENCY_BRAKE" else (2 if planning_needed else (1 if front.is_front_vehicle else 0)),
                    "right_object_distance": right_object.distance,
                    "right_object_ttc": right_object.ttc,
                    "right_object_type": right_object.object_type or right_object.actor_role,
                    "right_risk_level": right_object.risk_level,
                    "left_side_distance": left_side.distance,
                    "left_side_ttc": left_side.ttc,
                    "left_side_role": left_side.actor_role,
                    "left_side_risk_level": left_side.risk_level,
                    "sensor_overlay_enabled": DEBUG_DRAW_SENSOR_OVERLAY,
                    "steer": ego_control.steer,
                    "throttle": ego_control.throttle,
                    "brake": ego_control.brake,
                    "collision_count": len(collision_monitor.history),
                    "lap_distance": loop_route.progress_distance,
                    "lap_target_distance": loop_route.length,
                })
            frame += 1 # 增加帧计数器，推进仿真时间的计算，并在下一次循环中使用更新后的仿真时间进行控制逻辑判断和信息输出
            pace_realtime_playback(start_time, frame * FIXED_DELTA_SECONDS, args.playback_speed, args.free_run)

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

        elapsed = time.time() - start_time # 计算仿真总耗时，基于仿真开始的墙钟时间和当前时间的差值计算得到，并在仿真结束后输出结果
        print(
            "Scenario finished in {:.1f}s wall time. Collisions: {}".format(
                elapsed, len(collision_monitor.history)
            )
        )

    finally:
        """清理资源：恢复仿真世界的原始设置，销毁所有生成的演员，关闭显示窗口，并输出清理完成的提示信息，确保仿真环境干净整洁，不影响其他程序使用 CARLA"""
        if world is not None and original_settings is not None: # 恢复仿真世界的原始设置，避免影响其他程序使用 CARLA，如果 world 和 original_settings 都不为 None，则调用 restore_world 函数恢复世界设置
            restore_world(world, original_settings)
        for actor in reversed(actor_list): # 销毁所有生成的演员，避免资源泄漏，如果 actor_list 中有演员，则按照生成的逆序进行销毁，确保先销毁后生成的演员，最后输出清理完成的提示信息
            if actor is not None:
                try:
                    actor.destroy()
                except RuntimeError as exc:
                    print("Cleanup warning: failed to destroy actor {}: {}".format(actor.id, exc))
        if camera_display is not None: # 
            try:
                camera_display.close()
            except RuntimeError as exc:
                print("Cleanup warning: failed to close pygame display: {}".format(exc))
        print("Cleanup finished.")


if __name__ == "__main__":
    main()
