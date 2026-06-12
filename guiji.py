import argparse
import math
import random
import time

import carla

from actors import ( # 场景中涉及的各种演员生成函数，包括自车、前车、背景车辆、背景自行车和右侧过街自行车
    spawn_background_r344_bicycles,
    spawn_background_route_vehicles,
    spawn_right_side_bicycle_crossing,
    spawn_right_side_pedestrians, # 生成右侧过街行人，增加右侧物体避让的复杂性
    spawn_scenario, # 生成自车和前车，并返回自车的起始路点
    spawn_slow_right_lane_vehicle, # 生成右侧慢速车辆，增加右侧物体避让的复杂性
)

from config import ( # 仿真参数配置，包括服务器连接、仿真时间、车辆目标速度、换道长度、安全距离、碰撞和避让的 TTC 阈值等
    CLIENT_TIMEOUT,
    DEBUG_DRAW_INTERVAL_FRAMES, # 调试绘制的帧间隔，控制仿真中轨迹和目标点的绘制频率，避免过于密集导致画面混乱
    DEBUG_DRAW_LIFETIME,
    DEBUG_DRAW_LOOKAHEAD_DISTANCE,
    DEBUG_DRAW_TRAJECTORY,
    DEBUG_DRAW_TRAJECTORY_STEP,
    EGO_TARGET_SPEED,
    FIXED_DELTA_SECONDS,
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
    RIGHT_OBJECT_DETECT_DISTANCE,
    RIGHT_OBJECT_STOP_RELEASE_DISTANCE,
    RIGHT_OBJECT_STOP_DISTANCE,
    RIGHT_OBJECT_TTC_THRESHOLD,
    RIGHT_OBJECT_YIELD_SPEED,
    ROUTE_COMPLETION_HOLD_SECONDS,
    SAFE_DISTANCE,
    SIM_SECONDS,
    TRAFFIC_RANDOM_SEED,
    TTC_AVOID_THRESHOLD,
    TTC_BRAKE_THRESHOLD,
)
from control import SamplingMPCTracker # MPC 控制器实现
from control import select_best_route_offset_trajectory, select_return_to_base_trajectory
from display import CollisionMonitor, PygameDemoDisplay # 碰撞监测和仿真显示实现
from perception import VirtualGroundTruthSensor # 虚拟传感器实现，提供前车和右侧过街物体的距离、TTC 等信息
from route import LoopRoute # 固定路线实现，提供路线点、航向和转弯信息
from utils import get_speed, smooth_reference_for, speed_control, waypoint_steer # 工具函数，包括获取速度、速度控制和航向控制等

RIGHT_OBJECT_CLEAR_HOLD_SECONDS = 2.0
EMERGENCY_BRAKE_TTC_SECONDS = 1.8
EMERGENCY_BRAKE_DISTANCE = 8.0
PLAN_RETRY_LOG_INTERVAL = 1.0
RETURN_TO_BASE_CLEARANCE = 2.0

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


def plan_route_replacement(loop_route, ego_vehicle, front, obstacle_actors, sim_time):
    """生成一个可替换当前基础路线 s 区间的避障路径段。"""
    target_speed = 0.0 if front.actor_role == "lead" else max(0.0, getattr(front, "target_speed_along", 0.0))
    predicted_motion = target_speed * min(3.0, max(1.0, front.distance / max(front.closing_speed, 0.1)))
    base_length = max(14.0, min(52.0, LANE_CHANGE_LENGTH + predicted_motion))
    best_candidate, candidates = select_best_route_offset_trajectory(
        loop_route, ego_vehicle, front, base_length, obstacle_actors=obstacle_actors
    )
    print_avoidance_candidate_summary(best_candidate, candidates) # 打印候选路径数量、筛选结果和最终选中的关键参数，便于分析避障轨迹的生成和选择过程
    selected_trajectory = best_candidate.trajectory if best_candidate is not None else None
    if selected_trajectory is not None:
        print(
            "Route replacement planned at {:.2f}s: s={:.1f}-{:.1f}, peak_offset={:.2f}m, target={}, distance={:.1f}m, TTC={:.2f}s".format(
                sim_time,
                best_candidate.start_route_s,
                best_candidate.end_route_s,
                best_candidate.target_offset,
                right_object_label(front),
                front.distance,
                front.ttc if math.isfinite(front.ttc) else 99.99,
            )
        )
    return best_candidate, candidates


def plan_return_to_base(loop_route, ego_vehicle, obstacle_actors, sim_time):
    """生成从当前偏移回到基础路线的候选段。"""
    best_candidate, candidates = select_return_to_base_trajectory(
        loop_route, ego_vehicle, LANE_CHANGE_LENGTH, obstacle_actors=obstacle_actors
    )
    print_avoidance_candidate_summary(best_candidate, candidates)
    if best_candidate is not None:
        print(
            "Return-to-base planned at {:.2f}s: s={:.1f}-{:.1f}, start_offset={:.2f}m, length={:.1f}m.".format(
                sim_time,
                best_candidate.start_route_s,
                best_candidate.end_route_s,
                best_candidate.start_offset,
                best_candidate.length,
            )
        )
    return best_candidate, candidates


def print_avoidance_candidate_summary(best_candidate, candidates):
    """打印候选路径数量、筛选结果和最终选中的关键参数。"""
    valid_count = sum(1 for candidate in candidates if candidate.is_valid)
    if best_candidate is None:
        reasons = {}
        for candidate in candidates:
            reasons[candidate.reject_reason] = reasons.get(candidate.reject_reason, 0) + 1
        print("Avoidance candidates rejected: total={}, reasons={}.".format(len(candidates), reasons))
        return

    print(
        "Best avoidance candidate: valid={}/{}, length={:.1f}m, start_offset={:.2f}m, target_offset={:.2f}m, ay={:.2f}m/s^2, cost={:.2f}.".format(
            valid_count,
            len(candidates),
            best_candidate.length,
            best_candidate.start_offset,
            best_candidate.target_offset,
            best_candidate.lateral_accel,
            best_candidate.total_cost,
        )
    )


def front_planning_needed(front):
    if not front.is_front_vehicle or not math.isfinite(front.distance):
        return False
    close_slow_front_vehicle = (
        front.distance < LANE_CHANGE_LENGTH + 6.0
        and front.closing_speed > 2.0
    )
    return front.distance < SAFE_DISTANCE and (front.ttc < TTC_AVOID_THRESHOLD or close_slow_front_vehicle)


def front_emergency_brake_needed(front):
    if not front.is_front_vehicle or not math.isfinite(front.distance):
        return False
    return front.distance < EMERGENCY_BRAKE_DISTANCE or front.ttc < EMERGENCY_BRAKE_TTC_SECONDS


def apply_route_replacement(sensor, candidate):
    if candidate is None or candidate.trajectory is None:
        return None
    trajectory = candidate.trajectory
    sensor.apply_replacement_segment(
        candidate.start_route_s,
        candidate.end_route_s,
        trajectory.replacement_points(DEBUG_DRAW_TRAJECTORY_STEP),
        end_offset=candidate.target_offset,
    )
    return trajectory


def make_avoidance_target(front, obstacle_actors, target_offset):
    return {
        "actor_id": front.actor_id,
        "actor_role": front.actor_role,
        "actor": find_actor_by_id(obstacle_actors, front.actor_id),
        "target_offset": target_offset,
    }


def find_actor_by_id(actors, actor_id):
    if actor_id is None:
        return None
    for actor in actors:
        if actor is not None and getattr(actor, "id", None) == actor_id:
            return actor
    return None


def vehicle_half_length(actor, default_length=2.4):
    bbox = getattr(actor, "bounding_box", None)
    extent = getattr(bbox, "extent", None)
    if extent is None:
        return default_length
    return max(0.1, float(extent.x))


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
    """用绿色线显示有效候选避障轨迹，选中轨迹使用更亮更粗的线。"""
    if not candidates:
        return
    for candidate in candidates:
        if not candidate.is_valid:
            continue
        trajectory = candidate.trajectory
        is_selected = trajectory is selected_trajectory
        color = carla.Color(0, 255, 40) if is_selected else carla.Color(0, 170, 0)
        thickness = 0.08 if is_selected else 0.04
        s = 0.0
        previous = trajectory.location_at(s) + carla.Location(z=0.18)
        while s < trajectory.length:
            s = min(trajectory.length, s + DEBUG_DRAW_TRAJECTORY_STEP)
            current = trajectory.location_at(s) + carla.Location(z=0.18)
            world.debug.draw_line(previous, current, thickness=thickness, color=color, life_time=DEBUG_DRAW_LIFETIME)
            previous = current


def draw_trajectory_debug(world, ego_vehicle, tracking_route, selected_trajectory, candidates, frame):
    """统一绘制当前跟踪路线前视点和最近一次候选轨迹。"""
    if not DEBUG_DRAW_TRAJECTORY:
        return
    if frame % max(1, DEBUG_DRAW_INTERVAL_FRAMES) != 0:
        return
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
        mpc = SamplingMPCTracker() # 创建 MPC 控制器实例，用于后续的轨迹跟踪控制
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
        right_object_clear_since = None
        last_plan_failure_log_time = -999.0
        avoidance_candidates = []
        active_avoidance_target = None
        mpc_tracking_until_s = None
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
            ego_speed = get_speed(ego_vehicle)

            planning_needed = front_planning_needed(front)
            brake_needed = front.is_front_vehicle and front.ttc < TTC_BRAKE_THRESHOLD
            emergency_recovered = (
                not front.is_front_vehicle
                or front.distance > SAFE_DISTANCE + 8.0
                or front.ttc > TTC_BRAKE_THRESHOLD + 1.0
            )
            right_object_risk = (
                right_object.is_conflict_object
                and (
                    right_object.risk_level >= 2
                    or right_object.ttc < RIGHT_OBJECT_TTC_THRESHOLD
                    or right_object.distance < RIGHT_OBJECT_DETECT_DISTANCE
                )
            )

            route_changed_this_frame = False

            if state == "EMERGENCY_BRAKE":
                if emergency_recovered:
                    state = "ROUTE_FOLLOW"
                    print(
                        "Emergency brake recovered at {:.2f}s: distance={:.1f}m, TTC={:.2f}s".format(
                            sim_time,
                            front.distance,
                            front.ttc if math.isfinite(front.ttc) else 99.99,
                        )
                    )
                elif planning_needed:
                    best_candidate, avoidance_candidates = plan_route_replacement(
                        loop_route, ego_vehicle, front, obstacle_actors, sim_time
                    )
                    selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                    if selected_plan_trajectory is not None:
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
                    loop_route, ego_vehicle, obstacle_actors, sim_time
                )
                selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                if selected_plan_trajectory is not None:
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
                elif sim_time - last_plan_failure_log_time >= PLAN_RETRY_LOG_INTERVAL:
                    print(
                        "Return planning blocked at {:.2f}s: keep offset={:.2f}m, target={}.".format(
                            sim_time,
                            sensor.current_offset(),
                            active_target_label,
                        )
                    )
                    last_plan_failure_log_time = sim_time

            if state == "ROUTE_FOLLOW" and planning_needed and not route_changed_this_frame:
                best_candidate, avoidance_candidates = plan_route_replacement(
                    loop_route, ego_vehicle, front, obstacle_actors, sim_time
                )
                selected_plan_trajectory = apply_route_replacement(sensor, best_candidate)
                if selected_plan_trajectory is not None:
                    active_avoidance_target = make_avoidance_target(
                        front, obstacle_actors, best_candidate.target_offset
                    )
                    mpc_tracking_until_s = best_candidate.end_route_s + 2.0
                    route_changed_this_frame = True
                else:
                    if front_emergency_brake_needed(front):
                        state = "EMERGENCY_BRAKE"
                        print(
                            "Emergency brake at {:.2f}s: no collision-free replacement, target={}, distance={:.1f}m, TTC={:.2f}s".format(
                                sim_time,
                                right_object_label(front),
                                front.distance,
                                front.ttc if math.isfinite(front.ttc) else 99.99,
                            )
                        )
                    elif sim_time - last_plan_failure_log_time >= PLAN_RETRY_LOG_INTERVAL:
                        print(
                            "Replacement planning failed at {:.2f}s: keep current route and retry, target={}, distance={:.1f}m, TTC={:.2f}s".format(
                                sim_time,
                                right_object_label(front),
                                front.distance,
                                front.ttc if math.isfinite(front.ttc) else 99.99,
                            )
                        )
                        last_plan_failure_log_time = sim_time

            if right_object_risk:
                if last_right_object_actor_id != right_object.actor_id:
                    action = "started" if last_right_object_actor_id is None else "target changed"
                    print(
                        "Right object yield {} at {:.2f}s: target={}, distance={:.1f}m, TTC={:.2f}s, route_index={}.".format(
                            action,
                            sim_time,
                            right_object_label(right_object),
                            right_object.distance,
                            right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                            loop_route.last_index,
                        )
                    )
                    last_right_object_actor_id = right_object.actor_id
                right_object_clear_since = None
            elif last_right_object_actor_id is not None:
                if right_object_clear_since is None:
                    right_object_clear_since = sim_time
                elif sim_time - right_object_clear_since >= RIGHT_OBJECT_CLEAR_HOLD_SECONDS:
                    right_object_stop_active = False
                    last_right_object_actor_id = None
                    right_object_clear_since = None
                    print("Right object yield completed at {:.2f}s.".format(sim_time))

            if route_completion_time is not None:
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "EMERGENCY_BRAKE":
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            else:
                target_speed = EGO_TARGET_SPEED
                if brake_needed:
                    target_speed = min(target_speed, max(0.0, ego_speed - 5.0))
                if right_object_risk or right_object_clear_since is not None or right_object_stop_active:
                    target_speed = min(target_speed, RIGHT_OBJECT_YIELD_SPEED)

                tracking_route = sensor.tracking_route()
                base_route_s = project_actor_to_base_route(loop_route, ego_vehicle)["route_s"]
                if mpc_tracking_until_s is not None and base_route_s > mpc_tracking_until_s:
                    mpc_tracking_until_s = None
                replacement_segment_active = mpc_tracking_until_s is not None
                offset_route_active = abs(sensor.current_offset()) > 0.05
                use_mpc_tracking = (
                    tracking_route is not None
                    and tracking_route.is_valid
                    and (replacement_segment_active or offset_route_active)
                )

                if use_mpc_tracking:
                    ego_control = mpc.control(ego_vehicle, tracking_route, target_speed)
                else:
                    throttle, brake = speed_control(ego_speed, target_speed)
                    ego_control = carla.VehicleControl(
                        throttle=throttle,
                        brake=brake,
                        steer=loop_route.steer(ego_vehicle),
                    )

                if brake_needed:
                    ego_control.brake = max(ego_control.brake, 0.20)
                    ego_control.throttle = 0.0

                if right_object_risk and not right_object_stop_active and right_object.distance < RIGHT_OBJECT_STOP_DISTANCE:
                    right_object_stop_active = True
                    print(
                        "Right object hard stop engaged at {:.2f}s: target={}, distance={:.1f}m.".format(
                            sim_time, right_object_label(right_object), right_object.distance
                        )
                    )
                elif (
                    right_object_stop_active
                    and right_object.actor_id is not None
                    and right_object.distance > RIGHT_OBJECT_STOP_RELEASE_DISTANCE
                ):
                    right_object_stop_active = False
                    print(
                        "Right object hard stop released at {:.2f}s: target={}, distance={:.1f}m.".format(
                            sim_time, right_object_label(right_object), right_object.distance
                        )
                    )

                clear_waiting = (
                    right_object_clear_since is not None
                    and sim_time - right_object_clear_since < RIGHT_OBJECT_CLEAR_HOLD_SECONDS
                )
                if right_object_stop_active or clear_waiting:
                    ego_control.throttle = 0.0
                    ego_control.brake = max(ego_control.brake, 0.85)
            ego_vehicle.apply_control(ego_control) # 应用控制命令，控制自车的油门、制动和转向，根据当前状态和感知信息计算得到的控制命令进行应用，实现跟车、避障、右侧物体避让等行为

            draw_trajectory_debug(
                world,
                ego_vehicle,
                sensor.tracking_route(),
                selected_plan_trajectory,
                avoidance_candidates,
                frame,
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
                    "right_object_distance": right_object.distance,
                    "right_object_ttc": right_object.ttc,
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
