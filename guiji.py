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
import config
from control import SamplingMPCTracker # MPC 控制器实现
from control import select_best_route_offset_trajectory
from display import CollisionMonitor, PygameDemoDisplay # 碰撞监测和仿真显示实现
from perception import VirtualGroundTruthSensor # 虚拟传感器实现，提供前车和右侧过街物体的距离、TTC 等信息
from route import LoopRoute # 固定路线实现，提供路线点、航向和转弯信息
from utils import clamp
from utils import get_speed, speed_control, waypoint_steer # 工具函数，包括获取速度、速度控制和航向控制等

POST_AVOID_LANE_HOLD_SECONDS = 1.0
AVOID_REPLAN_COOLDOWN_SECONDS = 1.2
AVOID_REPLAN_MIN_PROGRESS = 4.0
RIGHT_OBJECT_CLEAR_HOLD_SECONDS = 2.0

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


def choose_avoidance_side(sensor):
    """根据邻道净空状况选择避障换道方向：优先左转，其次右转，无道则返回 None"""
    if sensor.lane_clear("left"): # 调用传感器的 lane_clear 方法检查左侧邻道是否在前后安全范围内无车，如果左侧邻道清空，则选择左换道避障
        return "left"
    if sensor.lane_clear("right"):
        return "right"
    return None


def lane_label(waypoint):
    """格式化路点的道路和车道编号，供避障诊断日志使用。"""
    if waypoint is None:
        return "None"
    return "road={}, lane={}, yaw={:.1f}".format(
        waypoint.road_id,
        waypoint.lane_id,
        waypoint.transform.rotation.yaw,
    )


def print_avoidance_lane_choice(carla_map, ego_vehicle, avoidance_side, sim_time):
    """打印本次避障选择的当前车道和目标邻道，便于判断弯道避障是否选错方向。"""
    current_wp = carla_map.get_waypoint(
        ego_vehicle.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
    )
    target_wp = current_wp.get_left_lane() if avoidance_side == "left" else current_wp.get_right_lane()
    print(
        "Avoidance lane choice at {:.2f}s: side={}, current=[{}], target=[{}]".format(
            sim_time,
            avoidance_side,
            lane_label(current_wp),
            lane_label(target_wp),
        )
    )
    return current_wp, target_wp


def plan_route_relative_avoidance(carla_map, loop_route, ego_vehicle, front, avoidance_side, sim_time):
    """打印邻道选择、生成多候选路线相对避障轨迹，并返回最佳候选轨迹。"""
    current_wp, target_wp = print_avoidance_lane_choice(
        carla_map, ego_vehicle, avoidance_side, sim_time
    )
    target_speed = 0.0 if front.actor_role == "lead" else max(0.0, getattr(front, "target_speed_along", 0.0))
    predicted_motion = target_speed * min(3.0, max(1.0, front.distance / max(front.closing_speed, 0.1)))
    base_length = max(14.0, min(52.0, LANE_CHANGE_LENGTH + predicted_motion))
    best_candidate, candidates = select_best_route_offset_trajectory(
        loop_route, ego_vehicle, target_wp, front, base_length
    )
    print_avoidance_candidate_summary(best_candidate, candidates)
    selected_trajectory = best_candidate.trajectory if best_candidate is not None else None
    return selected_trajectory, candidates


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


def avoidance_laterally_separated(trajectory, lateral):
    """判断避障过程中自车是否已经横向脱离原车道风险区。"""
    lateral_shift = abs(trajectory.lateral_offset - trajectory.start_offset)
    if lateral_shift < 0.1:
        return False
    completed_shift = abs(lateral - trajectory.start_offset)
    return completed_shift > min(1.4, lateral_shift * 0.55)


def build_avoidance_front_reference_points(trajectory, ego_vehicle):
    """Build the active front-perception path: current avoidance path plus route continuation."""
    progress, _ = trajectory.to_local(ego_vehicle.get_location())
    start_s = max(0.0, progress - 8.0)
    end_s = max(trajectory.length + SAFE_DISTANCE + 18.0, progress + SAFE_DISTANCE + 28.0)
    step = max(1.0, DEBUG_DRAW_TRAJECTORY_STEP)
    points = []
    sample_count = int(math.ceil((end_s - start_s) / step))
    for sample in range(sample_count + 1):
        s = min(end_s, start_s + sample * step)
        points.append(trajectory.location_at(s))
    return points


def avoid_replan_needed(front, current_target_actor_id, progress, sim_time, last_replan_time):
    """Return True when the active avoidance path sees a new or still-dangerous front risk."""
    if not front.is_front_vehicle:
        return False
    if not math.isfinite(front.distance):
        return False
    if progress < AVOID_REPLAN_MIN_PROGRESS:
        return False
    if sim_time - last_replan_time < AVOID_REPLAN_COOLDOWN_SECONDS:
        return False
    new_target = front.actor_id is not None and front.actor_id != current_target_actor_id
    urgent_same_target = front.ttc < TTC_BRAKE_THRESHOLD or front.distance < max(12.0, LANE_CHANGE_LENGTH * 0.55)
    return new_target or urgent_same_target


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


def draw_route_lookahead_marker(world, loop_route, lookahead_distance=DEBUG_DRAW_LOOKAHEAD_DISTANCE):
    """在正常路线前方约 lookahead_distance 处画红色目标标记。"""
    lookahead_steps = max(1, int(lookahead_distance / loop_route.step_distance))
    target_index = min(loop_route.last_index + lookahead_steps, len(loop_route.points) - 1)
    draw_debug_marker(world, loop_route.points[target_index], carla.Color(255, 0, 0))


def draw_avoidance_lookahead_marker(world, ego_vehicle, trajectory, lookahead_distance=DEBUG_DRAW_LOOKAHEAD_DISTANCE):
    """在当前避障轨迹前方约 lookahead_distance 处画红色目标标记。"""
    progress, _ = trajectory.to_local(ego_vehicle.get_location())
    target_s = min(trajectory.length + lookahead_distance, progress + lookahead_distance)
    draw_debug_marker(world, trajectory.location_at(target_s), carla.Color(255, 0, 0))


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


def draw_trajectory_debug(world, loop_route, ego_vehicle, state, trajectory, candidates, frame):
    """统一绘制路线/避障前视点和避障候选轨迹。"""
    if not DEBUG_DRAW_TRAJECTORY:
        return
    if frame % max(1, DEBUG_DRAW_INTERVAL_FRAMES) != 0:
        return
    if state == "AVOID" and trajectory is not None:
        draw_avoidance_candidates(world, candidates, trajectory)
        draw_avoidance_lookahead_marker(world, ego_vehicle, trajectory)
    elif state == "ROUTE_FOLLOW":
        draw_route_lookahead_marker(world, loop_route)


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

        # === AlphaBetaTracker init (from feature/perception-risk) ===
        try:
            from perception import AlphaBetaTracker
            tracker = AlphaBetaTracker() if getattr(config, 'TRACKER_ENABLED', True) else None
        except ImportError:
            tracker = None
            print("AlphaBetaTracker not implemented yet, skipping tracker.")
        _prev_ego_location = ego_vehicle.get_location()
        if RADAR_ENABLED:
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

        # === Side radar mount (from feature/perception-risk) ===
        side_radar = None
        if getattr(config, 'SIDE_RADAR_ENABLED', True):
            side_radar_bp = world.get_blueprint_library().find("sensor.other.radar")
            side_radar_bp.set_attribute("horizontal_fov",
                str(getattr(config, 'SIDE_RADAR_FOV_HORIZONTAL_DEG', 150.0)))
            side_radar_bp.set_attribute("vertical_fov", "15.0")
            side_radar_bp.set_attribute("range",
                str(getattr(config, 'SIDE_RADAR_RANGE', 30.0)))
            side_radar_bp.set_attribute("points_per_second",
                str(getattr(config, 'SIDE_RADAR_POINTS_PER_SECOND', 800)))
            side_radar_tf = carla.Transform(
                carla.Location(
                    x=getattr(config, 'SIDE_RADAR_MOUNT_X', 0.0),
                    y=getattr(config, 'SIDE_RADAR_MOUNT_Y', 1.0),
                    z=getattr(config, 'SIDE_RADAR_MOUNT_Z', 0.5)),
                carla.Rotation(yaw=90.0),
            )
            side_radar = world.spawn_actor(side_radar_bp, side_radar_tf, attach_to=ego_vehicle)
            actor_list.append(side_radar)

            def side_radar_callback(data):
                """Side radar callback."""
                sensor._side_radar_detections = data

            side_radar.listen(side_radar_callback)
            print("Side radar sensor mounted: range={:.0f}m, fov={:.0f}deg".format(
                getattr(config, 'SIDE_RADAR_RANGE', 30.0),
                getattr(config, 'SIDE_RADAR_FOV_HORIZONTAL_DEG', 150.0)))

        # === RGB + Semantic segmentation camera mount (from feature/perception-risk) ===
        if getattr(config, 'CAMERA_ENABLED', True):
            cam_x = getattr(config, 'CAMERA_FRONT_X', 1.5)
            cam_z = getattr(config, 'CAMERA_FRONT_Z', 1.4)
            camera_tf = carla.Transform(carla.Location(x=cam_x, z=cam_z))

            rgb_bp = world.get_blueprint_library().find("sensor.camera.rgb")
            rgb_bp.set_attribute("image_size_x",
                str(getattr(config, 'CAMERA_RGB_WIDTH', 800)))
            rgb_bp.set_attribute("image_size_y",
                str(getattr(config, 'CAMERA_RGB_HEIGHT', 600)))
            rgb_bp.set_attribute("fov",
                str(getattr(config, 'CAMERA_RGB_FOV', 90.0)))
            rgb_camera = world.spawn_actor(rgb_bp, camera_tf, attach_to=ego_vehicle)
            actor_list.append(rgb_camera)
            rgb_camera.listen(lambda image: None)  # placeholder
            print("RGB camera mounted: {}x{}".format(
                getattr(config, 'CAMERA_RGB_WIDTH', 800),
                getattr(config, 'CAMERA_RGB_HEIGHT', 600)))

            sem_bp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
            sem_bp.set_attribute("image_size_x",
                str(getattr(config, 'CAMERA_SEMANTIC_WIDTH', 800)))
            sem_bp.set_attribute("image_size_y",
                str(getattr(config, 'CAMERA_SEMANTIC_HEIGHT', 600)))
            sem_bp.set_attribute("fov",
                str(getattr(config, 'CAMERA_SEMANTIC_FOV', 90.0)))
            sem_camera = world.spawn_actor(sem_bp, camera_tf, attach_to=ego_vehicle)
            actor_list.append(sem_camera)

            def sem_camera_callback(image):
                """Semantic segmentation callback."""
                try:
                    import numpy as np
                    image.convert(carla.ColorConverter.CityScapesPalette)
                    array = np.frombuffer(image.raw_data, dtype=np.uint8)
                    array = np.reshape(array, (image.height, image.width, 4))
                    semantic_labels = array[:, :, 2].astype(int)
                    sensor.set_camera_classifications(semantic_labels)
                except Exception:
                    pass

            sem_camera.listen(sem_camera_callback)
            print("Semantic segmentation camera mounted: {}x{}".format(
                getattr(config, 'CAMERA_SEMANTIC_WIDTH', 800),
                getattr(config, 'CAMERA_SEMANTIC_HEIGHT', 600)))

        state = "ROUTE_FOLLOW" # 定义初始状态为路线跟踪，后续根据感知信息和事件进行状态转换，包括避障换道、紧急制动、右侧物体避让等
        trajectory = None # 定义当前避障换道轨迹，初始为 None，在需要避障时生成具体的换道轨迹供 MPC 跟踪使用
        start_time = time.time() # 记录仿真开始的墙钟时间，用于计算总仿真耗时，最后在仿真结束后输出结果
        frame = 0 # 定义仿真帧计数器，初始为 0，在主循环中每次迭代增加 1，用于计算当前仿真时间和控制逻辑的时间判断
        route_completion_time = None # 定义路线完成时间，初始为 None，在完成固定路线一圈时记录仿真时间，供后续判断是否继续运行一定时间后结束仿真
        right_object_stop_active = False
        last_right_object_actor_id = None
        right_object_clear_since = None
        post_avoid_lane_hold_until = None
        active_avoid_target_actor_id = None
        last_avoid_replan_time = -999.0

        avoidance_candidates = []

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

            # === Tracker predict & update (from feature/perception-risk) ===
            if 'tracker' in dir() and tracker is not None:
                ego_loc_now = ego_vehicle.get_location()
                ego_dx_local = ego_loc_now.x - _prev_ego_location.x
                ego_dy_local = ego_loc_now.y - _prev_ego_location.y
                _prev_ego_location = ego_loc_now

                tracker.predict(ego_dx_local, ego_dy_local)

                detections = []
                for actor in actor_list:
                    if actor is None or not actor.is_alive:
                        continue
                    if actor.id == ego_vehicle.id:
                        continue
                    if actor.type_id.startswith('vehicle.') or actor.type_id.startswith('walker.'):
                        loc = actor.get_location()
                        vel = actor.get_velocity()
                        detections.append({
                            'x': loc.x,
                            'y': loc.y,
                            'vx': vel.x,
                            'vy': vel.y,
                            'actor_id': actor.id,
                            'type': actor.type_id,
                        })
                tracker.update(detections)

            if state == "AVOID" and trajectory is not None:
                sensor.set_front_reference_points(build_avoidance_front_reference_points(trajectory, ego_vehicle))
            else:
                sensor.clear_front_reference_points()

            front = sensor.front_vehicle(use_route_reference=True) # 获取前车的感知信息；ROUTE_FOLLOW 使用原路线弧线参考，AVOID 使用避障轨迹加后续路线的联合参考
            right_object = sensor.right_side_object(loop_route.last_index) # 获取右侧过街物体的感知信息，调用传感器的 right_side_object 方法获取当前右侧过街物体的距离、TTC 等信息，供控制决策使用
            ego_speed = get_speed(ego_vehicle)

            close_slow_front_vehicle = (
                front.is_front_vehicle
                and front.distance < LANE_CHANGE_LENGTH + 6.0
                and front.closing_speed > 2.0
            )
            emergency_needed = ( # 判断是否需要进行紧急避障，基于前车的感知信息进行判断，如果前车确认为正前方车辆，并且距离小于安全距离，并且 TTC 小于避障阈值，则认为需要进行紧急避障
                front.is_front_vehicle # 从感知信息中获取前车是否确认为正前方车辆的布尔值（也就是返回是否存在一个正前方车辆）
                and front.distance < SAFE_DISTANCE # 存在正前方车辆的话，这里是最近的一个，判断其距离是否小于安全距离
                and (front.ttc < TTC_AVOID_THRESHOLD or close_slow_front_vehicle)
            )
            brake_needed = ( # 定义是否需要制动的条件，基于前车的感知信息进行判断，如果前车确认为正前方车辆，并且距离小于安全距离，并且 TTC 小于制动阈值，则认为需要进行制动
                front.is_front_vehicle
                and front.ttc < TTC_BRAKE_THRESHOLD
            )
            emergency_recovered = ( # 紧急制动恢复条件：前方目标消失、距离重新拉开，或 TTC 恢复到制动阈值以上，避免 EMERGENCY_BRAKE 成为永久状态
                not front.is_front_vehicle
                or front.distance > SAFE_DISTANCE + 8.0
                or front.ttc > TTC_BRAKE_THRESHOLD + 1.0
            )
            right_object_risk = ( # 定义右侧物体是否构成风险的条件，基于右侧过街物体的感知信息进行判断，如果右侧物体确认为冲突对象，并且 TTC 小于右侧物体风险阈值或者距离小于右侧物体检测距离，则认为构成风险
                right_object.is_conflict_object
                and (
                    right_object.risk_level >= 2
                    or right_object.ttc < RIGHT_OBJECT_TTC_THRESHOLD
                    or right_object.distance < RIGHT_OBJECT_DETECT_DISTANCE
                )
            )

            """以下的if语句为状态切换逻辑，根据当前状态和感知信息判断是否需要切换到避障状态、紧急制动状态或右侧物体避让状态，并设置相应的状态变量和输出相关信息"""

            if state == "ROUTE_FOLLOW" and emergency_needed:
                """从路线跟踪状态切换到避障状态，调用 choose_avoidance_side 函数根据邻道净空状况选择避障换道方向；避障/让行完成后仍回到 ROUTE_FOLLOW，允许再次触发前方避障。"""
                avoidance_side = choose_avoidance_side(sensor)
                if avoidance_side is not None:
                    """生成路线相对避障轨迹，供 MPC 跟踪使用，并设置状态为 AVOID 进行避障换道。"""
                    selected_trajectory, avoidance_candidates = plan_route_relative_avoidance(
                        carla_map, loop_route, ego_vehicle, front, avoidance_side, sim_time
                    )
                    if selected_trajectory is None:
                        state = "EMERGENCY_BRAKE"
                        print(
                            "Emergency brake only at {:.2f}s: no valid avoidance path, TTC={:.2f}s".format(
                                sim_time, front.ttc
                            )
                        )
                    else:
                        trajectory = selected_trajectory
                        active_avoid_target_actor_id = front.actor_id
                        last_avoid_replan_time = sim_time
                        state = "AVOID" # 设置状态为 AVOID 进行避障换道
                        print(
                            "Avoidance started at {:.2f}s: side={}, distance={:.1f}m, TTC={:.2f}s".format(
                                sim_time, avoidance_side, front.distance, front.ttc
                            )
                        )
                else: # 没有可用的换道方向，直接设置状态为 EMERGENCY_BRAKE 进行紧急制动
                    state = "EMERGENCY_BRAKE"
                    print(
                        "Emergency brake only at {:.2f}s: no adjacent clear lane, TTC={:.2f}s".format(
                            sim_time, front.ttc
                        )
                    )
            elif state == "EMERGENCY_BRAKE":
                if emergency_recovered:
                    state = "ROUTE_FOLLOW"
                    print(
                        "Emergency brake recovered at {:.2f}s: distance={:.1f}m, TTC={:.2f}s".format(
                            sim_time,
                            front.distance,
                            front.ttc if math.isfinite(front.ttc) else 99.99,
                        )
                    )
                elif emergency_needed:
                    avoidance_side = choose_avoidance_side(sensor)
                    if avoidance_side is not None:
                        selected_trajectory, avoidance_candidates = plan_route_relative_avoidance(
                            carla_map, loop_route, ego_vehicle, front, avoidance_side, sim_time
                        )
                        if selected_trajectory is not None:
                            trajectory = selected_trajectory
                            active_avoid_target_actor_id = front.actor_id
                            last_avoid_replan_time = sim_time
                            state = "AVOID"
                            print(
                                "Emergency brake switched to avoidance at {:.2f}s: side={}, distance={:.1f}m, TTC={:.2f}s".format(
                                    sim_time, avoidance_side, front.distance, front.ttc
                                )
                            )

            if state == "ROUTE_FOLLOW" and right_object_risk: # 从路线跟踪状态切换到右侧物体避让状态；右侧物体再次达到风险条件时也允许重新触发避让
                state = "RIGHT_OBJECT_YIELD" # 设置状态为 RIGHT_OBJECT_YIELD 进行右侧物体避让
                right_object_stop_active = False
                last_right_object_actor_id = right_object.actor_id
                right_object_clear_since = None
                print(
                    "Right object yield started at {:.2f}s: target={}, distance={:.1f}m, TTC={:.2f}s, route_index={}.".format(
                        sim_time,
                        right_object_label(right_object),
                        right_object.distance,
                        right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        loop_route.last_index,
                    )
                )

            if route_completion_time is not None:
                """如果已经完成固定路线一圈，并且继续运行的时间超过预设的保持时间，则提前终止仿真"""
                state = "ROUTE_HOLD"
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "AVOID" and trajectory is not None:
                """在避障状态下，使用 MPC 控制器跟踪避障换道轨迹，计算所需的控制命令，并应用控制；如果需要制动则增加制动值；如果当前轨迹点已经完成并且接近目标车道，则切换回车道保持状态"""
                target_speed = min(EGO_TARGET_SPEED, max(8.0, ego_speed))
                progress, lateral = trajectory.to_local(ego_vehicle.get_location())
                lateral_separated = avoidance_laterally_separated(trajectory, lateral)
                if emergency_needed and avoid_replan_needed(
                    front, active_avoid_target_actor_id, progress, sim_time, last_avoid_replan_time
                ):
                    avoidance_side = choose_avoidance_side(sensor)
                    if avoidance_side is not None:
                        selected_trajectory, avoidance_candidates = plan_route_relative_avoidance(
                            carla_map, loop_route, ego_vehicle, front, avoidance_side, sim_time
                        )
                        if selected_trajectory is not None:
                            trajectory = selected_trajectory
                            active_avoid_target_actor_id = front.actor_id
                            last_avoid_replan_time = sim_time
                            progress, lateral = trajectory.to_local(ego_vehicle.get_location())
                            lateral_separated = avoidance_laterally_separated(trajectory, lateral)
                            print(
                                "Avoidance replanned at {:.2f}s: target={}, side={}, distance={:.1f}m, TTC={:.2f}s".format(
                                    sim_time,
                                    right_object_label(front),
                                    avoidance_side,
                                    front.distance,
                                    front.ttc if math.isfinite(front.ttc) else 99.99,
                                )
                            )
                        elif front.ttc < TTC_BRAKE_THRESHOLD or front.distance < 10.0:
                            state = "EMERGENCY_BRAKE"
                            trajectory = None
                            avoidance_candidates = []
                            active_avoid_target_actor_id = None
                            print(
                                "Avoidance replanning failed at {:.2f}s: braking for target={}, distance={:.1f}m, TTC={:.2f}s".format(
                                    sim_time,
                                    right_object_label(front),
                                    front.distance,
                                    front.ttc if math.isfinite(front.ttc) else 99.99,
                                )
                            )
                    elif front.ttc < TTC_BRAKE_THRESHOLD or front.distance < 10.0:
                        state = "EMERGENCY_BRAKE"
                        trajectory = None
                        avoidance_candidates = []
                        active_avoid_target_actor_id = None
                        print(
                            "Avoidance replanning blocked at {:.2f}s: no adjacent lane for target={}, distance={:.1f}m, TTC={:.2f}s".format(
                                sim_time,
                                right_object_label(front),
                                front.distance,
                                front.ttc if math.isfinite(front.ttc) else 99.99,
                            )
                        )
                if state == "EMERGENCY_BRAKE":
                    ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)
                else:
                    ego_control = mpc.control(ego_vehicle, trajectory, target_speed) # 在避障状态下，使用 MPC 控制器跟踪避障换道轨迹，计算所需的控制命令，基于当前自车状态、预生成的换道轨迹和目标速度进行计算，供后续应用控制命令使用
                if state == "AVOID" and brake_needed and not lateral_separated:
                    """如果需要制动则增加制动值，基于前车的感知信息判断是否需要增加制动值，如果需要则将油门设置为0，并且将制动值增加到至少0.20，以增强避障时的安全性"""
                    ego_control.brake = max(ego_control.brake, 0.20)
                    ego_control.throttle = 0.0
                elif state == "AVOID" and lateral_separated and not front.is_front_vehicle:
                    ego_control.brake = min(ego_control.brake, 0.15)

                if state == "AVOID" and progress > trajectory.length + 2.0 and abs(lateral - trajectory.lateral_offset) < 0.65: # 如果当前轨迹点已经完成并且接近目标车道，则切换回路线跟踪状态
                    state = "ROUTE_FOLLOW"
                    avoidance_candidates = []
                    active_avoid_target_actor_id = None
                    post_avoid_lane_hold_until = sim_time + POST_AVOID_LANE_HOLD_SECONDS
                    print("Avoidance completed at {:.2f}s.".format(sim_time))

            elif state == "EMERGENCY_BRAKE":
                """在紧急制动状态下，直接应用全制动控制命令，油门为0，制动为1，方向盘不转动，最大限度地减小与前车的碰撞风险"""
                ego_control = carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0)

            elif state == "RIGHT_OBJECT_YIELD":
                if right_object.actor_id != last_right_object_actor_id:
                    print(
                        "Right yield target changed at {:.2f}s: {} -> {}, distance={:.1f}m, TTC={:.2f}s.".format(
                            sim_time,
                            last_right_object_actor_id if last_right_object_actor_id is not None else "none",
                            right_object_label(right_object),
                            right_object.distance if math.isfinite(right_object.distance) else 99.9,
                            right_object.ttc if math.isfinite(right_object.ttc) else 99.99,
                        )
                    )
                    last_right_object_actor_id = right_object.actor_id

                """在右侧物体避让状态下，降低目标速度以增加与右侧过街物体的安全距离；风险解除后回到路线跟踪，后续可再次触发避让"""
                if right_object_risk:
                    right_object_clear_since = None
                elif right_object_clear_since is None:
                    right_object_clear_since = sim_time

                target_speed = RIGHT_OBJECT_YIELD_SPEED # 在右侧物体避让状态下，降低目标速度以增加与右侧过街物体的安全距离，这里直接使用预设的 RIGHT_OBJECT_YIELD_SPEED 作为目标速度，供后续计算控制命令使用
                throttle, brake = speed_control(ego_speed, target_speed) # 使用 speed_control 函数计算所需的油门和制动值，基于当前自车速度和降低后的目标速度进行计算，供后续应用控制命令使用
                if not right_object_stop_active and right_object.distance < RIGHT_OBJECT_STOP_DISTANCE:
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
                    """如果右侧过街物体距离小于预设的停止距离，则认为需要紧急避让，增加制动值并将油门设置为0，以最大限度地减小与右侧过街物体的碰撞风险"""
                    throttle = 0.0
                    brake = max(brake, 0.85)
                ego_control = carla.VehicleControl( # 创建控制命令，基于计算得到的油门和制动值，以及航向控制值（这里直接使用当前路线点的航向进行控制），供后续应用控制命令使用
                    throttle=throttle,
                    brake=brake,
                    steer=loop_route.steer(ego_vehicle),
                )
                right_object_clear_confirmed = (
                    right_object_clear_since is not None
                    and sim_time - right_object_clear_since >= RIGHT_OBJECT_CLEAR_HOLD_SECONDS
                )
                if right_object_clear_confirmed:
                    """如果右侧物体风险已经解除，则切换回路线跟踪状态；后续再次达到风险条件时可重新进入避让"""
                    state = "ROUTE_FOLLOW"
                    right_object_stop_active = False
                    last_right_object_actor_id = None
                    right_object_clear_since = None
                    print("Right object yield completed at {:.2f}s.".format(sim_time))

            else: # 在 ROUTE_FOLLOW 状态下保持路线跟踪控制，如果需要制动则降低目标速度
                if brake_needed:
                    target_speed = min(EGO_TARGET_SPEED, max(0.0, ego_speed - 5.0))
                else:
                    target_speed = EGO_TARGET_SPEED
                throttle, brake = speed_control(ego_speed, target_speed)
                if post_avoid_lane_hold_until is not None and sim_time < post_avoid_lane_hold_until:
                    route_steer = clamp(waypoint_steer(ego_vehicle, carla_map), -0.25, 0.25)
                else:
                    post_avoid_lane_hold_until = None
                    route_steer = loop_route.steer(ego_vehicle)
                ego_control = carla.VehicleControl(
                    throttle=throttle,
                    brake=brake,
                    steer=route_steer,
                )

            ego_vehicle.apply_control(ego_control) # 应用控制命令，控制自车的油门、制动和转向，根据当前状态和感知信息计算得到的控制命令进行应用，实现跟车、避障、右侧物体避让等行为

            draw_trajectory_debug(world, loop_route, ego_vehicle, state, trajectory, avoidance_candidates, frame)

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
                    "front_actor_role": front.actor_role,
                    "front_risk_level": front.risk_level,
                    "right_object_distance": right_object.distance,
                    "right_object_ttc": right_object.ttc,
                    "right_object_type": right_object.object_type,
                    "right_risk_level": right_object.risk_level,
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
