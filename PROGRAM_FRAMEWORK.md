# CARLA 城市环形道路紧急避障程序框架

> 维护约定：后续如果需要更新选题说明、程序框架、模块职责、参数说明、运行方式或 PR/提交记录，只更新本文档，不再新建新的说明文件。

## 1. 最终选题

本项目最终选题确定为：

```text
城市交通流中自动驾驶车辆在环形道路区域内的紧急避障控制
```

核心目标不是单纯跟车，而是突出“避障”：

- 在城市道路交通流中运行。
- 选择一个环形或近似环形的道路区域作为测试路线。
- 控制自车沿路线安全行驶一圈。
- 在直线行驶时，前方车辆突然刹停，自车需要紧急制动并尽可能通过转向避障。
- 在右转弯时，右侧出现或行进非机动车辆，自车需要及时避让，可以通过制动、转向或二者结合完成。
- 整个过程中不发生碰撞，尽量不冲出车道，不出现明显失控。

当前程序入口文件：

```text
dazuoye/guiji.py
```

当前模块文件：

```text
dazuoye/config.py
dazuoye/utils.py
dazuoye/perception.py
dazuoye/control.py
dazuoye/route.py
dazuoye/actors.py
dazuoye/display.py
```

本文档是该程序后续唯一维护文档。

公式维护约定：

- 路径生成、TTC 风险计算、纯追踪/前视控制、五次多项式轨迹、MPC 跟踪和控制量映射的数学公式统一维护在本文档正文中。
- 后续如果修改 `route.py`、`perception.py`、`control.py`、`utils.py` 或 `guiji.py` 中的相关算法，必须同步更新本文档中的公式、参数含义和当前实现说明。
- 如果代码已修改但尚未完成验证，公式段可以先标注“待验证”，提交/PR 后再在 PR 提交记录中写明验证状态。

## 2. 场景设计

### 2.1 道路区域

目标道路区域：

```text
城市地图中的环形道路或近似闭环道路
```

选择原则：

- 道路具有明显城市交通属性。
- 存在直线段，便于布置“前车突然刹停”工况。
- 存在右转弯段，便于布置“右侧非机动车避让”工况。
- 路线能够形成一圈闭环或近似闭环，便于定义“安全行驶一圈”的任务目标。
- 尽量避免过多复杂路口干扰第一版验证。

当前代码已切换到 `Town10HD_Opt`，固定起点为 `TOWN10_START_SPAWN_INDEX = 141`。路线采用 Town10 内部短闭环，不再绕外侧大圈；直线段用于前车急停避障，后续十字路口右转段用于扩展右侧行人/非机动车避让工况。

### 2.2 交通流

场景中应包含：

- 自车，也就是被控制车辆。
- 前方机动车，用于触发直线段急停避障。
- 城市交通流车辆，用于提高场景真实感。
- 右侧非机动车辆，用于触发右转弯避让。

交通流第一版可以保持简单：

- 只生成少量背景车。
- 背景车不主动制造复杂冲突。
- 主要危险目标仍然是前车和右侧非机动车。

后续再扩展为更密集、更随机的交通流。

## 3. 两个核心避障工况

### 3.1 工况一：直线段前车突然刹停

触发位置：

```text
环形路线中的直线道路段
```

过程：

1. 自车沿直线段正常行驶。
2. 前方车辆在同车道内正常行驶。
3. 到达指定时间或指定位置后，前车突然紧急制动。
4. 自车检测到前车距离快速缩短、TTC 降低。
5. 自车先进行纵向制动。
6. 如果相邻车道或避让空间安全，则执行紧急转向避障。
7. 避障完成后恢复路线跟踪，继续沿环形道路行驶。

控制目标：

- 避免追尾前车。
- 制动过程尽量平稳但优先保证安全。
- 转向避障时不与旁车碰撞。
- 避障完成后能回到可继续行驶的道路路径。

### 3.2 工况二：右转弯时避让右侧非机动车

触发位置：

```text
环形路线中的右转弯道路段
```

非机动车目标可以是：

```text
自行车、摩托车、行人替代目标或 CARLA 中可用的两轮车 actor
```

过程：

1. 自车进入右转弯区域。
2. 右侧非机动车沿道路右侧、路口边缘或自车右前方区域运动。
3. 自车预测到右转轨迹与非机动车存在冲突。
4. 自车降低速度。
5. 如果空间允许，自车适当调整转向轨迹，避开非机动车。
6. 非机动车通过或风险解除后，自车继续右转并回到环形路线。

控制目标：

- 避免与右侧非机动车发生碰撞。
- 转弯时不压出道路边界。
- 不出现急剧方向盘振荡。
- 避让后继续完成一圈行驶任务。

## 4. 整体程序框架

新的程序框架按任务链路划分为 8 层：

```text
场景配置层
  ↓
环形路线规划层
  ↓
交通参与者生成层
  ↓
虚拟感知层
  ↓
风险评估层
  ↓
行为决策层
  ↓
轨迹生成与控制层
  ↓
可视化与评估层
```

当前程序已经具备的基础能力：

- CARLA 连接和地图加载。
- 自车与前车生成。
- 前车急停。
- 虚拟真值感知，并可选叠加噪声、FOV、漏检和前向毫米波雷达点云聚类。
- TTC 计算。
- 基于 `LoopRoute` 真实路线叠加避障起点局部右向五次横向增量的多候选避障轨迹。
- 采样式 MPC 跟踪，当前主流程使用路线相对轨迹代价计算。
- Town10 固定起点。
- Town10 固定短路线。
- 路线进度与一圈完成判断。
- 右转路线事件检测与右转前靠右准备。
- 5 辆沿固定路线行驶的背景车辆，速度使用固定随机种子生成并设置上限。
- `R344 -> R20` 右转处右侧关键非机动车直行目标。
- 3 辆 R344 右侧背景自行车，速度各异。
- 前方车辆和右侧非机动车虚拟感知支持关键目标与背景目标共同参与风险判断。
- `ROUTE_FOLLOW` 内联右侧目标减速/硬刹让行。
- 路线完成后的停车保持控制。
- pygame 摄像头显示。
- 碰撞监测。

### 4.1 当前模块职责

```text
config.py       场景、路线、风险阈值和控制参数
utils.py        通用数学、车辆速度、平滑参考线投影和道路辅助函数
perception.py   虚拟真值/雷达融合感知、基于平滑参考线的前车读取与右侧目标风险读取
control.py      平滑路线叠加局部五次偏移候选轨迹、路径约束/代价选择和采样式 MPC 跟踪
route.py        Town10 固定短路线、路线进度和转弯事件检测
actors.py       自车、前车、背景车辆和非机动车生成与运动
display.py      碰撞监测、CARLA 摄像头、pygame HUD 和显示窗口
guiji.py        CARLA 世界初始化、行为状态机、主循环和清理流程
```

模块依赖保持单向：配置与工具模块位于底层，业务模块依赖底层模块，`guiji.py` 只负责组装，避免模块之间循环导入。

后续代码需要重点补充：

- 背景交通流的视觉效果和密度继续调参。
- 右侧非机动车目标的参数微调。
- 右转弯区域识别与冲突判断的稳定性优化。
- 右转弯避让策略从第一版减速让行扩展为更完整的制动/转向组合。
- 紧急制动灯或车辆灯光状态控制。

## 5. 场景配置层

场景配置层负责统一管理地图、车辆、路线、风险阈值和控制参数。

当前已使用的集中参数包括：

```python
HOST = "localhost"
PORT = 2000
MAP_NAME = "Town10HD_Opt"
CLIENT_TIMEOUT = 120.0
FIXED_DELTA_SECONDS = 0.05
SIM_SECONDS = 90.0
TOWN10_START_SPAWN_INDEX = 141
TOWN10_ROUTE_STEP = 4.0
ROUTE_COMPLETION_HOLD_SECONDS = 4.0
RIGHT_OBJECT_TTC_THRESHOLD = 5.0
RIGHT_OBJECT_DETECT_DISTANCE = 34.0
RIGHT_OBJECT_STOP_DISTANCE = 13.0
RIGHT_OBJECT_STOP_RELEASE_DISTANCE = 14.5
RIGHT_OBJECT_YIELD_SPEED = 3.0
RIGHT_OBJECT_LONGITUDINAL_MIN = -30.0
RIGHT_OBJECT_LONGITUDINAL_MAX = 12.0
RIGHT_OBJECT_LATERAL_MIN = -3.0
RIGHT_OBJECT_LATERAL_MAX = 20.0
RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS = 6
RIGHT_OBJECT_R344_RIGHT_OFFSET = 2.4
RIGHT_OBJECT_R344_START_FORWARD_OFFSET = -3.5
RIGHT_OBJECT_R344_END_FORWARD_OFFSET = 42.0
TRAFFIC_RANDOM_SEED = 20260602
BACKGROUND_VEHICLE_ROUTE_INDICES = (32, 54, 76, 96, 108)
BACKGROUND_VEHICLE_SPEED_MIN = 7.0
BACKGROUND_VEHICLE_SPEED_MAX = 8.8
BACKGROUND_VEHICLE_EGO_CLEARANCE = 18.0
BACKGROUND_BICYCLE_FORWARD_OFFSETS = (-1.5,)
BACKGROUND_BICYCLE_RIGHT_OFFSETS = (4.6,)
BACKGROUND_BICYCLE_END_FORWARD_OFFSET = 48.0
BACKGROUND_BICYCLE_SPEED_MIN = 2.6
BACKGROUND_BICYCLE_SPEED_MAX = 4.3
DEBUG_DRAW_TRAJECTORY = True
DEBUG_DRAW_LOOKAHEAD_DISTANCE = 10.0
DEBUG_DRAW_TRAJECTORY_STEP = 2.0
DEBUG_DRAW_INTERVAL_FRAMES = 4
DEBUG_DRAW_LIFETIME = 0.25
```

后续加入场景开关时，可以再增加：

```python
ENABLE_TRAFFIC_FLOW = True
```

含义：

- `ENABLE_TRAFFIC_FLOW`：是否启用城市交通流。

安全阈值建议分为两类：

```python
FRONT_TTC_BRAKE_THRESHOLD
FRONT_TTC_AVOID_THRESHOLD
RIGHT_OBJECT_TTC_THRESHOLD
RIGHT_OBJECT_DISTANCE_THRESHOLD
```

其中：

- `FRONT_*` 用于前车急停。
- `RIGHT_OBJECT_*` 用于右转弯时右侧非机动车避让。

## 6. 环形路线规划层

环形路线规划层是新选题的关键新增内容。

目标：

- 在地图上选择一条闭环或近似闭环路线。
- 自车沿该路线行驶。
- 能判断车辆是否已经安全完成一圈。

建议设计：

```python
class RingRoute:
    def __init__(self, carla_map, start_waypoint):
        ...

    def build_route(self):
        ...

    def get_target_waypoint(self, ego_location):
        ...

    def progress(self, ego_location):
        ...

    def is_lap_completed(self):
        ...
```

主要职责：

- 保存路线 waypoint 序列。
- 提供当前最近路线点。
- 提供前视目标点。
- 记录自车沿路线的行驶进度。
- 判断是否完成一圈。

第一版实现可以不追求复杂全局规划，先手动选择一组关键 waypoint 或固定 spawn 点，并通过 `waypoint.next()` / `get_right_lane()` / `get_left_lane()` 生成路线。

当前代码实现采用 `LoopRoute` 预生成 Town10 固定短路线：

- 起点保持为 `TOWN10_START_SPAWN_INDEX = 141`。
- 路线步长为 `TOWN10_ROUTE_STEP = 4.0` 米。
- 通过 `TOWN10_SHORT_LOOP_BRANCH_OVERRIDES` 在 Town10 左侧路口选择中间连接路；第二个路口仍然转弯；在 `road13 lane -2` 选择 `road934 lane -2` 继续直行；随后 `road14 lane -2` 默认转弯；转弯后在 `road20 lane -2` 选择 `road875 lane -2` 继续直行，最后实际经过的路口按默认分支完成右转。
- 路线长度约 `520m`，不再绕最外侧大圈；到达路线终点后记录 `route_completion_time` 并直接施加停车控制，保持 `4s` 后结束仿真，避免继续追踪最后一个 waypoint 导致回摆。
- 通过 `TOWN10_RIGHT_TURN_PREPARE_LANE_CHANGES`、`TOWN10_RIGHT_TURN_PREPARE_MAX_X` 和航向阈值，在第一个大弯右转结束后的直道上进入同向右侧车道，运行输出中 `right_lane_before_turn=True`、`prepare_index=45`。
- 路线中的大弯右转仍保持原车道；后续十字路口右转前已在右侧车道，可作为右转避让行人/非机动车的重点工况位置。
- 进度跟踪采用“以上一次进度为锚点的前向窗口搜索”，并支持闭合到已走过路线上的点，避免路线末端靠近旧路段时误判。

### 6.1 当前路径生成与路线跟踪公式

当前 `route.py` 中 `LoopRoute` 的路线不是在线全局规划，而是从固定起点开始按固定步长向前展开 waypoint 序列。

设路线离散点为：

$$
P_i = (x_i,\ y_i,\ z_i)
$$

路线生成步长为：

$$
\Delta s = \texttt{TOWN10\_ROUTE\_STEP} = 4.0\ \mathrm{m}
$$

正常情况下，下一个路点来自 CARLA waypoint：

$$
W_{i+1} = W_i.\mathrm{next}(\Delta s)[0]
$$

$$
P_{i+1} = \mathrm{location}(W_{i+1})
$$

如果当前 `(road_id, lane_id)` 命中 `TOWN10_SHORT_LOOP_BRANCH_OVERRIDES`，则优先选择指定 `road_id` 的分支：

$$
W_{i+1} = \arg\min_{W \in \mathcal{C}_i}
\mathbf{1}\{\mathrm{road\_id}(W) \ne \mathrm{preferred\_road}\}
$$

其中 $\mathcal{C}_i$ 表示 `waypoint.next()` 返回的候选路点集合。实际代码中如果找到 `road_id == preferred_road` 的候选路点就直接选用，否则使用第一个候选路点。

路线长度按离散点数量估计：

$$
S_{\mathrm{route}} = (N - 1)\Delta s
$$

其中 `N` 为 `LoopRoute.points` 数量。

自车路线进度使用最近点索引估计。为避免路线闭合处误跳，搜索窗口以前一帧索引 `i_last` 为锚点：

$$
\mathcal{I}
= [i_{\mathrm{last}}-\mathrm{search\_back},\ i_{\mathrm{last}}+\mathrm{search\_ahead}]
$$

$$
i_{\mathrm{near}}
= \arg\min_{i \in \mathcal{I}} \left\| P_i - p_{\mathrm{ego}} \right\|
$$

当前累计进度为：

$$
i_{\max} \leftarrow \max(i_{\max},\ i_{\mathrm{near}})
$$

$$
S_{\mathrm{progress}}
= \min(S_{\mathrm{route}},\ i_{\max}\Delta s)
$$

纯追踪/前视转向当前用于正常路线跟踪。先根据前视距离 `L_d` 选择目标点：

$$
n_{\mathrm{lookahead}}
= \max\left(2,\ \left\lfloor\frac{L_d}{\Delta s}\right\rfloor\right)
$$

$$
i_{\mathrm{target}}
= \min(i_{\mathrm{near}} + n_{\mathrm{lookahead}},\ N-1)
$$

$$
P_{\mathrm{target}} = P_{i_{\mathrm{target}}}
$$

目标航向角：

$$
\psi_{\mathrm{target}}
= \operatorname{atan2}(y_{\mathrm{target}}-y_{\mathrm{ego}},\ x_{\mathrm{target}}-x_{\mathrm{ego}})
$$

航向误差归一化到 `(-pi, pi]`：

$$
e_{\psi}
= \operatorname{normalize\_angle}(\psi_{\mathrm{target}}-\psi_{\mathrm{ego}})
$$

归一化转向命令：

$$
\mathrm{steer}
= \operatorname{clamp}(1.8e_{\psi},\ -0.45,\ 0.45)
$$

该公式对应 `LoopRoute.steer()`；`utils.py` 中 `waypoint_steer()` 也使用同类前视航向误差控制，只是目标点来自 CARLA 当前车道的 `waypoint.next(lookahead)`。

## 7. 交通参与者生成层

该层负责生成和管理场景中的其他交通参与者。

建议拆分：

```python
spawn_ego_vehicle()
spawn_front_brake_vehicle()
spawn_background_traffic()
spawn_right_side_bicycle()
```

### 7.1 自车

推荐车型：

```text
vehicle.tesla.model3
```

原因：

- CARLA 示例中常用。
- 动力响应稳定。
- 适合演示自动驾驶控制。

### 7.2 前车

推荐车型：

```text
vehicle.lincoln.mkz_2020
```

作用：

- 在直线段前方行驶。
- 到达触发条件后急停。

### 7.3 背景交通流

背景车辆建议使用普通轿车、SUV、小型车。

当前第一版数量：

```text
5 辆
```

背景车辆不使用 Traffic Manager 自由自动驾驶，而是沿自车同一条 `LoopRoute` 做确定性路线进度推进。这样可以保证背景车路线与被控车一致，只通过初始位置和目标速度制造交通流差异。

当前 `BackgroundRouteVehicle.update()` 会在背景车从后方接近自车到 `BACKGROUND_VEHICLE_EGO_CLEARANCE = 18.0m` 内时暂停脚本进度，避免右转让行等长时间停车工况中背景车按固定路线硬追尾自车。

### 7.4 右侧非机动车

优先使用 CARLA 可用的两轮车蓝图，例如：

```text
vehicle.bh.crossbike
vehicle.diamondback.century
vehicle.gazelle.omafiets
```

如果对应蓝图不可用，可以使用摩托车或小型目标临时代替。

右侧非机动车的运动位置应布置在：

- 自车右转弯路径右侧。
- 自车右前方潜在冲突区。
- 或右转入口附近。

## 8. 虚拟感知层

当前代码已经有：

```python
VirtualGroundTruthSensor
FrontVehicleReading
RightSideObjectReading
```

当前虚拟感知已输出两类目标：

```python
FrontVehicleReading
RightSideObjectReading
```

当前感知层仍以虚拟真值和路线参考线为主演示底座，但已经合入感知增强开关：

- `SENSOR_NOISE_ENABLED`：对距离、接近速度和部分预测 TTC 叠加固定随机种子的高斯噪声。
- 前向/侧向 FOV：目标超出视场角或检测距离时不返回。
- 漏检模拟：远距离目标按固定概率漏检。
- `RADAR_ENABLED`：为 `True` 时在自车前部挂载 CARLA `sensor.other.radar`，通过 `set_radar_detections()` 输入点云，并用简单欧氏距离聚类生成前方候选目标。

当前默认 `RADAR_ENABLED = False`，原因是当前控制演示已经基于路线弧线虚拟感知完成验证；雷达聚类作为可启用增强能力保留，避免默认运行时突然改变前车识别稳定性。

### 8.1 前方车辆感知

继续保留：

- 前车距离。
- 相对速度。
- TTC。
- 横向偏移。
- 是否为本车道前方车辆。

当前 `guiji.py` 主流程统一调用 `sensor.front_vehicle()`。感知层维护两类路线数据：

- `_base_tracking_route`：由 `LoopRoute` 包装出的原始基础路线，只作为不可整体替换的基础参考。
- `_replacement_segments`：当前有效的局部替换段，每段只覆盖一个基础路线 $s$ 区间；新段与旧段重叠时，旧段会被移除后再合成，避免同一 $s$ 区间存在多个版本。
- `_tracking_route`：由基础路线与 replacement segments 合成出的当前真实跟踪路线，前车投影、TTC、最近前车判断、MPC 跟踪前视点都基于这条路线。

这样，如果触发避障的旧目标已经被 replacement segment 绕开，它相对当前合成路线的横向偏移会增大，不再反复触发同一碰撞走廊风险；如果避障偏移线前方又出现新的慢车/危险目标，仍会在当前合成路线坐标下被识别出来。避障段末尾不会立即拼回 $d=0$ 基础路线，而是继续沿 $C_{\mathrm{base}}(s)+d_{\mathrm{hold}}r(s)$ 行驶，直到回归段通过冲突检测。

当前前车判断使用的主动参考线可以写成：

$$
C_{\mathrm{active}}(s)=
\begin{cases}
C_{\mathrm{replace},k}(s), & s\in[s_{k,0},s_{k,1}] \\
C_{\mathrm{base}}(s)+d_{\mathrm{hold}}(s)r(s), & \text{otherwise}
\end{cases}
$$

其中 $C_{\mathrm{replace},k}(s)$ 来自 `RouteOffsetLaneChangeTrajectory.replacement_points()` 的采样结果；每个 replacement segment 还记录 `end_offset`，合成器在该段之后用这个偏移继续采样基础路线。普通避障段的 `end_offset = target_offset`，回归段的 `end_offset = 0`。

路线参考线模式可以理解为虚拟车道线/导航参考线传感器：目标位置和速度仍来自虚拟真值感知，但“前方”和“同车道”不再由自车当前直线 `forward/right` 一刀切判断，而是由前方道路局部弧线坐标判断。

路线参考线模式中的参考线由 `SmoothRouteReference` 表示。设平滑参考线为：

$$
C(s)=
\begin{bmatrix}
x(s)\\
y(s)
\end{bmatrix}
$$

对车辆位置 $p$，在当前进度附近的局部搜索窗口内求最近投影：

$$
s^*=\arg\min_s \|p-C(s)\|^2
$$

投影点和切向量为：

$$
C^*=C(s^*),\qquad
t(s^*)=
\frac{C'(s^*)}{\|C'(s^*)\|}
$$

道路右向单位向量为：

$$
r(s^*)=
\begin{bmatrix}
-t_y(s^*)\\
t_x(s^*)
\end{bmatrix}
$$

横向偏移为：

$$
d(p)=\operatorname{dot}_{2D}(p-C^*,\ r(s^*))
$$

自车和目标分别投影为 $(s_e,d_e)$、$(s_o,d_o)$，则前方目标沿参考线的弧长距离和横向差为：

$$
s_{\mathrm{front}} = s_o - s_e
$$

$$
d_{\mathrm{front}} = d_o - d_e
$$

当前只认为满足以下条件的目标是“同车道前方车辆”：

$$
s_{\mathrm{front}} > 0
$$

$$
|d_{\mathrm{front}}| < 0.45w_{\mathrm{lane}}
$$

自车和目标车速度投影到各自参考线切线方向：

$$
v_{e,\mathrm{route}} = \operatorname{dot}_{2D}(v_e,\ t(s_e))
$$

$$
v_{o,\mathrm{route}} = \operatorname{dot}_{2D}(v_o,\ t(s_o))
$$

`FrontVehicleReading.target_speed_along` 保存的就是 $v_{o,\mathrm{route}}$。后续避障规划会用它估计慢前车在换道时间内继续前进的距离；急刹前车 `actor_role == "lead"` 时按静止障碍处理，避免把已经急停的目标错误外推。

接近速度：

$$
v_{\mathrm{close}}
= v_{e,\mathrm{route}} - v_{o,\mathrm{route}}
$$

TTC 计算：

$$
\mathrm{TTC}_{\mathrm{front}} =
\begin{cases}
\dfrac{s_{\mathrm{front}}}{v_{\mathrm{close}}}, & v_{\mathrm{close}} > 0.1 \\
\infty, & v_{\mathrm{close}} \le 0.1
\end{cases}
$$

这样在弯道上，即使两车几何连线与自车当前朝向不重合，只要目标沿道路局部弧线位于自车前方、横向偏移仍属于同一车道，就可以被提前识别为前方目标，避免等到距离很近才触发避障。

如果没有可用的 `TrackingRoute`，前车感知才使用自车坐标系兜底。设：

$$
p_e = \text{ego location},\quad
p_o = \text{obstacle/target vehicle location}
$$

$$
f_e = \text{ego forward unit vector},\quad
r_e = \text{ego right unit vector}
$$

$$
v_e = \text{ego velocity},\quad
v_o = \text{target vehicle velocity}
$$

相对位置：

$$
\Delta p = p_o - p_e
$$

纵向距离和横向偏移：

$$
s = \operatorname{dot}_{2D}(\Delta p,\ f_e)
$$

$$
d = \operatorname{dot}_{2D}(\Delta p,\ r_e)
$$

自车坐标系兜底模式下，只认为满足以下条件的目标是“同车道前方车辆”：

$$
s > 0
$$

$$
|d| < 0.65w_{\mathrm{lane}}
$$

自车和目标车沿自车前向的速度分量：

$$
v_{e,\mathrm{forward}} = \operatorname{dot}_{2D}(v_e,\ f_e)
$$

$$
v_{o,\mathrm{forward}} = \operatorname{dot}_{2D}(v_o,\ f_e)
$$

接近速度：

$$
v_{\mathrm{close}}
= v_{e,\mathrm{forward}} - v_{o,\mathrm{forward}}
$$

TTC 计算：

$$
\mathrm{TTC}_{\mathrm{front}} =
\begin{cases}
\dfrac{s}{v_{\mathrm{close}}}, & v_{\mathrm{close}} > 0.1 \\
\infty, & v_{\mathrm{close}} \le 0.1
\end{cases}
$$

如果多个前方车辆满足条件，当前读取距离最近的车辆：

$$
\mathrm{front}
= \arg\min_{o \in \mathcal{O}_{\mathrm{front}}} s_o
$$

当前 `FrontVehicleReading` 已保留 `actor_id`、`actor_role`、`target_speed_along`、`lane_relative_lateral`、`is_same_lane` 和 `risk_level`。其中 `target_speed_along` 已用于慢车预测，`front_vehicles()` 保留为雷达/多目标候选读取入口；当前主状态机仍主要使用最近同车道前车的路线弧长距离、横向偏移、接近速度和 TTC。

### 8.2 右侧非机动车感知

当前第一版字段：

```python
@dataclass
class RightSideObjectReading:
    distance: float
    ttc: float
    is_conflict_object: bool
    actor_id: int
    actor_role: str
    longitudinal: float
    lateral: float
    risk_level: int
    is_moving_toward_conflict: bool
    predicted_ttc: float
    object_type: str
```

重点不只是判断“在不在右侧”，还要判断：

- 是否位于右转轨迹附近。
- 是否将与自车未来轨迹产生冲突。
- 是否处于自车右前方危险区域。

当前第一版使用几何规则：

```text
目标在自车右侧
目标距离小于阈值
目标位置接近右转目标路径
```

当前已经加入连续帧确认与风险等级：

$$
\mathrm{right\_confirm\_count}
\ge
\mathrm{RIGHT\_CONFIRM\_FRAMES}
$$

满足连续确认后才把候选目标标记为真正冲突目标，降低单帧误触发。右侧目标风险等级当前为：

```text
0 = 无风险
1 = 注意
2 = 警告/需要停车让行
3 = 危险
```

主状态机仍保留原有 `TTC/距离` 触发条件，同时允许 `risk_level >= 2` 触发右侧让行。

当前右侧目标 TTC 使用目标相对自车的径向接近速度。设：

$$
p_e = \text{ego location},\quad
p_o = \text{right-side object location}
$$

$$
v_e = \text{ego velocity},\quad
v_o = \text{right-side object velocity}
$$

$$
\Delta p = p_o - p_e
$$

欧氏距离：

$$
\mathrm{dist} = \left\|\Delta p\right\|
$$

从自车指向目标的单位向量：

$$
u = \frac{\Delta p}{\max(\mathrm{dist},\ 0.1)}
$$

相对速度：

$$
\Delta v = v_e - v_o
$$

接近速度：

$$
v_{\mathrm{close,right}}
= \operatorname{dot}_{2D}(\Delta v,\ u)
$$

右侧目标 TTC：

$$
\mathrm{TTC}_{\mathrm{right}} =
\begin{cases}
\dfrac{\mathrm{dist}}{v_{\mathrm{close,right}}}, & v_{\mathrm{close,right}} > 0.1 \\
\infty, & v_{\mathrm{close,right}} \le 0.1
\end{cases}
$$

当前右侧冲突目标还必须同时满足：

$$
\mathrm{is\_active} = \mathrm{True}
$$

$$
\mathrm{is\_conflict\_window}(\mathrm{route\_index}) = \mathrm{True}
$$

$$
-30.0 \le s_{\mathrm{right}} \le 12.0
$$

$$
-3.0 \le d_{\mathrm{right}} \le 20.0
$$

其中：

$$
s_{\mathrm{right}} = \operatorname{dot}_{2D}(\Delta p,\ f_e)
$$

$$
d_{\mathrm{right}} = \operatorname{dot}_{2D}(\Delta p,\ r_e)
$$

如果存在多个右侧目标，当前优先返回处于冲突窗口内且 TTC/距离更危险的目标；否则返回最近的非冲突目标用于显示。

## 9. 风险评估层

风险评估层负责把感知信息转换成风险等级。

建议定义风险类型：

```text
NO_RISK
FRONT_BRAKE_RISK
FRONT_COLLISION_RISK
RIGHT_SIDE_CONFLICT_RISK
```

### 9.1 前车急停风险

判断依据：

```text
前车是否在本车道前方
前车距离
自车与前车相对速度
TTC
相邻车道是否可用
```

当前 `guiji.py` 中前车风险实际使用两个布尔量：

$$
\mathrm{brake\_needed}
= \mathrm{front.is\_front\_vehicle}
\land
\left(\mathrm{front.ttc} < \mathrm{TTC\_BRAKE\_THRESHOLD}\right)
$$

为了避免弯道前车被提前感知后只持续跟车制动、反而错过可用避障距离，当前还定义了“近距离慢前车”触发项：

$$
\mathrm{close\_slow\_front\_vehicle}
=
\mathrm{front.is\_front\_vehicle}
\land
\left(\mathrm{front.distance} < \mathrm{LANE\_CHANGE\_LENGTH} + 6.0\right)
\land
\left(\mathrm{front.closing\_speed} > 2.0\right)
$$

$$
\mathrm{front\_planning\_needed}
= \mathrm{front.is\_front\_vehicle}
\land
\left(\mathrm{front.distance} < \mathrm{SAFE\_DISTANCE}\right)
\land
\left[
\left(\mathrm{front.ttc} < \mathrm{TTC\_AVOID\_THRESHOLD}\right)
\lor
\mathrm{close\_slow\_front\_vehicle}
\right]
$$

$$
\mathrm{front\_emergency\_brake\_needed}
=
\left(\mathrm{front.distance}<\mathrm{EMERGENCY\_BRAKE\_DISTANCE}\right)
\lor
\left(\mathrm{front.ttc}<\mathrm{EMERGENCY\_BRAKE\_TTC\_SECONDS}\right)
$$

`EMERGENCY_BRAKE` 恢复判定当前使用更宽的滞回阈值，避免紧急制动状态成为永久状态：

$$
\mathrm{emergency\_recovered}
= \neg\mathrm{front.is\_front\_vehicle}
\lor
\left(\mathrm{front.distance} > \mathrm{SAFE\_DISTANCE} + 8.0\right)
\lor
\left(\mathrm{front.ttc} > \mathrm{TTC\_BRAKE\_THRESHOLD} + 1.0\right)
$$

其中当前参数为：

```text
TTC_BRAKE_THRESHOLD = 4.5 s
TTC_AVOID_THRESHOLD = 3.6 s
SAFE_DISTANCE = 34.0 m
LANE_CHANGE_LENGTH = 28.0 m
EMERGENCY_BRAKE_DISTANCE = 8.0 m
EMERGENCY_BRAKE_TTC_SECONDS = 1.8 s
```

如果 `front_planning_needed(front)` 成立，程序直接生成左右多组 replacement 候选，并用候选轨迹与所有检测车辆的冲突判断筛掉危险路径；不再把 `lane_clear()` 作为能否规划的硬条件。若规划失败但 `front_emergency_brake_needed(front)` 仍为 `False`，车辆继续跟踪当前合成路线并在下一帧重试。

### 9.2 右侧非机动车风险

判断依据：

```text
自车是否处于右转弯阶段
非机动车是否在右侧危险区域
非机动车与自车未来路径是否冲突
距离是否低于阈值
TTC 是否低于阈值
```

当前 `guiji.py` 中右侧目标风险为：

$$
\mathrm{right\_object\_risk}
= \mathrm{right\_object.is\_conflict\_object}
\land
\left[
\left(\mathrm{right\_object.ttc} < \mathrm{RIGHT\_OBJECT\_TTC\_THRESHOLD}\right)
\lor
\left(\mathrm{right\_object.distance} < \mathrm{RIGHT\_OBJECT\_DETECT\_DISTANCE}\right)
\right]
$$

当前参数：

```text
RIGHT_OBJECT_TTC_THRESHOLD = 5.0 s
RIGHT_OBJECT_DETECT_DISTANCE = 34.0 m
RIGHT_OBJECT_STOP_DISTANCE = 13.0 m
RIGHT_OBJECT_STOP_RELEASE_DISTANCE = 14.5 m
RIGHT_OBJECT_YIELD_SPEED = 3.0 m/s
```

在 `ROUTE_FOLLOW` 中，如果 `right_object_risk` 成立或仍处于右侧目标清空确认期，自车目标速度降为 `RIGHT_OBJECT_YIELD_SPEED`。如果右侧目标距离进一步小于 `RIGHT_OBJECT_STOP_DISTANCE`，则激活硬刹停标志 `right_object_stop_active`，制动至少提升到：

$$
\mathrm{brake} = \max(\mathrm{brake},\ 0.85)
$$

$$
\mathrm{throttle} = 0.0
$$

硬刹停释放使用距离滞回，只有当当前右侧目标距离大于 `RIGHT_OBJECT_STOP_RELEASE_DISTANCE` 后才关闭 `right_object_stop_active`，避免目标距离在 13m 附近波动时反复刹停：

$$
\begin{cases}
\mathrm{stop\_active} \leftarrow \mathrm{True}, & d_{\mathrm{right}} < 13.0 \\
\mathrm{stop\_active} \leftarrow \mathrm{False}, & d_{\mathrm{right}} > 14.5 \\
\mathrm{stop\_active}\ \mathrm{保持不变}, & \text{otherwise}
\end{cases}
$$

此外，右侧让行不再在单帧 `right_object_risk == False` 时立即解除。当前代码使用 `RIGHT_OBJECT_CLEAR_HOLD_SECONDS = 2.0s` 做连续清空确认：

$$
\mathrm{right\_clear\_confirmed}
=
\left(t_{\mathrm{sim}} - t_{\mathrm{right\_clear\_since}}\right)
\ge 2.0
$$

在清空确认期间仍保持停车制动，避免右侧目标在行人/自行车之间切换、短暂漏检或返回 `none` 时自车过早起步。

右侧目标读取现在会携带 `actor_id`、`actor_role`、相对纵向距离和相对横向距离。运行日志会输出右侧让行开始、目标切换、硬刹激活和硬刹释放，便于判断“再次刹车”来自同一目标持续让行，还是新的右侧目标接管。

风险评估层应输出给行为决策层：

```python
risk_type
risk_level
recommended_action
```

## 10. 行为决策层

行为决策层负责根据风险选择驾驶行为。

当前代码中的 `state` 变量只区分前向行驶与紧急制动两种状态：

```text
ROUTE_FOLLOW
EMERGENCY_BRAKE
```

### 10.0 当前代码实际状态切换条件

当前 `guiji.py` 中状态变量初始化为：

$$
\mathrm{state}_0 = \mathrm{ROUTE\_FOLLOW}
$$

前方风险由 `front_planning_needed(front)` 和 `front_emergency_brake_needed(front)` 分级判断。规划触发仍使用 `SAFE_DISTANCE`、`TTC_AVOID_THRESHOLD` 和近距离慢车接近条件；进入 `EMERGENCY_BRAKE` 只看更近的硬阈值 `EMERGENCY_BRAKE_DISTANCE = 8.0m` 或 `EMERGENCY_BRAKE_TTC_SECONDS = 1.8s`，避免远距离潜在风险直接全制动。

当前状态切换条件如下。

| 状态 | 进入条件 | 退出条件 |
| --- | --- | --- |
| `ROUTE_FOLLOW` | 初始状态；`EMERGENCY_BRAKE` 风险解除后回到该状态；`EMERGENCY_BRAKE` 中重新规划到有效 replacement segment 后也回到该状态。 | 若 `front_planning_needed(front)` 成立，调用 `select_best_route_offset_trajectory()` 同时生成左右偏移候选，并用候选路径与所有车辆的冲突检测筛选；若存在有效候选，调用 `sensor.apply_replacement_segment()` 替换对应 $s$ 区间，状态仍保持 `ROUTE_FOLLOW`，段尾继续保持 `target_offset`；若自车车尾超过当前避让目标车头并满足安全余量，调用 `select_return_to_base_trajectory()` 生成 `current_offset -> 0` 回归候选，安全后才写入回归段；若回归不安全，继续保持当前 offset 并下一帧重试。若规划失败但风险尚未达到紧急阈值，保持当前合成路线并下一帧重试；若规划失败且 `front_emergency_brake_needed(front)` 成立，进入 `EMERGENCY_BRAKE`。右侧非机动车/行人风险只在该状态下限制目标速度或触发硬刹停，不再切换独立状态。路线完成后只通过 `route_completion_time` 施加停车控制，不再写入独立完成状态。 |
| `EMERGENCY_BRAKE` | `ROUTE_FOLLOW` 中需要规划但没有无冲突 replacement segment，且前车距离小于 `EMERGENCY_BRAKE_DISTANCE` 或 TTC 小于 `EMERGENCY_BRAKE_TTC_SECONDS`。 | 若 `emergency_recovered` 成立，回到 `ROUTE_FOLLOW`；若风险仍在但重新规划到有效 replacement segment，应用替换段后回到 `ROUTE_FOLLOW`；否则继续保持全制动。路线完成、碰撞、窗口关闭或仿真时间结束仍会提前终止主循环。 |



当前还有三个非状态机的提前结束条件：
$$
\mathrm{collision\_count} > 0
$$

$$
t_{\mathrm{sim}} \ge \mathrm{SIM\_SECONDS}
$$

$$
\text{pygame 窗口被用户关闭}
$$

后续如果继续细化右转避障，可以在 `ROUTE_FOLLOW` 内部再拆出更细的子阶段；这些名称只是设计备忘，不属于当前 `state` 变量：

```text
front_planning
right_turn_approach
right_object_yielding
right_object_avoidance
recover_to_base_route
route_finished_hold
```

### 10.1 ROUTE_FOLLOW

默认巡航状态，也是避障规划成功后的持续状态。

动作：

- 跟踪当前合成 `TrackingRoute`。
- 持续检测当前合成路线上的前方车辆和右侧非机动车/行人。
- 前方风险达到规划阈值时生成左右多组 replacement segment 候选，筛掉与所有检测车辆冲突的候选。
- 规划成功时仅替换对应 $s$ 区间，状态仍保持 `ROUTE_FOLLOW`。
- 右侧目标风险成立时降低目标速度，必要时激活 `right_object_stop_active` 硬刹停。

### 10.2 EMERGENCY_BRAKE

前方风险已经很近或 TTC 极小，且当前没有无冲突 replacement segment 时进入。

动作：

- 油门为 0。
- 制动为 1。
- 风险恢复或重新规划出有效 replacement segment 后回到 `ROUTE_FOLLOW`。

### 10.3 路线完成保持

完成一圈后不再写入独立状态，而是记录 `route_completion_time` 并直接施加停车控制。

动作：

- 油门为 0。
- 制动为 1。
- 保持 `ROUTE_COMPLETION_HOLD_SECONDS` 后输出结果并清理 actor 和传感器。

## 11. 轨迹生成与控制层

当前已有：

```python
RouteOffsetLaneChangeTrajectory
SamplingMPCTracker
```

后续应扩展为三类轨迹：

```text
正常路线跟踪轨迹
直线段紧急避障轨迹
右转弯避让轨迹
```

### 11.1 正常路线跟踪

用于沿环形道路行驶。

当前已使用 waypoint 前视控制，公式见 `6.1 当前路径生成与路线跟踪公式`。

后续可统一交给 MPC 跟踪。

### 11.2 前车急停避障轨迹

当前主流程在 `ROUTE_FOLLOW` 中触发前车规划时使用 `RouteOffsetLaneChangeTrajectory`。它生成的是基础路线上的局部 replacement segment，而不是独立状态轨迹：先把 `LoopRoute` 的真实路线点按累计弧长拟合成平滑参考线 $C(s)$，再沿该参考线的连续法向量叠加五次多项式横向避障增量。

注意：当前候选横向偏移不再依赖 `lane_clear()` 或单一目标邻道中心，而是围绕当前基础路线左右两侧生成多个目标偏移；每条候选再用路径与所有车辆的冲突检测筛掉明显危险段。

五次横向偏移采用“从当前偏移过渡到目标偏移，并在段尾保持目标偏移”的形式：

$$
d_{\mathrm{avoid}}(s)
=
\begin{cases}
d_0, & s \le 0 \\
\operatorname{blend}(d_0,d_t,t), & 0 < s < L \\
d_t, & s \ge L
\end{cases}
$$

$$
\operatorname{blend}(a,b,t)=a+(b-a)(10t^3-15t^4+6t^5)
$$

$$
t=\frac{s}{L}
$$

其中 $d_0$ 为规划开始时车辆相对基础路线的横向偏移，$d_t$ 为候选目标偏移。普通避障段成功后 `end_offset = d_t`，后续基础路线继续叠加该偏移；当自车车尾超过被避让车辆车头并满足安全余量后，才生成 `d_0 -> 0` 的回归候选，安全后写入 `end_offset = 0`。

设 `LoopRoute` 的离散路线点为：

$$
P_i=(x_i,\ y_i,\ z_i)
$$

先计算累计弧长：

$$
s_0=0,\qquad s_i=s_{i-1}+\|P_i-P_{i-1}\|
$$

然后使用三次样条分别拟合：

$$
x_{\mathrm{route}}=f_x(s),\qquad
y_{\mathrm{route}}=f_y(s),\qquad
z_{\mathrm{route}}=f_z(s)
$$

因此平滑道路参考线为：

$$
C(s)=
\begin{bmatrix}
f_x(s)\\
f_y(s)\\
f_z(s)
\end{bmatrix}
$$

参考线切向单位向量由样条导数得到：

$$
t(s)
=
\frac{
\begin{bmatrix}
f'_x(s)\\
f'_y(s)
\end{bmatrix}
}{
\sqrt{f'_x(s)^2+f'_y(s)^2}
}
$$

对应右向法向量为：

$$
r(s)=
\begin{bmatrix}
-t_y(s)\\
t_x(s)
\end{bmatrix}
$$

若当前运行环境没有 `scipy` 或 `CubicSpline` 导入失败，代码会回退到旧的线性插值和有限差分切向量，避免程序直接中断。

规划开始时车辆当前横向偏移为：

$$
d_0 = \operatorname{dot}_{2D}
\left(
p_{\mathrm{ego}}-C(s_{\mathrm{start}}),\
r(s_{\mathrm{start}})
\right)
$$

候选目标偏移由候选集合 $\mathcal{D}$ 给出，记为 $d_t$；普通避障段尾保持 $d_t$：

$$
d_{\mathrm{end}}=d_t
$$

五次曲线的避障横向偏移与代码 `avoidance_delta_at(s)` 一致，写成双段形式为：

$$
d_{\mathrm{avoid}}(s)
=
\begin{cases}
d_0, & s \le 0 \\
\operatorname{blend}(d_0,d_t,t), & 0 < s < L \\
d_t, & s \ge L
\end{cases}
$$

最终避障参考点为：

$$
P_{\mathrm{ref}}(s)
= C(s_{\mathrm{start}}+s)
+ d_{\mathrm{avoid}}(s) r(s_{\mathrm{start}}+s)
$$

其中：

- $s_{\mathrm{start}}=\texttt{loop\_route.last\_index}\cdot\texttt{loop\_route.step\_distance}$。
- $C(s)$ 为 `SmoothRouteReference` 通过累计弧长和 `CubicSpline` 得到的平滑路线参考线。
- $r(s)$ 为平滑参考线导数得到的道路右向单位向量。
- `d_0` 为规划开始时自车相对基础路线的道路右向距离。
- `d_t` 为候选目标横向偏移；普通避障段末端保持 `d_t`，回归段的目标偏移为 `0.0`。
- `L` 为换道纵向长度，当前由候选轨迹选择器在一组长度中筛选得到。

当前 `guiji.py` 不再直接把基础长度和目标邻道中心固定成唯一轨迹，而是调用 `control.py` 中的 `select_best_route_offset_trajectory()` 生成候选集合。横向五次函数本身不引入时间变量，目标车速度主要用于拉伸候选轨迹的纵向尺度和安全约束。

对于急刹前车，若 `actor_role == "lead"`，目标速度按 $0$ 处理：

$$
v_{o,\mathrm{plan}} = 0
$$

对于普通慢速前车，取路线方向目标速度：

$$
v_{o,\mathrm{plan}} = \max(0,\ v_{o,\mathrm{route}})
$$

先用当前前车距离和接近速度估计一个有限预测时间：

$$
t_{\mathrm{pred}}
=
\min\left(
3.0,\
\max\left(1.0,\ \frac{d_{\mathrm{front}}}{\max(v_{\mathrm{close}},0.1)}\right)
\right)
$$

目标车预测前进距离为：

$$
\Delta s_o = v_{o,\mathrm{plan}} t_{\mathrm{pred}}
$$

基础长度为：

$$
L_0 =
\max\left(
14.0,\
\min\left(52.0,\ \texttt{LANE\_CHANGE\_LENGTH}+\Delta s_o\right)
\right)
$$

候选纵向长度集合为：

$$
\mathcal{L}
=
\left\{
\operatorname{clamp}(\lambda L_0,\ 14.0,\ 56.0)
\mid
\lambda \in \{0.85,\ 1.00,\ 1.15,\ 1.30\}
\right\}
$$

当前横向候选不再只围绕一个目标邻道中心，而是围绕当前基础路线左右两侧生成多个目标偏移，包含保持/小偏移候选：

$$
\mathcal{D}
=
\{d_0+\alpha W_{\mathrm{lane}}\mid
\alpha\in[-1.20,-0.90,-0.60,-0.35,-0.20,0,0.20,0.35,0.60,0.90,1.20]\}
$$

对每一组 $(L,d_t)$，都构造一条 `RouteOffsetLaneChangeTrajectory`，段尾保持目标偏移。因此当前避障入口处理的是候选 replacement segment 集合：

$$
\mathcal{T}
=
\left\{
T(L,d_t)
\mid
L \in \mathcal{L},\ d_t \in \mathcal{D}
\right\}
$$

路径约束包括：

$$
L \ge 14.0
$$

$$
a_{y,\max}
=
\frac{10\sqrt{3}\left|d_t-d_0\right|}{3t_e^2}
\le 3.8\ \mathrm{m/s^2}
$$

其中：

$$
t_e = \frac{L}{\max(v_{\mathrm{ego}}, 4.0)}
$$

若前方车辆距离有限，还会在每条候选轨迹的预计换道时间 $t_e$ 内估计目标车继续前进的距离：

$$
d_{\mathrm{front,pred}}
=
d_{\mathrm{front}}
+ v_{o,\mathrm{plan}}t_e
$$

候选还会对 `obstacle_actors` 中所有车辆做简化时空冲突硬筛选。对每个候选采样点 $s_j$，估计车辆沿基础路线方向的预测弧长 $s_o(t_j)$，并根据自车/目标车包围盒半长半宽构造纵向和横向安全包络。若同时满足：

$$
\left|s_o(t_j)-s_j\right| \le l_{\mathrm{ego}}+l_o+\Delta_l
$$

$$
\left|d_o-d_{\mathrm{avoid}}(s_j)\right| \le w_{\mathrm{ego}}+w_o+\Delta_w
$$

则该候选被标记为 `candidate conflicts with vehicle` 或 `candidate conflicts with front vehicle` 并直接剔除。当前采样步长为 `1.0m`；触发本次规划的前车使用更大的安全余量。候选长度相对前车可用距离不再作为硬拒绝条件，而是进入安全代价。候选路径代价由安全性、舒适性、跟踪难度和目标偏移代价组成：

$$
J = 4.0J_s + 2.0J_c + J_t + 0.6J_d
$$

安全性代价：

$$
J_s
=
\frac{\max(0,\ 0.75L-d_{\mathrm{front,pred}}+4.0)^2}{25.0}
+
\max(0,\ t_e-\mathrm{TTC}_{\mathrm{front}}+0.4)^2
$$

舒适性代价：

$$
J_c
=
\left(\frac{a_{y,\max}}{3.8}\right)^2
+ 0.20\frac{\left|d_t-d_0\right|}{W_{\mathrm{lane}}}
$$

基础路线偏移代价：

$$
J_d
=
\frac{\left|d_t\right|}{W_{\mathrm{lane}}}
$$

跟踪难度代价当前用参考航向累计变化和最大横向斜率近似：

$$
J_t
=
0.35\sum_k
\left|
\operatorname{normalize\_angle}
\left(\psi_{\mathrm{ref},k+1}-\psi_{\mathrm{ref},k}\right)
\right|
+ 0.30 \max_k
\left|\frac{d d_{\mathrm{avoid}}}{ds}(s_k)\right|
$$

最终选择：

$$
T^*
=
\arg\min_{T_i\in\mathcal{T}_{\mathrm{valid}}} J_i
$$

如果 $\mathcal{T}_{\mathrm{valid}}$ 为空，`ROUTE_FOLLOW` 不会切入额外避障状态，而是保持当前合成路线并在下一帧重试；只有当前风险已经满足 `front_emergency_brake_needed(front)` 的近距离/TTC 阈值时，才进入 `EMERGENCY_BRAKE`。

普通避障段被采用后，主循环记录 `active_avoidance_target`。当目标仍存活时，回归触发采用“自车车尾超过障碍物车头”的几何条件：

$$
s_{\mathrm{ego}}-l_{\mathrm{ego}}
>
s_o+l_o+\Delta_{\mathrm{return}}
$$

其中 $\Delta_{\mathrm{return}}=2.0\mathrm{m}$。触发后只生成 `current_offset -> 0` 的回归候选，并复用同一套所有车辆冲突检测；安全才写入 replacement segment。若不安全，当前合成路线继续保持 `target_offset`。

`RouteOffsetLaneChangeTrajectory.to_local()` 使用 `SmoothRouteReference.project()` 在当前避障起点前后搜索平滑参考线上的最近弧长位置 $s^*$，再用该点的右向法向量计算车辆相对真实路线的横向避障量：

$$
d_{\mathrm{local}}
= \operatorname{dot}_{2D}
\left(
p-C(s^*),\
r(s^*)
\right)
$$

五次平滑函数：

$$
b(t) = 10t^3 - 15t^4 + 6t^5
$$

$$
d_{\mathrm{avoid}}(s)
=
\begin{cases}
d_0, & s \le 0 \\
\operatorname{blend}(d_0,d_t,t), & 0 < s < L \\
d_t, & s \ge L
\end{cases}
$$

$$
t=\frac{s}{L}
$$

边界条件：

$$
b(0)=0,\quad b(1)=1
$$

$$
b'(0)=0,\quad b'(1)=0
$$

$$
b''(0)=0,\quad b''(1)=0
$$

因此换道起点和终点的横向速度、横向加速度都为 0，轨迹相对平滑。

路线相对轨迹的五次横向偏移斜率为：

$$
b'(t) = 30t^2 - 60t^3 + 30t^4
$$

$$
\frac{d d_{\mathrm{avoid}}}{ds}
=
\begin{cases}
0, & s \le 0 \\
\frac{(d_t-d_0)b'(t)}{L}, & 0 < s < L \\
0, & s \ge L
\end{cases}
$$

$$
\psi_{\mathrm{ref}}
= \psi_{\mathrm{route}}(s_{\mathrm{start}}+s)
+ \arctan\left(\frac{d d_{\mathrm{avoid}}}{ds}\right)
$$

其中：

$$
\psi_{\mathrm{route}}(s)=\operatorname{atan2}(f'_y(s), f'_x(s))
$$

当前路线相对轨迹的实际代码使用平滑参考点叠加横向偏移后的最终轨迹有限差分计算参考航向，使参考航向与实际跟踪轨迹一致：

$$
\psi_{\mathrm{ref}}(s)
= \operatorname{atan2}
\left(
y_{\mathrm{ref}}(s+\Delta s_d)-y_{\mathrm{ref}}(\max(0,s-\Delta s_d)),
x_{\mathrm{ref}}(s+\Delta s_d)-x_{\mathrm{ref}}(\max(0,s-\Delta s_d))
\right)
$$

$$
\Delta s_d = \max(0.5,\ 0.25\Delta s)
$$

旧的 `QuinticLaneChangeTrajectory` 直线局部轨迹类已从 `control.py` 删除；当前 `guiji.py` 在 `ROUTE_FOLLOW` 和 `EMERGENCY_BRAKE` 重试规划中都生成 `RouteOffsetLaneChangeTrajectory` replacement segment。后续如果需要旧直线局部轨迹，可从 Git 历史恢复。

### 11.3 弯道避障轨迹

弯道避障不再使用触发瞬间固定直角坐标系生成整段轨迹。当前采用 `RouteOffsetLaneChangeTrajectory`：

```text
SmoothRouteReference 平滑路线 + 连续道路右方向五次横向增量
```

这样做的目的：

- 保留道路本身的转弯方向。
- 候选偏移左右同时生成，不再先用邻道净空函数决定方向。
- 弯道第二次避障与直道第一次避障共用同一套 replacement segment 规划与 MPC 跟踪接口。
- 避障段结束后合成路线继续保持目标偏移；只有回归候选安全时才回到基础路线，状态仍保持 `ROUTE_FOLLOW`。

当前右侧非机动车/行人冲突仍主要通过 `ROUTE_FOLLOW` 内的减速或停车让行处理，尚未单独生成右转转向避让轨迹。

### 11.4 MPC 跟踪公式

当前 `SamplingMPCTracker` 是采样式滚动优化控制器，不求解连续优化问题，而是枚举有限个转向和加速度候选，在预测时域内选择总代价最低的动作。

当前候选转向：

$$
\delta \in \delta_{\mathrm{prev}}
+ \{-0.45,\ -0.32,\ -0.20,\ -0.10,\ 0,\ 0.10,\ 0.20,\ 0.32,\ 0.45\}
$$

$$
\delta = \operatorname{clamp}(\delta,\ -0.65,\ 0.65)
$$

当前候选加速度：

$$
a \in \{-4.0,\ -2.0,\ -1.0,\ 0.0,\ 1.0\}\ \mathrm{m/s^2}
$$

预测步长和步数：

$$
\Delta t = \texttt{MPC\_DT} = 0.10\ \mathrm{s}
$$

$$
N = \texttt{MPC\_HORIZON\_STEPS} = 18
$$

采样式 MPC 使用简化运动学自行车模型。历史上的非路线相对局部轨迹分支已从 `SamplingMPCTracker.control()` 中删除；当前控制器只接受带 `is_route_relative=True` 的路线相对对象，主流程传入的是基础路线与 replacement segments 合成后的 `TrackingRoute`。如果误传旧式轨迹，代码会直接抛出错误，避免静默走旧逻辑。

当前轴距参数：

$$
\mathrm{WHEEL\_BASE} = 2.85\ \mathrm{m}
$$

对于当前主流程使用的合成 `TrackingRoute`，MPC 在全局坐标中预测：

$$
x_{k+1} = x_k + v_{k+1}\cos(\psi_k)\Delta t
$$

$$
y_{k+1} = y_k + v_{k+1}\sin(\psi_k)\Delta t
$$

$$
s_{k+1} = s_k + v_{k+1}\Delta t
$$

$$
\psi_{k+1}
= \operatorname{normalize\_angle}
\left(
\psi_k + \frac{v_{k+1}}{\mathrm{WHEEL\_BASE}}\tan(\delta)\Delta t
\right)
$$

其中 $s_k$ 是从 `RouteOffsetLaneChangeTrajectory.to_local()` 得到的路线相对进度。参考点和参考航向来自 `11.2`：

$$
P_{\mathrm{ref}}(s_k)
= (x_{\mathrm{ref}}(s_k),\ y_{\mathrm{ref}}(s_k))
$$

$$
\psi_{\mathrm{ref}}(s_k)
= \operatorname{atan2}
\left(
y_{\mathrm{ref}}(s_k+\Delta s_d)-y_{\mathrm{ref}}(\max(0,s_k-\Delta s_d)),
x_{\mathrm{ref}}(s_k+\Delta s_d)-x_{\mathrm{ref}}(\max(0,s_k-\Delta s_d))
\right)
$$

路线相对分支的横向/位置误差当前使用二维位置误差：

$$
e_p
= \sqrt{(x_k-x_{\mathrm{ref}}(s_k))^2 + (y_k-y_{\mathrm{ref}}(s_k))^2}
$$

$$
e_{\psi}
= \operatorname{normalize\_angle}(\psi_k-\psi_{\mathrm{ref}}(s_k))
$$

路线相对 MPC 默认使用加速度候选：

$$
a \in \{-4.0,\ -2.0,\ -1.0,\ 0.0,\ 1.0\}
$$

为了避免车辆在低速跟踪合成路线时继续选择负加速度并原地停住，若满足：

$$
v_0 < \max(3.0,\ 0.5v_{\mathrm{target}})
$$

则候选加速度收敛为：

$$
a \in \{0.0,\ 1.0\}
$$

单步代价为：

$$
J_k =
6.0e_p^2
+ 1.7e_{\psi}^2
+ 0.07e_v^2
+ 0.08\delta^2
+ 0.01a^2
+ 0.02k\left|\delta-\delta_{\mathrm{prev}}\right|
$$

控制器选择：

$$
(\delta^*, a^*) = \arg\min_{\delta, a} J
$$

加速度到 CARLA 油门/制动的映射：

$$
\mathrm{throttle} =
\begin{cases}
\operatorname{clamp}(0.25 + 0.18a^*,\ 0.0,\ 0.65), & a^* \ge 0 \\
0.0, & a^* < 0
\end{cases}
$$

$$
\mathrm{brake} =
\begin{cases}
0.0, & a^* \ge 0 \\
\operatorname{clamp}\left(-\dfrac{a^*}{7.5},\ 0.0,\ 1.0\right), & a^* < 0
\end{cases}
$$

最终输出：

$$
\mathrm{VehicleControl}
= (\mathrm{throttle},\ \mathrm{brake},\ \mathrm{steer}=\delta^*)
$$

### 11.5 速度控制公式

当前普通跟车、路线跟踪和右侧目标让行使用 `utils.py` 中的简单比例速度控制。

速度误差：

$$
e_v = v_{\mathrm{target}} - v_{\mathrm{current}}
$$

如果需要加速：

$$
\mathrm{throttle}
= \operatorname{clamp}(0.18 + 0.06e_v,\ 0.0,\ 0.75)
$$

$$
\mathrm{brake} = 0.0
$$

如果需要减速：

$$
\mathrm{throttle} = 0.0
$$

$$
\mathrm{brake}
= \operatorname{clamp}(-0.12e_v,\ 0.0,\ 0.75)
$$

注意：该速度控制是演示用比例控制，不是严格意义上的纵向 MPC。后续如果改为 PID、LQR 或纵向 MPC，必须同步更新本节公式。

## 12. 紧急制动灯与车辆灯光

新选题明确要求体现紧急制动。

后续代码中应加入车辆灯光控制：

```python
carla.VehicleLightState.Brake
```

建议规则：

- 正常巡航：关闭制动灯。
- 普通减速：根据 brake 控制量开启制动灯。
- 紧急制动：开启制动灯，并可考虑开启危险警示灯。

如果 CARLA 版本或车辆蓝图不支持完整灯光效果，应在文档和程序输出中说明。

## 13. pygame 可视化与演示

当前已有：

```python
DemoCamera
DemoHUD
PygameDemoDisplay
```

当前 pygame/HUD 显示：

```text
当前状态
已完成路线进度
当前工况：前车急停 / 右侧非机动车 / 正常行驶
前车 TTC
右侧目标 TTC
自车速度
制动值
转向值
碰撞次数
```

pygame 视角建议保留后车摄像头，同时 CARLA spectator 可以设置为俯视跟随，方便观察整体交通流和环形路线。

当前 `guiji.py` 默认按真实时间 1x 播放同步仿真，避免 pygame 动画在机器负载变化时忽快忽慢。直接运行入口：

```powershell
E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py
```

如需尽快跑完场景用于调试日志，可以使用自由推进模式：

```powershell
E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run
```

如需指定其他固定播放倍率，可以使用：

```powershell
E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --playback-speed 1.0
```

当前还通过 CARLA `world.debug` 增加了轨迹调试标记，便于在 pygame/CARLA 画面中观察当前规划目标：

- 在当前合成 `TrackingRoute` 前方约 `DEBUG_DRAW_LOOKAHEAD_DISTANCE = 10.0m` 处绘制红色竖向标记，表示自车下一段路线跟踪目标。
- 前方规划触发后，把有效候选 replacement segment 绘制为绿色线段，最终选中的候选使用更亮、更粗的绿色线段。
- 调试绘制由 `DEBUG_DRAW_TRAJECTORY` 控制；线段采样间隔为 `DEBUG_DRAW_TRAJECTORY_STEP = 2.0m`，每 `DEBUG_DRAW_INTERVAL_FRAMES = 4` 帧刷新一次，绘制生命周期为 `DEBUG_DRAW_LIFETIME = 0.25s`，因此视觉上保持连续，同时避免避障时每帧绘制大量候选线拖慢 1x 播放。

这些标记只用于演示和调试，不参与控制计算；如果画面过密或影响性能，可以在 `config.py` 中关闭 `DEBUG_DRAW_TRAJECTORY`。

注意：`--playback-speed 1.0` 只能在仿真循环跑得比真实时间快时主动等待，不能把已经超时的计算/渲染帧“加速回来”。如果避障时 MPC 计算或 debug 绘制耗时超过 `FIXED_DELTA_SECONDS = 0.05s`，画面仍会慢下来。因此当前候选轨迹 debug 绘制采用低频刷新，以降低避障段的渲染负担。

## 14. 评价指标

为了证明“安全行驶一圈”，建议记录以下指标：

```text
是否完成一圈
是否发生碰撞
是否成功避开前车急停
是否成功避开右侧非机动车
最大制动值
最大转向值
最小前车距离
最小前车 TTC
最小右侧目标距离
最小右侧目标 TTC
是否离开道路
是否压线或明显越界
```

最终成功条件建议定义为：

```text
完成环形路线 1 圈
碰撞次数为 0
前车急停工况避障成功
右转非机动车工况避让成功
车辆最终仍在可行驶道路上
```

## 15. 主程序执行流程

当前入口文件 `guiji.py` 中的 `main()` 已实现以下执行流程：

```text
1. 连接 CARLA 服务端
2. 加载或复用 Town10HD_Opt
3. 设置同步模式和固定仿真步长
4. 根据 `TOWN10_START_SPAWN_INDEX` 获取固定起点
5. 生成自车和前车
6. 挂载碰撞传感器和 pygame 摄像头
7. 构建 `LoopRoute` Town10 固定短路线
8. 在固定路线不同位置生成 5 辆较慢背景车辆
9. 在 R344 右侧生成 3 辆背景自行车
10. 在 `R344 -> R20` 右转处生成关键右侧直行非机动车目标
11. 进入同步仿真循环
12. 前车在 `LEAD_BRAKE_TIME` 后急停
13. 背景车辆沿 `LoopRoute` 做确定性进度推进，路线与自车完全一致，仅初始 index 和目标速度不同；背景自行车沿 R344 右侧直行
14. 自车基于当前合成路线检测最近前方车辆、TTC 和所有右侧非机动车风险
15. 自车在前方车辆风险触发后生成 replacement segment，规划失败且风险很近时进入 `EMERGENCY_BRAKE`
16. 自车在右侧非机动车风险触发后在 `ROUTE_FOLLOW` 中减速或硬刹让行
17. 避障/让行过程中继续跟踪当前合成 `TrackingRoute`
18. 到达路线终点后记录 `route_completion_time`
19. 自车方向盘回正、油门为 0、刹车为 1，停车保持 4 秒
20. 输出运行日志和碰撞次数
21. 恢复 world settings 并销毁 actor
```

后续扩展右侧非机动车工况时，再补充或优化：

```text
1. 更精确地绑定非机动车道或人行横道位置
2. 将右侧目标冲突判断升级为轨迹预测
3. 根据风险选择减速让行或转向避让
4. 记录右侧目标最小距离、TTC 和是否碰撞
```

## 16. 当前代码与新选题的关系

当前模块化程序可以视为新选题的第一阶段原型。

已经具备：

- 前车急停。
- 自车紧急制动。
- 自车横向转向避障。
- 五次多项式轨迹生成。
- MPC 跟踪。
- pygame 演示窗口。
- 碰撞监测。
- Town10 固定起点。
- Town10 固定短路线。
- 路线进度与完成判断。
- 右转路线段检测输出。
- 5 辆沿固定路线行驶的较慢背景车辆。
- 3 辆 R344 右侧背景自行车。
- `R344 -> R20` 右转处右侧关键非机动车直行目标。
- `ROUTE_FOLLOW` 内联右侧目标减速/硬刹让行。

尚未完成：

- 背景交通流密度、位置和速度的视觉效果调参。
- 右转弯区域识别的精细化。
- 右转弯非机动车转向避让策略。
- 紧急制动灯显示控制。
- 完整评价指标记录。

后续代码修改应围绕这些未完成项展开。

## 17. 建议开发顺序

已完成的基础步骤：

```text
1. 固定 Town10 城市路线和起点
2. 实现自车沿固定短路线稳定行驶一圈
3. 将现有前车急停避障工况嵌入直线段
4. 在后续十字路口右转前提前靠右
5. 路线完成后停车保持并输出碰撞结果
6. 在 `R344 -> R20` 右转处加入右侧非机动车直行目标和减速让行状态
7. 加入 5 辆路线背景车辆和 3 辆 R344 背景自行车
8. 将单文件 `guiji.py` 按配置、工具、感知、控制、路线、参与者和显示职责拆分为独立模块
```

下一步建议按以下顺序推进：

```text
第一步：观察背景车辆和背景自行车在 pygame/CARLA 视角中的位置效果并微调
第二步：将右转弯冲突判断从几何规则升级为更稳定的区域/轨迹预测
第三步：实现右转弯转向避让策略
第四步：加入制动灯和演示状态显示
第五步：记录评价指标并输出结果
```

这个顺序的好处是继续保持每一步都能单独验证，不会一次性把交通流、非机动车和避障控制全部混在一起调试。

## 18. 常见运行问题

### 18.1 地图加载较慢

如果看到：

```text
Loading map ...
```

说明 CARLA 正在切换地图，可能需要等待几十秒到一两分钟。

不要在 `client.load_world(MAP_NAME)` 时频繁按 `Ctrl+C`。

### 18.2 无法导入 carla

应使用 CARLA 对应的 Python 环境：

```powershell
E:/Anaconda_envs/envs/carla_env/python.exe
```

不要直接使用系统默认 Python。

### 18.3 pygame 窗口没有打开

可能原因：

- CARLA 服务端没有启动。
- 程序还在加载地图。
- 缺少 pygame。
- 缺少 numpy。

### 18.4 pygame 图像格式错误

当前程序已使用 numpy 将 CARLA camera 的 BGRA 数据转换为 RGB，再交给 pygame 显示。

## 19. PR 提交记录

### 2026-06-01 - 重构 pygame 演示显示层并忽略本地 Claude 配置

- 本次目标：重构 `guiji.py` 中的 pygame 演示显示层，降低显示逻辑和避障算法的耦合，并避免 `.claude/` 本地配置目录被提交。
- 主要改动：删除旧的 `PygameCameraDisplay` 实现；新增 `DemoCamera`、`DemoHUD`、`PygameDemoDisplay` 三个显示层类；主循环改为传入 `telemetry` 字典；新增 `.gitignore` 忽略 `.claude/`。
- 为什么这样改：后续还要继续修改环形路线、交通流和避障算法，显示层拆分后可以作为稳定的演示外壳，减少算法迭代时对 pygame 代码的影响。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile d:\17871\CARLA_0.9.15\WindowsNoEditor\PythonAPI\examples\dazuoye\guiji.py`，语法检查通过。
- 未覆盖风险：未启动 CARLA 服务端进行 pygame 窗口实际显示验证；未验证摄像头画面刷新、窗口关闭、`Esc`/`Q` 退出等运行时行为。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/.gitignore`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`26e8b07`
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-01 - 迁移到 Town10 固定闭环路线

- 本次目标：将当前演示从 Town04 直道场景迁移到 `Town10HD_Opt`，保留直道前车急停避障，并让自车在避障后继续完成固定路线一圈。
- 主要改动：将 `MAP_NAME` 改为 `Town10HD_Opt`；设置 Town10 固定 spawn 点；新增 `LoopRoute` 生成 744m 闭环 waypoint 路线；主循环改用闭环路线转向；HUD 增加一圈进度；完成一圈后再结束仿真。
- 为什么这样改：Town10 更适合后续扩展人行横道和非机动车避让场景；先把固定地点、直道急停避障和一圈行驶跑通，可以作为后续复杂场景的基础。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile d:\17871\CARLA_0.9.15\WindowsNoEditor\PythonAPI\examples\dazuoye\guiji.py`；已运行 `guiji.py`，结果显示 `Avoidance completed`、`完成 Town10 固定路线一圈`、`Collisions: 0`。
- 未覆盖风险：仅验证当前固定起点和路线；尚未验证其他 Town10 路段、右转非机动车、交通流、制动灯或真实传感器。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`38b701a`
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-01 - 优化 Town10 短路线与终点停车收尾

- 本次目标：在保持 Town10 固定起点和直道前车急停避障的基础上，缩短路线，避免绕外侧大圈；保留后续十字路口右转前靠右；路线完成后直接停车观察。
- 主要改动：为 `LoopRoute` 增加固定分支覆盖、短路线自闭合、右转事件检测、右转前靠右准备和基于上一进度锚点的前向最近点搜索；路线完成后进入 `ROUTE_HOLD`，方向盘回正、油门为 0、刹车为 1，并保持 `4s` 后结束；同步更新本文档。
- 为什么这样改：短路线更接近当前大作业场景调试需要，避免车辆绕 Town10 外侧大圈；右转前靠右更符合后续路口避让行人/非机动车的场景要求；终点停车可以避免继续追踪最后 waypoint 导致车辆回摆。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile dazuoye\guiji.py`；已实际运行 `guiji.py`，输出显示路线长度 `520.0m`、`131` 个 waypoint、`right_lane_before_turn=True`、`prepare_index=45`、最后路口右转事件 `right:72.9deg@126-130`、前车急停避障完成、进入 `ROUTE_HOLD` 后 `steer=+0.00`、`brake=1.00`、速度降到 `0.0m/s`，碰撞次数为 0。
- 未覆盖风险：尚未加入右转路口行人/非机动车目标、背景交通流、制动灯控制和真实传感器；目前验证集中在当前固定起点、固定路线和前车急停避障。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`d67077f`
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-01 - 剪枝收敛 Town10 演示旧逻辑

- 本次目标：对当前已跑通的 Town10 演示代码做一次保守剪枝和收敛，删除不再需要的旧逻辑，降低后续继续加入右转非机动车/行人避让场景时的维护成本。
- 主要改动：删除已被 `LoopRoute` 取代的旧 `LapTracker`；删除不再使用的 `LAP_MIN_DISTANCE` 和 `LAP_COMPLETION_RADIUS`；将起点选择收敛为 `get_town10_start_waypoint`，移除其他地图自动搜索直道起点的旧分支；删除右转事件中未使用的起止坐标字段；清理主循环中不再需要的外层 `avoidance_side` 变量；更新本文档。
- 为什么这样改：当前代码已经固定在 `Town10HD_Opt`、固定起点和 `LoopRoute` 短路线方案上，旧的一圈判断和其他地图起点搜索逻辑已经不再承担实际职责。先剪掉这些复杂度，可以让后续加非机动车/行人避让时更容易定位核心流程，也减少误读旧逻辑的风险。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile dazuoye\guiji.py`；已实际运行 `guiji.py`，输出显示路线长度 `520.0m`、`131` 个 waypoint、`right_lane_before_turn=True`、`prepare_index=45`、最后路口右转事件 `right:72.9deg@126-130`、前车急停避障完成、进入 `ROUTE_HOLD` 后停车保持，碰撞次数为 0。
- 未覆盖风险：本次只做保守剪枝，未重构 MPC、换道轨迹、显示层或路线核心生成逻辑；尚未加入右转路口行人/非机动车目标；尚未验证行人/非机动车目标出现后的横向/纵向避让策略。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`61fa715`
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-02 - 整理维护文档结构并同步当前框架

- 本次目标：按新的维护文档规则更新本文档，删除单独的维护记录段，并让前面的程序框架、场景配置和主流程与当前 `guiji.py` 保持一致。
- 主要改动：将文档维护约定改为使用 PR/提交记录；删除 `维护记录` 章节；修正主体中的 `Town04`、旧路线待办和旧目标流程描述；补充已有 PR/提交记录的提交代号；新增本次文档结构更新记录。
- 为什么这样改：维护文档不能只在末尾追加记录，否则前面的框架说明会继续误导读者。把主体内容和 PR/提交记录同时维护，可以让读者直接从正文看到当前系统状态。
- 如何验证：基于当前 `guiji.py` 和 `git log` 做文档一致性检查；本次只修改 Markdown 文档，未运行 CARLA 仿真。
- 未覆盖风险：本次没有改变程序代码；历史 PR 记录中的早期实验细节仍保留为历史上下文，没有重新验证早期版本运行结果。
- 需要 reviewer 重点看的文件：`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：6e14fc6
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-02 - 接入 R344-R20 右转非机动车让行工况

- 本次目标：在 Town10 固定路线的 `R344 -> R20` 大十字路口右转处加入右侧非机动车直行目标，并让自车在右转冲突窗口内减速让行。
- 主要改动：新增右侧非机动车风险阈值；新增精简后的 `RightSideObjectReading`、`RightSideBicycleCrossing` 和 `spawn_right_side_bicycle_crossing`；虚拟感知层增加右侧目标读取；主循环新增 `RIGHT_OBJECT_YIELD` 状态；pygame HUD 和终端日志增加右侧目标距离/TTC；根据实际运行结果调整非机动车生成偏移和触发时机；将非机动车轨迹改为锚定 `R344` 连接段右侧并沿 R344 方向直行，自车右转进入 `R20` 时让行；回退不稳定的视频录制实验；修正 `finally` 清理顺序，避免显示关闭异常阻断 actor 销毁；同步更新本文档主体框架。
- 为什么这样改：该路口是当前路线中的明确右转点，适合作为“右转避让右侧非机动车”的第一版落地工况。先用虚拟真值和减速让行跑通冲突触发链路，可以为后续转向避让、轨迹预测和评价指标打基础。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile dazuoye\guiji.py`，语法检查通过；已运行 `E:\Anaconda_envs\envs\carla_env\python.exe guiji.py`，日志确认 `Right-side bicycle ready`、`anchor_index=91`、`anchor_road=344`、`path=straight_along_r344`、`Right-side bicycle started`、`Right object yield started/completed` 均触发，仿真结束时 `Collisions: 0` 且 `Cleanup finished`；运行后查询 CARLA world，`ego`、`lead`、`right_side_bicycle` 残留列表为空。
- 未覆盖风险：本次运行环境提示未打开 pygame 动画窗口，视觉效果仍需在用户本机窗口中确认；第一版策略以减速让行为主，还未实现更完整的右转转向避让；尚未加入背景交通流和制动灯控制。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：a73e870
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-02 - 加入固定路线背景交通流

- 本次目标：在现有 Town10 固定路线场景中加入可复现的背景交通流，包括 5 辆较慢路线车辆和 3 辆 R344 右侧背景自行车，同时保留前车急停和关键右侧非机动车让行工况。
- 主要改动：新增固定随机种子和背景交通参数；新增 `BackgroundRouteVehicle`，使背景车辆按自车同一条 `LoopRoute` 做确定性进度推进，仅初始 index 和目标速度不同；新增背景车辆和 R344 背景自行车生成函数；虚拟感知层从只读取前车/关键非机动车扩展为读取最近前方车辆和所有右侧非机动车候选目标；背景目标达到原有距离/TTC 条件时也能触发现有避障/让行状态；碰撞日志补充 role/type 信息；剪枝收敛了右侧目标候选列表组织和背景车未使用速度字段；同步更新本文档主体框架。
- 为什么这样改：新选题需要城市交通流氛围，但当前阶段更重视可控和可复现实验。固定路线、固定数量和限速随机可以提高场景丰富度，同时避免全城随机交通流干扰核心避障验证；背景车不使用自由自动驾驶，避免偏离自车路线乱开。
- 如何验证：已运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile guiji.py`，语法检查通过；已运行 `E:\Anaconda_envs\envs\carla_env\python.exe guiji.py`，日志确认 5 辆 `background_vehicle_*` 以 `route_index=32,54,76,96,108` 和 `7.6~8.6m/s` 生成，3 辆 `background_bicycle_*` 和关键 `right_side_bicycle` 生成成功；前车急停避障和 `RIGHT_OBJECT_YIELD` 均触发，仿真结束时 `Collisions: 0` 且 `Cleanup finished`；运行后查询 CARLA world，`ego`、`lead`、`right_side_bicycle` 和 `background_*` 残留列表为空。
- 未覆盖风险：当前环境提示 pygame/numpy 不可用，背景交通视觉效果仍需在用户本机窗口中确认；背景车辆使用确定性路线推进，不是完整 Traffic Manager 行为，不具备真实跟车礼让能力，因此背景车速度和初始位置需要继续按演示画面微调；背景自行车会参与右侧让行风险判断，当前验证中让行持续时间变长，后续可按演示节奏继续调参。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：9fb69fa
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-03 - 分阶段拆分 CARLA 避障程序模块

- 本次目标：将已经跑通的单文件 `guiji.py` 按职责拆分为可供多人协作的独立模块，并保证拆分过程不改变现有路线、交通流、感知、避障和显示行为。
- 主要改动：新增 `config.py`、`utils.py`、`perception.py`、`control.py`、`route.py`、`actors.py` 和 `display.py`；将配置常量、通用工具、虚拟感知、轨迹与 MPC、固定路线、交通参与者、pygame 显示和碰撞监测分别迁移到对应模块；将 `guiji.py` 收敛为 CARLA 世界初始化、行为状态机、主循环和清理入口；使用显式导入并保持单向模块依赖；`display.py` 在 Windows 下自动补充当前 Conda 环境的 `Library\bin` DLL 搜索路径，并在显示依赖导入失败时输出具体原因；剪枝时将关键右侧非机动车与背景自行车统一为一个场景列表，删除两套更新和感知接口；删除拆分过程中使用的临时机械脚本和生成缓存。
- 为什么这样改：后续需要 3 至 4 人并行协作，继续集中修改一个大文件会增加冲突和审核难度。按职责拆分后，不同成员可以分别负责场景、感知、控制和显示验证，同时降低互相干扰。
- 如何验证：在拆出 `config.py/utils.py`、`perception.py/control.py`、`route.py/actors.py` 和 `display.py` 后分别运行完整 CARLA 场景；最终运行 `E:\Anaconda_envs\envs\carla_env\python.exe -m py_compile guiji.py config.py utils.py perception.py control.py route.py actors.py display.py` 和 `E:\Anaconda_envs\envs\carla_env\python.exe guiji.py`，日志确认前车急停避障、背景交通生成、`RIGHT_OBJECT_YIELD`、路线一圈完成和 `Cleanup finished` 均正常，最终 `Collisions: 0`；运行后查询 CARLA world，相关 actor 残留列表为空；未手动补充终端 `PATH` 时再次直接运行 `python.exe guiji.py`，pygame 动画渲染路径正常启用，未出现无窗口降级提示。
- 未覆盖风险：不同机器上的 Conda/Numpy 安装状态仍可能影响动画依赖加载；本次只重构文件组织，没有新增自动化单元测试；多人协作时若同时修改 `guiji.py` 状态机，仍需要通过分支和 PR 审核协调。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/config.py`、`dazuoye/utils.py`、`dazuoye/perception.py`、`dazuoye/control.py`、`dazuoye/route.py`、`dazuoye/actors.py`、`dazuoye/display.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：f7f91c1
- PR/分支信息：直接推送到 `origin/main`，未创建独立 PR。

### 2026-06-05 - 补充 LaTeX 公式与状态切换条件

- 本次目标：在唯一维护文档中用 LaTeX 形式详细补充路径生成、TTC 风险计算、五次多项式轨迹、MPC 跟踪和速度控制的数学公式，并写清当前状态机各状态的进入/退出条件。
- 主要改动：新增公式维护约定；补充 `LoopRoute` 路径生成、路线进度、前视转向公式；补充前车 TTC、右侧目标 TTC 和冲突窗口判断公式；补充前方急停风险、右侧目标风险的状态机触发条件；新增 `ROUTE_FOLLOW`、`AVOID`、`EMERGENCY_BRAKE`、`RIGHT_OBJECT_YIELD`、`ROUTE_HOLD` 的进入/退出条件表；将主要数学公式从文本形式调整为 Markdown LaTeX；补充五次多项式换道边界条件、参考航向计算、采样式 MPC 预测模型、代价函数和油门/制动映射。
- 为什么这样改：当前代码已经拆分为路线、感知、控制等模块，如果文档只描述流程，不写清数学公式，后续多人协作时很难判断算法修改是否改变了实际控制逻辑。把公式写入正文可以让代码、文档和后续 PR 审核保持一致。
- 如何验证：基于当前 `route.py`、`perception.py`、`control.py`、`utils.py` 和 `guiji.py` 做代码阅读与公式对应检查；本次只修改 Markdown 文档，未运行 CARLA 仿真。
- 未覆盖风险：未对公式进行独立数值单元测试；未运行仿真验证文档描述之外的控制效果；当前公式说明对应现有代码，后续如果控制算法改为 PID/LQR/纵向 MPC 或右转轨迹预测，需要再次同步更新。
- 需要 reviewer 重点看的文件：`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ae73bf1`；合并提交 `aa5d44f`。
- PR/分支信息：已通过 PR #5 从 `feature/decision-control` 合并到 `main`。

### 2026-06-05 - 合并路线跟踪状态并保留多次避障

- 本次目标：合并功能重复的 `FOLLOW` 和 `LANE_KEEP`，保留自车完成避障或右侧目标让行后再次触发前方紧急避障的能力。
- 主要改动：将正常行驶状态统一为 `ROUTE_FOLLOW`；初始状态、避障完成恢复状态、右侧目标让行完成恢复状态都回到 `ROUTE_FOLLOW`；前方紧急避障和右侧目标让行都从 `ROUTE_FOLLOW` 触发；删除默认控制分支中将 `ROUTE_HOLD` 混入路线跟踪的误导性描述；同步更新状态切换条件表。
- 为什么这样改：`FOLLOW` 和 `LANE_KEEP` 在当前实现中已经没有控制功能差异，只剩“是否经历过避障”的标签意义。合并后状态机更清楚，同时 `ROUTE_FOLLOW` 仍可多次触发前方避障，满足背景交通流和慢速车带来的重复避障需求。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile guiji.py`，语法检查通过；尚未运行 CARLA 场景仿真。
- 未覆盖风险：未验证连续多次实际避障的动态效果；未处理同时出现前方风险和右侧目标风险时更复杂的优先级仲裁；HUD 中显示的状态名会从 `FOLLOW/LANE_KEEP` 变为 `ROUTE_FOLLOW`；`EMERGENCY_BRAKE` 恢复逻辑由后续记录补充。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ae73bf1`；合并提交 `aa5d44f`。
- PR/分支信息：已通过 PR #5 从 `feature/decision-control` 合并到 `main`。

### 2026-06-05 - 增加紧急制动恢复出口

- 本次目标：避免自车进入 `EMERGENCY_BRAKE` 后永久停留在该状态。
- 主要改动：新增 `emergency_recovered` 判定；当前方目标消失、距离恢复到 `SAFE_DISTANCE + 8.0m` 以上，或 TTC 恢复到 `TTC_BRAKE_THRESHOLD + 1.0s` 以上时，从 `EMERGENCY_BRAKE` 回到 `ROUTE_FOLLOW`；如果前方风险仍在但邻道重新可用，则从 `EMERGENCY_BRAKE` 重新生成避障轨迹并进入 `AVOID`；同步更新状态切换表。
- 为什么这样改：紧急制动应是风险处置状态，不应成为永久停车状态。加入恢复和重新避障出口后，后续前方风险解除或邻道重新可用时，自车可以继续路线跟踪或再次尝试避障。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile guiji.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe guiji.py`，本次实景运行触发了两次 `AVOID`，但没有进入 `EMERGENCY_BRAKE`，因此恢复出口未被实景覆盖；运行最终在第二次 `AVOID` 后碰撞 `static.pole`，`Collisions: 1`。
- 未覆盖风险：`EMERGENCY_BRAKE -> ROUTE_FOLLOW` 和 `EMERGENCY_BRAKE -> AVOID` 只完成代码路径添加，尚未在实景中触发验证；弯道/右转附近继续使用直线五次换道轨迹仍可能导致靠边或撞静态杆，需后续单独处理弯道避障限制。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ae73bf1`；合并提交 `aa5d44f`。
- PR/分支信息：已通过 PR #5 从 `feature/decision-control` 合并到 `main`。

### 2026-06-05 - 将弯道避障改为真实路线叠加局部偏移轨迹

- 本次目标：解决弯道触发避障时，固定直角坐标系五次轨迹相对道路偏出的问题，使避障轨迹等于真实路线弯曲基线叠加避障起点局部坐标系中的五次横向增量。
- 主要改动：在 `control.py` 新增并收敛 `RouteOffsetLaneChangeTrajectory`，用 `LoopRoute` 的真实路线点作为基线，用每个路线点自身的道路右向量叠加五次横向增量；目标偏移 `target_offset` 改为由目标邻道中心相对避障起点路线点的道路右向距离计算，不再使用触发瞬间自车右向量；五次横向曲线从自车当前道路横向偏移 `start_offset` 过渡到目标偏移，避免第二次避障时参考轨迹起点与车辆实际位置不连续；`SamplingMPCTracker` 增加路线相对轨迹分支，在全局坐标中预测车辆运动并对真实路线叠加偏移后的参考点计算代价；路线相对轨迹的参考航向由最终参考轨迹差分计算；近距离触发时避障长度从固定 `28m` 收缩为动态长度；路线相对 MPC 的转向候选限幅为 `±0.45`，且在低速时禁止继续选择负加速度，避免 `AVOID` 中原地停住；`AVOID` 完成判定缓冲从 `8m` 收缩到 `2m`，并将完成后的当前车道保持时间从 `3s` 缩短到 `1s`；`perception.py` 的 `FrontVehicleReading` 新增 `actor_id` 和 `actor_role` 字段，仅用于诊断和后续决策扩展；同步更新本文档中的避障轨迹、状态切换和 MPC 公式。
- 为什么这样改：原轨迹把整段避障固定在触发瞬间的自车 `forward/right` 坐标系中，直道上可用，但在大右转弯上道路方向持续变化，容易出现“选择右邻道但实际相对道路切偏”的现象。当前实现保留真实路线本身的弯曲，再把五次避障增量叠加上去，更符合“生成轨迹 + 真实道路相对初始坐标系偏移”的建模思路。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py`，日志显示第一次直道避障完成；第二辆慢车避障在 `17.85s` 触发，`start_offset=1.31m`、`target_offset=3.50m`，`20.70s` 退出 `AVOID`，随后保持 `ROUTE_FOLLOW` 且未在 `AVOID` 中原地停住；右侧非机动车/行人让行、路线终点保持均完成，最终 `Collisions: 0`。
- 未覆盖风险：本次验证基于一次 CARLA 实景运行，未做多随机种子或不同慢车位置回归；新增的 `actor_id/actor_role` 目前仅用于诊断，尚未接入不同目标角色的差异化决策；右侧目标让行策略仍是低速/停车让行，尚未实现横向绕行。
- 需要 reviewer 重点看的文件：`dazuoye/control.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ae73bf1`；合并提交 `aa5d44f`。
- PR/分支信息：已通过 PR #5 从 `feature/decision-control` 合并到 `main`。

### 2026-06-05 - 删除未使用的旧直线局部轨迹类

- 本次目标：确认 `QuinticLaneChangeTrajectory` 是否仍被当前控制流程使用；如果未使用，则删除旧实现，降低控制模块维护成本。
- 主要改动：从 `control.py` 删除未被调用的 `QuinticLaneChangeTrajectory`；从 `guiji.py` 删除对应导入，并把避障轨迹生成注释改为当前实际使用的路线相对避障轨迹；同步更新本文档主体中的控制模块职责、轨迹生成说明和 PR 记录。
- 为什么这样改：当前 `AVOID` 和 `EMERGENCY_BRAKE -> AVOID` 都使用 `RouteOffsetLaneChangeTrajectory`，旧直线局部轨迹只会增加阅读成本，并可能让后续维护者误以为主流程仍存在两套路由。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py`，语法检查通过；已运行 `git -C dazuoye diff --check`，无空白错误，仅有 Windows 下 LF/CRLF 提示；本次未重新运行完整 CARLA 场景。
- 未覆盖风险：本次是死代码删除，未做完整实景回归；如果后续需要旧的固定直角坐标系直线换道对比实现，需要从 Git 历史恢复。
- 需要 reviewer 重点看的文件：`dazuoye/control.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`9ee9d5d`。
- PR/分支信息：推送到 `origin/feature/decision-control`，尚未创建新 PR。

### 2026-06-06 - 增加多候选避障路径生成与选择

- 本次目标：参考笔记中的路径生成、路径限制和最优路径选择思路，把前方避障从固定一条终点轨迹改为多条候选轨迹中筛选最优路径，同时忽略本地 PDF 资料文件。
- 主要改动：在 `control.py` 新增 `AvoidancePathCandidate` 和 `select_best_route_offset_trajectory()`；围绕基础避障长度和目标邻道中心生成多组 `RouteOffsetLaneChangeTrajectory` 候选；按目标车道边界、最小轨迹长度、最大横向加速度和前向距离约束筛掉无效路径；用安全性、舒适性、跟踪难度和终点居中误差组成总代价并选择最优候选；`guiji.py` 的 `ROUTE_FOLLOW -> AVOID` 与 `EMERGENCY_BRAKE -> AVOID` 入口改为使用候选选择结果，若没有有效候选则进入或保持 `EMERGENCY_BRAKE`；删除 `SamplingMPCTracker` 中旧的非路线相对局部轨迹跟踪分支，当前控制器只接受路线相对轨迹；`guiji.py` 增加默认 1x 实时播放入口、`--playback-speed` 和 `--free-run` 参数；`.gitignore` 增加 `分布式驱动车轨迹跟踪13.pdf`；同步更新本文档中的状态切换、轨迹选择公式、MPC 公式和运行方式。
- 为什么这样改：现有路线相对轨迹解决了弯道坐标系问题，但进入避障时仍相当于固定一条长度和终点都确定的轨迹。加入候选生成、约束筛选和代价选择后，可以表达“多路径规划，再选最优路径”的控制流程，也更贴近笔记中的路径限制和代价函数思想。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --help`，确认存在 `--playback-speed` 和 `--free-run` 入口；已运行默认 1x 入口 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py`，完整 CARLA 场景跑通，日志显示 `Playback mode: 1.0x realtime`，最终墙钟耗时 `70.1s`。第一次避障在 `6.65s` 触发，候选路径 `valid=10/12`，选择 `length=32.2m`、`target_offset=2.80m`、`ay=1.90m/s^2`、`cost=1.10`，并在 `12.25s` 完成；第二次避障在 `17.90s` 触发，候选路径 `valid=1/12`，选择 `length=16.9m`、`target_offset=2.80m`、`ay=3.52m/s^2`、`cost=4.72`，并在 `20.75s` 完成；右侧目标让行在 `37.70s` 触发并在 `55.20s` 完成；路线终点 `ROUTE_HOLD` 停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：当前是轻量候选路径选择，不是完整论文级分布式驱动车动力学规划；PDF 文本未在当前环境中成功抽取，第一版主要依据笔记和现有代码实现；本次只做了一次固定场景实景运行，未覆盖多随机种子、不同速度/距离组合或更多交通流密度；候选代价权重仍需要根据 pygame/CARLA 效果继续调参；1x 播放只保证同步仿真循环按墙钟等待，若 CARLA 服务端本身低于实时速度，画面仍会受机器性能限制。
- 需要 reviewer 重点看的文件：`dazuoye/control.py`、`dazuoye/guiji.py`、`dazuoye/.gitignore`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-06 - 收敛右转让行硬刹与背景车追尾保护

- 本次目标：确认右转避让处“启动后又停”的原因，并减少同一右转让行状态内的二次硬刹或背景车追尾风险。
- 主要改动：`RightSideObjectReading` 新增 `actor_id`、`actor_role`、相对纵向距离和相对横向距离；`guiji.py` 在右侧目标让行开始、目标切换、硬刹激活和硬刹释放时输出诊断日志；`RIGHT_OBJECT_YIELD` 中新增 `right_object_stop_active`，使用 `RIGHT_OBJECT_STOP_DISTANCE = 13.0m` 和 `RIGHT_OBJECT_STOP_RELEASE_DISTANCE = 14.5m` 做硬刹滞回；背景路线车辆在后方接近自车到 `BACKGROUND_VEHICLE_EGO_CLEARANCE = 18.0m` 内时暂停脚本进度，避免长时间让行后被背景车硬追尾；同步更新本文档中的参数、背景交通说明和右侧让行状态公式。
- 为什么这样改：实测日志显示右转让行不是重复进入 `RIGHT_OBJECT_YIELD`，而是在同一状态内右侧目标在自行车和两名行人之间切换，且硬刹释放条件过于保守会让自车长时间停留；长时间停留又会让按固定路线推进的背景车从后方追上。加入目标身份日志可以定位触发源，距离滞回可以避免阈值抖动，背景车后向保护可以让交通流不破坏核心避障演示。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；第一次 `--free-run` 验证显示 `Right object yield started` 只出现一次，但目标在 `right_side_pedestrian_1`、`right_side_pedestrian_2` 和 `right_side_bicycle` 间切换，随后因背景车 `background_vehicle_5` 追尾提前结束；修正后再次运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，日志显示右转让行在 `37.65s` 进入、硬刹在 `37.65s` 激活、`49.30s` 释放，释放后未再次硬刹，`54.95s` 完成右转让行；路线完成后进入 `ROUTE_HOLD`，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：本次只验证固定 Town10 起点、固定随机种子和 `--free-run` 运行；1x 实时播放视觉效果尚未重新人工确认；右转让行仍是低速/停车让行，没有实现右转横向绕行；目标切换日志用于诊断，尚未把不同角色接入差异化决策。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/perception.py`、`dazuoye/actors.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 调整右转冲突几何门限为右后方优先

- 本次目标：让 `R344 -> R20` 右转让行的冲突区域更符合“自车右转切入右侧直行非机动车流”的实际场景。
- 主要改动：在 `config.py` 中新增 `RIGHT_OBJECT_LONGITUDINAL_MIN/MAX` 和 `RIGHT_OBJECT_LATERAL_MIN/MAX`；将右侧目标几何门限从 `-8m <= longitudinal <= 34m`、`-14m <= lateral <= 18m` 调整为 `-30m <= longitudinal <= 12m`、`-3m <= lateral <= 20m`；`perception.py` 改为读取配置项，不再在感知函数里写死数字；同步更新本文档中的参数与几何门限公式。
- 为什么这样改：当前右转冲突主要来自自车右侧或右后方沿 R344 直行的非机动车。远前方目标通常更可能已经先通过冲突区域，反而不是主要威胁；因此几何门限应以后方和右侧为主，并保留少量前方、左侧容错。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，完整 CARLA 场景跑通，右侧目标让行在 `38.30s` 进入并在 `48.25s` 完成，路线终点 `ROUTE_HOLD` 停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：几何门限调整后的 pygame 画面观感仍需人工确认；如果目标生成位置或自车姿态变化较大，仍可能需要继续微调门限。
- 需要 reviewer 重点看的文件：`dazuoye/config.py`、`dazuoye/perception.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 增加前视目标与候选避障轨迹可视化

- 本次目标：在 pygame/CARLA 演示画面中标出自车当前跟踪目标和多条候选避障轨迹，便于观察避障规划是否符合预期。
- 主要改动：在 `config.py` 中新增 `DEBUG_DRAW_TRAJECTORY`、`DEBUG_DRAW_LOOKAHEAD_DISTANCE`、`DEBUG_DRAW_TRAJECTORY_STEP` 和 `DEBUG_DRAW_LIFETIME`；在 `guiji.py` 中新增红色前视标记和绿色候选轨迹绘制函数；`plan_route_relative_avoidance()` 返回选中轨迹的同时返回候选轨迹列表；主循环在 `AVOID` 状态持续绘制有效候选轨迹，在 `ROUTE_FOLLOW` 状态绘制路线前视目标；避障完成后清空候选轨迹缓存；同步更新本文档的可视化说明。
- 为什么这样改：当前已经有多候选路径生成与选择，但仅靠日志不方便判断轨迹在道路上的实际位置。把前视目标和候选轨迹画出来后，可以更直观看到自车正在跟踪哪条路线、候选轨迹是否过宽或偏离道路。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，完整 CARLA 场景跑通，第一次避障在 `6.65s` 触发且候选 `valid=10/12`，第二次避障在 `18.15s` 触发且候选 `valid=1/12`，右侧目标让行完成，路线终点停车保持完成，最终 `Collisions: 0`，`Cleanup finished`；运行过程中未出现 `world.debug` 绘制相关异常。
- 未覆盖风险：红色标记和绿色候选轨迹的画面效果仍需在 pygame/CARLA 窗口中人工确认；`world.debug` 绘制是调试可视化，不是真正的车辆灯光；如果调试线段过多，可能轻微影响画面性能。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 平滑弯道避障参考线显示

- 本次目标：减少弯道避障轨迹在画面上的折线感，优先处理路线右向量和调试线采样，不调整候选路径筛选策略。
- 主要改动：`RouteOffsetLaneChangeTrajectory` 中新增路线中心线浮点索引插值和前后差分右向量计算；轨迹生成、`to_local()` 横向投影、目标邻道中心偏移统一使用平滑右向量，不再直接使用单个 waypoint 的 `get_right_vector()`；曾将 `DEBUG_DRAW_TRAJECTORY_STEP` 调整为 `1.0m` 以观察更细轨迹，后续因 1x 播放性能压力恢复为 `2.0m`；同步更新本文档中的轨迹公式和可视化参数。
- 为什么这样改：弯道处 waypoint 朝向可能变化不均匀，直接叠加每个 waypoint 自身右向量容易让路线相对避障轨迹出现视觉折线。用路线中心线前后差分得到切线，再由切线计算右向量，可以让横向偏移方向随中心线连续变化。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，完整 CARLA 场景跑通，第一次避障 `valid=10/12` 并完成，第二次弯道避障 `valid=1/12` 并在 `21.10s` 完成，右侧目标让行和终点停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：本次没有改变“弯道只有一条有效候选轨迹”的筛选问题，也没有拉长弯道避障长度；第二次弯道避障仍为 `valid=1/12`，后续需要继续检查候选拒绝原因和前车距离约束；绿色轨迹是否真正更顺仍建议在 pygame/CARLA 画面中人工确认。
- 需要 reviewer 重点看的文件：`dazuoye/control.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 降低避障轨迹调试绘制开销

- 本次目标：解决 1x 播放模式下避障阶段明显变慢的问题。
- 主要改动：新增 `DEBUG_DRAW_INTERVAL_FRAMES = 4`；候选避障轨迹和前视目标不再每帧重画，而是每 4 帧刷新一次；将 `DEBUG_DRAW_LIFETIME` 从 `0.12s` 调整为 `0.25s`，避免降频后线段闪烁；同步更新本文档中的 pygame/CARLA 可视化说明。
- 为什么这样改：避障时可能存在多条有效候选轨迹，每条轨迹按 `1.0m` 采样绘制。如果每一帧都调用大量 `world.debug.draw_line()`，同步仿真单帧耗时会超过 `0.05s`，即使显示 `Playback mode: 1.0x realtime`，画面也会因为计算/渲染超时而变慢。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行默认 1x 入口 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py`，日志显示 `Playback mode: 1.0x realtime`，仿真运行到 `64s` 左右，最终墙钟耗时 `64.8s`，路线终点停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：如果机器渲染压力或 MPC 计算本身仍超过单帧预算，1x 仍可能在避障段短暂变慢；必要时可继续减少绘制内容，例如只画选中轨迹或关闭 `DEBUG_DRAW_TRAJECTORY`。
- 需要 reviewer 重点看的文件：`dazuoye/guiji.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 使用路线弧线参考修正弯道前车识别

- 本次目标：解决弯道上前车/慢车因为自车直角坐标横向投影偏差而被过晚识别，导致避障轨迹被压短、候选轨迹数量过少的问题。
- 主要改动：`VirtualGroundTruthSensor` 增加 `loop_route` 参考线输入；`front_vehicle()` 支持路线参考线模式和自车坐标兜底模式；正常路线跟踪阶段把自车和目标投影到 `LoopRoute` 前方局部弧线，使用沿路线弧长距离和相对路线横向偏移判断同车道前车；`AVOID` 状态下继续使用自车坐标系前车读取，避免已经绕开的原车道目标继续触发制动；路线坐标系下同车道横向阈值收紧为 `0.45 * lane_width`；`guiji.py` 新增 `close_slow_front_vehicle` 避障触发项，使距离进入换道窗口且接近速度明显时可以提前进入 `AVOID`；同步更新本文档中的感知公式和风险触发条件。
- 为什么这样改：弯道上两车都在同一车道时，两车连线不一定与自车当前朝向一致，旧的 `forward/right` 投影会把真实前车判断为横向偏离，直到距离很近才满足同车道条件。路线弧线参考更接近“车道线/导航参考线传感器”输出，可以更早得到合理的 $s,d$；同时避障过程中切回自车坐标兜底，避免绕开后仍盯着原车道障碍物。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，完整 CARLA 场景跑通。第一次急停避障在 `6.20s` 触发，距离 `33.9m`，候选 `valid=10/12`，选择长度 `36.0m`；第二次弯道慢车避障在 `16.25s` 触发，距离 `33.7m`，候选 `valid=12/12`，选择长度 `36.0m`，相比原先约 `13m`、`valid=1/12` 明显提前；右侧目标让行、终点停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：本次仍使用虚拟路线参考线，等价于车道线/导航参考线真值，尚未加入噪声和延迟；路线参考模式下远距离前车日志偶尔会显示较大弧长距离，但当前触发仍受 `SAFE_DISTANCE` 和避障窗口约束；弯道轨迹视觉顺滑程度仍建议在 pygame/CARLA 窗口中人工确认。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-07 - 加入慢车位移预测与避障后制动收敛

- 本次目标：针对弯道慢速前车继续前进导致避障轨迹纵向长度不足的问题，在不重写五次横向曲线的前提下，让候选避障长度和前向安全约束考虑目标车沿路线方向的预测位移；同时在自车已经横向绕开目标后减少不必要的大制动。
- 主要改动：`FrontVehicleReading` 新增 `target_speed_along`，记录目标车沿自车坐标或路线参考线方向的速度；`plan_route_relative_avoidance()` 根据目标角色区分急刹前车和普通慢车，急刹前车按静止障碍处理，普通慢车使用 $v_{o,\mathrm{route}}$ 估计预测位移并动态拉伸基础避障长度；`select_best_route_offset_trajectory()` 把候选长度上限扩展到 `56m`，并在候选约束和安全代价中使用预测后的前向可用距离；`AVOID` 状态新增横向分离判定，车辆已经离开原车道目标且感知层不再认为有同车道前车时，将制动收敛到较小上限；同步更新本文档中的前方感知、风险判断、候选轨迹和制动收敛公式。
- 为什么这样改：慢车不是静态障碍物，如果只用触发瞬间的前车距离限制候选长度，避障轨迹容易偏短，绕行结束时慢车仍在自车前方。用目标车路线方向速度做轻量预测，可以把“前车在换道期间继续前进的距离”反映到候选长度和安全代价中；横向分离后减小制动，则避免已经绕开目标还继续大力刹车。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile dazuoye/guiji.py dazuoye/control.py dazuoye/perception.py dazuoye/route.py dazuoye/actors.py`，语法检查通过；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe dazuoye/guiji.py --free-run`，完整 CARLA 场景跑通。第一次急停避障在 `6.20s` 触发，候选 `valid=10/12`，选择长度 `36.4m`，并在 `12.80s` 完成；第二次弯道慢车避障在 `16.10s` 触发，候选 `valid=12/12`，选择长度 `38.8m`，并在 `23.95s` 完成；避障后段日志显示制动收敛到 `0.15`，右侧目标让行和终点停车保持完成，最终 `Collisions: 0`，`Cleanup finished`。
- 未覆盖风险：当前只是基于路线方向速度的一阶预测，不是完整时空 Lattice/动态障碍物规划；目标速度、路线投影仍来自虚拟真值感知，尚未加入传感器噪声和延迟；只做了一次固定场景 `--free-run` 验证，pygame 画面观感和不同速度组合仍需人工继续观察。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/control.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`5d326ea`。
- PR/分支信息：已推送到 `origin/feature/decision-control`；GitHub 连接器创建 PR 时返回 403，PR 需手动在 GitHub 创建或授权后再创建。

### 2026-06-08 - 合并感知增强分支到决策控制分支

- 本次目标：把 `origin/feature/perception-risk` 中的感知增强内容合并到当前决策控制分支，同时保留已经验证过的路线弧线前车识别、多候选避障、慢车位移预测和右转让行控制逻辑。
- 主要改动：`perception.py` 重整为融合版，保留 `front_vehicle(use_route_reference=...)` 和 `target_speed_along`，新增 `front_vehicles()`、固定随机种子的距离/速度噪声、前向/侧向 FOV、远距离漏检、CARLA radar 点云输入和简单聚类；`RightSideObjectReading` 增加 `risk_level`、`predicted_ttc`、`object_type` 和连续帧确认；`guiji.py` 在配置开启时挂载前向雷达，并让右侧目标 `risk_level >= 2` 也能触发让行；`config.py` 新增感知增强和雷达配置，默认 `RADAR_ENABLED = False` 以保持主演示稳定；`.gitignore` 增加 `__pycache__/` 与 `*.pyc`，并从合并结果中移除远端误提交的 `.pyc` 文件。
- 为什么这样改：感知分支提供了更接近传感器输出的噪声、视场、漏检和雷达点云能力，但直接覆盖当前分支会丢失弯道避障和慢车预测逻辑。因此本次采用“当前控制分支为主、感知增强移植进来”的方式合并。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile .\dazuoye\guiji.py .\dazuoye\control.py .\dazuoye\perception.py .\dazuoye\route.py .\dazuoye\actors.py`，语法检查通过；已运行 `git -C .\dazuoye diff --check --cached`，无空白错误；已尝试运行 `E:/Anaconda_envs/envs/carla_env/python.exe .\dazuoye\guiji.py --free-run`，但 CARLA 服务端在 `localhost:2000` 等待 `120000ms` 后超时，未完成实景回归。
- 未覆盖风险：当前尚未完成完整 CARLA 场景回归；雷达模式默认关闭，尚未验证 `RADAR_ENABLED = True` 时的点云聚类效果；噪声/FOV/漏检可能改变触发时机，必要时可临时关闭 `SENSOR_NOISE_ENABLED` 进行对照；需要在 CARLA 服务端已启动且地图可加载时再次运行 `--free-run`。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`、`dazuoye/.gitignore`。
- 提交代号/Commit ID：`2de989b`。
- PR/分支信息：已在本地 `feature/decision-control` 完成合并提交；尚未推送。

### 2026-06-08 - 精简感知合并后的未接入接口

- 本次目标：在合并感知增强分支后进行剪枝收敛，减少当前主流程没有使用的临时接口和配置。
- 主要改动：删除未接入主循环的 `RiskAssessment` 数据结构和 `VirtualGroundTruthSensor.assess_risk()` 包装函数；删除未使用的 `RIGHT_PREDICTION_SECONDS` 配置；同步收敛维护文档中关于统一风险评估入口的描述。
- 为什么这样改：当前 `guiji.py` 状态机仍显式读取 `front_vehicle()` 和 `right_side_object()`，统一风险评估包装没有被调用，保留会增加维护者判断成本。先删掉未接入接口，可以让感知层聚焦在已经使用的噪声/FOV/雷达候选和右侧风险等级输出。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile .\dazuoye\guiji.py .\dazuoye\control.py .\dazuoye\perception.py .\dazuoye\route.py .\dazuoye\actors.py`，语法检查通过；已运行 `git -C .\dazuoye diff --check`，无空白错误，仅有 Windows 下 LF/CRLF 提示。
- 未覆盖风险：本次为未接入接口删除，尚未重新完成 CARLA 实景回归；如果后续确实要把状态机改成统一风险评估，需要重新设计并接入该接口，而不是沿用这次删除的未验证包装。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/config.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`bb65888`。
- PR/分支信息：已在本地 `feature/decision-control` 完成剪枝提交；尚未推送。

### 2026-06-08 - 使用联合参考轨迹支持 AVOID 中再次避障判断

- 本次目标：让前方风险判断跟随当前实际计划轨迹变化，避免旧目标被绕开后重复触发，同时在避障过程中仍能发现联合轨迹前方的新危险目标，并允许必要时重新规划。
- 主要改动：`perception.py` 新增临时 `FrontReferencePath`，支持把车辆投影到“避障轨迹 + 后续路线延伸”的采样参考线上；`guiji.py` 在 `AVOID` 状态下每帧生成并设置联合参考轨迹，其他状态清空回原始 `LoopRoute`；新增 `AVOID_REPLAN_COOLDOWN_SECONDS`、`AVOID_REPLAN_MIN_PROGRESS` 和 `avoid_replan_needed()`，当联合轨迹上出现新目标或同目标仍然紧急时重规划，重规划失败且风险很近时切入 `EMERGENCY_BRAKE`；删除 `right_object_yield_done`，右侧目标风险解除后允许后续再次触发 `RIGHT_OBJECT_YIELD`；新增 `RIGHT_OBJECT_CLEAR_HOLD_SECONDS = 2.0s` 连续清空确认，避免右侧目标短暂丢失时过早起步；同步更新本文档中的前车感知参考线、状态切换条件和待提交记录。
- 为什么这样改：只用原始路线投影会让避障过程中的“正在跟踪轨迹”和“风险判断轨迹”不一致；只用自车直线坐标又容易在弯道上误判。联合参考轨迹把当前 MPC 正在跟踪的避障路径作为感知参考，旧目标横向脱离后自然不再触发，而避障路径前方的新目标仍会被检测到。
- 如何验证：已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile .\dazuoye\guiji.py .\dazuoye\control.py .\dazuoye\perception.py .\dazuoye\route.py .\dazuoye\actors.py`，语法检查通过；已运行 `git -C .\dazuoye diff --check`，无空白错误，仅有 Windows 下 LF/CRLF 提示；第一次 `--free-run` 调试发现右侧让行在目标短暂变为 `none` 后过早退出，随后与 `right_side_pedestrian_2` 碰撞；加入连续清空确认后再次运行 `E:/Anaconda_envs/envs/carla_env/python.exe .\dazuoye\guiji.py --free-run`，第一次急停避障在 `5.85s` 触发并于 `12.35s` 完成，第二次弯道避障在 `16.05s` 触发并于 `23.75s` 完成，右侧让行在 `41.30s` 触发、`49.00s` 完成，路线终点 `ROUTE_HOLD` 停车保持完成，最终 `Collisions: 0`、`Cleanup finished`。
- 未覆盖风险：当前 `FrontReferencePath` 使用采样线段和车辆中心点投影，尚未投影车辆四角占据区域；AVOID 中途重规划虽然有冷却和最小进度限制，但本次固定场景没有出现 `Avoidance replanned` 日志，仍需要后续用更密集交通流验证重规划分支；雷达模式默认关闭，尚未验证雷达开启时与临时参考轨迹的关系；pygame 画面观感仍建议人工确认。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`3c10c4a`。
- PR/分支信息：本地待提交，尚未推送。

### 2026-06-10 - 使用样条平滑路线与轨迹投影

- 本次目标：减少弯道避障轨迹由离散 waypoint 和分段线性参考线造成的折线感，并让车辆在路线/轨迹坐标中的位置确定也统一使用平滑参考线投影。
- 主要改动：在 `utils.py` 新增公共 `SmoothRouteReference`，按离散路线点或临时轨迹点计算累计弧长，并使用 `scipy.interpolate.CubicSpline` 拟合 $x(s)$、$y(s)$、$z(s)$；`RouteOffsetLaneChangeTrajectory` 改为从平滑参考线读取位置、切向量和右向法向量；`to_local()` 改为在平滑参考线上搜索投影位置；候选轨迹起点偏移和目标邻道中心偏移也统一使用平滑参考线；`perception.py` 的原始 `LoopRoute` 前车投影改为使用同一平滑参考线的 `route_s/d`，`FrontReferencePath` 也为 AVOID 联合参考轨迹构造平滑参考线；Windows 下直接运行 `python.exe` 时自动补充当前 Conda 环境的 `Library\bin`，避免 scipy/numpy DLL 加载失败；同步更新本文档中的轨迹公式。
- 为什么这样改：单纯加密 waypoint 仍可能保留分段折线方向变化；使用弧长参数化样条后，参考线位置和法向量连续，五次横向偏移叠加到弯道上时更接近连续 Frenet 轨迹。同时，感知层和控制层使用同一类投影口径，可以减少“生成轨迹很平滑，但判断车辆位置仍按折线算”的不一致。
- 如何验证：已通过 `conda install -n carla_env scipy=1.7.3 -y` 安装 scipy；已确认补充 `Library\bin` 后 `scipy 1.7.3` 可导入；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile .\dazuoye\utils.py .\dazuoye\control.py .\dazuoye\perception.py .\dazuoye\guiji.py`，语法检查通过。本次按要求未实跑 CARLA。
- 未覆盖风险：尚未在 pygame/CARLA 画面中人工确认弯道轨迹观感；当前投影搜索是沿样条采样近似最近点，尚未实现严格连续优化投影；`FrontReferencePath` 的平滑参考线由避障轨迹采样点拟合而来，采样密度仍会影响细节。
- 需要 reviewer 重点看的文件：`dazuoye/utils.py`、`dazuoye/control.py`、`dazuoye/perception.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`f392f1a`。
- PR/分支信息：本地待提交，尚未推送。

### 2026-06-12 - 合并前车投影跟踪路线对象

- 本次目标：精简前车感知中的路线对象，避免固定全局路线和避障局部路线各自维护一套投影筛选逻辑。
- 主要改动：`perception.py` 用统一的 `TrackingRoute` 同时包装基础 `LoopRoute` 和 AVOID 阶段的临时避障采样路线；删除 `_route_front_vehicles()`、`_reference_path_front_vehicles()`、`_project_to_route()` 和 `_project_to_reference_path()`，统一为 `_tracking_route_front_vehicles()` 与 `_project_to_tracking_route()`；`guiji.py` 改为在 AVOID 中调用 `set_tracking_route_points()`，其他状态调用 `reset_tracking_route()`；同步更新本文档当前实现说明。
- 为什么这样改：原来“全局路线”和“局部路线”在代码里对应不同对象和不同函数，但核心都是把自车与目标投影到当前正在跟踪的路线。合并后感知层只关心当前 `TrackingRoute`，行为保持一致，后续再做多目标冲突验证时入口更清楚。
- 如何验证：已运行 `git diff --check`；已运行 `E:/Anaconda_envs/envs/carla_env/python.exe -m py_compile guiji.py control.py perception.py route.py actors.py config.py utils.py`。
- 未覆盖风险：本次尚未实跑 CARLA `--free-run`，只验证语法和 diff 空白；雷达开启模式仍保持原有优先级，未单独验证。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：本地未提交。
- PR/分支信息：本地未推送。

### 2026-06-12 - 用 replacement segments 合成路线收敛避障状态机

- 本次目标：按当前两态方案重构前方动态障碍物避障逻辑，让避障轨迹成为基础路线上的局部替换段，而不是独立 `AVOID` 状态或整条路线替换。
- 主要改动：`perception.py` 新增 `ReplacementSegment`，`VirtualGroundTruthSensor` 持有不可整体替换的 `_base_tracking_route`、有效 `_replacement_segments` 和合成 `_tracking_route`；`guiji.py` 删除显式 `AVOID` / `RIGHT_OBJECT_YIELD` / `ROUTE_HOLD` 状态分支，`ROUTE_FOLLOW` 下触发规划、应用 replacement segment、继续基于当前合成路线感知和控制，规划失败但未达到紧急阈值时保持当前路线下一帧重试；`control.py` 将候选轨迹改为左右多偏移 replacement segment，并用候选路径与所有车辆的简化时空冲突检测剔除危险候选；删除不再使用的 `lane_clear()` 硬条件接口。
- 为什么这样改：当前演示需要在避障过程中继续检测“当前避障段 + 后续基础路线”上的车辆，并允许再次规划。把避障作为局部 replacement segment 叠加到基础路线，可以让感知、TTC、最近前车判断、MPC 跟踪和后续重规划都使用同一条当前合成路线；左右候选统一生成后再做冲突筛选，也比先用邻道净空布尔量决定是否规划更贴近动态障碍物避障。
- 如何验证：已运行 `python -m py_compile guiji.py control.py perception.py route.py actors.py display.py utils.py config.py`，语法检查通过；已运行 `git diff --check`，无空白错误，仅有 Windows 下 LF/CRLF 提示；已用 `rg` 检查当前代码中不再存在 `AVOID` 状态分支、`RIGHT_OBJECT_YIELD` 状态分支、`lane_clear()` 调用和旧临时路线切换接口。
- 未覆盖风险：本次未启动 CARLA 实景回归，replacement segment 覆盖/重规划的画面轨迹、候选冲突阈值、右侧让行与背景车交互仍需在 `guiji.py --free-run` 和 1x pygame 演示中继续观察；候选冲突检测仍是车辆中心点级简化包络，尚未投影车辆四角。
- 需要 reviewer 重点看的文件：`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/control.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ea38b26`
- PR/分支信息：已推送至 `origin/feature/decision-control`。

### 2026-06-12 - 避障段保持偏移并增加安全回归候选

- 本次目标：让普通避障段成功后持续保持 `target_offset`，禁止段尾立即拼接 `d=0` 基础路线；超过目标后再通过安全回归候选回到基础路线。
- 主要改动：`control.py` 将 `RouteOffsetLaneChangeTrajectory.avoidance_delta_at()` 改为单段五次 `current_offset -> target_offset` 并保持目标偏移，修正横向斜率计算；新增 `select_return_to_base_trajectory()` 生成 `current_offset -> 0` 回归候选；候选目标偏移包含左右大/小偏移和保持偏移，并用车辆包围盒包络对所有障碍车做冲突硬筛选。`perception.py` 的 `ReplacementSegment` 增加 `end_offset`，合成路线在 segment 后继续采样 `base_route + end_offset`。`guiji.py` 记录 `active_avoidance_target`，在“自车车尾超过障碍物车头 + 2m”后尝试回归，回归不安全则保持当前 offset 下一帧重试。
- 为什么这样改：避障轨迹如果在段尾直接回到基础路线，会把后续路线重新压回障碍物所在走廊，导致候选轨迹与前车重合并反复规划。保持偏移并把回归也作为受碰撞检测约束的候选，可以让绕行和回归都由同一套几何安全逻辑决定。
- 如何验证：已运行 `python -m py_compile guiji.py control.py perception.py route.py actors.py display.py utils.py config.py`，语法检查通过；已运行 `git diff --check`，无空白错误，仅有 Windows 下 LF/CRLF 提示。
- 未覆盖风险：本次未启动 CARLA 实景回归；持续 offset 可能需要结合可视化继续调 `RETURN_TO_BASE_CLEARANCE`、回归长度和冲突包络余量；车辆包络仍按路线投影近似处理，未做完整多边形碰撞检测。
- 需要 reviewer 重点看的文件：`dazuoye/control.py`、`dazuoye/perception.py`、`dazuoye/guiji.py`、`dazuoye/PROGRAM_FRAMEWORK.md`。
- 提交代号/Commit ID：`ea38b26`
- PR/分支信息：已推送至 `origin/feature/decision-control`。
