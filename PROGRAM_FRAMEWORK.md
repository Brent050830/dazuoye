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
- 虚拟传感器。
- TTC 计算。
- 基于 `LoopRoute` 真实路线叠加避障起点局部右向五次横向增量的避障轨迹。
- 采样式 MPC 跟踪，当前主流程使用路线相对轨迹代价计算。
- Town10 固定起点。
- Town10 固定短路线。
- 路线进度与一圈完成判断。
- 右转路线事件检测与右转前靠右准备。
- 5 辆沿固定路线行驶的背景车辆，速度使用固定随机种子生成并设置上限。
- `R344 -> R20` 右转处右侧关键非机动车直行目标。
- 3 辆 R344 右侧背景自行车，速度各异。
- 前方车辆和右侧非机动车虚拟感知支持关键目标与背景目标共同参与风险判断。
- `RIGHT_OBJECT_YIELD` 减速让行。
- 路线完成后的 `ROUTE_HOLD` 停车保持。
- pygame 摄像头显示。
- 碰撞监测。

### 4.1 当前模块职责

```text
config.py       场景、路线、风险阈值和控制参数
utils.py        通用数学、车辆速度和道路辅助函数
perception.py   虚拟真值感知、前车与右侧目标风险读取
control.py      真实路线叠加局部五次偏移避障轨迹和采样式 MPC 跟踪
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
RIGHT_OBJECT_YIELD_SPEED = 3.0
RIGHT_OBJECT_R344_ANCHOR_BACK_STEPS = 6
RIGHT_OBJECT_R344_RIGHT_OFFSET = 3.0
RIGHT_OBJECT_R344_START_FORWARD_OFFSET = -8.0
RIGHT_OBJECT_R344_END_FORWARD_OFFSET = 22.0
TRAFFIC_RANDOM_SEED = 20260602
BACKGROUND_VEHICLE_ROUTE_INDICES = (32, 54, 76, 96, 108)
BACKGROUND_VEHICLE_SPEED_MIN = 7.0
BACKGROUND_VEHICLE_SPEED_MAX = 8.8
BACKGROUND_BICYCLE_FORWARD_OFFSETS = (-42.0, -30.0, -18.0)
BACKGROUND_BICYCLE_RIGHT_OFFSETS = (4.4, 1.8, 5.6)
BACKGROUND_BICYCLE_END_FORWARD_OFFSET = 36.0
BACKGROUND_BICYCLE_SPEED_MIN = 2.6
BACKGROUND_BICYCLE_SPEED_MAX = 4.3
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
- 路线长度约 `520m`，不再绕最外侧大圈；到达路线终点后进入 `ROUTE_HOLD` 收尾观察模式，方向盘回正、油门为 0、刹车为 1，直接停车并保持 `4s` 后结束仿真，避免继续追踪最后一个 waypoint 导致回摆。
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

### 8.1 前方车辆感知

继续保留：

- 前车距离。
- 相对速度。
- TTC。
- 横向偏移。
- 是否为本车道前方车辆。

当前 `perception.py` 中前车感知使用自车坐标系计算。设：

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

当前只认为满足以下条件的目标是“同车道前方车辆”：

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

当前 `FrontVehicleReading` 已保留 `actor_id` 和 `actor_role`，可用于区分 `lead`、慢速车和背景车角色；现阶段这些字段仅作为诊断信息和后续扩展入口，不参与避障触发条件或控制量计算。

### 8.2 右侧非机动车感知

当前第一版字段：

```python
@dataclass
class RightSideObjectReading:
    distance: float
    ttc: float
    is_conflict_object: bool
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

后续再升级为更稳定的轨迹预测和路口区域判定。

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
-8.0 \le s_{\mathrm{right}} \le 34.0
$$

$$
-14.0 \le d_{\mathrm{right}} \le 18.0
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

$$
\mathrm{emergency\_needed}
= \mathrm{front.is\_front\_vehicle}
\land
\left(\mathrm{front.distance} < \mathrm{SAFE\_DISTANCE}\right)
\land
\left(\mathrm{front.ttc} < \mathrm{TTC\_AVOID\_THRESHOLD}\right)
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
```

如果 `emergency_needed` 成立，程序再调用 `sensor.lane_clear("left")` 和 `sensor.lane_clear("right")` 判断邻道是否可用于换道。

邻道净空判断使用前后安全窗口：

$$
-\mathrm{LANE\_CLEAR\_REAR}
\le s_{\mathrm{neighbor}}
\le \mathrm{LANE\_CLEAR\_FRONT}
$$

当前参数：

```text
LANE_CLEAR_REAR = 18.0 m
LANE_CLEAR_FRONT = 45.0 m
```

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
RIGHT_OBJECT_YIELD_SPEED = 3.0 m/s
```

进入 `RIGHT_OBJECT_YIELD` 后，自车目标速度降为 `RIGHT_OBJECT_YIELD_SPEED`。如果右侧目标距离进一步小于 `RIGHT_OBJECT_STOP_DISTANCE`，则制动至少提升到：

$$
\mathrm{brake} = \max(\mathrm{brake},\ 0.85)
$$

$$
\mathrm{throttle} = 0.0
$$

风险评估层应输出给行为决策层：

```python
risk_type
risk_level
recommended_action
```

## 10. 行为决策层

行为决策层负责根据风险选择驾驶行为。

当前代码中的状态机已经包含前车避障和右侧非机动车第一版让行状态：

```text
ROUTE_FOLLOW
AVOID
EMERGENCY_BRAKE
RIGHT_OBJECT_YIELD
ROUTE_HOLD
```

### 10.0 当前代码实际状态切换条件

当前 `guiji.py` 中状态变量初始化为：

$$
\mathrm{state}_0 = \mathrm{ROUTE\_FOLLOW}
$$

风险布尔量定义见 `9.1` 和 `9.2`。邻道避障方向由 `choose_avoidance_side(sensor)` 给出：

$$
\mathrm{avoidance\_side}
=
\begin{cases}
\mathrm{left}, & \mathrm{lane\_clear(left)} \\
\mathrm{right}, & \neg\mathrm{lane\_clear(left)} \land \mathrm{lane\_clear(right)} \\
\mathrm{None}, & \text{otherwise}
\end{cases}
$$

当前状态切换条件如下。

| 状态 | 进入条件 | 退出条件 |
| --- | --- | --- |
| `ROUTE_FOLLOW` | 初始状态；`AVOID` 完成后回到该状态；`RIGHT_OBJECT_YIELD` 风险解除后也回到该状态。 | 若 $\mathrm{emergency\_needed}$ 且 $\mathrm{avoidance\_side}\ne\mathrm{None}$，进入 `AVOID`；若 $\mathrm{emergency\_needed}$ 且 $\mathrm{avoidance\_side}=\mathrm{None}$，进入 `EMERGENCY_BRAKE`；若 $\mathrm{right\_object\_risk}$ 且 $\neg\mathrm{right\_object\_yield\_done}$，进入 `RIGHT_OBJECT_YIELD`；若已完成路线并记录 `route_completion_time`，进入 `ROUTE_HOLD`。 |
| `AVOID` | 当前状态为 `ROUTE_FOLLOW`，且 $\mathrm{emergency\_needed}$ 成立，并且相邻车道存在可用避障方向；或 `EMERGENCY_BRAKE` 中风险仍存在但邻道重新可用。进入时基于 `loop_route.last_index`、目标邻道和车道宽度生成 `RouteOffsetLaneChangeTrajectory`。 | 若换道进度满足 $s_{\mathrm{traj}} > L_{\mathrm{lanechange}} + 2.0$ 且 $|d_{\mathrm{traj}} - D| < 0.65$，回到 `ROUTE_FOLLOW`；若 `route_completion_time` 已记录，进入 `ROUTE_HOLD`。 |
| `EMERGENCY_BRAKE` | 当前状态为 `ROUTE_FOLLOW`，且 $\mathrm{emergency\_needed}$ 成立，但左右邻道均不可用。 | 若 $\mathrm{emergency\_recovered}$ 成立，回到 `ROUTE_FOLLOW`；若 $\mathrm{emergency\_needed}$ 仍成立但 $\mathrm{avoidance\_side}\ne\mathrm{None}$，重新生成避障轨迹并进入 `AVOID`；否则继续保持全制动。路线完成、碰撞、窗口关闭或仿真时间结束仍会提前终止主循环。 |
| `RIGHT_OBJECT_YIELD` | 当前状态为 `ROUTE_FOLLOW`，且 $\mathrm{right\_object\_risk}$ 成立，并且 `right_object_yield_done == False`。 | 若 $\neg\mathrm{right\_object\_risk}$，回到 `ROUTE_FOLLOW`，并设置 `right_object_yield_done = True`；若 `route_completion_time` 已记录，进入 `ROUTE_HOLD`。 |
| `ROUTE_HOLD` | `loop_route.update(ego_vehicle)` 判断完成一圈后，主循环记录 `route_completion_time`；下一轮控制计算中进入 `ROUTE_HOLD`。 | 保持停车控制，直到 $t_{\mathrm{sim}} - t_{\mathrm{route\_completion}} \ge \mathrm{ROUTE\_COMPLETION\_HOLD\_SECONDS}$ 后跳出主循环并清理。 |



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

后续如果继续细化右转避障，可以在 `ROUTE_FOLLOW` 基础上继续拆出更细的阶段状态：

```text
ROUTE_FOLLOW
FRONT_EMERGENCY_BRAKE
FRONT_AVOIDANCE
RIGHT_TURN_APPROACH
RIGHT_OBJECT_YIELD
RIGHT_OBJECT_AVOIDANCE
RECOVER_TO_ROUTE
FINISHED
```

### 10.1 ROUTE_FOLLOW

默认巡航状态。

动作：

- 跟踪环形路线。
- 保持目标速度。
- 持续检测前方车辆和右侧非机动车。

### 10.2 FRONT_EMERGENCY_BRAKE

前方风险较高但不适合转向时进入。

动作：

- 油门为 0。
- 制动增大。
- 保持或轻微修正方向。
- 打开紧急制动灯。

### 10.3 FRONT_AVOIDANCE

前车急停且存在安全避让空间时进入。

动作：

- 生成紧急避障轨迹。
- MPC 跟踪轨迹。
- 同时保持必要制动。
- 避障后回到环形路线。

### 10.4 RIGHT_TURN_APPROACH

接近右转弯区域时进入。

动作：

- 降低目标速度。
- 加强右侧目标检测。
- 准备右转路径跟踪。

### 10.5 RIGHT_OBJECT_YIELD

右侧非机动车有冲突风险，但通过减速可以解决时进入。

动作：

- 主动制动或低速滑行。
- 等待非机动车通过。
- 保持转向轨迹不过度靠右。

### 10.6 RIGHT_OBJECT_AVOIDANCE

右侧非机动车风险更高，需要制动和转向共同避让时进入。

动作：

- 降低速度。
- 调整右转轨迹，使自车避开非机动车。
- 必要时扩大转弯半径。
- 风险解除后恢复路线。

### 10.7 RECOVER_TO_ROUTE

避障结束后的恢复状态。

动作：

- 重新寻找环形路线上的目标 waypoint。
- 平滑回到正常路线跟踪。
- 恢复目标速度。

### 10.8 FINISHED

完成一圈后进入。

动作：

- 减速停车。
- 输出评价结果。
- 清理 actor 和传感器。

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

当前主流程进入 `AVOID` 时使用 `RouteOffsetLaneChangeTrajectory`。它不再把整段避障轨迹固定成一条直线，而是保留 `LoopRoute` 的真实路线点作为道路弯曲基线，再沿每个路线点对应的道路右方向叠加五次多项式横向避障增量。

注意：当前横向偏移不是简单固定为一个车道宽。程序使用避障开始处路线点的道路右向量计算目标邻道中心偏移，并在轨迹上使用每个路线点自己的道路右向量叠加横向增量。这样弯道上横向偏移方向会随道路旋转，而不是固定在触发瞬间的自车右方向。

五次横向偏移仍为：

$$
d(s) = D\left(10t^3 - 15t^4 + 6t^5\right)
$$

$$
t = \frac{s}{L}
$$

用于快速完成横向避障。

设避障开始时的路线索引为：

$$
i_0 = \texttt{loop\_route.last\_index}
$$

路线离散参考点为 $P_i$，路线步长为 $\Delta s$。对避障轨迹纵向进度 $s$，对应的路线索引为：

$$
i(s) = i_0 + \frac{s}{\Delta s}
$$

实际代码中使用相邻路线点线性插值得到道路参考点：

$$
P_{\mathrm{route}}(s)
= (1-\alpha)P_{\lfloor i(s)\rfloor}
+ \alpha P_{\lfloor i(s)\rfloor+1}
$$

其中：

$$
\alpha = i(s)-\lfloor i(s)\rfloor
$$

路线点对应的道路右向单位向量为：

$$
r_{\mathrm{route}}(s)
$$

目标邻道中心横向偏移：

$$
d_1 = \operatorname{dot}_{2D}
\left(
p_{\mathrm{target\_lane}}-P_{\mathrm{route}}(0),\
r_{\mathrm{route}}(0)
\right)
$$

避障开始时车辆当前横向偏移为：

$$
d_0 = \operatorname{dot}_{2D}
\left(
p_{\mathrm{ego}}-P_{\mathrm{route}}(0),\
r_{\mathrm{route}}(0)
\right)
$$

五次曲线的避障横向增量为：

$$
d_{\mathrm{avoid}}(s)
= d_0 + (d_1-d_0)b(t)
$$

最终避障参考点为：

$$
P_{\mathrm{ref}}(s)
= P_{\mathrm{route}}(s)
+ d_{\mathrm{avoid}}(s) r_{\mathrm{route}}(s)
$$

其中：

- `i_0` 为进入 `AVOID` 时的 `loop_route.last_index`。
- `P_route(s)` 为真实路线上的插值点。
- `r_route(s)` 为路线点对应的道路右向单位向量。
- `d_0` 为进入 `AVOID` 时自车相对避障起点路线点的道路右向距离。
- `d_1` 为目标邻道中心相对避障起点路线点的道路右向距离。
- `L` 为换道纵向长度，当前根据前方目标距离动态取值：

$$
L = \max\left(14.0,\ \min\left(\texttt{LANE\_CHANGE\_LENGTH},\ d_{\mathrm{front}}+4.0\right)\right)
$$

`RouteOffsetLaneChangeTrajectory.to_local()` 使用当前车辆位置到最近路线点的弧长差作为进度，并使用最近路线点的道路右向量计算车辆相对真实路线的横向避障量：

$$
d_{\mathrm{local}}
= \operatorname{dot}_{2D}
\left(
p-P_{\mathrm{nearest\ route}},\
r_{\mathrm{route}}(s_{\mathrm{nearest}})
\right)
$$

五次平滑函数：

$$
b(t) = 10t^3 - 15t^4 + 6t^5
$$

$$
d_{\mathrm{avoid}}(s) = d_0 + (d_1-d_0)b(t),\quad t = \frac{s}{L}
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
= \frac{(d_1-d_0)b'(t)}{L}
$$

$$
\psi_{\mathrm{ref}}
= \psi_{\mathrm{route}}(s)
+ \arctan\left(\frac{d d_{\mathrm{avoid}}}{ds}\right)
$$

当前路线相对轨迹的实际代码使用最终参考点的有限差分计算参考航向，使参考航向与叠加后的真实轨迹一致：

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

旧的 `QuinticLaneChangeTrajectory` 直线局部轨迹类已从 `control.py` 删除；当前 `guiji.py` 的 `AVOID` 和 `EMERGENCY_BRAKE -> AVOID` 都生成 `RouteOffsetLaneChangeTrajectory`。后续如果需要旧直线局部轨迹，可从 Git 历史恢复。

### 11.3 弯道避障轨迹

弯道避障不再使用触发瞬间固定直角坐标系生成整段轨迹。当前采用 `RouteOffsetLaneChangeTrajectory`：

```text
LoopRoute 真实路线 + 每个路线点道路右方向五次横向增量
```

这样做的目的：

- 保留道路本身的转弯方向。
- 当右侧邻道可用时，先保留真实路线本身的弯曲，再把五次避障增量沿路线点自身的道路右方向叠加。
- 弯道第二次避障与直道第一次避障共用同一套 `AVOID` 状态和 MPC 跟踪接口。
- 避障完成后回到 `ROUTE_FOLLOW`，继续沿固定路线行驶。

当前右侧非机动车/行人冲突仍主要通过 `RIGHT_OBJECT_YIELD` 减速或停车让行处理，尚未单独生成右转转向避让轨迹。

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

采样式 MPC 使用简化运动学自行车模型。历史上的直线局部轨迹分支已删除；当前主流程使用后文的 `RouteOffsetLaneChangeTrajectory` 全局坐标预测分支。

直线局部轨迹版本曾使用如下局部坐标积分形式，当前仅作为公式背景保留：

$$
v_{k+1} = \max(0,\ v_k + a\Delta t)
$$

$$
s_{k+1} = s_k + v_{k+1}\cos(\psi_k)\Delta t
$$

$$
d_{k+1} = d_k + v_{k+1}\sin(\psi_k)\Delta t
$$

$$
\psi_{k+1}
= \operatorname{normalize\_angle}
\left(
\psi_k + \frac{v_{k+1}}{\mathrm{WHEEL\_BASE}}\tan(\delta)\Delta t
\right)
$$

当前轴距参数：

$$
\mathrm{WHEEL\_BASE} = 2.85\ \mathrm{m}
$$

每个预测步的误差：

$$
e_d = d_k - d_{\mathrm{ref}}(s_k)
$$

$$
e_{\psi}
= \operatorname{normalize\_angle}(\psi_k - \psi_{\mathrm{ref}}(s_k))
$$

$$
e_v = v_k - v_{\mathrm{target}}
$$

单步代价当前为：

$$
J_k =
6.0e_d^2
+ 1.7e_{\psi}^2
+ 0.07e_v^2
+ 0.08\delta^2
+ 0.01a^2
+ 0.02k\left|\delta-\delta_{\mathrm{prev}}\right|
$$

一个候选动作的总代价：

$$
J = \sum_{k=0}^{N-1} J_k
$$

对于当前主流程使用的 `RouteOffsetLaneChangeTrajectory`，MPC 改为在全局坐标中预测：

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

为了避免车辆在 `AVOID` 中已经低速时继续选择负加速度并原地停住，若满足：

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
14. 自车检测最近前方车辆、TTC、邻道净空和所有右侧非机动车风险
15. 自车在前方车辆风险触发后执行紧急制动和转向避障
16. 自车在右侧非机动车风险触发后进入 `RIGHT_OBJECT_YIELD` 减速让行
17. 避障/让行完成后继续跟踪 `LoopRoute`
18. 到达路线终点后进入 `ROUTE_HOLD`
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
- `RIGHT_OBJECT_YIELD` 减速让行状态。

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
- 提交代号/Commit ID：待提交。
- PR/分支信息：尚未推送；当前为待提交代码与文档更新记录。
