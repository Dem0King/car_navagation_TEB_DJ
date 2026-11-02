#  自主导航工程

> 一套跑在 `abot` 全向底盘 + 2D 激光雷达 + USB 摄像头上的比赛导航工程。
> 核心目标：**在随机挡板的 40cm 窄通道场地里，稳定地跑完 4 个识别点、4 个任务点和终点，并精确停进任务点。**

- ROS：Noetic · Ubuntu 20.04
- 定位：AMCL（omni 模型）
- 全局规划：`global_planner/GlobalPlanner`（Dijkstra；GridPath/GradientPath 见 8.1.2）
- 局部规划：`teb_local_planner`
- 底盘：全向轮（支持蟹行 / 横移）
- 主控脚本：`navigation_multi_goals.py`（比赛流程）+ `crab_dock.py`（精确对接）

---

## 目录

1. [快速开始](#1-快速开始)
2. [比赛流程](#2-比赛流程)
3. [系统架构](#3-系统架构)
4. [目录结构](#4-目录结构)
5. [运行方法](#5-运行方法)
6. [点位配置](#6-点位配置)
7. [核心模块](#7-核心模块)
8. [优化专题（重点）](#8-优化专题重点)
9. [调参速查表](#9-调参速查表)
10. [日志怎么读](#10-日志怎么读)
11. [已知问题与后续改进](#11-已知问题与后续改进)
12. [部署 / 同步 checklist](#12-部署--同步-checklist)
13. [改动记录](#13-改动记录)

---

## 1. 快速开始

```bash
# 1) 编译（Python 脚本也要保证包能被找到）
cd ~/yjx_ws
catkin_make

# 2) 脚本加执行权限（否则 rosrun 报 Cannot locate node）
chmod +x src/robot_slam/scripts/*.py src/robot_slam/scripts/*.sh

# 3) 先启动底盘 / 雷达 / 相机 / 地图 / AMCL / move_base（见第 5 节）

# 4) 启动比赛主控
roslaunch robot_slam multi_goal.launch

# 5) 等待语音唤醒，说："你好小创"
```

只跑点、不做识别和播报（调试最快）：

```bash
rosrun robot_slam point_check.py            # 图像4点 → 9个任务点 → 终点
rosrun robot_slam point_check.py list       # 只体检：列出所有点位和净空
rosrun robot_slam point_check.py 40         # 只跑任务点 40
rosrun robot_slam point_check.py seq 41,40,33,50
rosrun robot_slam point_check.py --plain    # 纯单次导航，不吸附、不封路、不重试
```

---

## 2. 比赛流程

```
语音唤醒 "你好小创"
   │
   ├─ 播放 bsks.mp3（比赛开始）
   │
   ├─ 图像点1 → VLM 识别 → 播放 1XX.mp3
   ├─ 图像点2 → VLM 识别 → 播放 2XX.mp3
   ├─ 图像点3 → VLM 识别 → 播放 3XX.mp3
   └─ 图像点4 → VLM 识别 → 播放 4XX.mp3
          │
          ▼
   按识别顺序依次前往 4 个任务点（31/32/33/40/41/42/49/50/51）
   每个点：两段式蟹行对接 → 只有真正达标才播报 XX.mp3
          │
          ▼
   前往 9 号终点（不设超时）→ 到达播报 end.mp3
          │
          ▼
   播放 bsjs.mp3（比赛结束）
```

关键行为：

- VLM 单次最多等 `15s`，最多重试 `2` 次；识别失败不会终止比赛，会跳过该图像。
- 任务点**只在对接真正成功时**播报到站语音，避免"没到也算完成"。
- 终点只在 `move_base` 真正返回 `SUCCEEDED` 时播报到达语音。

---

## 3. 系统架构

### 3.1 节点与数据流

```
                    ┌──────────────────────────────┐
 /scan_filtered ───▶│ AMCL (omni)                  │──▶ map→odom TF
                    └──────────────────────────────┘
 /scan_filtered ───▶┌──────────────────────────────┐
 /virtual_wall ────▶│ move_base                    │
                    │  ├─ global_costmap (map,2cm) │──▶ /move_base/global_costmap/costmap(_updates)
                    │  ├─ local_costmap  (odom,2cm)│──▶ /move_base/local_costmap/costmap(_updates)
                    │  ├─ GlobalPlanner (Dijkstra) │
                    │  └─ TEB (polygon footprint)  │
                    └──────────────┬───────────────┘
                                   │ /cmd_vel
                                   ▼
                    ┌──────────────────────────────┐
 /asrpro_node/start ▶│ navigation_multi_goals.py    │
 /abot_vlm_node/*   ▶│  ├─ 比赛流程 / VLM / 音频     │
                    │  ├─ 虚拟墙(封路换路)          │
                    │  ├─ 目标点吸附(snap)          │
                    │  └─ crab_dock(CrabDock)       │──▶ /cmd_vel（对接阶段直发）
                    └──────────────────────────────┘
```

### 3.2 坐标系

| 坐标系 | 用途 |
|---|---|
| `map` | 全局代价地图、AMCL、全局路径、任务点坐标 |
| `odom` | 局部代价地图、TEB、蟹行对接的阶段3闭环 |
| `base_link` | 机器人本体，footprint 定义在这里 |

> ⚠️ 蟹行对接阶段3 **必须在 odom 系闭环**，因为局部代价地图的 `global_frame` 是 `odom`；
> 拿 map 坐标去查 odom 地图会查错位置（map→odom 会随 AMCL 修正漂移）。

### 3.3 机器人 footprint

```yaml
# costmap_common_params.yaml
footprint: [[0.215, 0.145], [0.215, -0.145], [-0.135, -0.145], [-0.135, 0.145]]
footprint_padding: 0.0
```

- 车体约 `0.35m × 0.29m`，内切半径 `0.135m`，外接半径 `0.262m`。
- **代价地图和 TEB 的 footprint 必须完全一致**，改一个要改两个：
  - `params/carto/costmap_common_params.yaml`
  - `params/carto/teb_local_planner_params.yaml` 的 `footprint_model.vertices`

---

## 4. 目录结构

```
src/
├── robot_slam/                 ★ 本工程核心
│   ├── launch/
│   │   ├── multi_goal.launch          比赛主控（当前实际使用）
│   │   ├── competition.launch         旧的一键启动模板（见 11.3）
│   │   ├── positioning.launch         地图 + AMCL
│   │   ├── record_points.launch       录点
│   │   └── include/
│   │       ├── move_base.launch.xml
│   │       ├── amcl.launch.xml
│   │       └── amcl_positioning.launch.xml
│   ├── params/carto/
│   │   ├── costmap_common_params.yaml     footprint / 观测源 / 虚拟墙
│   │   ├── global_costmap_params.yaml     全局代价地图 + 膨胀参数
│   │   ├── local_costmap_params.yaml      局部代价地图（3m 滚动窗口）
│   │   ├── global_planner_params.yaml     全局规划器
│   │   ├── teb_local_planner_params.yaml  TEB（调参重点）
│   │   ├── move_base.yaml                 频率 / 恢复行为 / make_plan
│   │   └── dwa_local_planner_params.yaml  未使用（保留）
│   ├── scripts/
│   │   ├── navigation_multi_goals.py      比赛主控
│   │   ├── crab_dock.py                   两段式蟹行对接
│   │   ├── point_check.py                 跑点/体检工具
│   │   └── set_pose.sh                    发布 /initialpose
│   ├── maps/
│   │   ├── my_lab.yaml
│   │   └── my_lab.pgm                     2cm 分辨率，736×736
│   ├── rviz/
│   └── src/navigate.cpp
├── abot_base/                 底盘相关（外部依赖的 abot_bringup 不在此仓库）
├── abot_sr/                   语音识别（asrpro）
└── abot_vision/               VLM 视觉识别
```

> 仓库根目录还有两份历史资料：`优化cpu.md`（CPU 采样分析）、`问题报告.txt`（多轮实车日志）。

---

## 5. 运行方法

### 5.1 依赖

| 依赖 | 说明 |
|---|---|
| `abot_bringup` | 底盘驱动、URDF/TF、雷达（不在本仓库，机器人上已装） |
| `amcl` / `move_base` / `map_server` | ROS Navigation |
| `teb_local_planner` / `global_planner` | 局部 / 全局规划 |
| `usb_cam` | 摄像头 |
| `abot_sr` / `abot_vision` | 语音唤醒 / VLM |
| `mplayer` | 语音播报 |

### 5.2 启动顺序

```bash
# ① 底盘 + 雷达 + 相机 + 地图 + AMCL + move_base
#    按机器人自带的 bringup 启动（本仓库的 competition.launch 是模板，见 11.3）

# ② 比赛主控
roslaunch robot_slam multi_goal.launch

# ③ 检查启动日志
#    [对接] 初始化完成: footprint 4 个顶点, 总安全余量 0.025m
#    [吸附] make_plan 服务已就绪
#    等待语音唤醒，请说：你好小创
```

### 5.3 只导航不识别

```bash
rosrun robot_slam point_check.py list        # 先体检所有点位
rosrun robot_slam point_check.py --no-finish # 跑全流程但不跑终点
```

`point_check.py` 会打印每个点的：

- 到真障碍的净空（`_clearance`，从致死格量，不把膨胀带算进去）
- 到达/失败、用时、落点误差

### 5.4 手动设置初始位姿

```bash
rosrun robot_slam set_pose.sh 0.55 -0.15 90
```

---

## 6. 点位配置

点位**只在 `navigation_multi_goals.py` 里维护一处**（`point_check.py` 直接复用）。

### 6.1 图像识别点

| 序号 | x | y | yaw(°) |
|---|---|---|---|
| 1 | 1.526 | -0.287 | 84.26 |
| 2 | 2.941 | -1.567 | -0.01 |
| 3 | 1.557 | -2.845 | -86.41 |
| 4 | 0.364 | -1.541 | 179.12 |

### 6.2 任务点（3×3 网格）

| | x≈1.22 | x≈1.6 | x≈2.0 |
|---|---|---|---|
| **y≈-1.14** | 31 (1.221,-1.135) | 32 (1.634,-1.140) | 33 (2.025,-1.135) |
| **y≈-1.51** | 40 (1.221,-1.507) | 41 (1.574,-1.514) | 42 (1.980,-1.544) |
| **y≈-1.94** | 49 (1.184,-1.928) | 50 (1.592,-1.946) | 51 (2.044,-1.941) |

- 点距约 `0.37~0.40m`，通道很窄，挡板是随机摆的。
- 同一列的 x 最多相差 6cm —— 点位是人工示教的，**不是理想网格**。这是"点被挡板挤偏"的根源之一。

### 6.3 终点

```
9号终点 = (3.202, 0.051, -3.1°)
```

---

## 7. 核心模块

### 7.1 `navigation_multi_goals.py` —— 比赛主控

职责：

| 功能 | 关键实现 |
|---|---|
| 语音唤醒 | 轮询 `/asrpro_node/start != 255` |
| 音频播报 | `mplayer`，失败自动重试 2 次，播放前 `pkill mplayer` 防止声卡被占 |
| VLM 识别 | 写 `/abot_vlm_node/im_flag=1`，等待 `im_flog` 变成合法任务号且时间戳更新 |
| 普通导航 | `goto_with_timeout`（25s）/ `goto_no_timeout`（终点） |
| 走不通换路 | 虚拟墙封路 + 换落点（`goto_with_fallback`） |
| 目标点吸附 | `make_plan` 预检 + 按净空换落点 |
| 任务点对接 | 交给 `crab_dock.CrabDock` |
| 全局代价地图副本 | 订阅 `costmap` + `costmap_updates`，本地维护网格 |

### 7.2 `crab_dock.py` —— 两段式全向蟹行对接

**为什么不让 move_base 直接开到任务点？**

- 任务点周围挡板随机、点位示教有偏差，全局规划器只按"点 + 膨胀"判断可达，不保证车体多边形真的进得去；
- TEB 在极窄处容易振荡、报 `trajectory is not feasible`；
- 对接要求位置误差 ≤3cm、朝向误差 ≤3.4°，move_base 的默认容差做不到。

**四阶段：**

```
阶段0 距离预检
  dist(cur, P) > 0.35m ? 阶段1 : 跳过阶段1
        │
阶段1 八方向搜索 + 预对接
  在目标周围 8 个方向、0.25m 处取预对接点 AP
  要求 AP→P 的整段扫掠安全
  用 move_base 导航到 AP
  落点必须在 [0.10, 0.45]m 内，且实际走廊复检通过
        │  找不到 → 5cm 内微调目标位姿再试（见 8.7.5）
        │  仍找不到 → 失败，绝不直冲目标
        ▼
阶段2 对接前安全确认（用实际位姿）
  cur→P 整段扫掠 + 目标点本身能否停下车
        │
阶段3 20Hz 闭环蟹行
  世界系方向 + yaw 闭环
  位置 ≤3cm 且 yaw ≤3.4°，连续 5 个周期 → 成功
  每个周期用局部地图做 0.35s 前瞻动态防撞
        │
阶段4 结果返回（成功才播报到站）
```

关键常量：

| 常量 | 值 | 含义 |
|---|---|---|
| `DOCK_DIST` | 0.25m | 预对接点到目标点的距离 |
| `DOCK_SPEED` | 0.10m/s | 最终对接速度 |
| `POS_TOL` / `YAW_TOL` | 0.03m / 0.06rad | 最终容差 |
| `STABLE_N` | 5 | 连续达标周期数 |
| `DOCK_HZ` | 20Hz | 闭环频率 |
| `AP_NAV_TIMEOUT` | 25s | 导航到 AP 的超时 |
| `DIR_RETRY` | 1 | 首选方向失败后再试 1 个 |
| `LOOKAHEAD_T` | 0.35s | 动态防撞前瞻时间 |
| `MARGIN` | **0.025m** | 静态/动态安全余量（见 8.7.3） |
| `YAW_SWEEP` | **0.06rad** | 静态扫掠的 yaw 包络（见 8.7.4） |

### 7.3 `point_check.py` —— 跑点 / 体检工具

- 复用主脚本的点位和导航逻辑，只去掉 VLM 与语音。
- `list`：只算净空，判断"车停得下"（门槛 `SNAP_MIN_CLEAR=0.17m`）。
- `--plain`：不吸附、不封路、不重试，最能暴露点位本身的问题。
- 默认带吸附，但**不发虚拟墙**，避免封路影响后续判断。

---

## 8. 优化专题（重点）

这一节按"为什么改、改成什么、代价是什么"记录所有优化。

### 8.1 全局代价地图 / 全局规划器

#### 8.1.1 膨胀半径 0.25 → 0.10

```yaml
# global_costmap_params.yaml
resolution: 0.02
inflation_radius: 0.10
cost_scaling_factor: 3.5
```

**问题**：40cm 走廊，两侧各 0.135m 硬阻挡带 → 可走宽度只剩 0.13m，挡板被激光标厚一点、拐角两侧同时挤，全局规划器就报 `Failed to get a plan`。上一版曾整场 218s 里有 100s 浪费在失败的点上。

**改动**：`inflation_radius: 0.25 → 0.10`，可走宽度变成 0.16m，目标点可达门槛从 9.5cm 降到 6.5cm。

**为什么仍安全**：

- 全局规划器本来就是"点机器人 + 膨胀"的简化模型；
- 真正保证离墙距离的是 TEB：polygon footprint + `min_obstacle_dist`；
- 全局路径只管"走哪条廊道"，TEB 允许偏离全局路径（`global_plan_viapoint_sep=-1`、`weight_viapoint=1`）；
- 万一全局选了一条物理上过不去的路，主控的虚拟墙会封路逼它换。

> 注意：`inflation_radius (0.10) < 内切半径 (0.135)` 时，膨胀带内的格子会发布为 `99`（内切膨胀），
> 全局规划器把它当不可通行；但 `crab_dock._min_clear` **只把 `>=100`（致死格）当障碍**，
> 所以对接测的是到真障碍的距离，不是到膨胀带边缘的距离。

#### 8.1.2 GridPath / GradientPath 的取舍

```yaml
GlobalPlanner:
  use_dijkstra: true
  use_quadratic: true
  use_grid_path: false        # 当前值
  default_tolerance: 0.15
  allow_unknown: true
  lethal_cost: 253
  neutral_cost: 50
  cost_factor: 3.0
```

- `true` = GridPath（沿网格边逐格回溯），`false` = GradientPath（沿负梯度下降）。
- 文件注释记录：**GradientPath 在"高代价平台区"（=膨胀带里）会原地打转**，日志刷
  `NO PATH!` / `Failed to get a plan from potential when a legal potential was found`；
  机器人基本一直站在膨胀带边上 → 这个失效容易必现。GridPath 不需要梯度收敛，不会出这个问题。
- 当前文件里 `use_grid_path` 仍是 `false`。如果实车日志出现上述两条报错，
  第一件事就是把它改成 `true`。若当前版本跑得很稳，保持 `false` 也可以（路径更平滑）。
- `default_tolerance: 0.50 → 0.15`：配合两段式停靠，全局规划需要更准确地到 AP。
- `use_dijkstra: true`：比 A* 稳定。

### 8.2 局部代价地图 / TEB

#### 8.2.1 局部地图 2.0×2.0 → 3.0×3.0

```yaml
# local_costmap_params.yaml
global_frame: odom
rolling_window: true
width: 3.0
height: 3.0
resolution: 0.02
inflation_radius: 0.0
```

- 原 2.0m → 只有 ±1.0m，而 TEB 的 `max_global_plan_lookahead_dist=1.5`，局部目标会跑到地图外；
- 3.0m 后前后左右各 ±1.5m，刚好覆盖 TEB 的前瞻。
- `inflation_radius: 0.0`：局部地图只负责"障碍在哪里"，离墙距离交给 TEB 的 `min_obstacle_dist` / `inflation_dist`。每调大 1cm，等于每面墙在 TEB 眼里胖 1cm，窄缝直接顶死。

#### 8.2.2 `inflation_dist: 0.15 → 0.13`

```yaml
min_obstacle_dist: 0.03
inflation_dist: 0.13
weight_obstacle: 40
weight_inflation: 1.0
```

- `inflation_dist` 必须显著大于 `min_obstacle_dist`，TEB 才会建立 `EdgeInflatedObstacle`；
  原来 0.02/0.03 只差 1cm，`weight_inflation` 等于没生效。
- 0.15 在 40cm 走廊里**永远无法满足**（车边离墙只有 5cm），两侧永久惩罚 → 优化器算出的轨迹通不过可行性检查 → 313 条 `trajectory is not feasible`。
- 0.13 能过，但会蹭着障碍过——在单边余量 2.5~5cm、AMCL 定位误差本身 2~5cm 的走廊里，刮蹭是物理极限的必然结果，"能过"优先。

#### 8.2.3 优化器迭代 5/4 → 7/5

```yaml
no_inner_iterations: 7
no_outer_iterations: 5
```

- 共 20 → 35 次迭代，收敛更好，`not feasible` 明显减少。
- 开销约 1.75 倍，靠 8.8 的 CPU 优化腾出来。

#### 8.2.4 速度与前瞻

```yaml
max_vel_x: 0.40
max_vel_x_backwards: 0.12
max_vel_y: 0.35
max_vel_theta: 1.00
acc_lim_x: 0.8 / acc_lim_y: 0.7 / acc_lim_theta: 1.20
max_global_plan_lookahead_dist: 1.5
xy_goal_tolerance: 0.08
yaw_goal_tolerance: 0.12
```

- 全程全向底盘：`min_turning_radius=0`，`weight_kinematics_nh=0`，`weight_kinematics_forward_drive=0`。
- move_base 只需要到 AP（0.25m 外），容差 0.08/0.12 足够；精确对接交给 `crab_dock`。
- `dt_ref` 当前 0.25，注释记录了 `0.25 → 0.18` 的细化方向：轨迹时间分辨率更细、窄处更贴合，但更耗 CPU。
- 刮蹭还想再少：按顺序试 `weight_optimaltime 1.5 → 1.0`、`max_vel_x 0.40 → 0.30`、`max_vel_theta 1.0 → 0.6`（注释里已记录，但当前文件仍是 1.5）。

#### 8.2.5 放弃同伦类规划

```yaml
enable_homotopy_class_planning: false
enable_multithreading: true
max_number_classes: 2
```

- 同伦类规划机制本身生效（日志有 `Parallel planning in distinctive topologies enabled`），
  但算力扛不住：控制周期被拖到 0.2~2.0s，车收不到指令，连直缝都过不去。
- 降到 2 类同样不行 → 改用**虚拟墙封路（8.5）**，让全局规划器换路。

### 8.3 定位（AMCL）

```xml
<param name="odom_model_type" value="omni"/>
<param name="min_particles" value="1500"/>   <!-- 1000 → 1500 -->
<param name="max_particles" value="3000"/>
<param name="update_min_d" value="0.02"/>    <!-- 0.03 → 0.02 -->
<param name="update_min_a" value="0.03"/>    <!-- 0.05 → 0.03 -->
<param name="laser_max_beams" value="80"/>
<param name="laser_model_type" value="likelihood_field"/>
<param name="transform_tolerance" value="1.0"/>
```

- 粒子下限抬高、滤波更新更频繁 → 定位更贴合，减少"因为定位漂而蹭"。
- 全向底盘必须用 `odom_model_type=omni`。
- **定位误差 2~5cm 是窄处刮蹭的隐形推手**：即使所有规划参数都对，定位偏 3cm 也会让"物理上 5cm 的余量"变成 2cm。
  想要更保守，可在 `crab_dock.py` 把 `LOC_MARGIN` 从 0 加到 0.01~0.02。

### 8.4 move_base 频率 / 恢复 / 超时

```yaml
controller_frequency: 30.0      # 20 → 30
controller_patience: 10.0
planner_frequency: 3.0
planner_patience: 5.0
oscillation_timeout: 12.0       # 5.0 → 12.0
oscillation_distance: 0.10
recovery_behaviors:
  - conservative_reset          # 只保留这一条
conservative_reset:
  reset_distance: 3.0
  layer_names: ["obstacle_layer"]
```

- 30Hz → 每 1.3cm 一条速度指令（20Hz 是 2cm），指令之间的"盲走"更短，窄处更不容易蹭。
- 默认 4 条恢复行为会烧 60s+；只保留一次"清代价地图"。
- `oscillation_timeout: 5 → 12`：原值太急，车还在慢慢蹭就被判振荡 → 触发恢复 → abort。
- `conservative_reset.reset_distance=3.0`：静态地图里没有挡板，全靠激光实时标；清太狠等于把挡板记忆忘掉，所以保留 3m 内不清。

#### 8.4.1 `make_plan` 的两个开关

```yaml
make_plan_clear_costmap: false
make_plan_add_unreachable_goal: false
```

这两个是"吸附触发判据"的前提：

- `make_plan_clear_costmap: true` 会先把机器人周围 0.52m 的地图清成自由空间，
  把"被膨胀盖住的目标点"洗干净 → 服务谎报可达；
- `make_plan_add_unreachable_goal: true` 会把够不着的原目标追加到路径末尾，分不清真正的可达点。

### 8.5 走不通就封路（虚拟墙，方案 B）

代码：`seal_ahead()` / `clear_walls()` / `goto_with_fallback()`

```yaml
VW_AHEAD_DIST = 0.30      # 在车头前方 0.30m 处封路
VW_GRID_STEP  = 0.15      # 3×3 点阵 ≈ 0.3m 见方
SEAL_MIN_GOAL_DIST = 0.37 # 封路点离目标太近就不封（否则把目标围死）
```

流程：

1. 图像点第一次 25s 没走到 → `seal_ahead(goal)` 在车头前方打 3×3 虚拟墙；
2. 等代价地图标记生效 → 换落点重试一次；
3. 每一次新导航开始前 `clear_walls()` 清掉上一腿的封路（封路是"这一腿走不通"的证据，留着会累积把后面的路堵死）。

实现细节：

- 虚拟墙是 `PointCloud2`（`/virtual_wall`），`marking: true, clearing: false`；
- `clear_walls()` 发空点云后**会回读代价地图确认真的清掉了**，最多等 2s；
- 如果日志出现 `发了空点云, 但代价地图上仍有封路点没消失` → 去 RViz 看 `/move_base/global_costmap/costmap`，必要时重启 move_base。

> 任务点和终点**不发虚拟墙**：任务点由 `crab_dock` 负责，终点是最后一腿，封路只会帮倒忙。

### 8.6 目标点吸附（snap）

代码：`snap_goal_if_unreachable()` / `snap_goal()`

**两个角色一定要分清：**

| 角色 | 判据 |
|---|---|
| **要不要动** | 问规划器：`make_plan` 原目标点可达吗？可达 → 绝不干预 |
| **挪到哪里** | 用**净空**，不能用"规划器说可达" |

为什么不能用规划器选点：规划器判可达的门槛只有 `(硬阻挡带 − 0.04) ≈ 6~10cm`，
而车半宽是 0.14m —— 一个离挡板 8cm 的点，规划器说能到，车物理上根本停不进去。

参数：

```python
SNAP_RADIUS      = 0.15   # 搜索半径；40cm 格距下挡板都在离任务点 0.19m 以外，不会吸到隔壁点
SNAP_STEP        = 0.02   # = 地图分辨率
SNAP_MIN_CLEAR   = 0.17   # = 车半宽 0.14 + TEB min_obstacle_dist 0.03
SNAP_FLOOR_CLEAR = 0.14   # 兜底：低于这个值车物理上停不下
```

行为：

- 目标点净空 ≥0.17 → 原样返回；
- 否则在 0.15m 圆内由近到远找最近的达标点；
- 找不到达标点 → 用净空最大的点（≥0.14）；
- 再不行 → 原目标点，日志明确打印"最大净空"。

### 8.7 两段式蟹行精确对接（本项目最大的优化）

#### 8.7.1 为什么拆成两段

| 直接 move_base 到点的问题 | 两段式怎么解决 |
|---|---|
| TEB 在极窄处振荡、`not feasible` | 只在 AP 处让 move_base 负责，最后 0.25m 由 20Hz 闭环直发速度 |
| 全局规划只按"点 + 膨胀"判断，车体不一定进得去 | 阶段1 用 **footprint 多边形** 做 8 方向走廊扫掠 |
| 到达精度只有 8~12cm | 阶段3 闭环到 3cm / 3.4° |
| 对接过程中可能撞到新出现的障碍 | 阶段3 每周期用**局部地图**做 0.35s 前瞻防撞 |

#### 8.7.2 静态 / 动态地图分离

```python
_pick_map(mode):
    mode='static'  → 全局代价地图（map 系）
    mode='dynamic' → 局部代价地图（odom 系），过期就直接停车，不回退全局
```

- 阶段1/2 的走廊可能在 2~3m 外，局部滚动窗口只有 ±1.5m，会误判"安全" → 必须用全局图；
- 阶段3 是实时防撞，必须用局部图；局部图过期就停车，不能拿 1~2 秒前的数据继续走。

#### 8.7.3 安全余量标定史（⚠️ 最重要的调参记录）

```python
MARGIN = CLEAR_MIN + RES_MARGIN + LOC_MARGIN + YAW_MARGIN
```

| 版本 | MARGIN | 实车结果 |
|---|---|---|
| 按规格字面 | ≈0.17m | 每个方向都不安全，对接永远失败 |
| 初版 | **0.04m** | 40cm 通道能过，但差 2~4mm 就误杀 |
| 2026-10-18 第一轮 | **0.03m** | 40 实测 0.038、33 实测 0.036，仍被 0.03 判死（第二轮跑出 0.028 / 0.023） |
| 2026-10-18 第二轮 | **0.025m** | 当前值；配合 yaw 包络收窄 + 5cm 微调落点 |

当前值：

```python
CLEAR_MIN  = 0.015   # 车体轮廓到真障碍的最低几何净空
RES_MARGIN = 0.010   # 半格，覆盖"到格心量距"的量化误差
LOC_MARGIN = 0.000   # 定位不确定性；想保守可加 0.01~0.02
YAW_MARGIN = 0.000
MARGIN     = 0.025
```

> ⚠️ `MARGIN=0.025` 是"实测到障碍格心"的距离，障碍真实表面可能再近半格（≈1cm）。
> 车宽 0.29m 在 0.37~0.40m 通道里单边物理余量约 4~5cm，这个值仍在物理可过范围内，
> 但已经是"能过"而不是"宽裕"。**实车若出现擦挡板，先把 `CLEAR_MIN` 加回 0.02。**

#### 8.7.4 yaw 包络 0.12 → 0.06

```python
YAW_SWEEP = 0.06   # ±3.4°
```

- 0.12rad（±6.9°）时，车体在 y 方向被撑胖约 2.5cm：
  `0.215*sin(6.9°) + 0.145*cos(6.9°) − 0.145 ≈ 0.025m`。
- 实测任务点33唯一的 -45° 走廊就卡在 **+7° 包络采样**上（净空 0.023），正常朝向其实够。
- 对接时 yaw 是闭环的，控制器保证 `|yaw_err| ≤ YAW_TOL = 0.06rad`；阶段3 还会用**实际位姿 + 实际指令转向**做动态扫掠兜底。
- 静态预检只是"这条路大概能不能过"，用 ±`YAW_TOL` 就够，不该额外放大车体。

#### 8.7.5 5cm 内微调落点（随机挡板的救援）

```python
DOCK_SNAP_RADIUS   = 0.05   # 最大微调距离
DOCK_SNAP_STEP     = 0.01   # 搜索步长
DOCK_SNAP_TRY      = 2      # 每一圈最多试几个候选
DOCK_SNAP_MAX_TRIES= 4      # 整个搜索最多试几个候选
```

触发条件：

- 阶段1 八方向全堵；
- 或阶段2 直查失败（常见于跳过阶段1、离目标只剩 0.35m）。

做法：

1. 在目标点周围 1~5cm 的圆环上采样；
2. 先要求**位姿本身站得下**（目标点净空 ≥MARGIN）；
3. 按圈由近到远，圈内按**真实净空从大到小**排序（用 `cap=0.30` 量，避免都被截成 MARGIN 导致排序失效）；
4. 对候选跑完整八方向搜索，找到第一个有安全走廊的就用；
5. 只动位置**不动朝向**——车体 0.35×0.29，一转投影只会更宽；
6. 成功日志打印：`微调 Xcm: (原坐标) → (新坐标)`；最终再打印相对标称点的偏移。

> 背景：任务点 3×3 网格的点距 0.37~0.40m，同一列 x 最多差 6cm（人工示教），
> 随机挡板一挤，点就可能落在"车体离挡板只剩 2~3cm"的位置。原地挪 2~5cm 就能停进去。

#### 8.7.6 坐标系 / 时效 / 故障处理

| 机制 | 说明 |
|---|---|
| map→odom 每周期重算 | AMCL 随时可能修正 map→odom，用开机那次的旧坐标会让物理位置偏掉 |
| 局部地图时效 `MAP_AGE_LOCAL=0.5s` | 过期立刻停车，不回退全局图 |
| 全局地图时效 `MAP_AGE_GLOBAL=1.6s` | 它是 1Hz 发布的，不能要求太新 |
| TF 连续失败 `TF_FAIL_MAX=5` | 第一次失败就发零速度，连续 5 次退出 |
| `finally` 强制停车 | 成功/失败/异常/中断都保证发零速度 |

### 8.8 CPU / 系统层优化

来自 `优化cpu.md` 的实测采样：

```
%Cpu(s): 46.9 us, 9.9 sy, 42.9 id      ← 只有 43% 空闲
  %CPU  进程
  225.0  rviz          ← 2.25 个核
  100.0  whoopsie      ← Ubuntu 崩溃上报，纯垃圾
  100.0  gnome-shell   ← 桌面
  100.0  abot_driver   ← 驱动刷屏
```

**不改进任何导航参数就能腾出 5~6 个核：**

| 动作 | 腾出 | 风险 |
|---|---|---|
| `sudo systemctl stop whoopsie && sudo systemctl disable whoopsie` | 1 核 | 零 |
| RViz 用完就关（或挪到另一台电脑 / `nice -n 15`） | 2.25 核 | 零 |
| 驱动刷屏改成写日志 | ~1 核 | 零 |
| 跑点模式不起 `usb_cam` / VLM / ASR | 1.5~2 核 | 零 |
| AnyDesk 不用时断开 | 0.44 核 | 零 |

腾出 CPU 后才敢上：

- `controller_frequency 20 → 30`
- AMCL 粒子 1000 → 1500、更新 0.03/0.05 → 0.02/0.03
- TEB 迭代 5/4 → 7/5

### 8.9 音频 / VLM 容错

- **音频**：`mplayer` 可能被残留进程占声卡导致秒退 → 播放前 `pkill -f mplayer`，失败自动重试 2 次，并打印每次耗时。
- **VLM**：15s 超时 + 2 次重试；只接受 `31/32/33/40/41/42/49/50/51` 且时间戳比触发前新的结果；无效值只做限频告警。
- **唤醒**：`/asrpro_node/start != 255`，读到后立刻清回 255，避免重复触发。

---

## 9. 调参速查表

### 9.1 导航走不通 / 卡顿

| 现象 | 先看 | 调整方向 |
|---|---|---|
| `Failed to get a plan` | 全局代价地图、膨胀带 | 全局 `inflation_radius` 已 0.10，不建议再大 |
| `trajectory is not feasible` | TEB `inflation_dist` / 迭代次数 | `inflation_dist` 已 0.13；迭代 7/5；再不行降速 |
| `Control loop missed` | CPU | 关 RViz/whoopsie，检查跑导航时的 `top` |
| 车在原地慢慢蹭被判振荡 | `oscillation_timeout` | 已从 5 放宽到 12 |
| 某条缝永远走不通 | 虚拟墙 | 确认 `/virtual_wall` 有数据、代价地图能看到 |

### 9.2 对接失败

| 日志 | 含义 | 调整 |
|---|---|---|
| `八个方向都没有安全走廊` | 目标位姿/周围通道太窄 | 看 `@ (x,y,角度)`；必要时 `CLEAR_MIN` 微调 |
| `阶段2: cur→P 走廊不安全` | 直查路径被挡 | 现在会自动走 5cm 微调落点 |
| `阶段2: 目标点本身停不下车` | 目标位姿 footprint 放不下 | 微调落点；或检查点位是否示教偏了 |
| `阶段1导航失败` | AP 规划/到达失败 | 换方向；检查 AP 是否被膨胀盖住 |
| `局部代价地图缺失/过期` | 局部图不发布 | 查 `/move_base/local_costmap/costmap` 与 CPU |
| `前方 0.35s 的扫掠区域不安全` | 动态防撞触发 | 看 `@ (x,y,角度)`，确认是真障碍还是定位漂 |

### 9.3 净空相关

| 参数 | 位置 | 作用 |
|---|---|---|
| `SNAP_MIN_CLEAR` | nav 主控 | 吸附合格净空（点中心到障碍） |
| `CLEAR_MIN/RES_MARGIN` | crab_dock | 对接安全余量（footprint 到障碍） |
| `min_obstacle_dist` | TEB | 轨迹优化期望净空 |
| `inflation_dist` | TEB | 膨胀惩罚带宽度 |
| `inflation_radius` | 全局代价地图 | 全局规划硬阻挡带 |

---

## 10. 日志怎么读

### 10.1 正常对接

```
[对接] 任务点41: 距目标 1.24m, 目标 (1.574, -1.514, -2.3°)
[对接] 方向  +90°: 走廊通过, AP 净空 0.300m(排序用)
[对接] 有 8 个安全方向, 首选 +90°(净空 0.300m)
[对接] 方向 +90° 阶段1成功: 距目标 0.26m, 走廊净空 0.025m
[对接] 阶段2 通过: 走廊净空 0.025m, 目标点净空 0.025m
[对接] 闭环开始(odom系): 目标 (...) ← map系 (...)
[对接] 达标 1/5 (dist 0.027, yaw_err 0.002)
...
[对接] 对接成功: dist 0.017m, yaw_err 0.005rad, 用时 3.1s
[对接] 任务点41 成功: 最终 dx=... dist=0.004 ...
```

### 10.2 关键诊断字段

| 字段 | 含义 |
|---|---|
| `最小净空 0.0xxm @ (x, y, θ°)` | 扫掠中第一次低于阈值的点；θ 是叠加 yaw 包络后的测试朝向 |
| `微调 Xcm: (a,b) → (c,d)` | 原坐标没有走廊，落点被微调了 |
| `落点相对标称点偏移` | 最终停的位置离示教点有多远 |
| `达标 n/5` | 连续达标计数；中断说明在容差边缘振荡 |
| `state=3` | `GoalStatus.SUCCEEDED` |
| `state=4` | `ABORTED`（规划/控制失败） |
| `state=2` | `PREEMPTED`（被取消/超时） |

### 10.3 常用诊断命令

```bash
# 谁在吃 CPU（一定要在车跑导航时取）
top -b -n1 -o %CPU | head -15

# TEB 可行性 / CPU 卡顿计数
grep -c "not feasible"        ~/.ros/log/最新/move_base-*.log
grep -c "Control loop missed" ~/.ros/log/最新/move_base-*.log

# 定位漂不漂
rosrun tf tf_echo map odom

# 看全局代价地图（RViz）
#   Add → Costmap2D → /move_base/global_costmap/costmap
#   重点确认挡板有没有被标上、虚拟墙有没有清掉
```

---

## 11. 已知问题与后续改进

### 11.1 9 号终点偶发"规划器到不了"

实车日志：

```
[吸附] 规划器到不了 (3.20, 0.05), 启用吸附换落点
[吸附] (3.20, 0.05) 周围 0.15m 内找不到能停车的点(最大净空 0.14m), 用原目标点
```

- 终点附近最大净空只有 0.14m，低于 `SNAP_MIN_CLEAR=0.17`，吸附放弃；
- `goto_no_timeout` 会一直尝试，但被 `Ctrl+C` 打断，无法判断最终能否到达；
- **建议**：终点也加一次"小范围、低门槛"的落点搜索（参考 `crab_dock` 的 5cm 微调思路），或把终点的合格净空按实际场地单独放宽。

### 11.2 虚拟墙清不掉

`virtual_wall` 是 `clearing: false`，发空点云只是"不再标注"，格子要靠激光射线或代价地图重置才会消失。
`clear_walls()` 会回读确认并告警：

```
[封路] 发了空点云, 但代价地图上仍有封路点没消失 → 去 RViz 确认...
```

处理：RViz 确认；必要时重启 `move_base`。跑点模式可以加 `--plain` 避免封路。

### 11.3 `competition.launch` 已过时

- 它启动的节点是 `competition_main.py`，但当前脚本名是 `navigation_multi_goals.py`；
- 它依赖 `abot_bringup`（不在本仓库）；
- 当前实际使用 `multi_goal.launch`（只启动主控节点，导航栈由机器人自带 bringup 启动）。

若要用一键启动，需要把节点 `type` 改成 `navigation_multi_goals.py`，并确认 `abot_bringup` 存在。

### 11.4 `multi_goal.launch` 里的旧参数

`goalListX/Y/Yaw` 是旧版主控用的，当前 `navigation_multi_goals.py` 不读取它们；点位在脚本里维护。
保留不影响运行，但别误以为改 launch 里的数就能改点位。

### 11.5 `amcl_positioning.launch.xml` 有坏路径

里面的 `robot_localization` 加载 `$(find your_pkg)/config/ekf_params.yaml`，`your_pkg` 不存在。
比赛定位走 `amcl.launch.xml`，不使用这个文件；如果要用，先修路径。

### 11.6 八方向 + 固定 0.25m AP 的局限

- 只有 8 个方向、固定距离；如果随机挡板留下的开口不在这些方向上，仍可能找不到走廊；
- 目前靠 5cm 微调落点缓解；
- 后续可选：每 22.5° 搜索、AP 距离 0.25/0.35/0.45 多档。

### 11.7 定位误差是刮蹭的根因之一

- 单边余量 2.5~5cm，AMCL 误差 2~5cm，刮蹭是物理极限的必然；
- 想更保守：`LOC_MARGIN=0.01~0.02`、降速、或继续调 AMCL；
- 但 `LOC_MARGIN` 调大会重新触发"窄通道判死"，要配合实车验证。

### 11.8 `dwa_local_planner_params.yaml` 未使用

当前局部规划器是 TEB；DWA 文件是历史遗留，改它没有效果。

---

## 12. 部署 / 同步 checklist

- [ ] `chmod +x src/robot_slam/scripts/*.py src/robot_slam/scripts/*.sh`
- [ ] `catkin_make` 通过，`rospack find robot_slam` 正常
- [ ] 地图路径正确：`maps/my_lab.yaml` 的 `image:` 是绝对路径，必须和机器人上的实际路径一致
- [ ] 音频文件在 `/home/abot/yuyin/`：`bsks.mp3`、`<图序号><任务号>.mp3`（如 `141.mp3`、`450.mp3`）、`<任务号>.mp3`（如 `41.mp3`）、`end.mp3`、`bsjs.mp3`
- [ ] 启动后确认：`总安全余量 0.025m`、`make_plan 服务已就绪`
- [ ] 跑前体检：`rosrun robot_slam point_check.py list`
- [ ] RViz 确认：全局代价地图有挡板、虚拟墙已清、AMCL 粒子云收敛
- [ ] 跑点模式关掉 RViz / usb_cam / VLM / ASR，腾 CPU
- [ ] 同步 `crab_dock.py` 后，确认机器人上跑的是新版本（看启动日志里的余量值）

---

## 13. 改动记录

| 日期 | 模块 | 改动 |
|---|---|---|
| 2026-10-05 实测 | 全局代价地图 | `inflation_radius 0.25 → 0.10`，解决 40cm 走廊 `Failed to get a plan` |
| 参数调优阶段 | 全局规划器 | `default_tolerance 0.50 → 0.15`；GridPath/GradientPath 取舍见 8.1.2 |
| 参数调优阶段 | TEB | `inflation_dist 0.15 → 0.13`、迭代 5/4 → 7/5、`weight_optimaltime 1.5 → 1.0`（注释记录） |
| 参数调优阶段 | move_base | 控制 20 → 30Hz、振荡超时 5 → 12s、恢复行为只留 conservative_reset |
| 参数调优阶段 | AMCL | 粒子 1000 → 1500、更新 0.03/0.05 → 0.02/0.03 |
| 参数调优阶段 | 主控 | 虚拟墙封路（方案 B）、`make_plan` 吸附触发判据 |
| 2026-10-18 | crab_dock | 总安全余量 0.040 → 0.030（量化余量整格 → 半格） |
| 2026-10-18 | crab_dock | 最小净空位置写入失败日志 `@ (x,y,θ)` |
| 2026-10-18 | crab_dock | 修复 `_maybe_snap_ap` 度/弧度混用 |
| 2026-10-18 | crab_dock | 总余量 0.030 → 0.025；`YAW_SWEEP 0.12 → 0.06`；新增 5cm 微调落点；`goto_task_point` 阶段2失败后自动走 AP 重试 |

---

## 附：关键文件速查

| 想改什么 | 改哪里 |
|---|---|
| 点位 / 超时 / 封路 / 吸附 | `src/robot_slam/scripts/navigation_multi_goals.py` 顶部常量与 `CompetitionController` |
| 对接安全余量 / 速度 / 容差 | `src/robot_slam/scripts/crab_dock.py` 顶部常量 |
| 全局规划 / 膨胀 | `src/robot_slam/params/carto/global_costmap_params.yaml`、`global_planner_params.yaml` |
| 局部规划 / 窄缝通过 | `src/robot_slam/params/carto/teb_local_planner_params.yaml` |
| 频率 / 恢复 / make_plan | `src/robot_slam/params/carto/move_base.yaml` |
| footprint | `costmap_common_params.yaml` + `teb_local_planner_params.yaml`（两处必须一致） |
| 定位 | `src/robot_slam/launch/include/amcl.launch.xml` |
| 地图 | `src/robot_slam/maps/my_lab.yaml` / `my_lab.pgm` |