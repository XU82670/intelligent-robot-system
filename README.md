# 智能机器人系统设计与实现

> 毕业设计项目：基于分层架构的智能机器人软件系统，覆盖感知、导航、运动控制、任务决策、人机交互、数据管理六大模块。

本项目对应答辩 PPT《智能机器人系统设计与实现》，采用**纯 Python 标准库**实现核心算法，开箱即用、可直接运行仿真与测试，无需安装任何第三方依赖。

---

## 系统架构

采用五层分层架构，各层职责清晰、通过标准接口协同：

```
┌─────────────────────────────────────────────────┐
│  应用层 (hmi)        CLI 交互 / HTTP 远程控制    │
├─────────────────────────────────────────────────┤
│  决策层 (decision)   有限状态机 / 任务调度       │
├─────────────────────────────────────────────────┤
│  感知层 (perception) 目标识别 / 激光雷达扫描     │
├─────────────────────────────────────────────────┤
│  执行层 (motion)     差分驱动运动学 / PID 控制   │
├─────────────────────────────────────────────────┤
│  数据支撑层 (core, data_manager)                 │
│  消息总线 / 配置管理 / 日志 / 数据记录 / 数据集  │
└─────────────────────────────────────────────────┘
         导航层 (navigation)：栅格地图 / A*规划 / 避障 / 定位
```

---

## 模块说明

| 模块 | 路径 | 核心功能 |
|---|---|---|
| 感知 | `intelligent_robot/perception/` | 目标检测（颜色/YOLO 可插拔）、激光雷达射线投射扫描 |
| 导航 | `intelligent_robot/navigation/` | 占用栅格地图、A* 路径规划、人工势场避障、轮式里程计定位 |
| 运动控制 | `intelligent_robot/motion_control/` | 差分驱动正/逆运动学、PID 控制器（抗积分饱和） |
| 决策 | `intelligent_robot/decision/` | 有限状态机（IDLE→NAVIGATE→AVOID→TASK→RETURN→DONE）、多航点任务 |
| 人机交互 | `intelligent_robot/hmi/` | 交互式 CLI、HTTP REST API + Web 控制台 |
| 数据管理 | `intelligent_robot/data_manager/` | JSONL 数据记录、数据集构建与分层划分 |
| 系统集成 | `intelligent_robot/robot.py` | Robot 顶层类，组装全部模块，统一 start/stop/goto/update/status |

---

## 快速开始

### 环境要求

- Python 3.9+（核心功能纯标准库，**无需 pip install**）
- 可选：`numpy` + `opencv-python`（启用真实相机目标检测）

### 运行仿真演示

```bash
python main.py --sim
```

输出示例：

```
step=  0  state=NAVIGATE  pose=(0.52,0.50)  v=0.80 w=0.00
step= 20  state=NAVIGATE  pose=(1.85,0.62)  v=0.72 w=0.31
...
仿真结束: success=True, steps=120, final=(4.42,4.48), dist_to_goal=0.105m
```

### 交互式 CLI

```bash
python main.py
robot> start
robot> goto 2 3
robot> status
robot> stop
robot> quit
```

### HTTP 远程控制

```bash
python main.py --http --port 8080
```

打开浏览器访问 `http://127.0.0.1:8080`，可在 Web 控制台启动/停止机器人、下发导航目标、实时查看状态。

REST 接口：

```bash
# 查看状态
curl http://127.0.0.1:8080/status

# 下发命令
curl -X POST http://127.0.0.1:8080/command \
  -H "Content-Type: application/json" \
  -d '{"cmd":"goto","args":{"x":2.0,"y":3.0}}'
```

### 运行单元测试

```bash
python -m unittest discover -s tests -v
```

覆盖：A* 路径规划、PID 控制、差分驱动运动学、状态机、消息总线、数据集划分、端到端仿真。

---

## 配置说明

配置文件：`config/config.json`

| 配置项 | 说明 | 默认值 |
|---|---|---|
| `motion.wheel_radius` | 轮半径 (m) | 0.05 |
| `motion.wheel_base` | 轮距 (m) | 0.30 |
| `motion.max_linear_speed` | 最大线速度 (m/s) | 0.8 |
| `motion.pid` | PID 参数 {kp, ki, kd} | 1.2 / 0.05 / 0.15 |
| `perception.detector` | 检测器类型 mock/color/yolo | mock |
| `perception.lidar_rays` | 激光雷达射线数 | 36 |
| `navigation.grid_resolution` | 栅格分辨率 (m/cell) | 0.1 |
| `navigation.avoid_range` | 避障触发范围 (m) | 0.8 |
| `decision.goal_tolerance` | 到达目标容差 (m) | 0.15 |
| `hmi.http_port` | HTTP 服务端口 | 8080 |

---

## 项目结构

```
intelligent-robot-system/
├── intelligent_robot/          # 核心包
│   ├── core/                   # 数据支撑层：消息总线、配置、日志
│   ├── perception/             # 感知层：目标检测、激光雷达
│   ├── navigation/             # 导航层：栅格地图、A*、避障、定位
│   ├── motion_control/         # 执行层：运动学、PID
│   ├── decision/               # 决策层：状态机、任务
│   ├── hmi/                    # 应用层：CLI、HTTP
│   ├── data_manager/           # 数据管理：记录、数据集
│   └── robot.py                # 系统集成：Robot 顶层类
├── simulator/                  # 2D 世界仿真器
├── tests/                      # 单元测试 + 端到端测试
├── config/                     # 配置文件（系统配置、任务航点）
├── docs/                       # 设计文档
├── main.py                     # 系统入口
├── pyproject.toml              # 项目元数据
├── requirements.txt            # 可选依赖
├── Dockerfile                  # 容器化部署
├── LICENSE
└── README.md
```

---

## 核心算法

### A* 路径规划
在占用栅格地图上使用 A* 算法（曼哈顿启发函数），支持 8 邻域移动与对角穿墙检测，规划出从起点到目标的最短无碰路径。

### PID 控制
增量式 PID 控制器，带输出限幅与抗积分饱和，用于航向闭环控制。

### 人工势场避障
目标吸引力 + 障碍排斥力合成虚拟力，大角度差时原地转向、小角度差时减速前进，实现局部动态避障。

### 差分驱动运动学
正运动学（左右轮速度 → 机器人 v,w）与逆运动学（v,w → 左右轮速度），支持位姿积分。

### 有限状态机
事件驱动 + 上下文自动推进，状态：IDLE → NAVIGATE ⇄ AVOID → TASK → RETURN → DONE，支持进入/退出回调。

---

## 扩展与对接真实硬件

本项目为软件系统框架，对接真实机器人时：

1. **感知层**：将 `MockDetector` 替换为 `SimpleColorDetector`（OpenCV）或实现 `YoloDetector.detect()`，接入真实相机；`LidarSimulator` 替换为真实激光雷达驱动（如 RPLIDAR）。
2. **执行层**：`DifferentialDrive.to_wheel_speeds()` 输出的左右轮角速度下发给电机驱动（如串口/PWM）。
3. **定位层**：`OdometryLocalizer` 替换为 AMCL/SLAM（如 ROS 2 `nav2_amcl`），接口保持 `update()` 返回 `Pose`。
4. **通信**：`MessageBus` 可替换为 ROS 2 DDS，主题命名保持一致。

---

## License

MIT License
