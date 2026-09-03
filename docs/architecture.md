# 系统架构设计文档

## 1. 设计目标

构建一个模块化、可扩展、可测试的智能机器人软件系统，覆盖从环境感知到运动执行的完整链路，满足毕业设计对系统完整性与可演示性的要求。

## 2. 分层架构

系统采用五层分层架构，自顶向下：

### 2.1 应用层 (Application Layer)
- **职责**：人机交互、远程管理、状态监控
- **组件**：`RobotCLI`（交互式命令行）、`RobotHTTPServer`（HTTP REST + Web 控制台）
- **接口**：通过 `Robot` 统一接口下发命令、查询状态

### 2.2 决策层 (Decision Layer)
- **职责**：任务解析、状态调度、行为决策
- **组件**：`StateMachine`（有限状态机）、`Mission`（多航点任务）
- **状态流转**：
  ```
  IDLE → NAVIGATE ⇄ AVOID → TASK → RETURN → DONE → IDLE
            ↓
          ERROR
  ```

### 2.3 感知层 (Perception Layer)
- **职责**：环境感知、目标识别、障碍探测
- **组件**：`ObjectDetector`（可插拔目标检测）、`LidarSimulator`（激光雷达扫描）
- **输出**：检测结果列表（`Detection`）、激光扫描（`LidarScan`）

### 2.4 执行层 (Execution Layer)
- **职责**：运动控制、电机驱动
- **组件**：`DifferentialDrive`（差分驱动运动学）、`PID`（PID 控制器）
- **输入**：期望 (v, w)；**输出**：左右轮角速度

### 2.5 数据支撑层 (Data Support Layer)
- **职责**：通信中间件、配置管理、日志、数据记录
- **组件**：`MessageBus`（发布/订阅）、`load_config`（配置加载）、`Recorder`（JSONL 记录）、`DatasetBuilder`（数据集划分）

### 2.6 导航层 (Navigation Layer) — 横切
- **职责**：建图、定位、路径规划、避障
- **组件**：`GridMap`（占用栅格）、`AStarPlanner`（A* 规划）、`PotentialFieldController`（人工势场避障）、`OdometryLocalizer`（里程计定位）

## 3. 模块间通信

采用发布/订阅消息总线解耦：

| 主题 | 发布者 | 订阅者 | 数据 |
|---|---|---|---|
| `perception/detections` | 检测器 | 决策层 | `Detection[]` |
| `perception/lidar` | 激光雷达 | 避障控制 | `{nearest, ranges}` |
| `navigation/goal` | HMI | 规划器 | `{x, y, path_len}` |
| `motion/state` | 运动控制 | 记录器 | `{pose, v, w, state}` |
| `system/start` | Robot | 全部 | `{timestamp}` |

## 4. 控制周期

每个 `update(dt)` 周期执行：

1. **感知**：激光雷达扫描 → 最近障碍距离；目标检测
2. **决策**：根据上下文更新状态机
3. **规划**：NAVIGATE 状态下跟踪路径点，PID 控制航向
4. **避障**：AVOID 状态下人工势场计算 (v, w)
5. **执行**：运动学积分更新位姿
6. **记录**：写入 JSONL 日志

## 5. 可扩展性

- **检测器可插拔**：实现 `ObjectDetector.detect()` 即可接入新模型
- **定位可替换**：实现 `update()` 返回 `Pose` 即可接入 AMCL/SLAM
- **地图格式**：`GridMap` 可扩展为 PGM/标准地图加载
- **通信中间件**：`MessageBus` 可替换为 ROS 2 DDS
