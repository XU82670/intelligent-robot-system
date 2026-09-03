# API 参考

## Robot（系统集成）

```python
from intelligent_robot.robot import Robot

robot = Robot(config_path="config/config.json")
robot.start()
robot.goto(2.0, 3.0)
for _ in range(200):
    robot.update(dt=0.1)
print(robot.status())
robot.stop()
```

### 方法

| 方法 | 说明 |
|---|---|
| `Robot(config_path=None, config=None)` | 构造，加载配置并组装全部模块 |
| `start()` | 启动系统，复位状态机与定位 |
| `stop()` | 停止系统，关闭记录器 |
| `goto(x, y) -> bool` | 导航到目标点，返回是否规划成功 |
| `load_mission(mission)` | 加载多航点任务 |
| `update(dt=0.1) -> dict` | 执行一个控制周期，返回状态 |
| `status() -> dict` | 返回当前状态快照 |
| `add_wall(x0, y0, x1, y1)` | 在地图中添加矩形障碍（仿真用） |

### status() 返回

```json
{
  "state": "NAVIGATE",
  "started": true,
  "pose": {"x": 1.2, "y": 0.8, "theta": 0.5},
  "goal": [2.0, 3.0],
  "waypoints_remaining": 15,
  "nearest_obstacle": 1.25,
  "velocity": {"v": 0.6, "w": 0.2}
}
```

## 核心类

### AStarPlanner
```python
from intelligent_robot.navigation.path_planner import AStarPlanner
planner = AStarPlanner(grid)
path = planner.plan(start_cell, goal_cell)       # 栅格坐标
path = planner.plan_xy((0.5, 0.5), (2.0, 2.0))  # 世界坐标
```

### PID
```python
from intelligent_robot.motion_control.pid import PID
pid = PID(kp=1.2, ki=0.05, kd=0.15, output_limits=(-1.5, 1.5))
output = pid.update(error)
pid.reset()
```

### DifferentialDrive
```python
from intelligent_robot.motion_control.kinematics import DifferentialDrive
dd = DifferentialDrive(wheel_radius=0.05, wheel_base=0.30)
left, right = dd.to_wheel_speeds(v=0.5, w=0.3)   # 逆运动学
v, w = dd.from_wheel_speeds(left, right)           # 正运动学
new_pose = dd.integrate(pose, v, w, dt)            # 位姿积分
```

### StateMachine
```python
from intelligent_robot.decision.state_machine import StateMachine, DecisionContext
fsm = StateMachine()
ctx = DecisionContext(has_goal=True, goal_reached=False)
state = fsm.update(ctx)  # 自动推进
fsm.transition("NAVIGATE")  # 显式转换
```

### MessageBus
```python
from intelligent_robot.core.message_bus import MessageBus
bus = MessageBus()
bus.subscribe("topic", callback)
bus.publish("topic", data)
```

### Recorder
```python
from intelligent_robot.data_manager.recorder import Recorder
rec = Recorder("logs")
rec.record("motion/state", {"x": 1.0, "y": 2.0})
rec.close()
```

### DatasetBuilder
```python
from intelligent_robot.data_manager.dataset import DatasetBuilder
ds = DatasetBuilder(records)
split = ds.split(train=0.7, val=0.2, test=0.1, stratified=True)
print(ds.stats())
```

## HTTP API

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | `/` | Web 控制台页面 |
| GET | `/status` | 返回机器人状态 JSON |
| POST | `/command` | 下发命令 `{"cmd":"goto","args":{"x":1,"y":2}}` |
