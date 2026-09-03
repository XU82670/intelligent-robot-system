"""应用层：交互式命令行 HMI。对应 PPT「应用层-人机交互」。

RobotCLI 提供 REPL：start / goto x y / status / stop / quit。
可在终端直接控制机器人，也可被 HTTP 服务复用。
"""
from __future__ import annotations

import shlex
from typing import Optional


class RobotCLI:
    """机器人交互式命令行。

    命令：
      start            启动系统
      goto <x> <y>     导航到目标点
      status           查看当前状态
      stop             停止并复位
      quit             退出
    """

    def __init__(self, robot) -> None:
        self.robot = robot
        self.running = False

    def help_text(self) -> str:
        return (
            "可用命令:\n"
            "  start            启动系统\n"
            "  goto <x> <y>     导航到目标点 (米)\n"
            "  status           查看当前状态\n"
            "  stop             停止并复位\n"
            "  quit             退出\n"
        )

    def handle(self, line: str) -> str:
        """处理一行命令，返回输出文本。"""
        line = line.strip()
        if not line:
            return ""
        try:
            parts = shlex.split(line)
        except ValueError as exc:
            return f"命令解析错误: {exc}"
        cmd = parts[0].lower()

        if cmd in ("help", "?"):
            return self.help_text()
        if cmd == "start":
            self.robot.start()
            return "系统已启动。"
        if cmd == "goto":
            if len(parts) < 3:
                return "用法: goto <x> <y>"
            try:
                x, y = float(parts[1]), float(parts[2])
            except ValueError:
                return "坐标必须是数字。"
            ok = self.robot.goto(x, y)
            return f"已下发导航目标 ({x}, {y})，规划{'成功' if ok else '失败（无路径）'}。"
        if cmd == "status":
            return self._format_status(self.robot.status())
        if cmd == "stop":
            self.robot.stop()
            return "系统已停止。"
        if cmd == "quit":
            self.running = False
            return "再见。"
        return f"未知命令: {cmd}（输入 help 查看帮助）"

    def run(self) -> None:
        """启动 REPL 循环（阻塞）。"""
        self.running = True
        print("智能机器人系统 CLI（输入 help 查看帮助）")
        while self.running:
            try:
                line = input("robot> ")
            except (EOFError, KeyboardInterrupt):
                print()
                break
            out = self.handle(line)
            if out:
                print(out)

    @staticmethod
    def _format_status(s: dict) -> str:
        pose = s.get("pose", {})
        return (
            f"状态: {s.get('state', '?')}\n"
            f"位姿: x={pose.get('x', 0):.2f} y={pose.get('y', 0):.2f} "
            f"theta={pose.get('theta', 0):.2f} rad\n"
            f"目标: {s.get('goal', '无')}\n"
            f"路径点剩余: {s.get('waypoints_remaining', 0)}\n"
            f"最近障碍: {s.get('nearest_obstacle', 'inf')} m"
        )
