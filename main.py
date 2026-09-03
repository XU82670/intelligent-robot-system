#!/usr/bin/env python3
"""系统入口：启动智能机器人系统（CLI + HTTP 服务）。

用法:
    python main.py                  # 启动 CLI 交互模式
    python main.py --http           # 启动 HTTP 远程控制服务
    python main.py --sim            # 运行一次仿真演示
    python main.py --config config/config.json
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from intelligent_robot.hmi.cli import RobotCLI
from intelligent_robot.hmi.server import RobotHTTPServer
from intelligent_robot.robot import Robot


def main() -> None:
    parser = argparse.ArgumentParser(description="智能机器人系统")
    parser.add_argument("--config", default=None, help="配置文件路径 (JSON)")
    parser.add_argument("--http", action="store_true", help="启动 HTTP 远程控制服务")
    parser.add_argument("--sim", action="store_true", help="运行仿真演示")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP 服务地址")
    parser.add_argument("--port", type=int, default=8080, help="HTTP 服务端口")
    args = parser.parse_args()

    if args.sim:
        from simulator.run_simulation import run

        run(verbose=True)
        return

    robot = Robot(config_path=args.config)
    cli = RobotCLI(robot)

    if args.http:
        robot.start()
        server = RobotHTTPServer(args.host, args.port, robot, cli)
        print(f"HTTP 服务已启动: http://{args.host}:{args.port}")
        print("按 Ctrl+C 停止。")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\n正在停止...")
        finally:
            server.stop()
            robot.stop()
        return

    # 默认 CLI 模式
    cli.run()


if __name__ == "__main__":
    main()
