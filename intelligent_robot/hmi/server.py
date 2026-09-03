"""应用层：HTTP 远程控制服务。对应 PPT「应用层-远程管理」。

基于标准库 http.server，提供 REST 接口：
  GET  /status        返回机器人状态 JSON
  POST /command       下发命令 {"cmd": "goto", "args": {"x":1, "y":2}}
  GET  /              简单控制页 HTML
纯标准库实现，无需 Flask 等依赖。
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional


class RobotHTTPHandler(BaseHTTPRequestHandler):
    """HTTP 请求处理器。robot 实例通过 server.robot 注入。"""

    server: "RobotHTTPServer"  # type: ignore

    def _send_json(self, data: dict, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send_html(self._control_page())
            return
        if self.path == "/status":
            self._send_json(self.server.robot.status())
            return
        self._send_json({"error": "not found"}, status=404)

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/command":
            self._send_json({"error": "not found"}, status=404)
            return
        length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(length) if length else b"{}"
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            self._send_json({"error": "invalid JSON"}, status=400)
            return
        result = self.server.cli.handle(self._payload_to_line(payload))
        self._send_json({"result": result})

    @staticmethod
    def _payload_to_line(payload: dict) -> str:
        cmd = payload.get("cmd", "")
        args = payload.get("args", {})
        if cmd == "goto":
            return f"goto {args.get('x', 0)} {args.get('y', 0)}"
        return cmd

    def _send_html(self, html: str) -> None:
        body = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _control_page() -> str:
        return """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>智能机器人控制台</title>
<style>body{font-family:sans-serif;max-width:640px;margin:40px auto;padding:0 16px}
button{padding:8px 16px;margin:4px;cursor:pointer}
#status{background:#f4f6f1;padding:16px;border-radius:8px;white-space:pre-wrap}</style></head>
<body><h2>智能机器人控制台</h2>
<div><button onclick="cmd('start')">启动</button>
<button onclick="cmd('stop')">停止</button>
<button onclick="refresh()">刷新状态</button></div>
<p>目标点: <input id="x" value="2" size="4"> <input id="y" value="2" size="4">
<button onclick="goto()">导航</button></p>
<h3>状态</h3><div id="status">加载中...</div>
<script>function refresh(){fetch('/status').then(r=>r.json()).then(d=>{document.getElementById('status').textContent=JSON.stringify(d,null,2)})}
function cmd(c){fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:c})}).then(r=>r.json()).then(d=>{alert(d.result);refresh()})}
function goto(){const x=document.getElementById('x').value,y=document.getElementById('y').value;fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'goto',args:{x:parseFloat(x),y:parseFloat(y)}})}).then(r=>r.json()).then(d=>{alert(d.result);refresh()})}
refresh();setInterval(refresh,2000);</script></body></html>"""

    def log_message(self, format: str, *args) -> None:  # 静默访问日志
        pass


class RobotHTTPServer(ThreadingHTTPServer):
    """带 robot 与 cli 注入的 HTTP 服务器。"""

    def __init__(self, host: str, port: int, robot, cli) -> None:
        super().__init__((host, port), RobotHTTPHandler)
        self.robot = robot
        self.cli = cli
        self._thread: Optional[threading.Thread] = None

    def start_in_background(self) -> None:
        self._thread = threading.Thread(target=self.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.shutdown()
        self.server_close()
