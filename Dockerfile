FROM python:3.11-slim

WORKDIR /app

# 核心系统纯标准库，无需安装依赖；如需感知增强可取消注释
# RUN pip install --no-cache-dir numpy opencv-python-headless

COPY . .

EXPOSE 8080

# 默认启动 HTTP 远程控制服务
CMD ["python", "main.py", "--http", "--host", "0.0.0.0", "--port", "8080"]
