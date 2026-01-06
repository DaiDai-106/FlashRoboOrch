"""
启动 WebSocket 逆解服务的脚本

使用方法:
    python tests/start_service.py                    # 默认配置启动
    python tests/start_service.py --port 8080        # 指定端口
    python tests/start_service.py --reload           # 开发模式（自动重载）
    python tests/start_service.py --host 127.0.0.1   # 只监听本地
"""
import uvicorn
import sys
import argparse
from pathlib import Path

# 添加项目根目录到 Python 路径，确保可以导入 robots_orchestra
project_root = Path(__file__).parent.parent
src_path = str(project_root / "src")
if src_path not in sys.path:
    sys.path.insert(0, src_path)


def main():
    """启动 WebSocket 服务"""
    parser = argparse.ArgumentParser(description="启动 WebSocket 逆解服务")
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="服务监听地址 (默认: 0.0.0.0)"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="服务监听端口 (默认: 8000)"
    )
    parser.add_argument(
        "--reload",
        action="store_true",
        help="启用自动重载（开发模式）"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=1,
        help="工作进程数 (默认: 1)"
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("WebSocket 逆解服务")
    print("=" * 60)
    print(f"服务地址: ws://{args.host}:{args.port}/ws/<client_id>")
    print(f"监听地址: {args.host}:{args.port}")
    print(f"自动重载: {'启用' if args.reload else '禁用'}")
    print(f"工作进程: {args.workers}")
    print("=" * 60)
    print("按 Ctrl+C 停止服务")
    print("=" * 60)
    print()
    
    # 启动服务
    uvicorn.run(
        "robots_orchestra.services.websocket_service:app",
        host=args.host,
        port=args.port,
        reload=args.reload,
        workers=args.workers if not args.reload else 1,  # reload 模式下只能使用单进程
        log_level="info"
    )


if __name__ == "__main__":
    main()

