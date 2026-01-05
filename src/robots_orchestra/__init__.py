"""Robots Orchestra - 机器人编排系统"""
from pathlib import Path

# 声明目前的包中囊括的URDF及其对应的几何内容
PROJECT_ROOT = Path(__file__).parent
SCENE_DIR = PROJECT_ROOT / "scene"

__all__ = ["SCENE_DIR", "PROJECT_ROOT"]
