import abc
from typing import Any, Dict, Optional


class ServiceBase(abc.ABC):
    def __init__(self, name: str):
        self.name = name

    # 所有子类必须实现这个方法。
    @abc.abstractmethod
    async def execute(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        pass

    # 简单的日志记录
    def log(self, message: str):
        print(f"[{self.name}] {message}")