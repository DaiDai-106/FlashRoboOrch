import numpy as np
from typing import Dict, Optional, List, Any
from pydantic import BaseModel
from data_model import tensor1f, tensor2f, tensor3f, tensor4f
from enum import IntEnum

"""websocket 错误码的定义"""
class StatusCode(IntEnum):
    SUCCESS = 200              # 成功：算出来了
    BAD_REQUEST = 400          # 格式不对 (比如 json 缺括号)
    VALIDATION_ERROR = 422     # 参数不对 (比如 target_frame 只有6个数，但需要7个)
    UNKNOWN_COMMAND = 404      # 指令不对 (比如发了 "fly_to_moon")
    SERVER_ERROR = 500         # 咱们代码崩了 (比如除以零)
    TIMEOUT = 504              # 计算超时 (IK 算了 10秒还没结果)
    IK_NO_SOLUTION = 1001      # 求解失败：目标点不可达 (太远了)


"""websocket 基础请求的定义"""
class WsRequestEnvelope(BaseModel):
    command: str              # 对应 "solve_ik"
    task_id: str              # 建议放在外层，这样报错时也能带上 ID
    payload: Dict[str, Any]   # 这里先用 Dict 接收，稍后再具体校验

"""websocket 基础请求的定义"""
class WsResponseEnvelope(BaseModel):
    task_id: str              # 把请求的 ID 原样还回去，方便前端匹配
    code: int = 200           # 状态码, 默认成功 
    data: Optional[Dict[str, Any]] = None
    message: Optional[str] = None


""" 具体业务层功能payload的数据定义"""

""" 1. 逆解请求和相应的的定义"""
class WsIkRequest(BaseModel):
    robot_type : str
    target_frame: tensor1f         # qt 7 x y z rx ry rz rw
    joint_seed: Optional[tensor1f]
    iter_rate: Optional[tensor1f]    # 迭代步长
    iter_number : Optional[int] = 20 # 默认 20次 

class WsIkResponse(BaseModel):
    output: tensor2f # 全部逆解的组合