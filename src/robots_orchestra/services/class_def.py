import asyncio
import numpy as np

from typing import Dict, Optional, List
from pydantic import BaseModel
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, HTTPException
from data_model import tensor1f, tensor2f, tensor3f, tensor4f

# 求逆解的请求和回复 
class IKRequest(BaseModel):
    task_id: str
    robot_type : str
    base_frame: tensor1f # qt 7 x y z rx ry rz rw
    target_frame: tensor1f # qt 7 x y z rx ry rz rw
    joint_seed: tensor1f # 提供的轴值seed
    iter_rate : tensor1f # 迭代率 如果有值就会启用迭代算法求解逆解，可以是空
    iter_number : int # 默认 20次 

class IKResponse(BaseModel):
    task_id: str
    output: tensor2f # 全部逆解的组合
    message: str = ""
    code: int = 0
