# 驱动测试 
import json
from ctypes import *
from pathlib import Path
from robots_orchestra.driver.marvin.robot import Marvin_Robot , DCSS

class BytesEncoder(json.JSONEncoder):
    """自定义 JSON 编码器，处理 bytes 类型"""
    def default(self, obj):
        if isinstance(obj, bytes):
            # 将 bytes 转换为整数（对于单个字节）或 base64 字符串（对于多个字节）
            if len(obj) == 1:
                return int(obj[0])
            else:
                import base64
                return base64.b64encode(obj).decode('utf-8')
        return super().default(obj)

def main():
    marvin = Marvin_Robot()
    
    dcss = DCSS() #订阅数据
    data = marvin.subscribe(dcss)

    formatted_data = json.dumps(data, indent=2, cls=BytesEncoder)
    print(formatted_data)

if __name__ == "__main__":
    main()
