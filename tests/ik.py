"""
逆解服务测试程序( WebSocket 版本）
测试 FastAPI WebSocket 逆解服务的功能
使用信封模式( envelope )进行通信
"""
import asyncio
import numpy as np
import json
import websockets
from typing import Dict, Any, Optional
import ampl

WS_BASE_URL = "ws://localhost:8000/ws"


def create_envelope_request(
    command: str,
    task_id: str,
    payload: Dict[str, Any]
) -> Dict[str, Any]:
    """创建信封格式的请求"""
    return {
        "command": command,
        "task_id": task_id,
        "payload": payload
    }


def create_ik_payload(
    robot_type: str = "fanuc_crx10ia",
    target_frame: list = None,
    joint_seed: list = None,
    iter_rate: list = None,
    iter_number: int = 20
) -> Dict[str, Any]:
    """创建逆解请求的 payload"""
    if target_frame is None:
        # 默认 target_frame: 一个合理的目标位姿
        # (x, y, z, rx, ry, rz, rw) - 四元数格式
        target_frame = [0.5, 0.3, 0.4, 0.0, 0.0, 0.707, 0.707]
    
    if joint_seed is None:
        # 默认关节种子值（6个关节）
        joint_seed = [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    
    payload = {
        "robot_type": robot_type,
        "target_frame": target_frame,
        "iter_number": iter_number,
        "joint_seed": joint_seed,
        "iter_rate": iter_rate
    }
    
    if joint_seed is not None:
        payload["joint_seed"] = joint_seed
    
    if iter_rate is not None:
        payload["iter_rate"] = iter_rate
    
    return payload


async def send_ws_request(
    client_id: str,
    request_data: Dict[str, Any],
    timeout: float = 30.0
) -> Optional[Dict[str, Any]]:
    """发送 WebSocket 请求并接收响应"""
    ws_url = f"{WS_BASE_URL}/{client_id}"
    try:
        async with websockets.connect(ws_url, ping_interval=None) as websocket:
            # 发送请求
            await websocket.send(json.dumps(request_data))
            
            # 接收响应
            response_text = await asyncio.wait_for(websocket.recv(), timeout=timeout)
            response = json.loads(response_text)
            return response
    except asyncio.TimeoutError:
        print(f"  ✗ 请求超时")
        return None
    except websockets.exceptions.ConnectionClosed:
        print(f"  ✗ WebSocket 连接已关闭")
        return None
    except ConnectionRefusedError:
        print(f"  ✗ 无法连接到 WebSocket 服务")
        return None
    except Exception as e:
        print(f"  ✗ 连接错误: {e}")
        return None


async def test_ik_sync():
    """测试 1: 同步逆解（不使用迭代）"""
    print("=" * 60)
    print("测试 1: 同步逆解（不使用迭代算法）")
    print("=" * 60)
    
    ep = np.array([-1.06048286e-03, 7.06752344e-01, -3.53376555e-04, 7.07460158e-01,
                   7.00689569e-01, -1.49299150e-01, 7.85309730e-01], dtype=np.float64)
    payload = create_ik_payload(
        robot_type="fanuc_crx10ia",
        target_frame=ep.tolist(),
        iter_rate=None  # 不使用迭代
    )
    
    request = create_envelope_request(
        command="solve_ik",
        task_id="test_sync_001",
        payload=payload
    )
    
    result = await send_ws_request("test_client_1", request)
    
    if result:
        print(f"✓ 请求成功")
        print(f"  Task ID: {result.get('task_id', 'unknown')}")
        print(f"  Code: {result.get('code', 'unknown')}")
        print(f"  Message: {result.get('message', 'unknown')}")
        data = result.get('data', {})
        if data and 'output' in data:
            output = data['output']
            print(f"  逆解数量: {len(output)}")
            if output:
                print(f"  第一个逆解: {output[0]}")
        print()
        return result
    else:
        print(f"✗ 请求失败")
        print()
        return None


async def test_ik_iter():
    """测试 2: 迭代逆解"""
    print("=" * 60)
    print("测试 2: 迭代逆解（使用迭代算法）")
    print("=" * 60)
    
    iter_rate = [1e-3, 1e-2, 0.0, 0.0, 5e-4, 1e-4]
    ep = np.array([-1.06048286e-03, 7.06752344e-01, -3.53376555e-04, 7.07460158e-01,
                   7.00689569e-01, -1.49299150e-01, 7.85309730e-01], dtype=np.float64)
    
    payload = create_ik_payload(
        robot_type="fanuc_crx10ia",
        joint_seed=[0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
        iter_rate=iter_rate,
        iter_number=20,
        target_frame=ep.tolist()
    )
    
    request = create_envelope_request(
        command="solve_ik",
        task_id="test_iter_001",
        payload=payload
    )
    
    result = await send_ws_request("test_client_2", request)
    
    if result:
        print(f"✓ 请求成功")
        print(f"  Task ID: {result.get('task_id', 'unknown')}")
        print(f"  Code: {result.get('code', 'unknown')}")
        print(f"  Message: {result.get('message', 'unknown')}")
        data = result.get('data', {})
        if data and 'output' in data:
            output = data['output']
            print(f"  逆解数量: {len(output)}")
            if output:
                print(f"  第一个逆解: {output[0]}")
        print()
        return result
    else:
        print(f"✗ 请求失败")
        print()
        return None


async def test_heartbeat():
    """测试 3: 心跳检测"""
    print("=" * 60)
    print("测试 3: 心跳检测")
    print("=" * 60)
    
    request = create_envelope_request(
        command="heartbeat",
        task_id="test_heartbeat_001",
        payload={}
    )
    
    result = await send_ws_request("test_client_3", request)
    
    if result:
        print(f"✓ 心跳成功")
        print(f"  Task ID: {result.get('task_id', 'unknown')}")
        print(f"  Code: {result.get('code', 'unknown')}")
        print(f"  Message: {result.get('message', 'unknown')}")
        data = result.get('data', {})
        print(f"  Data: {data}")
        print()
        return result
    else:
        print(f"✗ 心跳失败")
        print()
        return None


async def test_error_cases():
    """测试 4: 错误情况处理"""
    print("=" * 60)
    print("测试 4: 错误情况处理")
    print("=" * 60)
    
    # 测试 4.1: 无效的机器人类型
    print("\n4.1 测试无效的机器人类型...")
    payload = create_ik_payload(robot_type="invalid_robot")
    request = create_envelope_request(
        command="solve_ik",
        task_id="test_error_001",
        payload=payload
    )
    
    result = await send_ws_request("test_client_4", request)
    if result:
        print(f"  响应 Code: {result.get('code', 'unknown')}")
        print(f"  响应 Message: {result.get('message', 'unknown')}")
    else:
        print(f"  请求失败")
    
    # 测试 4.2: 无效的 target_frame 长度
    print("\n4.2 测试无效的 target_frame 长度...")
    payload = create_ik_payload(
        robot_type="fanuc_crx10ia",
        target_frame=[0.0, 0.0, 0.0]  # 长度不对，应该是7
    )
    request = create_envelope_request(
        command="solve_ik",
        task_id="test_error_002",
        payload=payload
    )
    
    result = await send_ws_request("test_client_5", request)
    if result:
        print(f"  响应 Code: {result.get('code', 'unknown')}")
        print(f"  响应 Message: {result.get('message', 'unknown')}")
    else:
        print(f"  请求失败")
    
    # 测试 4.3: 未知的命令
    print("\n4.3 测试未知的命令...")
    request = create_envelope_request(
        command="fly_to_moon",
        task_id="test_error_003",
        payload={}
    )
    
    result = await send_ws_request("test_client_6", request)
    if result:
        print(f"  响应 Code: {result.get('code', 'unknown')}")
        print(f"  响应 Message: {result.get('message', 'unknown')}")
    else:
        print(f"  请求失败")
    
    # 测试 4.4: 无效的请求格式（缺少字段）
    print("\n4.4 测试无效的请求格式...")
    invalid_request = {
        "command": "solve_ik",
        # 缺少 task_id 和 payload
    }
    
    result = await send_ws_request("test_client_7", invalid_request)
    if result:
        print(f"  响应 Code: {result.get('code', 'unknown')}")
        print(f"  响应 Message: {result.get('message', 'unknown')}")
    else:
        print(f"  请求失败")
    
    print()


async def test_concurrent_requests():
    """测试 5: 并发请求测试（多个 WebSocket 连接）"""
    print("=" * 60)
    print("测试 5: 并发请求测试（多 WebSocket 连接）")
    print("=" * 60)
    
    async def single_request(client_id: str, task_id: str):
        payload = create_ik_payload()
        request = create_envelope_request(
            command="solve_ik",
            task_id=task_id,
            payload=payload
        )
        result = await send_ws_request(client_id, request)
        if result and result.get('code') == 200:
            return True, task_id
        else:
            return False, task_id
    
    # 并发发送5个请求（每个请求使用独立的 WebSocket 连接和 client_id）
    tasks = [
        single_request(f"concurrent_client_{i}", f"concurrent_task_{i}")
        for i in range(5)
    ]
    results = await asyncio.gather(*tasks)
    
    success_count = sum(1 for success, _ in results if success)
    print(f"  并发请求数: 5")
    print(f"  成功数: {success_count}")
    print(f"  失败数: {5 - success_count}")
    print()


async def check_service_health():
    """检查服务是否运行（通过 WebSocket 连接测试）"""
    print("检查服务状态...")
    try:
        ws_url = f"{WS_BASE_URL}/health_check"
        async with websockets.connect(ws_url, ping_interval=None) as websocket:
            test_request = create_envelope_request(
                command="heartbeat",
                task_id="health_check",
                payload={}
            )
            await websocket.send(json.dumps(test_request))
            
            try:
                response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                result = json.loads(response)
                print(f"✓ WebSocket 服务运行正常 ({WS_BASE_URL}/<client_id>)")
                print(f"测试响应: {result.get('message', 'unknown')}")
                return True
            except asyncio.TimeoutError:
                print(f"⚠ WebSocket 连接成功，但响应超时")
                return True  # 连接成功就算服务可用
    except websockets.exceptions.InvalidURI:
        print(f"✗ WebSocket URL 格式错误: {WS_BASE_URL}")
        return False
    except ConnectionRefusedError:
        print(f"✗ 无法连接到 WebSocket 服务 ({WS_BASE_URL})")
        print(f"  请确保服务已启动，可以使用以下命令启动:")
        print(f"  uvicorn robots_orchestra.services.websocket_service:app --reload")
        return False
    except Exception as e:
        print(f"✗ 检查服务状态时出错: {e}")
        return False


async def main():
    """主测试函数"""
    print("\n" + "=" * 60)
    print("逆解服务测试程序（WebSocket 信封模式）")
    print("=" * 60 + "\n")
    
    # 检查服务是否运行
    if not await check_service_health():
        print("服务未运行，请先启动服务")
        return

    await test_ik_sync()

    print("按回车继续...")
    input()  # 等待用户输入回车
    print("继续执行下一步")

    await test_ik_iter()
    await test_heartbeat()
    await test_error_cases()
    await test_concurrent_requests()
    
    print("=" * 60)
    print("测试完成")
    print("=" * 60)


if __name__ == "__main__":

    solver = ampl.ArmBase("fanuc_crx10ia", ampl.ArmType.CRX6, 6)
    solver.set_base(np.eye(4))
    # bounds = solver.joint_limits()
    qt_link = np.zeros(( 7, 7), dtype=np.float64)
    qs_fanuc = np.array([0.001] * 6, dtype=np.float64)
    solver.fk_links( qs_fanuc, qt_link )
    param_ik_iter=np.array([1e-3,1e-2,0,0,5e-4, 1e-4],dtype=np.float64)
    ep = qt_link[-1]
    print(ep)

    print("按回车继续...")
    input()  # 等待用户输入回车
    print("继续执行下一步")
    # 运行异步测试
    asyncio.run(main())