import cv2
import numpy as np
import ampl
import time
import requests
import json
from pathlib import Path
from typing import Optional


def tic(print_cmd: bool = False):
    global timer_global
    timer_global = time.perf_counter_ns()
    if print_cmd:
        print("# TIC")


def toc(print_cmd: bool = True, end='\n'):
    global timer_global
    delta = (float(time.perf_counter_ns()) - timer_global) / 1e6
    if print_cmd:
        print(f"# TOC = {delta} MS", end=end)
    return delta


def download_image(url: str, save_path: Path, timeout: int = 30) -> bool:
    """从URL下载图片到指定路径
    
    Args:
        url: 图片的URL地址
        save_path: 保存路径（Path对象）
        timeout: 请求超时时间（秒）
    
    Returns:
        bool: 下载是否成功
    """
    try:
        response = requests.get(url, timeout=timeout, stream=True)
        response.raise_for_status()
        
        # 确保目录存在
        save_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 写入文件
        with open(save_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=8192):
                f.write(chunk)
        
        print(f"成功下载图片: {url} -> {save_path}")
        return True
    except Exception as e:
        print(f"下载图片失败 {url}: {e}")
        return False


def download_camera_images(response_data: dict, task_id: str, cache_dir: Optional[Path] = None) -> dict:
    """从响应数据中提取URL并下载图片到cache目录
    
    Args:
        response_data: API响应的JSON数据（包含color_url、depth_url、mask_url等）
        task_id: 任务ID，用于创建子目录
        cache_dir: 缓存目录路径，如果为None则使用默认路径
    
    Returns:
        dict: 包含下载结果和本地路径
        {
            'color_path': Path or None,
            'depth_path': Path or None,
            'mask_path': Path or None,
            'intrinsic_path': Path or None,
            'color_success': bool,
            'depth_success': bool,
            'mask_success': bool
        }
    """
    if cache_dir is None:
        # 使用默认路径：controller/camera_cache
        cache_dir = Path(__file__).parent / "camera_cache"
    
    # 创建任务子目录
    task_dir = cache_dir / task_id
    task_dir.mkdir(parents=True, exist_ok=True)
    
    result = {
        'color_path': None,
        'depth_path': None,
        'mask_path': None,
        'intrinsic_path': None,
        'color_success': False,
        'depth_success': False,
        'mask_success': False
    }
    
    # 下载color图片
    if 'color_url' in response_data and response_data['color_url']:
        color_path = task_dir / "color.png"
        if download_image(response_data['color_url'], color_path):
            result['color_path'] = color_path
            result['color_success'] = True
    
    # 下载depth图片
    if 'depth_url' in response_data and response_data['depth_url']:
        depth_path = task_dir / "depth.png"
        if download_image(response_data['depth_url'], depth_path):
            result['depth_path'] = depth_path
            result['depth_success'] = True
    
    # 下载mask图片
    if 'mask_url' in response_data and response_data['mask_url']:
        mask_path = task_dir / "mask.png"
        if download_image(response_data['mask_url'], mask_path):
            result['mask_path'] = mask_path
            result['mask_success'] = True
    
    # 如果有intrinsic URL，也下载
    if 'intrinsic' in response_data and response_data['intrinsic']:
        intrinsic_path = task_dir / "intrinsic.json"
        if download_image(response_data['intrinsic'], intrinsic_path):
            result['intrinsic_path'] = intrinsic_path
    
    return result


def depth2pcd(task_dir: Optional[Path] = None):
    """从深度图生成点云
    
    Args:
        task_dir: 包含图片的目录路径，如果为None则使用默认路径
    """
    if task_dir is None:
        # 使用默认路径：controller/camera_cache/whatever
        task_dir = Path(__file__).parent / "camera_cache" / "whatever"
    
    F = Path(task_dir)

    dI = cv2.imread(str(F / "depth.png"), cv2.IMREAD_ANYDEPTH)
    cI = cv2.imread(str(F / "color.png"))
    mI = cv2.imread(str(F / "mask.png"), 0)
    
    # 从intrinsic.json中读取color类型的K矩阵
    intrinsic_json_path = F / "intrinsic.json"
    K = None
    if intrinsic_json_path.exists():
        with open(intrinsic_json_path, 'r') as f:
            intrinsic_data = json.load(f)
        # 查找type为"color"的项，提取其K矩阵
        for item in intrinsic_data:
            if item.get('type') == 'color' and 'K' in item:
                K = np.array(item['K'])
                break
        if K is None:
            print(f"警告: 在intrinsic.json中未找到color类型的K矩阵")
    else:
        print(f"错误: 找不到intrinsic.json文件: {intrinsic_json_path}")
        return
    
    if K is None:
        print(f"错误: 无法读取相机内参K矩阵")
        return
    
    print( K )
    xyzI = np.zeros((dI.shape[0], dI.shape[1], 3), dtype=np.float32)
    tic()
    ampl.image_depth_to_xyz(
        dI, K[0, 0], K[1, 1], K[0, 2], K[1, 2], xyzI, scale=1e-3, z_max_after_scale=1.5
    )
    toc()
    if 1:
        SKIP = 1
        xyzIo = xyzI[::SKIP, ::SKIP, :]
        cIo = cI[::SKIP, ::SKIP, :]
        mask = xyzIo[:, :, 2] > 0.0
        ampl.write_pointcloud(
            str(F / "pcd.ply"), xyzIo[mask], colorI=cIo[mask], is_bgr=True)
    # cv2.imshow("mask", mI)
    # cv2.waitKey(0)


