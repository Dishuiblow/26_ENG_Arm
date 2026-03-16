import numpy as np
from scipy.interpolate import CubicSpline

def process_trajectory_scipy(recorded_data, target_dt=0.01):
    """
    recorded_data: List[List[float]] 原始轨迹点
    target_dt: 目标采样间隔 (秒), 默认10ms
    """
    if not recorded_data: return None, None
    
    data = np.array(recorded_data) # (N, 6)
    n_points = len(data)
    
    # 假设原始采样也是约10ms一次
    t_raw = np.linspace(0, n_points * 0.01, n_points)
    
    # 构造三次样条插值器
    # axis=0 表示沿着时间轴对每一列(每个关节)分别插值
    cs = CubicSpline(t_raw, data, axis=0, bc_type='natural')
    
    # 生成新的时间轴 (这里示例保持时长一致，也可以用来做变速)
    t_new = np.arange(0, t_raw[-1], target_dt)
    
    # 计算新轨迹
    fitted_data = cs(t_new)
    
    return t_new, fitted_data