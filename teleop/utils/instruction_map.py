import numpy as np


class LowPassFilter:
    """低通滤波器，用于平滑数据"""
    def __init__(self, alpha=0.15):
        self.alpha = alpha
        self._value = 0.0
        self._last_value = 0.0

    def update(self, new_value, max_accel=1.5):
        delta = new_value - self._last_value
        delta = np.clip(delta, -max_accel, max_accel)
        filtered = self.alpha * (self._last_value + delta) + (1 - self.alpha) * self._value
        self._last_value = filtered
        self._value = filtered
        return self._value


class ControlDataMapper:
    """
    控制数据映射器
    将原始输入数据（范围 -32768 到 32768）映射到控制参数
    """
    def __init__(self):
        # 速度滤波器
        self._filters = {
            'x_vel': LowPassFilter(alpha=0.15),
            'y_vel': LowPassFilter(alpha=0.15),
            'yaw_vel': LowPassFilter(alpha=0.15)
        }
        
        # 高度累积值（松开后保持不变）
        self._height_value = 0.0
        
    def update(self, x_raw, y_raw, rx_raw, ry_raw):
        """
        更新并映射控制参数
        
        参数:
            x_raw: 横向摇杆原始值 (-1 到 1)
            y_raw: 纵向摇杆原始值 (-1 到 1)
            rx_raw: 右摇杆横向原始值 (-1 到 1)
            ry_raw: 右摇杆纵向原始值 (-1 到 1)
            
        返回:
            dict: 包含 x_vel, y_vel, yaw_vel, height 的字典
        """
        # 映射前进速度 (y_raw -> x_vel)
        raw = self._map_forward_velocity(y_raw)
        x_vel = self._filters['x_vel'].update(raw, max_accel=1.0)
        
        # 映射横向速度 (x_raw -> y_vel)
        raw = self._map_lateral_velocity(x_raw)
        y_vel = self._filters['y_vel'].update(raw, max_accel=1.0)
        
        # 映射偏航速度 (rx_raw -> yaw_vel)
        raw = self._map_yaw_velocity(rx_raw)
        yaw_vel = self._filters['yaw_vel'].update(raw, max_accel=1.0)
        
        # 更新高度（松开后保持当前值）
        self._update_height(ry_raw)
        
        return {
            'x_vel': x_vel,
            'y_vel': y_vel,
            'yaw_vel': yaw_vel,
            'height': self._height_value
        }
    
    def _update_height(self, raw_value):
        """
        更新高度值
        松开摇杆时保持当前高度不变（不会掉下来）
        
        参数:
            raw_value: 原始值 (-1 到 1)
        """
        deadzone = 0.05
        max_range = 1.0  # 最大范围 [-1, 1]
        
        if abs(raw_value) < deadzone:
            # 松开摇杆时，保持当前高度不变
            pass
        else:
            sign = 1 if raw_value > 0 else -1
            intensity = (abs(raw_value) - deadzone) / (1.0 - deadzone)
            smooth = 6*intensity**5 - 15*intensity**4 + 10*intensity**3
            height_value = sign * smooth * max_range
            
            # 更新高度值
            self._height_value = -height_value
    
    def _map_forward_velocity(self, value):
        """映射前进速度"""
        return self._smooth_map(value, -0.3, 0.3)
    
    def _map_lateral_velocity(self, value):
        """映射横向速度"""
        return self._smooth_map(value, -0.3, 0.3)
    
    def _map_yaw_velocity(self, value):
        """映射偏航速度"""
        return self._smooth_map(value, -1.57, 1.57)
    
    def _smooth_map(self, value, out_min, out_max, deadzone=0.05):
        """
        平滑映射函数
        使用死区和平滑曲线将输入值映射到输出范围
        
        参数:
            value: 输入值 (-1 到 1)
            out_min: 输出最小值
            out_max: 输出最大值
            deadzone: 死区大小
        """
        if abs(value) < deadzone:
            return 0.0
        t = (abs(value) - deadzone) / (1.0 - deadzone)
        t = np.clip(t, 0.0, 1.0)
        smooth = 6 * t**5 - 15 * t**4 + 10 * t**3
        return smooth * (out_max if value > 0 else out_min)
    
    def reset_height(self, height=0.0):
        """重置高度值"""
        self._height_value = height
    
    def get_current_height(self):
        """获取当前高度值"""
        return self._height_value
