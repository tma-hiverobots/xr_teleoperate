import numpy as np


class LowPassFilter:
    """Low-pass filter for smoothing data"""
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
    Control data mapper for mobile base and elevation
    """
    def __init__(self):
        # Velocity filters
        self._filters = {
            'x_vel': LowPassFilter(alpha=0.15),
            'y_vel': LowPassFilter(alpha=0.15),
            'yaw_vel': LowPassFilter(alpha=0.15)
        }
        
        # Height accumulated value (remains unchanged after release)
        self._height_value = 0.0
        
    def update(self, x_raw, y_raw, rx_raw, ry_raw,rbutton_A,rbutton_B):
        """
        Update and map control parameters
        
        Args:
            x_raw: Lateral joystick raw value (-1 to 1)
            y_raw: Longitudinal joystick raw value (-1 to 1)
            rx_raw: Right joystick horizontal raw value (-1 to 1)
            ry_raw: Right joystick vertical raw value (-1 to 1)
            rbutton_A: Right button A raw value (0 or 1)
            rbutton_B: Right button B raw value (0 or 1)
        Returns:
            dict: Dictionary containing x_vel, y_vel, yaw_vel, height
        """
        # Map forward velocity 
        raw = self._map_forward_velocity(x_raw)
        x_vel = self._filters['x_vel'].update(raw, max_accel=1.0)
        
        # Map lateral velocity
        raw = self._map_lateral_velocity(y_raw)
        y_vel = self._filters['y_vel'].update(raw, max_accel=1.0)
        
        # Map yaw velocity 
        raw = self._map_yaw_velocity(rx_raw, -2.62, 2.62)
        yaw_vel = self._filters['yaw_vel'].update(raw, max_accel=1.0)
        
        # Update height (remains at current value after release)
        self._update_height_button(rbutton_A,rbutton_B)
        
        return {
            'x_vel': x_vel,
            'y_vel': y_vel,
            'yaw_vel': yaw_vel,
            'height': self._height_value
        }
    def _update_height_button(self,rbutton_A,rbutton_B):
        """
        Update height value
        Height remains unchanged when joystick is released (won't drop down)
        
        Args:
            rbutton_A: Right button A raw value (0 or 1)
            rbutton_B: Right button B raw value (0 or 1)
        """
        if rbutton_A:
            self._height_value = 0.2
        elif rbutton_B:
            self._height_value = -0.2
        else:
            self._height_value = 0.0
    def _update_height(self, raw_value):
        """
        Update height value
        Height remains unchanged when joystick is released (won't drop down)
        
        Args:
            raw_value: Raw value (-1 to 1)
        """
        deadzone = 0.05
        max_range = 1.0 
        
        if abs(raw_value) < deadzone:
            self._height_value = 0.0
        else:
            sign = 1 if raw_value > 0 else -1
            intensity = (abs(raw_value) - deadzone) / (1.0 - deadzone)
            smooth = 6*intensity**5 - 15*intensity**4 + 10*intensity**3
            height_value = sign * smooth * max_range
            
            self._height_value = height_value
    
    def _map_forward_velocity(self, value):
        return self._smooth_map(value, -0.2, 0.2)
    
    def _map_lateral_velocity(self, value):
        return self._smooth_map(value, -0.6, 0.6)
    
    def _map_yaw_velocity(self, value,min_value,max_value):
        return self._smooth_map(value,min_value,max_value)
    
    def _smooth_map(self, value, out_min, out_max, deadzone=0.05):
        """
        Smooth mapping function
        Maps input value to output range using deadzone and smooth curve
        
        Args:
            value: Input value (-1 to 1)
            out_min: Output minimum value
            out_max: Output maximum value
            deadzone: Deadzone size
        """
        if abs(value) < deadzone:
            return 0.0
        t = (abs(value) - deadzone) / (1.0 - deadzone)
        t = np.clip(t, 0.0, 1.0)
        smooth = 6 * t**5 - 15 * t**4 + 10 * t**3
        return smooth * (out_max if value > 0 else out_min)
    
    def reset_height(self, height=0.0):
        """Reset height value"""
        self._height_value = height
    
    def get_current_height(self):
        """Get current height value"""
        return self._height_value