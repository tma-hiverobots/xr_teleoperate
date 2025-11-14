# dex_dds_helper.py
import numpy as np
from robot_control.hand_retargeting import HandRetargeting, HandType  # 这行名字你对一下
# 上面这个类名你在 dex_server.py 里已经有了，就照那个 import 路径来:contentReference[oaicite:2]{index=2}

class DexDDSTeleopHelper:
    def __init__(self):
        self.retarget = HandRetargeting(HandType.UNITREE_DEX3)
        self.last_q = np.zeros(14, dtype=np.float64)   # 左7右7
        # self.temp_limit_factor = np.ones(14, dtype=np.float64)

    def update_from_xr(self, left_pts75, right_pts75, trigger_val: float):
        """
        left_pts75/right_pts75: 你在 teleop 里已经拿到的 25*3 = 75 个数的扁平数组
        trigger_val: 扳机量 0~1
        """

        left_pts75 = np.asarray(left_pts75,dtype = np.float64)
        right_pts75 = np.asarray(right_pts75,dtype = np.float64)

        left_xyz = left_pts75.reshape(25,3)
        right_xyz = left_pts75.reshape(25,3)
        #1) 做 retarget
        q14 = self.retarget.retarget(left_xyz, right_xyz)  # -> np.array(14)

        #2) 扳机逻辑：按下就保持上一帧 / 或者你也可以直接在这里写成抓取姿态
        if trigger_val > 0.6:
            # 保持上一帧
            q14 = self.last_q.copy()

            # 也可以改成 “强行抓取”，例如：
            # q14[7:] = np.array([1.0,1.1,1.2,1.0,0.9,0.8,0.6])
        else:
            self.last_q = q14.copy()
        # q14 = np.array([-1.0,-1.0,-1.7,1.55,1.75,1.55,1.75])
        # 3) 温度限制（如果有的话）: q14 *= self.temp_limit_factor

        return q14

    def update_temp(self, right_temp_list):
        """
        可选：如果你从 rt/dex3/right/state 拿到了温度，就在这里限制
        """
        for i, t in enumerate(right_temp_list):
            if t > 100:
                self.temp_limit_factor[7+i] = 0.0
            elif t > 80:
                self.temp_limit_factor[7+i] = 0.5
            else:
                self.temp_limit_factor[7+i] = 1.0

