import numpy as np
import time
from numpy.random import f
import pinocchio as pin
import logging_mp
logging_mp.basic_config(level=logging_mp.INFO)
logger_mp = logging_mp.get_logger(__name__)
class GravityFeedforward:
    """
    用于计算关节重力补偿前馈力矩的类。
    初始化时加载 URDF 和模型，调用 compute() 计算 tau_ff。
    """

    def __init__(self, urdf_path, joint_names=None, root_joint=None):
        """
        参数:
        --------
        urdf_path : str
            URDF 文件路径

        joint_names : list[str], optional
            如果只需要部分关节（例如上身某几个电机），
            可以传 joint_names 来提取对应的 index。

        root_joint : pinocchio.JointModel, optional
            如果机器人不是 floating base，保持默认即可。
        """

        # 1. 加载 URDF 模型
        if root_joint is None:
            self.model = pin.buildModelFromUrdf(urdf_path)
        else:
            self.model = pin.buildModelFromUrdf(urdf_path, root_joint)

        self.data = self.model.createData()

        # 2. 获取关节索引
        if joint_names is None:
            # 默认使用模型的所有关节
            self.joint_id_list = [
                jid for jid in range(1, self.model.njoints)
                if self.model.joints[jid].nq > 0
            ]
        else:
            # 只使用指定关节
            self.joint_id_list = []
            for name in joint_names:
                jid = self.model.getJointId(name)
                if jid == 0:
                    raise ValueError(f"Joint name {name} not found in URDF.")
                self.joint_id_list.append(jid)

        # 生成 joint 顺序到 model.q 的索引（非常重要）
        self.idxs = []
        for jid in self.joint_id_list:
            idx_q = self.model.joints[jid].idx_q
            self.idxs.append(idx_q)

        self.idxs = np.array(self.idxs)
        self.n = len(self.idxs)
    # ------------------------------------------------------------------

    def compute(self, q):
        """
        计算重力补偿前馈扭矩 tau_ff。

        参数:
        --------
        q : ndarray [n,]
            当前关节角度（对应 joint_names 或模型所有 actuated joints）

        返回:
        --------
        tau_ff : ndarray [n,]
            重力补偿扭矩
        """

        # 1. 构造完整的 model.q（包含所有关节）
        full_q = np.zeros(self.model.nq)
        full_q[self.idxs] = q

        # 2. RNEA 计算 G(q)
        G_full = pin.rnea(
            self.model,
            self.data,
            full_q,
            np.zeros(self.model.nv),
            np.zeros(self.model.nv)
        )

        G_full = np.array(G_full).reshape(-1)

        # 3. 只输出你关心关节的前馈力矩
        tau_ff = G_full[self.idxs]
        # print(f"q[0]:{q[0]}")
        if q[0]>1.5:
            tau_ff = tau_ff-5
        return tau_ff