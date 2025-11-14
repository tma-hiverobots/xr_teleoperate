import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
# 下面这两个 import 名称风格，跟我们看到的 LowCmd_ / LowState_ 是一样的写法。
# HandCmd_ / HandState_ 在 SDK 里一般放在 unitree_hg.msg.dds_ 下面
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.utils.thread import RecurrentThread

################################################################
# 1. 订阅手的状态，拿回来打印，方便调试
################################################################
last_hand_state = None

def HandStateHandler(msg: HandState_):
    global last_hand_state
    last_hand_state = msg
    # 这里你可以打印你关心的内容，比如每根手指的当前角度/力
    # 我不知道 HandState_ 具体字段叫什么（可能是 finger_pos[], finger_tau[], error_code 等）
    # 所以我放一个安全打印：列出全部属性名，第一次看完你就知道字段叫什么了
    # 你可以先注释掉，等第一次回调触发再打开
    # print("[HandState DEBUG dir(msg)]:", dir(msg))

################################################################
# 2. 主程序
################################################################
if __name__ == "__main__":
    """
    用法:
      python3 hand_dds_demo.py lo           # 例如仿真用ChannelFactoryInitialize(1, "lo")的话你就改成那样
      python3 hand_dds_demo.py enxXYZ       # 或真实机器人网卡名，比如 "eth0"
    """

    if len(sys.argv) < 2:
        # 如果你没给网卡/接口名，就走一个保底
        # 仿真里 unitree 会要求 ChannelFactoryInitialize(1)
        # 真机一般 ChannelFactoryInitialize(0, "eth0")
        # 我这里随便假设真实机器人: 通道0 + 网卡名eth0
        ChannelFactoryInitialize(0, "enx9c69d31ecd9b")
        dds_channel_hint = 0
    else:
        # 你可以按需要改成 ChannelFactoryInitialize(1, sys.argv[1])（仿真）或 ChannelFactoryInitialize(0, sys.argv[1])（真机）
        ChannelFactoryInitialize(0, sys.argv[1])
        dds_channel_hint = 0

    print(f"[INFO] DDS init done (channel hint {dds_channel_hint}). Safety check: 确保机器人周围没有障碍物！")
    input("按 Enter 继续发送手指命令...\n")

    # 创建hand命令publisher (左手为例)
    hand_cmd_pub = ChannelPublisher("rt/dex3/left/cmd", HandCmd_)
    hand_cmd_pub.Init()

    # 创建hand状态subscriber，方便看当前状态
    hand_state_sub = ChannelSubscriber("rt/dex3/left/state", HandState_)
    hand_state_sub.Init(HandStateHandler, 10)

    ############################################################
    # 准备一个 HandCmd_ 消息
    ############################################################
    cmd = HandCmd_

    # ==== 这一段是最关键：你要把手指目标写进 cmd ====
    # 下面我先假设这个结构里有:
    #   cmd.mode           手的控制模式，比如 0=idle, 1=position control (举例)
    #   cmd.finger_pos[i]  第 i 根手指的目标角度/张开程度 (rad 或 normalized)
    #   cmd.finger_kp[i]   每根手指的刚度
    #   cmd.finger_kd[i]   每根手指的阻尼
    #
    # 注意：真实字段名要根据你SDK里 HandCmd_ 的定义来对上。
    # 你可以在python里 print(dir(cmd)) 看真实字段叫什么，然后把下面对应上就行。
    #
    # 下面是“把整只手张开”的一个例子：

    # 假设有5个手指/关节组，目标都张开到 0.8
    try:
        open_val = 0.8
        for i in range(len(cmd.finger_pos)):
            cmd.finger_pos[i] = open_val
            cmd.finger_kp[i]  = 5.0     # 刚度
            cmd.finger_kd[i]  = 0.5     # 阻尼
        cmd.mode = 1  # position mode (示例)
    except AttributeError:
        # 如果上面字段名不对，会进这里
        # 你就打印看看 HandCmd_ 里到底长什么样
        print("[ERROR] HandCmd_ 字段名跟假设不一致。请运行 print(dir(cmd)) 看一下实际字段名，然后把上面那段对上。")
        print("HandCmd_ available fields:", dir(cmd))
        # 我们还是继续往下发，哪怕是空的 cmd，至少通道是通的
        pass

    ############################################################
    # 循环写命令 + 打印状态
    ############################################################
    rate_hz = 50.0
    period = 1.0 / rate_hz
    t0 = time.time()
    while True:
        # 写命令
        #hand_cmd_pub.Write(cmd)

        # 打印一下当前状态（每0.5秒打印一次）
        now = time.time()
        if now - t0 > 0.5:
            t0 = now
            if last_hand_state is None:
                print("[STATE] 还没收到 hand state ...")
            else:
                # 这里同样我不知道 HandState_ 真实字段，所以先通用打印
                # 你拿到一次输出之后，就可以改成更清楚的：
                #   print("finger_pos_now:", last_hand_state.finger_pos)
                #   print("contact_force:", last_hand_state.contact_force)
                print("[STATE] 收到手的状态 (字段列表如下，记下来你需要的字段):")
                print(dir(last_hand_state))

        time.sleep(period)
