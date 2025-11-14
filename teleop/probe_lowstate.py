# from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
# from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
# import time

# def cb(msg: LowState_):
#     print("[OK] lowstate received once.")
#     exit(0)

# # 仿真=1, 接口=lo
# ChannelFactoryInitialize(1)
# sub = ChannelSubscriber("rt/lowstate", LowState_)
# sub.Init(cb, 10)

# print("Waiting lowstate...")
# while True:
#     time.sleep(0.1)
#!/usr/bin/env python3
# check_lowstate.py
# import argparse, sys, time, threading

# # 1) 尽量用你这版 SDK 的导入路径；失败再回退
# try:
#     from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
# except Exception:
#     from unitree_sdk2py.msg.dds_ import LowState_  # 兼容老路径

# from unitree_sdk2py.core.channel import (
#     ChannelSubscriber,
#     ChannelFactoryInitialize,
# )

# got_once = False

# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("--channel", type=int, default=1, help="DDS channel/domain (default: 1)")
#     ap.add_argument("--iface",   type=str, default="lo", help="network interface (lo|wlp8s0|eth0...)")
#     ap.add_argument("--timeout", type=float, default=10.0, help="overall timeout seconds")
#     args = ap.parse_args()

#     print(f"[INFO] Python: {sys.executable}")
#     print(f"[INFO] Init DDS: ChannelFactoryInitialize({args.channel}, '{args.iface}')")
#     ChannelFactoryInitialize(args.channel, args.iface)

#     # 2) 构建订阅者（回调 + 轮询双保险）
#     sub = ChannelSubscriber("rt/lowstate", LowState_)

#     def cb(msg: LowState_):
#         global got_once
#         if not got_once:
#             got_once = True
#             tick = getattr(msg, "tick", -1)
#             nmot = len(getattr(msg, "motor_state", []))
#             print(f"[OK] lowstate received (tick={tick}, motors={nmot}).")
#             sys.exit(0)  # 收到一次就退出

#     # 队列深度 10（与你原代码一致）
#     sub.Init(cb, 10)

#     # 3) 轮询兜底线程（有些环境回调可能被阻断，轮询更直接）
#     def poll_loop():
#         global got_once
#         last_warn = time.time()
#         while not got_once:
#             msg = sub.Read(0.25)  # 非阻塞
#             if msg is not None:
#                 got_once = True
#                 tick = getattr(msg, "tick", -1)
#                 nmot = len(getattr(msg, "motor_state", []))
#                 print(f"[OK] lowstate received (tick={tick}, motors={nmot}) via poll.")
#                 sys.exit(0)
#             # 每 5 秒提示一次
#             if time.time() - last_warn > 5.0:
#                 print("[WAIT] still waiting lowstate on rt/lowstate ...")
#                 last_warn = time.time()

#     th = threading.Thread(target=poll_loop, daemon=True)
#     th.start()

#     # 4) 总超时控制
#     print(f"[INFO] Waiting lowstate on rt/lowstate (channel={args.channel}, iface='{args.iface}'), timeout={args.timeout}s ...")
#     t0 = time.time()
#     try:
#         while time.time() - t0 < args.timeout:
#             time.sleep(0.1)
#         if not got_once:
#             print("[WARN] No lowstate within timeout. Likely causes:")
#             print("       - Isaac 未实际发布 lowstate（任务/开关未走 lowcmd/lowstate 管线）")
#             print("       - 频道/接口不一致（确保两端 ChannelFactoryInitialize(1, 'lo') 同机）")
#             print("       - 你在用 --motion/arm_sdk 路径：Sim 默认没人听这个话题")
#             sys.exit(2)
#     except KeyboardInterrupt:
#         print("\n[INFO] Interrupted.")
#         sys.exit(130)



# if __name__ == "__main__":
#     main()
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
import time, sys

def cb(msg: LowState_):
    print("[OK] lowstate received once. tick=", getattr(msg,"tick",-1))
    sys.exit(0)

ChannelFactoryInitialize(1, "lo")  # 或 "lo"（同机优先推荐）
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(cb, 10)
print("Waiting lowstate on channel=1 iface='los' ...")
t0=time.time()
while True:
    m = sub.Read(0.2)    # 轮询兜底
    if m is not None:
        print("[OK] lowstate via poll. tick=", getattr(m,"tick",-1))
        break
    if time.time()-t0 > 10:
        print("[WARN] 10s no data; 常见原因：Isaac未发布 / 接口或频道不一致 / 类型不一致")
        sys.exit(2)




