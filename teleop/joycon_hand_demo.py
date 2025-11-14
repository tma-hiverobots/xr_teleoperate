# joycon_hand_demo.py
# Minimal Joy-Con hand tracking: IMU (acc+gyro) → attitude + relative position (ZUPT),
# ZL/ZR to reset origin for each hand.

import time
import math
import threading
import numpy as np

# ---- Joy-Con library (pip install joycon-python) ----
try:
    from pyjoycon import JoyCon, get_L_id, get_R_id
except Exception as e:
    raise SystemExit(
        "Missing dependency. Please:\n"
        "  pip install joycon-python numpy\n"
        f"Import error: {e}"
    )

DT = 0.01         # 100 Hz
G  = np.array([0, 0, -9.81])

# ===== Simple Attitude integrator (gyro-only; good enough short-term). =====
# 你后面可以换成更完整的 Madgwick/Mahony，但这个最小可跑。
def quat_mul(q, r):
    w,x,y,z = q; a,b,c,d = r
    return np.array([
        w*a - x*b - y*c - z*d,
        w*b + x*a + y*d - z*c,
        w*c - x*d + y*a + z*b,
        w*d + x*c - y*b + z*a
    ], dtype=float)

def quat_from_omega(omega, dt):
    # omega (rad/s) -> small-angle quaternion over dt
    angle = np.linalg.norm(omega) * dt
    if angle < 1e-9:
        return np.array([1,0,0,0], dtype=float)
    axis = omega / (np.linalg.norm(omega) + 1e-12)
    s = math.sin(0.5*angle)
    return np.array([math.cos(0.5*angle), axis[0]*s, axis[1]*s, axis[2]*s], dtype=float)

def quat_to_R(q):
    w,x,y,z = q
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w),   2*(x*z + y*w)],
        [2*(x*y + z*w),   1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)
    return R

def normalize_quat(q):
    return q / (np.linalg.norm(q) + 1e-12)

# ===== One-hand tracker =====
class JoyconHand:
    def __init__(self, side="L", dt=DT):
        self.side = side  # "L" or "R"
        # Connect Joy-Con
        if side == "L":
            jid = get_L_id()
        else:
            jid = get_R_id()
        if jid is None:
            raise RuntimeError(f"{side} Joy-Con not found (not paired via Bluetooth?)")
        self.jc = JoyCon(*jid)

        # State
        self.dt = dt
        self.q  = np.array([1.,0.,0.,0.])   # orientation (wxyz)
        self.R0 = np.eye(3)                 # zero-orientation
        self.p  = np.zeros(3)               # position (relative)
        self.v  = np.zeros(3)               # velocity
        self.p0 = np.zeros(3)               # zero-position
        self.lock = threading.Lock()
        self.running = False

        # ZUPT thresholds
        self.zupt_gyro = np.deg2rad(3.0)    # rad/s
        self.zupt_acc  = 0.15 * 9.81        # m/s^2 deviation from 1g

        # Scale (depends on Joy-Con IMU config – these defaults work reasonably in practice)
        # If you see wrong magnitudes, tune these two gains.
        self.GYRO_SCALE  = math.radians(2000.0 / 32768.0)   # ~rad/s per LSB (assume ±2000 dps)
        self.ACCEL_SCALE = 9.81 * (8.0 / 32768.0)           # ~m/s^2 per LSB (assume ±8g)

    def start(self):
        if self.running: return
        self.running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self.running = False

    def _loop(self):
        # Ask joycon-python to enable IMU if not already (library usually does)
        # Warm-up
        time.sleep(0.05)

        while self.running:
            st = self.jc.get_status()  # dict with 'accel', 'gyro', 'buttons', etc.
            # Raw readings
            ax = st["accel"]["x"] * self.ACCEL_SCALE
            ay = st["accel"]["y"] * self.ACCEL_SCALE
            az = st["accel"]["z"] * self.ACCEL_SCALE
            gx = st["gyro"]["x"]  * self.GYRO_SCALE
            gy = st["gyro"]["y"]  * self.GYRO_SCALE
            gz = st["gyro"]["z"]  * self.GYRO_SCALE

            # Integrate orientation (gyro-only small-angle)
            dq = quat_from_omega(np.array([gx,gy,gz]), self.dt)
            self.q = normalize_quat(quat_mul(self.q, dq))
            R_body_to_world = quat_to_R(self.q)

            # Linear accel in world (remove gravity)
            acc_world = R_body_to_world @ np.array([ax,ay,az]) + G  # NOTE: Joy-Con accel points "up" as -G typically; adjust sign if needed.

            # ZUPT (Zero-Velocity Update) to curb drift
            gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
            acc_norm  = math.sqrt(ax*ax + ay*ay + az*az)
            zupt = (gyro_norm < self.zupt_gyro) and (abs(acc_norm - 9.81) < self.zupt_acc)
            if zupt:
                self.v *= 0.0
            else:
                self.v += acc_world * self.dt
                self.p += self.v * self.dt

            # Buttons → origin reset
            btns = st.get("buttons", {})
            if self.side == "L":
                pressed_reset = (btns.get("zl", False) is True)
            else:
                pressed_reset = (btns.get("zr", False) is True)

            if pressed_reset:
                # Set current pose as origin (zero)
                self.R0 = R_body_to_world.copy()
                self.p0 = self.p.copy()

            # Export
            with self.lock:
                # Orient relative to zero
                R_rel = R_body_to_world @ self.R0.T
                p_rel = self.p - self.p0
                self._R = R_rel
                self._p = p_rel
                self._buttons = btns

            time.sleep(self.dt)

    def get_pose(self):
        with self.lock:
            return self._R.copy(), self._p.copy(), dict(self._buttons)

# ===== Pretty print utils =====
def fmt_vec(v):
    return f"[{v[0]: .3f}, {v[1]: .3f}, {v[2]: .3f}]"

def main():
    print("Connecting Joy-Cons…")
    try:
        L = JoyconHand("L", dt=DT)
        print("Left Joy-Con connected.")
    except Exception as e:
        L = None
        print(f"Left Joy-Con not available: {e}")

    try:
        R = JoyconHand("R", dt=DT)
        print("Right Joy-Con connected.")
    except Exception as e:
        R = None
        print(f"Right Joy-Con not available: {e}")

    if not L and not R:
        print("No Joy-Con connected. Pair via Bluetooth first.")
        return

    if L: L.start()
    if R: R.start()

    print("\nControls:")
    print("  Left hand:  hold ZL to set current pose as origin (position=0, yaw align).")
    print("  Right hand: hold ZR to set current pose as origin (position=0, yaw align).")
    print("Press Ctrl+C to quit.\n")

    try:
        while True:
            lines = []
            if L:
                RL, pL, bL = L.get_pose()
                lines.append(f"L pos {fmt_vec(pL)}  ZL={bL.get('zl',False)}  SL={bL.get('sl',False)} SR={bL.get('sr',False)}")
            if R:
                RR, pR, bR = R.get_pose()
                lines.append(f"R pos {fmt_vec(pR)}  ZR={bR.get('zr',False)}  SL={bR.get('sl',False)} SR={bR.get('sr',False)}")
            print("\r" + " | ".join(lines) + " " * 10, end="", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        if L: L.stop()
        if R: R.stop()
        print("\nStopped.")

if __name__ == "__main__":
    main()
