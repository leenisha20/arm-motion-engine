"""Fixed-point PID with clamping anti-windup + DC-motor plant (float). Bit-accurate controller model.

Controller (integer math, floor shifts like Verilog '>>>'):
  e   = ref - pos                          Q3.13
  i  += e ; clamp to +-I_MAX
  d   = e - e_prev
  u   = (KP*e + KI*i + KD*d) >> 8          gains Q8.8 -> output units
  u   = clamp(u, +-U_MAX)
Plant (float): v += (KU*u - KD_PLANT*v)*dt ; p += v*dt   (p in Q3.13 LSB units)
"""
import math, os, sys
sys.path.insert(0, "model")
import traj_model as tm

DT = 1e-3
KU = 400.0       # plant gain: LSB/s^2 per unit of u
KD_PLANT = 4.0     # viscous damping, 1/s
U_MAX = 32767
I_MAX = 40000
KP, KI, KD = 1024, 13, 38400     # Q8.8 -> 4.0, 0.05, 150


class PID:
    def __init__(self, kp=KP, ki=KI, kd=KD, i_max=I_MAX, windup_protect=True):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_max = i_max
        self.protect = windup_protect
        self.i = 0
        self.e_prev = 0

    def step(self, ref, pos):
        e = ref - pos
        self.i += e
        if self.protect:
            self.i = max(-self.i_max, min(self.i_max, self.i))
        d = e - self.e_prev
        self.e_prev = e
        u = (self.kp * e + self.ki * self.i + self.kd * d) >> 8
        return max(-U_MAX, min(U_MAX, u))


class Plant:
    def __init__(self):
        self.v = 0.0
        self.p = 0.0

    def step(self, u):
        self.v += (KU * u - KD_PLANT * self.v) * DT
        self.p += self.v * DT
        return int(round(self.p))


def run(ref_seq, ctrl, n_extra=0):
    plant = Plant()
    pos = 0
    out = []
    for r in ref_seq:
        u = ctrl.step(r, pos)
        pos = plant.step(u)
        out.append(pos)
    return out


def step_metrics(y, target):
    peak = max(y)
    overshoot = max(0.0, (peak - target) / target * 100.0)
    band = 0.02 * target
    settle = len(y)
    for k in range(len(y) - 1, -1, -1):
        if abs(y[k] - target) > band:
            settle = k + 1
            break
    else:
        settle = 0
    return overshoot, settle


def main():
    os.makedirs("results", exist_ok=True)

    # 1. step response
    target = 8192   # 1 rad
    y = run([target] * 1500, PID())
    os_pct, ts = step_metrics(y, target)
    sse = abs(y[-1] - target)

    # 2. quintic tracking (0 -> 1.5 rad)
    q0, q1 = 0, 12288
    n = tm.min_log2n(q1 - q0)
    ref = [tm.traj(q0, q1, k, n) for k in range(1 << n)] + [q1] * 300
    yt = run(ref, PID())
    track = max(abs(a - b) for a, b in zip(ref, yt))
    rms = math.sqrt(sum((a - b) ** 2 for a, b in zip(ref, yt)) / len(ref))

    # 3. anti-windup: large step that saturates the actuator
    big = 24000
    y_p = run([big] * 2500, PID(windup_protect=True))
    y_u = run([big] * 2500, PID(windup_protect=False))
    os_p, _ = step_metrics(y_p, big)
    os_u, _ = step_metrics(y_u, big)

    with open("results/pid_baseline.csv", "w") as f:
        f.write("test,metric,value\n")
        f.write("step,overshoot_pct,%.3f\n" % os_pct)
        f.write("step,settling_samples_2pct,%d\n" % ts)
        f.write("step,steady_state_err_lsb,%d\n" % sse)
        f.write("quintic,max_track_err_lsb,%d\n" % track)
        f.write("quintic,rms_track_err_lsb,%.2f\n" % rms)
        f.write("saturation,overshoot_with_antiwindup_pct,%.3f\n" % os_p)
        f.write("saturation,overshoot_without_antiwindup_pct,%.3f\n" % os_u)

    print("PID baseline (dt=%.0f ms, Kp=%.1f Ki=%.3f Kd=%.1f)"
          % (DT * 1e3, KP / 256.0, KI / 256.0, KD / 256.0))
    print("Step (1 rad)     : overshoot %.2f %%, settling %d ms, steady-state err %d LSB"
          % (os_pct, ts, sse))
    print("Quintic tracking : max err %d LSB (%.2e rad), RMS %.1f LSB"
          % (track, track / 8192.0, rms))
    print("Saturating step  : overshoot with anti-windup %.2f %%, without %.2f %%"
          % (os_p, os_u))
    ok = (os_pct < 25.0 and ts < 1200 and sse <= 3 and os_p <= os_u)
    print("PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
