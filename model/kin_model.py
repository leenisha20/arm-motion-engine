"""2-link planar arm kinematics: float reference + fixed-point (CORDIC-based) model.

Fixed point: positions Q2.14 (1.0 = 16384), angles Q3.13 (1.0 = 8192).
Trig is done with the same CORDIC algorithms as the RTL (sin/cos rotation, atan2 vectoring).
"""
import math, os, random, sys
sys.path.insert(0, "model")
import cordic_model as cm          # sin/cos, Q2.14 angle in
import cordic_vec_model as vm      # atan2/mag, Q3.13 angle out

L1, L2 = 0.30, 0.25
PS = 1 << 14       # position scale
AS = 1 << 13       # angle scale


# ---------- floating-point reference ----------
def ik_float(x, y):
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    t2 = math.atan2(math.sqrt(1 - c2 * c2), c2)          # elbow-down
    t1 = math.atan2(y, x) - math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    return t1, t2


def fk_float(t1, t2):
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


# ---------- fixed-point models ----------
def sincos_q(angle_q13):
    """sin/cos of a Q3.13 angle using the Q2.14 CORDIC (quadrant-reduced). Returns Q2.14 (cos, sin)."""
    pi = vm.PI_FX
    a = angle_q13
    # wrap to [-pi, pi]
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    sign = 1
    half = pi // 2
    if a > half:
        a, sign = pi - a, -1       # cos flips sign
    elif a < -half:
        a, sign = -pi - a, -1
    a14 = a * 2                    # Q3.13 -> Q2.14
    c, s = cm.cordic_sincos(a14)
    return sign * c, s


def fk_fixed(t1, t2):
    c1, s1 = sincos_q(t1)
    c12, s12 = sincos_q(t1 + t2)
    l1, l2 = int(round(L1 * PS)), int(round(L2 * PS))
    x = (l1 * c1 + l2 * c12) >> 14
    y = (l1 * s1 + l2 * s12) >> 14
    return x, y


def ik_fixed(x, y):
    l1, l2 = int(round(L1 * PS)), int(round(L2 * PS))
    r2 = (x * x + y * y) >> 14                       # Q2.14
    num = r2 - ((l1 * l1 + l2 * l2) >> 14)
    c2 = (num << 14) // (2 * ((l1 * l2) >> 14)) if num >= 0 else -(((-num) << 14) // (2 * ((l1 * l2) >> 14)))
    c2 = max(-PS + 1, min(PS - 1, c2))               # Q2.14
    s2 = int(math.isqrt(max(0, PS * PS - c2 * c2)))  # Q2.14 (sqrt block in RTL later)
    t2, _ = vm.cordic_atan2(s2, c2)                  # Q3.13
    k1 = l1 + ((l2 * c2) >> 14)
    k2 = (l2 * s2) >> 14
    a_phi, _ = vm.cordic_atan2(k2, k1)
    a_tgt, _ = vm.cordic_atan2(y, x)
    return a_tgt - a_phi, t2


def main():
    random.seed(3)
    rmin, rmax = abs(L1 - L2) + 0.03, L1 + L2 - 0.03
    pts = []
    for _ in range(500):
        r = random.uniform(rmin, rmax)
        a = random.uniform(-math.pi, math.pi)
        pts.append((r * math.cos(a), r * math.sin(a)))

    e_ik, e_rt, e_fk = [], [], []
    for xf, yf in pts:
        x, y = int(round(xf * PS)), int(round(yf * PS))
        # IK fixed vs float (angles, rad)
        t1f, t2f = ik_float(x / PS, y / PS)
        t1, t2 = ik_fixed(x, y)
        d1 = abs(t1 / AS - t1f)
        d1 = min(d1, 2 * math.pi - d1)
        e_ik.append(max(d1, abs(t2 / AS - t2f)))
        # FK fixed vs float, using the fixed angles
        xf2, yf2 = fk_float(t1 / AS, t2 / AS)
        xx, yy = fk_fixed(t1, t2)
        e_fk.append(max(abs(xx / PS - xf2), abs(yy / PS - yf2)))
        # round trip: target -> IK -> FK -> position error
        e_rt.append(math.hypot(xx - x, yy - y) / PS)

    os.makedirs("results", exist_ok=True)
    print("2-link IK/FK, L1=%.2f L2=%.2f, %d targets" % (L1, L2, len(pts)))
    print("IK angle error (fixed vs float): max %.2e rad" % max(e_ik))
    print("FK error (fixed vs float)      : max %.2e" % max(e_fk))
    print("Round-trip position error      : max %.2e (%.2f mm if reach = 550 mm)"
          % (max(e_rt), max(e_rt) / 0.55 * 550))
    print("PASS" if max(e_rt) < 2e-3 else "FAIL")


if __name__ == "__main__":
    main()
