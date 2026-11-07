"""Bit-accurate CORDIC vectoring-mode golden model: atan2 and magnitude.

Inputs  x, y : signed Q2.14 (1.0 = 16384)
Outputs angle: signed Q3.13 (1.0 = 8192), range +-pi
        mag  : signed Q2.14 = K * sqrt(x^2+y^2) (gain K NOT removed)
Internally x,y carry GUARD extra fractional bits to cut truncation error.
"""
import math, os, random

FRAC_IN = 14
FRAC_ANG = 13
ITER = 14
GUARD = 4
MIN_MAG = 800          # skip vectors with |x|,|y| both below this (~0.05)
SCALE_IN = 1 << FRAC_IN
SCALE_ANG = 1 << FRAC_ANG


def to_in(v):
    return int(round(v * SCALE_IN))


def to_ang(v):
    return int(round(v * SCALE_ANG))


ATAN = [to_ang(math.atan(2.0 ** -i)) for i in range(ITER)]
PI_FX = to_ang(math.pi)

K = 1.0
for _i in range(ITER):
    K *= math.sqrt(1.0 + 2.0 ** (-2 * _i))


def cordic_atan2(y_fx, x_fx, guard=GUARD, iters=ITER):
    """Returns (angle_Q3.13, mag_Q2.14 including gain K)."""
    z = 0
    if x_fx < 0:
        x_fx, y_fx = -x_fx, -y_fx
        z = PI_FX if y_fx <= 0 else -PI_FX
    x, y = x_fx << guard, y_fx << guard
    for i in range(iters):
        if y < 0:
            x, y, z = x - (y >> i), y + (x >> i), z - ATAN[i]
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + ATAN[i]
    return z, x >> guard


def make_points():
    random.seed(1)
    pts = []
    for k in range(360):
        a = -math.pi + 2 * math.pi * k / 360
        for r in (0.1, 0.15, 0.5):
            pts.append((r * math.cos(a), r * math.sin(a)))
    for _ in range(40):
        pts.append((random.uniform(-0.55, 0.55), random.uniform(-0.55, 0.55)))
    out = []
    for xf, yf in pts:
        x_fx, y_fx = to_in(xf), to_in(yf)
        if abs(x_fx) < MIN_MAG and abs(y_fx) < MIN_MAG:
            continue
        out.append((x_fx, y_fx))
    return out


def errors(pts, guard):
    ea_l, em_l = [], []
    for x_fx, y_fx in pts:
        ang, mag = cordic_atan2(y_fx, x_fx, guard)
        d = abs(ang - math.atan2(y_fx, x_fx) * SCALE_ANG)
        ea_l.append(min(d, abs(d - 2 * PI_FX)))
        em_l.append(abs(mag - K * math.hypot(x_fx, y_fx)))
    return max(ea_l), max(em_l)


def main():
    pts = make_points()
    os.makedirs("tb", exist_ok=True)
    os.makedirs("results", exist_ok=True)

    print("Guard-bit sweep (%d vectors):" % len(pts))
    print("guard | max angle err (LSB) | max mag err (LSB)")
    rows = []
    for g in range(0, 7):
        ma, mm = errors(pts, g)
        rows.append((g, ma, mm))
        print("%5d | %19.2f | %17.2f" % (g, ma, mm))
    with open("results/vec_guard_sweep.csv", "w") as f:
        f.write("guard,max_angle_err_lsb,max_mag_err_lsb\n")
        for g, ma, mm in rows:
            f.write("%d,%.2f,%.2f\n" % (g, ma, mm))

    lines = []
    for x_fx, y_fx in pts:
        ang, mag = cordic_atan2(y_fx, x_fx, GUARD)
        lines.append("%04x%04x%04x%04x" % (x_fx & 0xFFFF, y_fx & 0xFFFF,
                                          ang & 0xFFFF, mag & 0xFFFF))
    with open("tb/cordic_vec_vectors.hex", "w") as f:
        f.write("\n".join(lines) + "\n")
    with open("tb/cordic_vec_count.txt", "w") as f:
        f.write("%d\n" % len(pts))

    ma, mm = errors(pts, GUARD)
    print("\nChosen GUARD = %d: angle %.2f LSB (%.2e rad), mag %.2f LSB"
          % (GUARD, ma, ma / SCALE_ANG, mm))
    print("PASS" if ma <= 8 and mm <= 8 else "FAIL")


if __name__ == "__main__":
    main()
