"""Quintic trajectory generator, bit-accurate integer model (floor shifts like Verilog '>>>').

Angles: signed Q3.13.  Internal tau and polynomial: unsigned Q0.F.
  t  = k << (F - n)           (N = 2^n steps)
  t2 = (t*t)  >> F
  t3 = (t2*t) >> F
  s  = (t3 * (10<<F - 15*t + 6*t2)) >> F
  q  = q0 + (((q1 - q0) * s) >> F)
"""
import math, os, random

F = 20
VMAX = 200.0     # LSB per sample
AMAX = 4.0       # LSB per sample^2
NMIN_LOG2, NMAX_LOG2 = 6, 10


def traj(q0, q1, k, n, f=F):
    one = 1 << f
    t = k << (f - n)
    t2 = (t * t) >> f
    t3 = (t2 * t) >> f
    poly = (10 << f) - 15 * t + 6 * t2
    s = (t3 * poly) >> f
    return q0 + (((q1 - q0) * s) >> f)


def traj_float(q0, q1, k, n):
    tau = k / float(1 << n)
    s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    return q0 + (q1 - q0) * s


def min_log2n(dq, vmax=VMAX, amax=AMAX):
    dq = abs(dq)
    if dq == 0:
        return NMIN_LOG2
    n_v = 1.875 * dq / vmax
    n_a = math.sqrt(5.7735 * dq / amax)
    n_min = max(n_v, n_a)
    n = NMIN_LOG2
    while (1 << n) < n_min and n < NMAX_LOG2:
        n += 1
    return n


def make_moves():
    random.seed(21)
    PI = 25736
    moves = [(0, 0), (-3000, -2990), (-PI, PI), (PI, -PI), (100, 5000)]
    while len(moves) < 24:
        moves.append((random.randint(-PI, PI), random.randint(-PI, PI)))
    return moves


def sweep(moves):
    rows = []
    for f in range(10, 23, 2):
        worst = 0.0
        for q0, q1 in moves:
            n = min_log2n(q1 - q0)
            for k in range(0, (1 << n) + 1):
                worst = max(worst, abs(traj(q0, q1, k, n, f) - traj_float(q0, q1, k, n)))
        rows.append((f, worst))
    return rows


def main():
    moves = make_moves()
    os.makedirs("tb", exist_ok=True)
    os.makedirs("results", exist_ok=True)

    max_err, max_v, max_a, ends_ok = 0.0, 0.0, 0.0, True
    mlines, elines, total = [], [], 0
    for q0, q1 in moves:
        n = min_log2n(q1 - q0)
        N = 1 << n
        q = [traj(q0, q1, k, n) for k in range(N + 1)]
        if q[0] != q0 or q[N] != q1:
            ends_ok = False
        for k in range(N + 1):
            max_err = max(max_err, abs(q[k] - traj_float(q0, q1, k, n)))
        v = [abs(q[k + 1] - q[k]) for k in range(N)]
        a = [abs(q[k + 2] - 2 * q[k + 1] + q[k]) for k in range(N - 1)]
        max_v = max(max_v, max(v))
        max_a = max(max_a, max(a) if a else 0)
        mlines.append("%04x%04x%04x" % (q0 & 0xFFFF, q1 & 0xFFFF, n))
        elines += ["%04x" % (x & 0xFFFF) for x in q]
        total += N + 1

    with open("tb/traj_moves.hex", "w") as f:
        f.write("\n".join(mlines) + "\n")
    with open("tb/traj_expected.hex", "w") as f:
        f.write("\n".join(elines) + "\n")
    with open("tb/traj_count.txt", "w") as f:
        f.write("%d\n" % len(moves))

    rows = sweep(moves)
    with open("results/traj_fsweep.csv", "w") as f:
        f.write("frac_bits,max_pos_err_lsb\n")
        for fb, e in rows:
            f.write("%d,%.3f\n" % (fb, e))

    print("Quintic trajectory: %d moves, %d samples, F=%d" % (len(moves), total, F))
    print("F sweep (frac bits | max position error vs float, LSB):")
    for fb, e in rows:
        print("  %2d | %.3f" % (fb, e))
    print("Max position error : %.3f LSB (%.2e rad)" % (max_err, max_err / 8192.0))
    print("Peak velocity      : %.1f LSB/sample (limit %.0f)" % (max_v, VMAX))
    print("Peak accel (meas.) : %.1f LSB/sample^2 (limit %.0f, +-2 rounding noise)" % (max_a, AMAX))
    print("Endpoints exact    : %s" % ends_ok)
    ok = ends_ok and max_err <= 3.0 and max_v <= VMAX + 1 and max_a <= AMAX + 2
    print("PASS" if ok else "FAIL")


if __name__ == "__main__":
    main()
