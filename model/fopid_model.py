"""Fractional-order PID (Gruenwald-Letnikov, truncated memory M) vs plain PID, same plant/tests.

  e   = ref - pos                                   Q3.13
  I   = clamp( (sum_j wi[j]*e[k-j]) >> WS , +-I_MAX )   integral of order lambda (alpha=-lambda)
  D   = (sum_j wd[j]*e[k-j]) >> WS                      derivative of order mu   (alpha=+mu)
  u   = clamp( (KP*e + KI*I + KD*D) >> 8 , +-U_MAX )
Weights: w0=1, wj = w(j-1)*(1-(alpha+1)/j), quantized to Q.WS integers.
"""
import math, os, sys
import numpy as np
sys.path.insert(0, "model")
import pid_model as pm
import traj_model as tm

WS = 28
M_DEFAULT = 128


def gl_weights(alpha, M, ws=WS):
    w = [1.0]
    for j in range(1, M):
        w.append(w[-1] * (1.0 - (alpha + 1.0) / j))
    return np.array([int(round(x * (1 << ws))) for x in w], dtype=np.int64)


class FOPID:
    def __init__(self, lam=1.0, mu=1.0, M=M_DEFAULT):
        self.wi = gl_weights(-lam, M)
        self.wd = gl_weights(mu, M)
        self.hist = np.zeros(M, dtype=np.int64)

    def step(self, ref, pos):
        e = ref - pos
        self.hist[1:] = self.hist[:-1]
        self.hist[0] = e
        i = int(np.dot(self.wi, self.hist)) >> WS
        i = max(-pm.I_MAX, min(pm.I_MAX, i))
        d = int(np.dot(self.wd, self.hist)) >> WS
        u = (pm.KP * e + pm.KI * i + pm.KD * d) >> 8
        return max(-pm.U_MAX, min(pm.U_MAX, u))


def evaluate(make):
    target = 8192
    y = pm.run([target] * 1500, make())
    os_pct, ts = pm.step_metrics(y, target)
    sse = abs(y[-1] - target)
    q0, q1 = 0, 12288
    n = tm.min_log2n(q1 - q0)
    ref = [tm.traj(q0, q1, k, n) for k in range(1 << n)] + [q1] * 300
    yt = pm.run(ref, make())
    err = [a - b for a, b in zip(ref, yt)]
    mx = max(abs(x) for x in err)
    rms = math.sqrt(sum(x * x for x in err) / len(err))
    ys = pm.run([24000] * 2500, make())
    os_s, _ = pm.step_metrics(ys, 24000)
    return dict(os=os_pct, ts=ts, sse=sse, mx=mx, rms=rms, os_sat=os_s)


def main():
    os.makedirs("results", exist_ok=True)

    # self-check: lambda = mu = 1 weights equal plain PID
    assert all(v == (1 << WS) for v in gl_weights(-1.0, 16)), "integral weights != 1"
    wd = gl_weights(1.0, 8)
    assert wd[0] == (1 << WS) and wd[1] == -(1 << WS) and all(v == 0 for v in wd[2:]), "derivative weights"
    print("Self-check OK: lambda=mu=1 gives running-sum integral and e-e_prev derivative")

    base = evaluate(lambda: pm.PID())
    print("\nPlain PID baseline: overshoot %.2f%%, settling %d ms, sse %d LSB, "
          "quintic max %d / rms %.1f LSB, sat overshoot %.2f%%"
          % (base["os"], base["ts"], base["sse"], base["mx"], base["rms"], base["os_sat"]))

    grid = [0.6, 0.7, 0.8, 0.9, 1.0]
    rows = []
    print("\nFO-PID sweep (M=%d):" % M_DEFAULT)
    print(" lam   mu  | over%%  settle  sse | quintic max   rms")
    for lam in grid:
        for mu in grid:
            r = evaluate(lambda: FOPID(lam, mu))
            rows.append((lam, mu, r))
            print(" %.1f  %.1f | %5.2f  %6d  %3d | %10d  %6.1f"
                  % (lam, mu, r["os"], r["ts"], r["sse"], r["mx"], r["rms"]))

    ok_rows = [t for t in rows if t[2]["os"] < 25.0 and t[2]["sse"] <= 10]
    pool = ok_rows if ok_rows else rows
    lam_b, mu_b, best = min(pool, key=lambda t: t[2]["rms"])

    print("\nMemory-length sweep at lambda=%.1f, mu=%.1f:" % (lam_b, mu_b))
    print("   M | quintic rms  max | over%%  sse")
    msweep = []
    for M in (16, 32, 64, 128, 256):
        r = evaluate(lambda: FOPID(lam_b, mu_b, M))
        msweep.append((M, r))
        print(" %3d | %10.1f %5d | %5.2f  %3d" % (M, r["rms"], r["mx"], r["os"], r["sse"]))

    with open("results/fopid_sweep.csv", "w") as f:
        f.write("lambda,mu,overshoot_pct,settling_ms,sse_lsb,quintic_max_lsb,quintic_rms_lsb\n")
        for lam, mu, r in rows:
            f.write("%.1f,%.1f,%.3f,%d,%d,%d,%.2f\n" % (lam, mu, r["os"], r["ts"], r["sse"], r["mx"], r["rms"]))
    with open("results/fopid_memory_sweep.csv", "w") as f:
        f.write("M,quintic_rms_lsb,quintic_max_lsb,overshoot_pct,sse_lsb\n")
        for M, r in msweep:
            f.write("%d,%.2f,%d,%.3f,%d\n" % (M, r["rms"], r["mx"], r["os"], r["sse"]))
    with open("results/pid_vs_fopid.csv", "w") as f:
        f.write("controller,overshoot_pct,settling_ms,sse_lsb,quintic_max_lsb,quintic_rms_lsb,sat_overshoot_pct\n")
        f.write("PID,%.3f,%d,%d,%d,%.2f,%.3f\n" % (base["os"], base["ts"], base["sse"], base["mx"], base["rms"], base["os_sat"]))
        f.write("FO-PID (lam=%.1f mu=%.1f M=%d),%.3f,%d,%d,%d,%.2f,%.3f\n"
                % (lam_b, mu_b, M_DEFAULT, best["os"], best["ts"], best["sse"], best["mx"], best["rms"], best["os_sat"]))

    print("\n=== PID vs best FO-PID (lambda=%.1f, mu=%.1f, M=%d) ===" % (lam_b, mu_b, M_DEFAULT))
    print("              overshoot  settling  sse   quintic max  quintic rms")
    print("PID         : %7.2f%%  %6d ms  %3d  %10d  %10.1f" % (base["os"], base["ts"], base["sse"], base["mx"], base["rms"]))
    print("FO-PID      : %7.2f%%  %6d ms  %3d  %10d  %10.1f" % (best["os"], best["ts"], best["sse"], best["mx"], best["rms"]))
    print("PASS")


if __name__ == "__main__":
    main()
