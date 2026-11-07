import sys
sys.path.insert(0, "model")
import pid_model as pm
import fopid_model as fm


class FOPIDg(fm.FOPID):
    def __init__(self, kp, ki, kd, lam, mu, M=16):
        super().__init__(lam, mu, M)
        self.g = (kp, ki, kd)

    def step(self, ref, pos):
        kp, ki, kd = self.g
        e = ref - pos
        self.hist[1:] = self.hist[:-1]
        self.hist[0] = e
        i = int(self.wi.dot(self.hist)) >> fm.WS
        i = max(-pm.I_MAX, min(pm.I_MAX, i))
        d = int(self.wd.dot(self.hist)) >> fm.WS
        u = (kp * e + ki * i + kd * d) >> 8
        return max(-pm.U_MAX, min(pm.U_MAX, u))


def safe_eval(make):
    try:
        return fm.evaluate(make)
    except (OverflowError, ValueError):
        return None


def ok(r):
    return r is not None and r["os"] < 25.0 and r["sse"] <= 10


# --- best PID (wider grid than before)
bp = None
for kp in (2048, 4096, 8192, 16384, 32768):
    for kd in (153600, 307200, 614400, 1228800):
        for ki in (0, 13, 64):
            r = safe_eval(lambda: pm.PID(kp=kp, ki=ki, kd=kd))
            if ok(r) and (bp is None or r["rms"] < bp[3]["rms"]):
                bp = (kp, ki, kd, r)

# --- best FO-PID (gains + orders)
bf = None
cnt = 0
for kp in (1024, 2048, 4096, 8192, 16384):
    for kd in (38400, 76800, 153600, 307200):
        for ki in (0, 13, 64):
            for lam in (0.6, 0.8, 1.0):
                for mu in (0.6, 0.8, 1.0):
                    cnt += 1
                    r = safe_eval(lambda: FOPIDg(kp, ki, kd, lam, mu))
                    if ok(r) and (bf is None or r["rms"] < bf[5]["rms"]):
                        bf = (kp, ki, kd, lam, mu, r)

print("PID configs tried: 60, FO-PID configs tried: %d" % cnt)
kp, ki, kd, r = bp
print("Best PID   : Kp=%.0f Ki=%.2f Kd=%.0f | rms %.1f max %d os %.2f%% sse %d"
      % (kp / 256, ki / 256, kd / 256, r["rms"], r["mx"], r["os"], r["sse"]))
kp2, ki2, kd2, lam, mu, r2 = bf
print("Best FO-PID: Kp=%.0f Ki=%.2f Kd=%.0f lam=%.1f mu=%.1f | rms %.1f max %d os %.2f%% sse %d"
      % (kp2 / 256, ki2 / 256, kd2 / 256, lam, mu, r2["rms"], r2["mx"], r2["os"], r2["sse"]))

# --- robustness: plant gain scaled
print("\nRobustness (plant gain KU scaled), quintic rms / overshoot%% / sse:")
ku0 = pm.KU
for scale in (0.5, 1.0, 2.0):
    pm.KU = ku0 * scale
    a = safe_eval(lambda: pm.PID(kp=kp, ki=ki, kd=kd))
    b = safe_eval(lambda: FOPIDg(kp2, ki2, kd2, lam, mu))
    fa = "unstable" if a is None else "%.1f / %.2f / %d" % (a["rms"], a["os"], a["sse"])
    fb = "unstable" if b is None else "%.1f / %.2f / %d" % (b["rms"], b["os"], b["sse"])
    print("  KU x%.1f | PID: %-22s | FO-PID: %s" % (scale, fa, fb))
pm.KU = ku0
