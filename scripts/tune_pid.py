import sys, itertools
sys.path.insert(0, "model")
import pid_model as pm
import fopid_model as fm

best = None
rows = []
for kp in (1024, 2048, 4096, 8192, 16384):
    for kd in (38400, 76800, 153600, 307200):
        for ki in (0, 13, 64):
            r = fm.evaluate(lambda: pm.PID(kp=kp, ki=ki, kd=kd))
            rows.append((kp, ki, kd, r))
            if r["os"] < 25.0 and r["sse"] <= 10:
                if best is None or r["rms"] < best[3]["rms"]:
                    best = (kp, ki, kd, r)

with open("results/pid_tuning_sweep.csv", "w") as f:
    f.write("kp_q8,ki_q8,kd_q8,overshoot_pct,settling_ms,sse_lsb,quintic_max,quintic_rms\n")
    for kp, ki, kd, r in rows:
        f.write("%d,%d,%d,%.3f,%d,%d,%d,%.2f\n" % (kp, ki, kd, r["os"], r["ts"], r["sse"], r["mx"], r["rms"]))

kp, ki, kd, r = best
print("Configs tried: %d" % len(rows))
print("Best PID: Kp=%.1f Ki=%.3f Kd=%.1f (Q8.8 %d,%d,%d)" % (kp/256, ki/256, kd/256, kp, ki, kd))
print("  overshoot %.2f%%, settling %d ms, sse %d LSB, quintic max %d, rms %.1f"
      % (r["os"], r["ts"], r["sse"], r["mx"], r["rms"]))
print("FO-PID (M=16) reference: quintic max 147, rms 65.7")
