import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

cyc, ref, pos = [], [], []
with open("results/loop1_trace.csv") as f:
    for row in csv.DictReader(f):
        cyc.append(int(row["cycle"]))
        ref.append(int(row["ref"]) / 8192.0)
        pos.append(int(row["pos"]) / 8192.0)
err = [(a - b) * 1000 for a, b in zip(ref, pos)]

fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
ax[0].plot(cyc, ref, label="reference (quintic)")
ax[0].plot(cyc, pos, label="joint position")
ax[0].set_ylabel("angle (rad)")
ax[0].legend()
ax[0].grid(True)
ax[1].plot(cyc, err)
ax[1].set_ylabel("error (mrad)")
ax[1].set_xlabel("sample")
ax[1].grid(True)
fig.suptitle("Hardware closed loop: traj_gen + PID + DC-motor plant")
fig.tight_layout()
fig.savefig("results/loop1_tracking.png", dpi=150)
print("Plot saved -> results/loop1_tracking.png")
