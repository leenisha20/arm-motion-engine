import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

n, ref, a, b = [], [], [], []
with open("results/loop2_trace.csv") as f:
    for row in csv.DictReader(f):
        n.append(int(row["n"]))
        ref.append(int(row["ref"]) / 8192.0)
        a.append(int(row["pid"]) / 8192.0)
        b.append(int(row["fopid"]) / 8192.0)
ea = [(r - x) * 1000 for r, x in zip(ref, a)]
eb = [(r - x) * 1000 for r, x in zip(ref, b)]

fig, ax = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
ax[0].plot(n, ref, "k--", label="reference")
ax[0].plot(n, a, label="PID")
ax[0].plot(n, b, label="FO-PID")
ax[0].set_ylabel("angle (rad)")
ax[0].legend()
ax[0].grid(True)
ax[1].plot(n, ea, label="PID")
ax[1].plot(n, eb, label="FO-PID")
ax[1].set_ylabel("tracking error (mrad)")
ax[1].set_xlabel("controller sample")
ax[1].legend()
ax[1].grid(True)
fig.suptitle("Hardware loop: PID vs fractional-order PID, same plant")
fig.tight_layout()
fig.savefig("results/loop2_pid_vs_fopid.png", dpi=150)
print("Plot saved -> results/loop2_pid_vs_fopid.png")
