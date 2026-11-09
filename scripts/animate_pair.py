import csv, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

L1, L2, AS = 0.30, 0.25, 8192.0
PS = 16384.0
BASE_A, BASE_B = (0.0, 0.0), (0.75, 0.0)   # drawing offset only
TGT_A = (6000 / PS, 3000 / PS)
TGT_B = (4500 / PS, 4000 / PS)

rows = []
with open("results/pair_trace.csv") as f:
    for r in csv.DictReader(f):
        rows.append([int(r[k]) for k in ("clk", "p1a", "p2a", "p1b", "p2b")])
assert len(rows) > 10, "trace too short, run ./scripts/run_pair.sh first"

def arm(base, t1, t2):
    x1 = base[0] + L1 * math.cos(t1)
    y1 = base[1] + L1 * math.sin(t1)
    x2 = x1 + L2 * math.cos(t1 + t2)
    y2 = y1 + L2 * math.sin(t1 + t2)
    return [base[0], x1, x2], [base[1], y1, y2]

fig, ax = plt.subplots(figsize=(7, 4))
ax.set_xlim(-0.65, 1.4)
ax.set_ylim(-0.65, 0.65)
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)
ax.set_title("Two-arm leader-follower (RTL simulation joint angles)")
ax.plot(TGT_A[0] + BASE_A[0], TGT_A[1] + BASE_A[1], "rx", ms=10, label="target A")
ax.plot(TGT_B[0] + BASE_B[0], TGT_B[1] + BASE_B[1], "bx", ms=10, label="target B")
la, = ax.plot([], [], "o-", lw=4, color="tab:red")
lb, = ax.plot([], [], "o-", lw=4, color="tab:blue")
ta, = ax.plot([], [], "-", lw=1, color="tab:red", alpha=0.5)
tb, = ax.plot([], [], "-", lw=1, color="tab:blue", alpha=0.5)
txt = ax.text(-0.6, 0.55, "", fontsize=9)
ax.legend(loc="upper right", fontsize=8)
hist_a, hist_b = [], []

def update(i):
    clk, p1a, p2a, p1b, p2b = rows[i]
    xa, ya = arm(BASE_A, p1a / AS, p2a / AS)
    xb, yb = arm(BASE_B, p1b / AS, p2b / AS)
    la.set_data(xa, ya)
    lb.set_data(xb, yb)
    hist_a.append((xa[2], ya[2]))
    hist_b.append((xb[2], yb[2]))
    ta.set_data([p[0] for p in hist_a], [p[1] for p in hist_a])
    tb.set_data([p[0] for p in hist_b], [p[1] for p in hist_b])
    txt.set_text("clock %d" % clk)
    return la, lb, ta, tb, txt

anim = FuncAnimation(fig, update, frames=len(rows), interval=50, blit=False)
anim.save("docs/arm_pair_demo.gif", writer=PillowWriter(fps=20), dpi=70)
print("saved docs/arm_pair_demo.gif with %d frames" % len(rows))
