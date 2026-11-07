import math, random
random.seed(42)
L1, L2, PS = 0.30, 0.25, 16384
rmin, rmax = abs(L1 - L2) + 0.04, L1 + L2 - 0.04
pts = []
for i in range(24):
    r = rmin + (rmax - rmin) * (i % 6) / 5.0
    a = -math.pi * 0.9 + 2 * math.pi * 0.9 * random.random()
    pts.append((int(round(r * math.cos(a) * PS)), int(round(r * math.sin(a) * PS))))
with open("tb/arm_targets.hex", "w") as f:
    f.write("\n".join("%04x%04x" % (x & 0xFFFF, y & 0xFFFF) for x, y in pts) + "\n")
with open("tb/arm_count.txt", "w") as f:
    f.write("%d\n" % len(pts))
print("wrote %d targets" % len(pts))
