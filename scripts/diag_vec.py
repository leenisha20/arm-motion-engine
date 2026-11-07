import sys, math
sys.path.insert(0, "model")
import cordic_vec_model as m

rows = []
for k in range(360):
    a = -math.pi + 2 * math.pi * k / 360
    for r in (0.05, 0.15, 0.5):
        x, y = m.to_in(r * math.cos(a)), m.to_in(r * math.sin(a))
        ang, mag = m.cordic_atan2(y, x)
        d = abs(ang - math.atan2(y, x) * m.SCALE_ANG)
        ea = min(d, abs(d - 2 * m.PI_FX))
        em = abs(mag - m.K * math.hypot(x, y))
        rows.append((ea, em, r, a))

for r in (0.05, 0.15, 0.5):
    sub = [t for t in rows if t[2] == r]
    print("radius %.2f: max angle err %.1f LSB, max mag err %.1f LSB"
          % (r, max(t[0] for t in sub), max(t[1] for t in sub)))
worst = max(rows)
print("worst angle case: err %.1f LSB at radius %.2f, angle %.2f rad" % (worst[0], worst[2], worst[3]))
