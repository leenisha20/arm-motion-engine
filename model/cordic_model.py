"""Bit-accurate fixed-point CORDIC golden model (rotation mode, sin/cos).

Format: signed Q2.14 in a 16-bit word (1.0 = 16384).
Valid input range: |theta| <= pi/2 (quadrant reduction comes later).
"""
import math

W = 16          # word length
FRAC = 14       # fractional bits
ITER = 14       # CORDIC iterations
SCALE = 1 << FRAC


def to_fx(v):
    return int(round(v * SCALE))


def to_float(v):
    return v / SCALE


ATAN = [to_fx(math.atan(2.0 ** -i)) for i in range(ITER)]

K = 1.0
for _i in range(ITER):
    K *= math.sqrt(1.0 + 2.0 ** (-2 * _i))
X0 = to_fx(1.0 / K)


def cordic_sincos(theta_fx, iters=ITER):
    """Returns (cos, sin) in Q2.14 for theta in Q2.14 radians."""
    x, y, z = X0, 0, theta_fx
    for i in range(iters):
        if z >= 0:
            x, y, z = x - (y >> i), y + (x >> i), z - ATAN[i]
        else:
            x, y, z = x + (y >> i), y - (x >> i), z + ATAN[i]
    return x, y


def main():
    import os
    n = 201
    thetas = [-math.pi / 2 + i * math.pi / (n - 1) for i in range(n)]
    errs_c, errs_s, lines = [], [], []

    for t in thetas:
        t_fx = to_fx(t)
        c, s = cordic_sincos(t_fx)
        ref_c = math.cos(to_float(t_fx))
        ref_s = math.sin(to_float(t_fx))
        errs_c.append(abs(c - ref_c * SCALE))
        errs_s.append(abs(s - ref_s * SCALE))
        lines.append("%04x%04x%04x" % (t_fx & 0xFFFF, c & 0xFFFF, s & 0xFFFF))

    os.makedirs("tb", exist_ok=True)
    os.makedirs("results", exist_ok=True)
    with open("tb/cordic_vectors.hex", "w") as f:
        f.write("\n".join(lines) + "\n")

    max_c, max_s = max(errs_c), max(errs_s)
    print("Word: Q%d.%d, iterations: %d, gain K = %.6f" % (W - FRAC, FRAC, ITER, K))
    print("Max |error| cos: %.2f LSB (%.2e)" % (max_c, max_c / SCALE))
    print("Max |error| sin: %.2f LSB (%.2e)" % (max_s, max_s / SCALE))
    print("Vectors written: %d -> tb/cordic_vectors.hex" % n)
    print("PASS" if max(max_c, max_s) <= 8 else "FAIL (error above 8 LSB)")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.figure(figsize=(7, 3.5))
        plt.plot(thetas, errs_c, label="cos error")
        plt.plot(thetas, errs_s, label="sin error")
        plt.xlabel("angle (rad)")
        plt.ylabel("abs error (LSB)")
        plt.title("CORDIC error vs ideal (Q2.14, %d iterations)" % ITER)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig("results/cordic_error.png", dpi=150)
        print("Plot saved -> results/cordic_error.png")
    except ImportError:
        print("matplotlib not found, skipping plot")


if __name__ == "__main__":
    main()
