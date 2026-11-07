import math, random

random.seed(7)
vals = [0, 1, 2, 3, 4, 15, 16, 17, 255, 256, 16383, 16384,
        (1 << 28) - 1, 1 << 28, (1 << 28) - 16384 * 16384 + 1]
vals = [v for v in vals if 0 <= v <= (1 << 28)]
vals += [random.randint(0, 1 << 28) for _ in range(2000)]

with open("tb/isqrt_vectors.hex", "w") as f:
    f.write("\n".join("%08x%08x" % (v, math.isqrt(v)) for v in vals) + "\n")
with open("tb/isqrt_count.txt", "w") as f:
    f.write("%d\n" % len(vals))
print("wrote %d sqrt vectors" % len(vals))
