import math
sig = 0.0
try:
    k = 0.0001
    while True:
        if sig < -2.998 and sig > -3.03:
            break

        x = math.radians(5.0) * k
        right = (abs(math.sin(x))/x)
        sig = 10 * math.log10(right)
        k = k + 0.001

except KeyboardInterrupt:
    pass

print sig
print k
