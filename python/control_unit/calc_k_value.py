import math
x = 0.0
try:
    k = 0.0001
    while True:
        if x > 2.9999 and x < 3.1:
            break
        x = math.acos(0.08727) * k
        k = k + 0.001
except KeyboardInterrupt:
    pass

print x
print k
