import math
import numpy as np

# Degrees to Radian
def d2r(a):
    return a * math.pi

print("sin(0) =", math.sin(0*math.pi/180))
print("sin(+90) =", math.sin(90*math.pi/180))
print("sin(-90) =", math.sin(-90*math.pi/180))

print("cos(0) =", math.cos(0*math.pi/180))
print("cos(+90) =", math.cos(90*math.pi/180))
print("cos(-90) =", math.cos(-90*math.pi/180))