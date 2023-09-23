import math 
import numpy as np

# Robot constraints
a = 0.132 
b = 0.032
c = 0.01
d = 0.17 
e = 0.17 
end_tx = 0.03
end_tz = 0.01

def IK(x, y, z):
    pass

def DK(j1, j2, j3):

    # Obtained from DH table
    x=b*math.cos(j1) - c*math.sin(j1) - d*math.cos(j1)*math.sin(j2) + e*math.cos(j1)*math.cos(j2)*math.cos(j3) - e*math.cos(j1)*math.sin(j2)*math.sin(j3)
    y=c*math.cos(j1) + b*math.sin(j1) - d*math.sin(j1)*math.sin(j2) + e*math.cos(j2)*math.cos(j3)*math.sin(j1) - e*math.sin(j1)*math.sin(j2)*math.sin(j3)
    z=a + e*math.sin(j2 + j3) + d*math.cos(j2)
    # End efector traslation
    x += end_tx
    z += end_tz
    return (x, y, z)
