import math

L1 = 0.17 # Lenght of link 1 in meters
L2 = 0.17 # Lenght of link 1 in meters

# Resolve inverse kinematics for a position of the end-effector relative to ground 0.0 
def IK(point, tcpOffset):
   x, y, z = point
   
   J1 = 0
   
   num = math.pow(x, 2) + math.pow(z, 2) - math.pow(L1, 2) - math.pow(L2, 2) 
   den = -2.0 * math.pow(L2, 2) 
   J2 = math.acos(math.sqrt(num / den))
   
   J3 = math.acos((x + L2 * math.cos(J2)) / L1)
   
   return (J1, J2, J3)

def DK(angles, tcpOffset):
    pass

print(IK((0.2,0,0.2), None))