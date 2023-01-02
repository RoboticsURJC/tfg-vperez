#!/usr/bin/python3
import sympy as sym
import matplotlib.pyplot as plt
import math
import sympy as sym


# Car constraints
CAR_WIDTH = 2
CAR_LEN = 3.5
CAR_MAX_STEER_ANGLE = math.radians(30)

# Algorith params
N_CHILDS = 5 # Always odd number
DISTANCE_BTW_CHILDS = 0.5

precomputed = []

x = sym.Symbol('x')
y = sym.Symbol('y')

class ReferenceSystem:
    x = 0
    y = 0
    orientation = 0

def precomputeChilds():
    
    childs = []
    costs = []
    steer_angles = []
    
    angle_increment = (2 * CAR_MAX_STEER_ANGLE) / (N_CHILDS - 1)
    
    for i in range(N_CHILDS):
        angle = -CAR_MAX_STEER_ANGLE + i * angle_increment
    
        # Get rear axle line
        # Because math is calculated in relative coord system, slope is always 0
        axle_line = y + CAR_LEN
        ideal_cost = DISTANCE_BTW_CHILDS  
              
        # Get direction line
        if math.isclose(angle, 0.0):
            # If is front
            child_point = (0, DISTANCE_BTW_CHILDS)   
            cost = ideal_cost 
            child_orientation = 0
            
        else: 
            aux_x, aux_y = (math.cos(angle + math.pi), math.sin(angle +  math.pi))
            direction_line = (aux_y / (aux_x)) * x - y
            
            # Rotation circle
            center = sym.solve((direction_line, axle_line), (x, y))
            rotation_circ = getCircunferenceExpresion(center[x], center[y], distance((center[x], center[y]), (0,0)))
            
            # Get both childs
            childs_circ = getCircunferenceExpresion(0, 0, DISTANCE_BTW_CHILDS)   
            solutions = circunferencesIntersection(childs_circ, rotation_circ)
            
            # Select only the valid one
            child_point = max(solutions[0], solutions[1], key=lambda tuple: tuple[1])
            
            # Compute child final orientation
            child_slope =  -1 / ((child_point[1] - center[y]) / (child_point[0] - center[x]))
                     
            child_orientation = math.atan(child_slope)
         
            # Because atan returns angle between 90 and -90 degrees
            if angle > 0:
                child_orientation += math.pi        
            
            child_orientation -= math.pi/2 # Taking Y axis as angle start
            
            # Compute cost
            # "Move" center of trajectory to 0,0 (Traslating all points)
            dx = -center[x]
            dy = -center[y]
            
            rot_center = (0,0)
            parent = (dx, dy)
            child = (child_point[0] + dx, child_point[1] + dy)
            
            beta = math.atan2(child[1], child[0])
            alpha = math.atan2(parent[1], parent[0])
            theta = abs(beta - alpha)
            
            # (theta / 2pi) * 2pi*r
            real_cost = theta * distance(parent, rot_center)
            
            # Amplify the cost
            cost = ((real_cost - ideal_cost) * 4000) + real_cost
        
        print(child_orientation)    
               
        childs.append((child_point, child_orientation))
        costs.append(cost)
        steer_angles.append(angle)
        
    
    return (childs, costs, steer_angles)

def getCircunferenceExpresion(center_x, center_y, radius):
      
    ec = sym.expand((x - center_x)**2) + sym.expand((y - center_y)**2) - radius**2
    return ec

def circunferencesIntersection(circunference1_ec, circunference2_ec):
    
    intersection_line = circunference1_ec - circunference2_ec  
    solution = sym.solve((intersection_line, circunference1_ec), (x, y))
    return (solution[0], solution[1])

def distance(A, B):
    dx = B[0] - A[0]
    dy = B[1] - A[1]
    
    return math.sqrt(dx**2 + dy**2)

def rel2abs(point, relative_reference_system):
    px, py = point
    
    angle = relative_reference_system.orientation
       
    # Rotate with current angle
    rotx = px * math.cos(angle) - py * math.sin(angle)
    roty = px * math.sin(angle) + py * math.cos(angle)
    
    absx = relative_reference_system.x + rotx
    absy = relative_reference_system.y + roty
    
    return (absx,absy)

def showWaypoints(waypoints):
    
    for waypoint in waypoints:
        plt.plot([waypoint.x], [waypoint.y], 'ro')
    
    plt.show()
 

def expand(parent):
    
    childs = []
  
    pre_childs, _, _ = precomputed
    
    for i in range(N_CHILDS):
        
        child = ReferenceSystem()
    
        pre_child_pose = pre_childs[i][0]
        pre_child_orientation = pre_childs[i][1]
        
        child_pose = rel2abs(pre_child_pose, parent)
        
        child.x = child_pose[0]
        child.y = child_pose[1]
        
        child.orientation = parent.orientation + pre_child_orientation
    
        childs.append(child)
        
        
    return childs
     
precomputed = precomputeChilds()


parent = ReferenceSystem()
parent.x = 2
parent.y = 3
parent.orientation = math.radians(45)

childs = expand(parent)

plt.plot([2], [3], 'r*')

for child in childs:
    
    #print(str(child.x) + " " + str(child.y) + " " + str(child.orientation) )
    
    plt.plot([child.x], [child.y], 'ro')
    

new_childs = expand(childs[-1])

for new_child in new_childs:

    
    plt.plot([new_child.x], [new_child.y], 'bo')
    


plt.xlim([0, 5])
plt.ylim([0, 5])
plt.axis('equal')


plt.show()



'''
for child in precomputed_childs[0]:
    
    px, py = child[0]
    plt.plot([px], [py], 'ro')

plt.plot([0], [0], 'r*')

plt.show()
'''