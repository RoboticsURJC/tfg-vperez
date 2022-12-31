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

precomputed_childs = []

x = sym.Symbol('x')
y = sym.Symbol('y')

class Waypoint:
    x = 0
    y = 0
    orientation = 0

def precomputeChilds():
    
    childs = []
    
    angle_increment = (2 * CAR_MAX_STEER_ANGLE) / (N_CHILDS - 1)
    
    for i in range(N_CHILDS):
        angle = -CAR_MAX_STEER_ANGLE + i * angle_increment
    
        # Get rear axle line
        # Because math is calculated in relative coord system, slope is always 0
        axle_line = y + CAR_LEN
                
        # Get direction line
        if math.isclose(angle, 0.0):
            # If is front
            child_point = (0, DISTANCE_BTW_CHILDS)    
        else: 
            aux_x, aux_y = (math.cos(angle + math.pi), math.sin(angle + math.pi))
            direction_line = (aux_y / (aux_x)) * x - y
            
            # Rotation circle
            solution = sym.solve((direction_line, axle_line), (x, y))
            rotation_circ = getCircunferenceExpresion(solution[x], solution[y], distance((solution[x], solution[y]), (0,0)))
            
            # Get both childs
            childs_circ = getCircunferenceExpresion(0, 0, DISTANCE_BTW_CHILDS)   
            solutions = circunferencesIntersection(childs_circ, rotation_circ)
            
            # Select only the valid one
            child_point = max(solutions[0], solutions[1], key=lambda tuple: tuple[1])
    
        childs.append(child_point)
    
    return childs

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
    rotx = px * math.cos(-angle) - py * math.sin(-angle)
    roty = px * math.sin(-angle) + py * math.cos(-angle)
    
    absx = relative_reference_system.x + rotx
    absy = relative_reference_system.y + roty
    
    return (absx,absy)

def showWaypoints(waypoints):
    
    for waypoint in waypoints:
        plt.plot([waypoint.x], [waypoint.y], 'ro')
    
    plt.show()
 

precomputed_childs = precomputeChilds()

'''
for child in precomputed_childs:
    plt.plot([child[0]], [child[1]], 'ro')

plt.plot([0], [0], 'r*')

plt.show()

'''