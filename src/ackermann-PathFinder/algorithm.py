#!/usr/bin/python3
import sympy as sym
import matplotlib.pyplot as plt
import math
import sympy as sym


# Car constraints
CAR_WIDTH = 2
CAR_HEIGHT = 3.5
CAR_MAX_STEER_ANGLE = math.radians(30)

# Algorith params
N_CHILDS = 5 # Always odd number
DISTANCE_BTW_CHILDS = 0.5

x = sym.Symbol('x')
y = sym.Symbol('y')

class Waypoint:
    x = 0
    y = 0
    orientation = 0

def getCircunferenceExpresion(center_x, center_y, radius):
      
    ec = sym.expand((x - center_x)**2) + sym.expand((y - center_y)**2) - radius**2
    return ec

def circunferencesIntersection(circunference1_ec, circunference2_ec):
    
    intersection_line = circunference1_ec - circunference2_ec  
    solution = sym.solve((intersection_line, circunference1_ec), (x, y))
    
    return (sym.N(solution[0]), sym.N(solution[1]))

def distance(waypointA, waypointB):
    dx = waypointB.x - waypointA.x 
    dy = waypointB.y - waypointA.y
    
    return math.sqrt(dx**2 + dy**2)

def getChildWaypoint(waypoint, angle):
    
    axis_line = 0
    direction_line = 0
    rotation_centre = 0
    
    pass
    
    
    
    
    
    
def expand(waypoint):
    childs = []
    actions = []
    costs = []
    
    angle_increment = (2 * CAR_MAX_STEER_ANGLE) / (N_CHILDS - 1)
    
    for i in range(N_CHILDS):
        angle = -CAR_MAX_STEER_ANGLE + i * angle_increment
        
        child = getChildWaypoint(waypoint, angle)
        action = angle
        
        childs.append(child)
        actions.append(action)
 
    return (childs, actions, costs)

def showWaypoints(waypoints):
    
    for waypoint in waypoints:
        plt.plot([waypoint.x], [waypoint.y], 'ro')
    
    plt.show()
    
waypoint = Waypoint

waypoint.x = 1
waypoint.y = 2
waypoint.orientation = math.radians(45)
expand(1)

showWaypoints([waypoint])

