#!/usr/bin/python3
import sympy as sym
import matplotlib.pyplot as plt
import math
# Car constraints
CAR_WIDTH = 2
CAR_HEIGHT = 3.5
CAR_MAX_STEER_ANGLE = math.radians(30)

# Algorith params
NODE_CHILDS = 5
DISTANCE_BTW_CHILDS = 0.5

class Waypoint:
    x = 0
    y = 0
    orientation = 0

class Circunsference:
    x = 0
    y = 0
    radius = 0

def circunferenceIntersectPoint(circ1, circ2):
    pass
 
def expand(waypoint):
    
    angle_increment = (2 * CAR_MAX_STEER_ANGLE) / (NODE_CHILDS-1)
    
    for i in range(NODE_CHILDS):
        pass
        


def showWaypoints(waypoints):
    
    for waypoint in waypoints:
        plt.plot([waypoint.x], [waypoint.y], 'ro')
    
    plt.show()
    
waypoint = Waypoint

waypoint.x = 1
waypoint.y = 2
waypoint.orientation = math.radians(45)

showWaypoints([waypoint])

