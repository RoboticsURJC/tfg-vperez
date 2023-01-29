#!/usr/bin/python3
import sympy as sym
import matplotlib.pyplot as plt
import math
import sympy as sym
import utils


# Car constraints
CAR_WIDTH = 2 # Meters
CAR_LEN = 10 # Meters
CAR_MAX_STEER_ANGLE = math.radians(30) # Radians. Steer range: -value to +value

# Algorith params
N_CHILDS = 9 # Always odd number
DISTANCE_BTW_CHILDS = 1 # Meters
GOAL_THRESHOLD = 0.5 # Meters
MAX_ASTAR_ITERATIONS = 10000

precomputed = []

id_count = 0 # First ID is 0

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
            print(str(center[x]) + " " + str(center[y]))
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
            cost = ((real_cost - ideal_cost) * 1000) + real_cost
        
               
        childs.append((child_point, child_orientation))
        print(cost)
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

def heuristic(actual, goal):
    
    dx = goal.x - actual.x
    dy = goal.y - actual.y
    
    return math.sqrt(dx**2 + dy**2)
    
    
    
'''
Expands the given waypoint and return a list of:
(child_waypoint, cost_to_reach_him, steer_to_reach_him, child_id)
'''
def new_expand(parent_waypoint, parent_steer):
    global id_count
    
    precomputed_waypoints, precomputed_costs, precomputed_steers = precomputed
    
    childs_data = []
    
    max_steer = max(precomputed_steers)
    
    for child_index in range(N_CHILDS):
        
        child_steer = precomputed_steers[child_index]
        
  
        # Only generated a valid child taking care of steer limitations
        
        # TODO Manera cutre de decir mayor o igual a 0 en floats: CORREGIR esto es para pruebas
        if max_steer - (abs(child_steer) + abs(parent_steer)) > -0.01 :
               
            # Compute the absolute child   
            child_waypoint = ReferenceSystem()
                
            rel_child_xy = precomputed_waypoints[child_index][0]
            rel_child_theta = precomputed_waypoints[child_index][1]
        
            child_x, child_y = rel2abs(rel_child_xy, parent_waypoint)
            
            # Fill new child data
            child_waypoint.x = child_x
            child_waypoint.y = child_y
            child_waypoint.orientation = parent_waypoint.orientation + rel_child_theta
            
            id_count += 1
            
            childs_data.append( (child_waypoint, precomputed_costs[child_index], child_steer, id_count) )
            
        
    return childs_data
    
def new_AstarSearch(start_waypoint, end_waypoint):
    
    frontier = utils.PriorityQueue() # Frontier data: (waypoint, parents <list>, costs <list> , steers <lists>, id)
    explored = []
    
    # Start
    frontier.push( (start_waypoint, [start_waypoint], [0], [0], id_count), 0)
    
    iterations = 0
    
    while not frontier.isEmpty():
        
        waypoint, parents, costs, steers, waypoint_id = frontier.pop()
        
        if isGoal(waypoint, end_waypoint) or iterations == MAX_ASTAR_ITERATIONS:
            return parents
            
        
        # If is an unexplored node
        if waypoint_id not in explored:
            
            explored.append(waypoint) # Add it
            
            childs = new_expand(waypoint, steers[-1])
                                  
            for child_data in childs:
                
                child_waypoint, child_cost, child_steer, child_id = child_data
                       
                child_cost += heuristic(child_waypoint, end_waypoint) # Add heuristic
                
                child_parents = parents.copy()
                child_parents.append(child_waypoint)
                
                child_costs = costs.copy()
                child_costs.append(child_cost)
                
                child_steers = steers.copy()
                child_steers.append(child_steer)
                
                frontier.push( (child_waypoint, child_parents, child_costs, child_steers, child_id) , child_cost)
                
            
        iterations += 1
        
    
    print("A star search failed!")
    return []

def expand(parent, parent_steer):
    
    childs = [] # List of waypoints
    angles = []
  
    pre_childs, _, pre_steers = precomputed
    
    for i in range(N_CHILDS):
        # Optimización necesaria. Usar el rango al máximo TODO
        valid_node = False
        
        if parent_steer > 0:
            max_angle_in_next = max(pre_steers) - parent_steer
            
            if pre_steers[i] < max_angle_in_next:
                valid_node = True
            
        else:
            min_angle_in_next = min(pre_steers) + parent_steer
            
            if pre_steers[i] > min_angle_in_next:
                valid_node = True
                    
        
        if valid_node:
            child = ReferenceSystem()
            
            pre_child_pose = pre_childs[i][0]
            pre_child_orientation = pre_childs[i][1]
            
            child_pose = rel2abs(pre_child_pose, parent)
            
            child.x = child_pose[0]
            child.y = child_pose[1]
            
            child.orientation = parent.orientation + pre_child_orientation
        
            childs.append(child)
            angles.append(pre_steers[i])
        
        
    return (childs, angles)

def isGoal(actual, goal):
    
    return distance((actual.x, actual.y), (goal.x, goal.y)) < GOAL_THRESHOLD

def waypointInList(waypoint, input_list):
    
    for wp in input_list:
        
        if math.isclose(wp.x, waypoint.x) and math.isclose(wp.y, waypoint.y) and math.isclose(wp.orientation, waypoint.orientation):
            return True
         
    return False

def AStarSearch(start, goal):
     
    _, precomputed_costs, precomputed_angles = precomputed
    
    frontier = utils.PriorityQueue()
    explored = []

    # Node structure -> (actual_waypoint, [(child, total_cost, steer_angle), ...]
    start_node = (start, [(start, 0, 0)])
    frontier.push(start_node, 0) # (node, cost)

    iterations = 0
    
    while not frontier.isEmpty():
        
        iterations += 1
        
        actual_waypoint, data = frontier.pop()
        
        if isGoal(actual_waypoint, goal) or iterations > 10000:
            return data
        
        if not waypointInList(actual_waypoint, explored):
            
            explored.append(actual_waypoint)

            childs, angles = expand(actual_waypoint, data[-1][2])
            
            
            actual_cost = data[-1][1]
            
            child_id = 0
            for child in childs:
                
                new_cost = 1 #actual_cost + precomputed_costs[child_id]
                
                #TODO este angulo no esta bien. NO funciona ya asi hay que guardarse el real.
                new_angle = angles[child_id]
                
                new_data = data.copy() 
                new_data.append( (child, new_cost, new_angle)) 

                estimated_cost =  new_cost + heuristic(child, goal)
                
                node = (child, new_data)
                frontier.push(node, estimated_cost)
                
                child_id += 1
                  
    return []

def drawCar(waypoint):
    points = [(0,0), (CAR_WIDTH/2, 0), (CAR_WIDTH/2, 0.25), (CAR_WIDTH/2, -0.25), (CAR_WIDTH/2, 0),
            (-CAR_WIDTH/2, 0), (-CAR_WIDTH/2, 0.25), (-CAR_WIDTH/2, -0.25), (-CAR_WIDTH/2, 0),
            (0, 0), (0, -CAR_LEN), (CAR_WIDTH/2, -CAR_LEN), (CAR_WIDTH/2, -CAR_LEN + 0.25), (CAR_WIDTH/2, -CAR_LEN -0.25), (CAR_WIDTH/2, -CAR_LEN),
            (-CAR_WIDTH/2, -CAR_LEN), (-CAR_WIDTH/2, -CAR_LEN + 0.25), (-CAR_WIDTH/2, -CAR_LEN -0.25), (-CAR_WIDTH/2, -CAR_LEN)]

    x = []
    y = []

    for p in points:
        
        absolute = rel2abs(p, waypoint)
        
        x.append(absolute[0])
        y.append(absolute[1])
    
    plt.plot(x, y, 'b-')

precomputed = precomputeChilds()


start = ReferenceSystem()
start.x = 2
start.y = 3
start.orientation = math.radians(135)


goal = ReferenceSystem()
goal.x = 8
goal.y = 8
goal.orientation = math.radians(150)


waypoints = new_AstarSearch(start, goal)

plt.plot([start.x], [start.y], 'r*')
plt.plot([goal.x], [goal.y], 'r*')

if len(waypoints) > 0:
    
    for waypoint in waypoints:
        plt.plot([waypoint.x], [waypoint.y], 'bo')
    
else:
    print("Path not found")   

drawCar(start)

plt.xlim(-10, 20)
plt.ylim(-10, 10)

ax = plt.gca().set_aspect('equal', adjustable='box')

plt.show()

'''
parent = ReferenceSystem()
parent.x = 2
parent.y = 3
parent.orientation = math.radians(45)

childs = expand(parent)

plt.plot([2], [3], 'r*')

for child in childs:
    
    #print(str(child.x) + " " + str(child.y) + " " + str(child.orientation) )
    
    plt.plot([child.x], [child.y], 'ro')
    

new_childs = expand(childs[-4])

for new_child in new_childs:

    
    plt.plot([new_child.x], [new_child.y], 'bo')
    


plt.xlim([0, 5])
plt.ylim([0, 5])
plt.axis('equal')


plt.show()




for child in precomputed_childs[0]:
    
    px, py = child[0]
    plt.plot([px], [py], 'ro')

plt.plot([0], [0], 'r*')

plt.show()
'''