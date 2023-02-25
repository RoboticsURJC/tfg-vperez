import grblAPI
import math
import os, time


grbl = grblAPI.Grbl()

if not grbl.start():
    print("Unable to start grbl\nDo you have permissions?")
    exit(1)

goal = float(input("Introduce X position: "))

grbl.goToXYZ(goal, 0, 0, feedrate = 8)

done = False

while not done:
    
    x, _, _ = grbl.getXYZ()
    
    os.system('clear')
    
    print("Status: " + grbl.getStatus())
    print("Position: " + str(x))
        
    time.sleep(0.05)
    
    if math.isclose(x, goal):
        done = True
        
print("Done")

grbl.stop()
