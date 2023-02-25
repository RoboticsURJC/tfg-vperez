import grblAPI
import math

grbl = grblAPI.Grbl()

if not grbl.start():
    print("Unable to start grbl\nDo you have permissions?")
    exit(1)
    
grbl.goToXYZ(1, 0, 0, feedrate = 8)

while not math.isclose(grbl.getXYZ()[0], 1.0):
    
    print("Status: " + grbl.getStatus())
    print("Position: " + grbl.getXYZ()[0])

grbl.stop()