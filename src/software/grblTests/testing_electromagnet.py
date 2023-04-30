import grblAPI
import math
import os, time


grbl = grblAPI.Grbl()

if not grbl.start():
    print("Unable to start grbl\nDo you have permissions?")
    exit(1)


	
magnetForce = int(input("Strength from 0 to 1000: "))

grbl.setSpindleSpeed(magnetForce)
grbl.enableSpindle()

print("Magnet enable for 4 secs")
time.sleep(4)
print("Magnet off")

grbl.disableSpindle()

grbl.stop()
