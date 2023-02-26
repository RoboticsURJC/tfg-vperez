import grblAPI
import math
import os, time


grbl = grblAPI.Grbl()

if not grbl.start():
    print("Unable to start grbl\nDo you have permissions?")
    exit(1)

if grbl.loadConfig("grblConfig.txt"):
    print("Configuration loaded successfully!")
        
print("Done")

grbl.stop()
