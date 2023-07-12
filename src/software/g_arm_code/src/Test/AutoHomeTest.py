import sys

sys.path.append('..') # Added to import robot class that is in other folder

import robot

rob = robot.Robot()

# At start, robot does the autohome
if not rob.start():
    print("Start error")

print(rob.getAngles())

rob.stop()