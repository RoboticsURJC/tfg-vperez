import robot


rob = robot.Robot()

if not rob.start():
    print("Failed to start")
    exit(1)
    

    
rob.stop()