import robot

rob = robot.Robot()

if not rob.start():
    print("Start error")

rob.stop()