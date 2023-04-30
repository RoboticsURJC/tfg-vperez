import serial
import math, time
import threading as th

currentAngle = 0.0
currentSpeed = 0.0
currentState = 'UNDEFINED'
done = False

serialObj = None
port = '/dev/ttyS0'


def grblConnect():
    global serialObj
    try:
        serialObj = serial.Serial(port, 115200)
        print("Connected with grbl")
    except:
        print("Unable to connect with grbl...")
        exit(1)


def grblDisconnect():
    serialObj.close()
    print("Grbl disconnected")


def updateInfo():

    global currentAngle, currentSpeed, currentState

    while not done:
    
    
        if serialObj.isOpen():

            serialObj.write('?'.encode())

            data = str(serialObj.readline())

            if '<' in data:

                # Remove unnecesary characters
                msg = data[3:][:-6]

                fields = msg.split('|')

                currentState = fields[0]
                currentAngle = float(fields[1][5:].split(',')[0]) * 2.0 * math.pi
                currentSpeed = float(fields[2][3:].split(',')[0])

        else:
            print("Device disconnected manually!")

        time.sleep(0.25) 


def sendAngle(angle):
    
    order = "G01X" + str(angle/360.0) + "F8"
    
    serialObj.write(bytes(order.encode()))
    
    serialObj.write('\r'.encode())
    serialObj.write('\n'.encode())

def resetZero():

    wcoZero = "G10L2X0Y0Z0"
    useAbsoluteMM = "G21G90"
    
    
    serialObj.write(bytes(useAbsoluteMM.encode()))
    serialObj.write('\r'.encode())
    serialObj.write('\n'.encode())
    
    
    serialObj.write(bytes(wcoZero.encode()))
    serialObj.write('\r'.encode())
    serialObj.write('\n'.encode())

    
def main():
    global done
    
    grblConnect()

    infoThread = th.Thread(target=updateInfo)
   
    goalAngle = float(input("Introduce target angle in degrees: "))
    
    resetZero()
    sendAngle(goalAngle)
    
    infoThread.start()

    while not done:

        print(chr(27) + "[2J")  # Clean terminal
        
        print("State: " + currentState)
        print("Goal angle: " + str(goalAngle))
        print("Current angle: " + str(math.degrees(currentAngle)) + "")
        print("Current speed: " + str(currentSpeed) + " turns/min")
        time.sleep(0.1)
        
        done = math.isclose(currentAngle, math.radians(goalAngle))
    
    
    # Reprint result
    
    print(chr(27) + "[2J")  # Clean terminal
        
    print("State: " + currentState)
    print("Goal angle: " + str(goalAngle))
    print("Current angle: " + str(math.degrees(currentAngle)) + "")
    print("Current speed: " + str(currentSpeed) + " turns/min")
    time.sleep(0.1)
        
    print("Done")
            
    infoThread.join()  # Stop timer
    grblDisconnect()

    


if __name__ == "__main__":
    main()
