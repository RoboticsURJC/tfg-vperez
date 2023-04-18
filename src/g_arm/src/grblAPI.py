import serial
import time
import threading as th

STATUS_REPORT_FREQUENCY = 5 # Hz

RECEIVE_TIMEOUT = 0.1 # Secs
HOMING_TIMEOUT = 8 # Secs

class Grbl:

    # Private
    __machinePosition = "0.0,0.0,0.0"
    __machineStatus = "Undefined"
    __machineSwitchStatus = ""
    __serialBus = None
    __feedbackThread = None
    __threadRunning = False
    __threadAlive = True

    def __init__(self): 
        self.__feedbackThread = th.Thread(target=self.__feedbackThreadFunc) 
                
    def __sendOrder(self, order):
        self.__serialBus.write(bytes(order.encode()))
        self.__serialBus.write('\r'.encode())
    

    def __updateInformation(self):
        # Ask for status report 
        reportObtained = False
        startTs = time.time()
        
        self.__sendOrder('?')
    
        while not reportObtained:
            
            received = str(self.__serialBus.readline())
            
            if '<' in received and '>' in received:
                reportObtained = True

            # If last to much time, abort
            if time.time() - startTs > 0.15:
                break
            
        if reportObtained:
            # Parse report message
            statusMsg = received[3:][:-6]

            variables = statusMsg.split('|')
            
            self.__machineStatus = variables[0]
            self.__machinePosition = variables[1][5:]
            
            # Parse switch status
            if len(variables) == 4 and 'Pn' in variables[3]:
                self.__machineSwitchStatus = variables[3][3:]
        
    def __feedbackThreadFunc(self):
        
        # Update information when grbl connection is working
        
        while self.__threadAlive:
            
            if self.__threadRunning:
               
                self.__updateInformation()                    
                time.sleep(1.0 / STATUS_REPORT_FREQUENCY)
            
    # Public

    def start(self, port='/dev/ttyUSB0'):
        
        try:
            self.__serialBus = serial.Serial(port, 115200)
        except:
            return False
        
        self.__threadAlive = True # Keep alive
        self.__threadRunning = True # Run thread
        self.__feedbackThread.start()
        time.sleep(1) # Wait all start
        return True
    
    def stop(self):

        if self.__threadRunning:
            
            self.__threadAlive = False # Kill thread
            self.__feedbackThread.join() # Wait thread to end
            self.__serialBus.close() # Close bus
        
    def loadConfig(self, filename):
        
        # Stop status thread to have a clean bus
        self.__threadRunning = False
        
        time.sleep(1.0 / STATUS_REPORT_FREQUENCY) # Wait to have clean bus
        
        try:
            file = open(filename, 'r')
        except:
            print("File not found!")
            self.__threadRunning = True # Reactivate thread
            return False
        
        commands = file.readlines()
        
        done = False
        i = 0
        
        while not done:
            
            command = commands[i].replace('\n', '') # Remove \n
            
            self.__sendOrder(command)
            
            # Wait for response
            responseValid = False # Only response is valid if contains error or ok. Other messages does not matter
            
            startTs = time.time()
            
            receive = ''
            
            while not responseValid:
                
                receive = str(self.__serialBus.readline())
                
                responseValid = ("error" in receive or "ok" in receive)
            
                if time.time() - startTs > RECEIVE_TIMEOUT:
                    break
            
            if responseValid:
                if "error" in receive:
                    print("Grbl response: " + receive[2:][:-5])
                    self.__threadRunning = True # Reactivate thread
                    return False                
                i += 1
                         
            done = (i == len(commands))
                       
        self.__threadRunning = True  # Reactivate thread
        return True
           
    def getXYZ(self):

        x = float(self.__machinePosition.split(',')[0])
        y = float(self.__machinePosition.split(',')[1])
        z = float(self.__machinePosition.split(',')[2])
        return (x, y, z)

    def getStatus(self):
        return self.__machineStatus
    
    def getSwitchStatus(self):
        return self.__machineSwitchStatus

    def setSpindleSpeed(self, speed):
        
        # Ensure speed is in range 0-1000
        if speed < 0:
            speed = 0
        if speed > 1000:
            speed = 1000
        
        order = "S" + str(speed)
        self.__sendOrder(order)
    
    def setZero(self):
        self.__sendOrder("G92 X0 Y0 Z0")
    
    def setCoordinates(self, x, y, z):
        self.__sendOrder("G92 " + "X" + str(x) + " Y" + str(y) + " Z" + str(z))
        
    def enableSpindle(self):
        self.__sendOrder("M3")
    
    def disableSpindle(self):
        self.__sendOrder("M5")
                 
    def asyncAxisMove(self, axis, value, feedrate, relative):
        
        if axis not in 'XYZ':
            print("Wrong axis!") 
            return 
                
        if relative:
            order = "G01G21G91" + axis + str(value) + "F" + str(feedrate)     
        else:
            order = "G01G21G91" + axis + str(value) + "F" + str(feedrate) 
            
        self.__sendOrder(order)
    
    def asyncXYZMove(self, position, feedrate, relative):
        if len(position) != 3:
            print("Wrong position!") 
            return
        
        coords = 'X' + str(position[0]) + 'Y' + str(position[1]) + 'Z' + str(position[2])
        
        if relative:
            order = "G01G21G91" + coords + "F" + str(feedrate)     
        else:
            order = "G01G21G91" + coords + "F" + str(feedrate) 
            
        self.__sendOrder(order)
            
    def softReset(self):
        # Send Hold
        self.__sendOrder('!')
        time.sleep(1)
        # Send control-x
        self.__serialBus.write(bytes.fromhex('18'))
        self.__serialBus.write('\r'.encode())

    def resume(self):
        self.__sendOrder('~')
    
    def syncMove(self, axis, value, feedrate, relative):
        
        self.asyncAxisMove(axis, value, feedrate, relative)
        
        self.waitForRunToIdle()
            
