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
    __port = None
    __feedbackThread = None
    __threadRunning = False
    __threadAlive = True

    def __init__(self, port='/dev/ttyS0'):
        self.__port = port 
        self.__feedbackThread = th.Thread(target=self.__feedback) 
                
    def __sendOrder(self, order):
        self.__serialBus.write(bytes(order.encode()))
        self.__serialBus.write('\r'.encode())
        #self.__serialBus.write('\n'.encode())
    

    def __feedback(self):
        
        # Update information when grbl connection is working
        
        while self.__threadAlive:
            
            if self.__threadRunning:
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
                                        
                time.sleep(1.0 / STATUS_REPORT_FREQUENCY)
            
    # Public

    def start(self):
        
        try:
            self.__serialBus = serial.Serial(self.__port, 115200)
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
    
    def enableSpindle(self):
        self.__sendOrder("M3")
    
    def disableSpindle(self):
        self.__sendOrder("M5")
    
    def goToXYZ(self, x, y, z, feedrate):
        
        order = "G01 G90" + "X" + str(x) + "Y" + str(y) + "Z" + str(z) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
    def setAbsX(self, value, feedrate):
        order = "G01 G90" + "X" + str(value) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
    def setAbsY(self, value, feedrate):
        order = "G01 G90" + "Y" + str(value) + "F" + str(feedrate)       
        self.__sendOrder(order)
    
    def setAbsZ(self, value, feedrate):
        order = "G01 G90" + "Z" + str(value) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
    
    def setRelX(self, value, feedrate):
        order = "$J=G01 G91" + "X" + str(value) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
    def setRelY(self, value, feedrate):
        order = "$J=G01 G91" + "Y" + str(value) + "F" + str(feedrate)       
        self.__sendOrder(order)
    
    def setRelZ(self, value, feedrate):
        order = "$J=G01 G91" + "Z" + str(value) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
    def asyncMove(self, axis, value, feedrate, relative=True):
        
        if axis not in 'XYZ':
            return 
        
        if relative:
            order = "$J=G21 G91" + axis + str(value) + "F" + str(feedrate)     
        else:
            order = "$J=G21 G90" + axis + str(value) + "F" + str(feedrate) 
            
        self.__sendOrder(order)
    
    def syncMove(self, axis, value, feedrate, relative=True):
        
        if axis not in 'XYZ':
            print("Wrong axis!") 
        
        if relative:
            order = "$J=G21G91" + axis + str(value) + "F" + str(feedrate)     
        else:
            order = "$J=G21G90" + axis + str(value) + "F" + str(feedrate)
            
        self.__sendOrder(order)
        
        time.sleep(0.025) # Sleep for 25ms
        
        print(self.__machineStatus)
        
        while self.__machineStatus != "Idle":
            pass
            
