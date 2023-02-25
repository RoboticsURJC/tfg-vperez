import serial
import time
import threading as th

STATUS_REPORT_FREQUENCY = 5 # Hz

class Grbl:

    # Private
    __machinePosition = "0.0,0.0,0.0"
    __machineStatus = "Undefined"
    __serialBus = None
    __port = None
    __feedbackThread = None
    __threadRunning = False

    def __init__(self, port='/dev/ttyS0'):
        self.__port = port 
        self.__feedbackThread = th.Thread(target=self.__feedback) 
                
    def __sendOrder(self, order):
        self.__serialBus.write(bytes(order.encode()))
        self.__serialBus.write('\r'.encode())
        self.__serialBus.write('\n'.encode())

    def __feedback(self):
        
        # Update information when grbl connection is working
        
        while self.__threadRunning:
            
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
                        
            time.sleep(1.0 / STATUS_REPORT_FREQUENCY)
            
    # Public

    def start(self):
        
        try:
            self.__serialBus = serial.Serial(self.__port, 115200)
        except:
            return False
        
        self.__threadRunning = True
        self.__feedbackThread.start()
        return True
    
    def stop(self):

        self.__threadRunning = False # Stop thread
        self.__feedbackThread.join() # Wait thread to end
        self.__serialBus.close() # Close bus
               
    def getXYZ(self):

        x = float(self.__machinePosition.split(',')[0])
        y = float(self.__machinePosition.split(',')[1])
        z = float(self.__machinePosition.split(',')[2])
        return (x, y, z)

    def getStatus(self):
        return self.__machineStatus

    def goToXYZ(self, x, y, z, feedrate):
        
        order = "G01" + "X" + str(x) + "Y" + str(y) + "Z" + str(z) + "F" + str(feedrate)     
        self.__sendOrder(order)
    
   