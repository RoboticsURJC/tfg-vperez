import serial
import time
import threading as th

STATUS_REPORT_FREQUENCY = 5 # Hz

class grbl:

    # Private
    __machinePosition = "0.0,0.0,0.0"
    __machineState = "Undefined"
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
            
            
            
        
        
        
        done = False

        self.__sendOrder('?')
        startTs = time.time()

        while not done:

            received = str(self.__serialBus.readline())
            
            # If received is a status command
            if '<' in received and '>' in received:
                
                # Parse
                statusMsg = received[3:][:-6]

                variables = statusMsg.split('|')
                
                self.__machineState = variables[0]
                self.__machinePosition = variables[1][5:]
                done = True

            # Timeout
            if time.time() - startTs > STATUS_TIMEOUT:
                done = True

    # Public

    def start(self):
        
        try:
            self.__serialBus = serial.Serial(self.__port, 115200)
            self.__feedbackThread.start()
            return True
        except:
            print("Unable to connect with grbl... Do you have permissions to access " + self.__port + " ?")
            return False
    
    def stop(self):

        __threadRunning = False # Stop thread
        self.__feedbackThread.join() # Wait thread to end
        self.__serialBus.close() # Close bus
               
    def getXYZ(self):

        x = float(self.__machinePosition.split(',')[0])
        y = float(self.__machinePosition.split(',')[1])
        z = float(self.__machinePosition.split(',')[2])
        return (x, y, z)

    def getState(self):
        return self.__machineState

    def goToXYZ(self, x, y, z, feedrate):
        
        order = "G01" + "X" + str(x) + "Y" + str(y) + "Z" + str(z) + "F" + feedrate       
        self.__sendOrder(order)
    
   