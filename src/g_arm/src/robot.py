import math, time
import grblAPI


RELATION_X_RAD = 2*math.pi # Relation between x in grbl and radians: X=1 -> 2pi radians
RELATION_Y_RAD = 2*math.pi # Relation between x in grbl and radians: Y=1 -> 2pi radians
RELATION_Z_RAD = 2*math.pi # Relation between x in grbl and radians: Z=1 -> 2pi radians

START_X_RAD = math.pi # Real angle when machine is at X=0 
START_Y_RAD = math.pi # Real angle when machine is at Y=0 
START_Z_RAD = math.pi # Real angle when machine is at Z=0  

class Robot:
    
    def __init__(self):      
        self.__grbl = grblAPI.Grbl()
    
    def start(self, port='/dev/ttyUSB0'):
        
        if not self.__grbl.start(port):
            print("Unable to start communications in " + port + ". Do you have permissions?")
            return False
        
        #self.__grbl.syncMove('Y', 10,10)
        #self.__grbl.syncMove('Y', 10,10, True)
        time.sleep(1)
        self.autohome()
        
        return True

    
    def stop(self):
        self.__grbl.stop()
         
    # Setters
    def setAngles(self, baseAngle, joint1Angle, joint2Angle):
       self.__grbl.goToXYZ(baseAngle / RELATION_X_RAD, joint1Angle / RELATION_Y_RAD, joint2Angle / RELATION_Z_RAD)
    
    # Getters    
    def getAngles(self):
        X, Y, Z = self.__grbl.getXYZ()       
        return (X * RELATION_X_RAD, Y * RELATION_Y_RAD, Z * RELATION_Z_RAD)
    '''
    def autohome(self):
        print("Homing...")
        
        while 'Y' not in self.__grbl.getSwitchStatus():  
           self.__grbl.syncMove('Y', 1,10, True)
           
        print("Done!")
    '''
    def autohome(self):
        self.__grbl.asyncMove('Y', 20,10, True)
        time.sleep(2)
        self.__grbl.softReset()
        
        
    # Private
    __grbl = None 
