import math, time
import grblAPI


RELATION_X_DEG = 1 # Relation between x in grbl and degrees
RELATION_Y_DEG = 1 # Relation between x in grbl and degrees
RELATION_Z_DEG = 1 # Relation between x in grbl and degrees

X_ZERO_REAL_ANGLE = 100 # Real angle when robot is zero X
Y_ZERO_REAL_ANGLE = 15 # Real angle when robot is zero X
Z_ZERO_REAL_ANGLE = -45 # Real angle when robot is zero X

class Robot:
    
    def __init__(self):      
        self.__grbl = grblAPI.Grbl()
    
    def start(self, port='/dev/ttyUSB0'):
        
        if not self.__grbl.start(port):
            print("Unable to start communications in " + port + ". Do you have permissions?")
            return False

        time.sleep(1)
        self.autohome()
        
        return True
   
    def stop(self):
        self.__grbl.stop()
         
   
    def getAngles(self):
        XGrbl, YGrbl, ZGrbl = self.__grbl.getXYZ()
        X0, Y0, Z0 = self.zeroGrblPosition   
        
        X = ((XGrbl-X0) * RELATION_X_DEG) + X_ZERO_REAL_ANGLE
        Y = ((YGrbl-Y0) * RELATION_Y_DEG) + Y_ZERO_REAL_ANGLE
        Z = ((ZGrbl-Z0) * RELATION_Z_DEG) + Z_ZERO_REAL_ANGLE
        
        return (X, Y, Z)


    def autohome(self):
        print("Homing...")
        
        # Reach Z switch
        print("Calibrating Z...")
        self.__grbl.asyncAxisMove('Z', -360, 200, True)
        while not 'Z' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
        
        # Reach Y switch
        print("Calibrating Y...")
        self.__grbl.asyncXYZMove((0, 360, 360), 10, True)
        while not 'Y' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
        
        # Reach X switch
        print("Calibrating X...")
        self.__grbl.asyncAxisMove('X', 360, 600, True)
        while not 'X' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
                
        self.zeroGrblPosition = self.__grbl.getXYZ()
        print("Calibrating done!")

        
    # Private
    __grbl = None 
    zeroGrblPosition = (0.0, 0.0, 0.0)
