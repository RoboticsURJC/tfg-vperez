import math, time
from g_arm.g_arm_lib import grblAPI

RELATION_X_DEG = 1 # Relation between x in grbl and degrees
RELATION_Y_DEG = 1 # Relation between x in grbl and degrees
RELATION_Z_DEG = 1 # Relation between x in grbl and degrees

X_ZERO_REAL_ANGLE = 135 # Real angle when robot is zero X
Y_ZERO_REAL_ANGLE = 15 # Real angle when robot is zero Y
Z_ZERO_REAL_ANGLE = -36 # Real angle when robot is zero Z

class Robot:
    
    def __init__(self):      
        self.__grbl = grblAPI.Grbl()
    
    def startSerial(self, port='/dev/ttyUSB0'):
        
        if not self.__grbl.startSerial(port):
            return False

        time.sleep(1)
        self.autohome()
        
        return True
   
    def startTelnet(self, address='192.168.4.1', port=23):
        
        if not self.__grbl.startTelnet(address, port):
            return False

        time.sleep(1)
        self.autohome()
        
        return True
   
    def stop(self):
        self.__grbl.stop()
         
    def getStatus(self):
        
        if self.__status != "CALIBRATING":
            return self.__grbl.getStatus()
        else:
            return self.__status
    
    def getAngles(self):
        
        if self.__calibrated:
            XGrbl, YGrbl, ZGrbl = self.__grbl.getXYZ()
            X0, Y0, Z0 = self.zeroGrblPosition   
            
            X = ((XGrbl-X0) * RELATION_X_DEG) + X_ZERO_REAL_ANGLE
            Y = ((YGrbl-Y0) * RELATION_Y_DEG) + Y_ZERO_REAL_ANGLE
            Z = ((ZGrbl-Z0) * RELATION_Z_DEG) + Z_ZERO_REAL_ANGLE
            
            return (X, Y, Z)
        else:
            return (None, None, None)

    def setAngles(self, j1, j2, j3):  
        
        X0, Y0, Z0 = self.zeroGrblPosition 
        
        XGrbl = ((j1 - X_ZERO_REAL_ANGLE) / RELATION_X_DEG) + X0
        YGrbl = ((j2 - Y_ZERO_REAL_ANGLE) / RELATION_Y_DEG) + Y0
        ZGrbl = ((j3 - Z_ZERO_REAL_ANGLE) / RELATION_Z_DEG) + Z0
        
            
        self.__grbl.asyncXYZMove((XGrbl, YGrbl, ZGrbl), feedrate=2000, relative=False)

    def toolPWM(self, value):
        self.__grbl.setSpindleSpeed(int(value*1000))
    
    def autohome(self):
        self.__status = "HOMING"
        print("[INFO] [Robot] Starting calibration")
        
        # Reach Z switch
        print("[INFO] [Robot] Calibrating Joint 3")
        self.__grbl.asyncAxisMove('Z', -360, 200, True)
        while not 'Z' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
        
        # Reach Y switch
        print("[INFO] [Robot] Calibrating Joint 2")
        self.__grbl.asyncXYZMove((0, 360, 360), 100, True)
        while not 'Y' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
        
        # Reach X switch
        print("[INFO] [Robot] Calibrating Joint 1")
        self.__grbl.asyncAxisMove('X', 360, 400, True)
        while not 'X' in self.__grbl.getSwitchStatus():
            pass
        self.__grbl.softReset()
        time.sleep(2) # Wait for restart
                
        self.zeroGrblPosition = self.__grbl.getXYZ()
        print("[INFO] [Robot] Calibration done")

        self.__calibrated = True
        self.__status = "IDDLE"
                    
    # Private
    __grbl = None 
    zeroGrblPosition = (0.0, 0.0, 0.0)
    __calibrated = False
    __status = "IDDLE"
