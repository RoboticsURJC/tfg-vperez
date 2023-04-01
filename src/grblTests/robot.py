import math
import grblAPI


RELATION_X_RAD = 2*math.pi # Relation between x in grbl and radians: X=1 -> 2pi radians
RELATION_Y_RAD = 2*math.pi # Relation between x in grbl and radians: Y=1 -> 2pi radians
RELATION_Z_RAD = 2*math.pi # Relation between x in grbl and radians: Z=1 -> 2pi radians

START_X_RAD = math.pi # Real angle when machine is at X=0 
START_Y_RAD = math.pi # Real angle when machine is at Y=0 
START_Z_RAD = math.pi # Real angle when machine is at Z=0  

class Robot:
    
    def __init__(self):
        
        pass
    
    def start(self, port='/dev/ttyS0'):
        
        if not self.__grbl.start():
            print("Unable to start communications in " + port + ". Do you have permissions?")
            return False
        
        if not self.__grbl.autohome(): # Init 0,0 correctly
            print("Unable to perform homing")
            return False
        
    
    def setBaseAngle():
        pass

    def joint2():
        pass        
        
    
    
    # Private
    __grbl = grblAPI.Grbl()
    
