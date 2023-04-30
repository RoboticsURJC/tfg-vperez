
class Kinematics:
    
    __l1 = None
    __l2 = None
    __tcp_offset = None

    def __init__(self, l1, l2, tcp_offset):
        
        self.__l1 = l1
        self.__l2 = l2
        self.__tcp_offset = tcp_offset

    def inverseKinematics(self, x, y):
       
        pass

    def directKinematics(self, joint1, joint2, joint3):

        pass
