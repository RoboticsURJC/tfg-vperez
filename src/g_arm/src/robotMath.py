from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import math

import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

# Resolve inverse kinematics for a position of the end-effector relative to ground 0.0
def IK(point, tcpOffset):
    
    arm = Chain.from_urdf_file("../../g_arm_description/urdf/base.urdf.xacro")
    
    '''
    # Create chain manually
    arm = Chain(name='left_arm', links=[
        
        URDFLink(
        name="base",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
        URDFLink(
        name="motors",
        origin_translation=[2, 0, 10],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
        URDFLink(
        name="link1",
        origin_translation=[0, 10, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
        URDFLink(
        name="link2",
        origin_translation=[2, 0, 10],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 0],
        ),
    ])
    
    '''

    
    
    ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

    arm.plot(arm.inverse_kinematics([2, 2, 2]), ax)
    matplotlib.pyplot.show()


def DK(angles, tcpOffset):
    pass

IK(0, 0)