from DM_CAN import *
import math as cm
import numbers
try:
    import numpy as np
except ImportError:
    np = None

gloria_max = -2.9

class MotorComm:
    def __init__(self):
        pass

    def getPosRad(self, Motor):
        return Motor.getPosition()

    def getAngle(self, Motor):
        return Motor.getPosition() * (180.0 / cm.pi)
    
    def getAngle100(self, Motor):
        """
        将电机位置(期望 0~gloria_max)线性映射为 0~100
        超出范围自动夹紧
        """
        pos = Motor.getPosition()
        return (pos / gloria_max) * 100.0

    def getVelRad(self, Motor):
        return Motor.getVelocity()

    def getVel(self, Motor):
        return Motor.getVelocity() * (30.0 / cm.pi)

    def getTorque(self, Motor):
        return Motor.getTorque()
    
    def getTMos(self, Motor):
        return Motor.T_Mos

    def getTRoto(self, Motor):
        return Motor.T_Roto

    ##########################################

    def rad_to_angle(self, rad):
        return rad * 180.0 / cm.pi
