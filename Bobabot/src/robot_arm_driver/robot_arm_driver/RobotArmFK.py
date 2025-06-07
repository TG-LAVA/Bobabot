from robot_arm_driver.DHTables import *
from typing import List
import numpy as np
import math

"""
A class for performing forward kinematics analysis on a robotic arm using DH parameters.

This class computes homogeneous transformation matrices for each joint, extracts joint positions,
calculates the Jacobian for velocity visualization, and checks for collisions and angle limits.

Author: Yinuo Liu
Date: 6/3/2025


<if it reached this position, it means that the system will not collide. so there are no collision check here>
"""
class RobotArmFK:

    #constant defination
    NEG_90_DEGREES = math.radians(-90)
    POS_90_DEGREES = math.radians(90)
    FIXED_JOINT_ANGLE = 0
    MATRIX_SIZE_T = 4
    MATRIX_SIZE_ANG = 3
    LAST_ELEMENT = -1
    Z_COORD_ROW = 3
    Z_COORD_COL = 2

    JOINT_LIMIT_MIN = -2*np.pi
    JOINT_LIMIT_MAX =   2*np.pi

    LAST_ELEMENT = -1                 # Index for last element (end effector)
    POSITION_VECTOR_DIM = 3           # Dimension of position vector (x, y, z)
    POSITION_INDEX = 3                # Column index for position component in transformation matrix
    INITIAL_COORDINATE_VECTOR = np.array([[0], [0], [1]])  # Unit Z-axis vector for rotation

    INITIAL_COORDINATE = np.array([[0], [0], [1]])




    # constructor to create class-level variables
    def __init__(self):
        self.DHTables = DHTables()
        self.DummyJointGeneral = []
        self.NonModified()

    # the NonModified function is to add joints to the robot arm by using stand DH transform parameters,
    # and store it for future use.
    def NonModified(self):
        # DH table parameters defination
        # self.DHTables.addHomoMatrix(DHTableParams(d=0.093, theta=math.radians(0), a=0, alpha=math.radians(90)))
        # self.DHTables.addHomoMatrix(DHTableParams(d=0, theta=math.radians(90-9.505), a=0.126, alpha=0))
        # self.DHTables.addHomoMatrix(DHTableParams(d=0, theta=math.radians(9.505), a=0.130, alpha=0))
        # self.DHTables.addHomoMatrix(DHTableParams(d=0, theta=0, a=0.147, alpha=math.radians(90)))
        # self.DHTables.addHomoMatrix(DHTableParams(
        #     d = 0.098387, # ↑  0.093
        #     theta = -0.025277, # -1.45° 
        #     a = -0.010, # X  10 mm
        #     alpha = 1.504593)) # 86.2°（≈ π/2 − 3.8°）
        # self.DHTables.addHomoMatrix(DHTableParams(
        #     d = -0.00457, #  -4.6 mm
        #     theta = 1.406384, # 80.6°
        #     a = 0.127188, #  0.126
        #     alpha = 0.004803)) # 0. 0
        
        # self.DHTables.addHomoMatrix(DHTableParams(
        #     d = -0.009972, # -10 mm
        #     theta = 0.1214, # 6.96°
        #     a = 0.124257, #  0.130 → 124 mm
        #     alpha = 0.014343)) # 0.82°   
        # self.DHTables.addHomoMatrix(DHTableParams(
        #     d = -0.004797, # -4.8 mm
        #     theta = -0.0012, # -2.12°
        #     a = 0.153497, #  0.147 → 153 mm
        #     alpha = 1.572647)) # 90.1°
        self.DHTables.addHomoMatrix(DHTableParams(
            d=0.091,        theta=-0.054555,  a=-0.010,    alpha=1.565351))   # link-1

        self.DHTables.addHomoMatrix(DHTableParams(
            d=0.009441,     theta= 1.489953,  a=0.127451,  alpha=0.008170))   # link-2

        self.DHTables.addHomoMatrix(DHTableParams(
            d=-0.010,       theta= 0.121548,  a=0.123048,  alpha=0.016081))   # link-3

        self.DHTables.addHomoMatrix(DHTableParams(
            d=0.008789,     theta=-0.038747,  a=0.156341,  alpha=1.573905))   # link-4
    '''
        apply forward kinemetics to the DH table created.
        if Velcalc = Ture, return the full transformation matrix for
        jacobian matrix calculation and collision calculation
    '''
    def FKanalysis(self):
        DHMatrix = self.DHTables.DHMatrix
        dh_params = self.DHTables.getDHParam()
        for i in range (len(dh_params)):
            print(np.degrees(self.DHTables.dh_params[i].theta))

        #forward kinemetics algorithm
        positions = [np.identity(self.MATRIX_SIZE_T)]  # initial position
        positions_jacob = [np.identity(self.MATRIX_SIZE_T)]  # initial position
        T_total = np.identity(self.MATRIX_SIZE_T)
        for dh in range(len(dh_params)):
            T = DHMatrix(dh_params[dh])
            positions_jacob.append(T_total@T)
            T_total = T_total@T
            positions.append(T_total)
        return positions
    

    def EditAngle(self,angles:List[float]):
        for i in range (len(angles)):
            self.DHTables.dh_params[i].theta = math.radians(angles[i])