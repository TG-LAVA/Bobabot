import math
import numpy as np
from typing import List

'''
This file is for Defining the DH parameters(normal and modified)
store them into arrays for calculation and provide a general transformation
from DH parameters to the Homogeneous transform.

Author: Yinuo Liu
Date: 6/3/2025
'''




'''
DHTableParams is the struct to store the DH parameters
'''
class DHTableParams:
    def __init__(self, d, theta, a, alpha,fixed = False):
        self.d = d
        self.theta = theta  
        self.a = a        
        self.alpha = alpha
        self.fixedjoint = fixed


"""
A class to manage DH parameters and compute related 
homogeneous transformation matrices for robotic arms. It supports both standard 
and modified DH transform.
"""
class DHTables:
    """
    Initialize the DH tables.
    Parameters:
        modified (bool): If True, use modified DH parameters
    """
    def __init__(self):
        # List to store DH parameters (each parameter is an instance of DHTableParams)
        self.dh_params: List[DHTableParams] = []
        self.dh_params_const = []
        # Counter for the number of joints that are not fixed (i.e., joints that can rotate)
        self.__dh_params_count__ = 0

    def getDHParam(self) -> List[DHTableParams]:
        dh_params_copy = []
        for i in range(len(self.dh_params)):
            p = self.dh_params[i]
            new_param = DHTableParams(
                d=p.d,
                theta=p.theta + self.dh_params_const[i],
                a=p.a,
                alpha=p.alpha,
                fixed=p.fixedjoint
            )
            dh_params_copy.append(new_param)
        return dh_params_copy


    """
    Add a new set of DH parameters to the table along with its dummy joint flag.

    Parameters:
        dh_param (DHTableParams): The DH parameters for a joint.
        DummyJoint (bool): Indicates whether this joint is a dummy joint.
    
    This function has been designed for flexable parameters, people can add any parameters they like to this function
    """
    def addHomoMatrix(self, dh_param:DHTableParams):
        # Append the new DH parameters to the list
        self.dh_params_const.append(dh_param.theta)
        dh_param.theta = 0
        self.dh_params.append(dh_param)
        # If it is a normal joint, increase the joint counter
        if not dh_param.fixedjoint:
            self.__dh_params_count__ += 1

    """
    Return the number of active (non-fixed) joints.
    """
    def GetJointNumber(self):
        return self.__dh_params_count__

    """
    Calculate the homogeneous transformation matrix based on the given DH parameters.

    Parameters:
        M (DHTableParams): An object containing DH parameters (d, theta, a, alpha, fixedjoint).

    Returns:
        numpy.ndarray: The 4x4 homogeneous transformation matrix.
    """
    def DHMatrix(self, M:DHTableParams):
        T = np.array([])  # Initialize T as an empty array
        T = np.array([
            [math.cos(M.theta), -math.sin(M.theta) * math.cos(M.alpha),  math.sin(M.theta) * math.sin(M.alpha), M.a * math.cos(M.theta)],
            [math.sin(M.theta),  math.cos(M.theta) * math.cos(M.alpha), -math.cos(M.theta) * math.sin(M.alpha), M.a * math.sin(M.theta)],
            [0,                  math.sin(M.alpha),                      math.cos(M.alpha),                   M.d],
            [0,                  0,                                      0,                                   1]
        ])
        return T



