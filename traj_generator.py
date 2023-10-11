#!/usr/bin/env python3.8
from spatialmath import * 
from spatialmath.base import * 
from spatialmath.base import sym
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import warnings
from scipy import linalg
from roboticstoolbox import *
from math import pi
from swift import Swift

class TrajectoryGenerator:
    # Constructor:
    def __init__(self, serialLink, qInit, Tb):
        # Internal robot model
        self.serialLink = serialLink
        # Initial joint configuration
        self.q = qInit
        # Transform: centre of board wrt. base
        self.Tb = Tb
        self.serialLink.base = self.Tb

    def moveTo(self,Tp):        
        Te = self.serialLink.fkine(self.q)
        Td = Te.inv()*Tp
        print("Start end-effector transform: ")
        print(Te)
        dPos = Td.t
        dAngle = Td.eul()
        
        # 3.5 Resolved Motion Rate Control
        steps = 50
        # 3.6
        angleE = Te.eul()
        x1 = np.append(Te.t,angleE[0])
        x2 = np.append(Tp.t,angleE[0])
        delta_t = 0.05 # Discrete time step
        # 3.7
        x = np.zeros([4,steps])
        s = trapezoidal(0,1,steps).q
        for i in range(steps):
            x[:,i] = x1 *(1-s[i]) + s[i]*x2 # Create trajectory
        # 3.8
        q_matrix = np.zeros([steps,self.serialLink.n])
        # 3.9
        q_matrix[0,:] = self.serialLink.ikine_LM(Te, q0=self.q, mask = [1,1,1,0,0,0]).q # Solve for jointangles
        # 3.10
        for i in range(steps-1):
            xdot = (x[:,i+1] - x[:,i])/delta_t # Calculate velocity at discrete time step
            J = self.serialLink.jacob0(q_matrix[i,:]) # Get the jacobian at the current state
            J = J[:self.serialLink.n,:] # Take only first n rows
            q_dot = linalg.pinv(J) @ xdot # Solve velocities via RMRC
            q_matrix[i+1,:] = q_matrix[i,:] + delta_t * q_dot # Update next joint state
        
        self.q = q_matrix[steps-1,:]
        
        print("Target end-effector transform: ")
        print(Tp)
        print("Realised end-effector transform: ")
        print(robot.fkine(self.q))

# test robot
ql = [-pi, pi]
l1 = DHLink(d = .1, a = 0, alpha = pi/2, qlim = ql)
l2 = DHLink(d= 0, a= .5, alpha= 0,offset=pi/2, qlim = ql)
l3 = DHLink(d= 0, a= .5, alpha= 0, qlim = ql)
l4 = DHLink(d= 0, a= .2, alpha= 0, qlim = ql)
robot = DHRobot([l1, l2, l3, l4], name= 'test_robot')

qInit = np.zeros([1,4])

Tb = SE3(transl(0,.5,0))

trajGen = TrajectoryGenerator(robot,qInit,Tb)
trajGen.moveTo( SE3(transl(1,1,1)) )
