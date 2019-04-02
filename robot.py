#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 15:55:37 2019

@author: jash
"""

import vrep
import math as m
import numpy as np
import random as r

class robot:
    def __init__(self, clientID, L, r):
        self.clientID = clientID
        # Get the necessary handles
        self.pioneer = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)
        self.motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
        self.motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
        self.usensor = []
        for i in range(1,17):
            err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
            self.usensor.append(s)
        # Init sensors
        for i in range(16):
            _, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, self.usensor[i], vrep.simx_opmode_streaming)
            
        self.L = L
        self.r = r
        self.sState = []
        self.sPoints = []
        
    def setDestination(self, xf, yf):
        self.xf = xf
        self.yf = yf
    
    def setWheelsVel(self, ur, ul):
        """ """
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorR, ur, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(self.clientID, self.motorL, ul, vrep.simx_opmode_streaming)

    def setPointCtrl(self, Kv, Kh):
        """ Control of the robot using the set point control """
        # Get robot position and orientation
        _, rP = vrep.simxGetObjectPosition(self.clientID, self.pioneer, -1, vrep.simx_opmode_blocking)
        _, rOr = vrep.simxGetObjectOrientation(self.clientID, self.pioneer, -1, vrep.simx_opmode_blocking)

        # Get the desired velocity and rotation angle of the robot
        v = Kv * m.sqrt( (self.xf - rP[0]) ** 2 + (self.yf - self.rP[1]) ** 2 )
        thetaD = m.atan2( self.yf - rP[1], self.xf - rP[0])
        gamma = Kh * (thetaD - rOr[2])
            
        # Compute the angular velocities of each wheel
        ur = (1/self.r) * v + (self.L / (2 * self.r)) * gamma
        ul = (1/self.r) * v - (self.L / (2 * self.r)) * gamma
        self.setWheelsVel(self.clientID, self.motorR, self.motorL, ur, ul)
        error = m.sqrt( (self.xf - rP[0]) ** 2 + (self.yf - rP[1]) ** 2 )

        return error
        
    def getPoints(self, pTrue, pFalse, w, h, s):
        """ Get the position and orientation of the sensors. Get the points of the detected objects """
        def q2R(x,y,z,w):
            """ Transforms a quaternion to a rotation matrix """
            R = np.zeros((3,3))
            R[0,0] = 1-2*(y**2+z**2)
            R[0,1] = 2*(x*y-z*w)
            R[0,2] = 2*(x*z+y*w)
            R[1,0] = 2*(x*y+z*w)
            R[1,1] = 1-2*(x**2+z**2)
            R[1,2] = 2*(y*z-x*w)
            R[2,0] = 2*(x*z-y*w)
            R[2,1] = 2*(y*z+x*w)
            R[2,2] = 1/2*(x**2+y**2)
            return R

        sPosition, sState, sPoints, sPointsW = [], [], [], []
        for i in range(16):
            _, position = vrep.simxGetObjectPosition(self.clientID, self.usensor[i], -1, vrep.simx_opmode_blocking)
            _, quat = vrep.simxGetObjectQuaternion(self.clientID, self.usensor[i], -1, vrep.simx_opmode_blocking)
            _, state, point, detectedObj, norm = vrep.simxReadProximitySensor(self.clientID, self.usensor[i], vrep.simx_opmode_buffer)
            sPointsW.append(np.linalg.norm(point))
            row, col = self.getGridCoord(position[0], position[1], w, h, s)
            sPosition.append([row, col])
            R = r.random()
            pS = [0, 0, 1]
            if state:
                B = R < pTrue
                if B:
                    state = False
                    pS = [0, 0, 1]
                else:
                    state = True
                    pS = point
            else:
                B = R < pFalse
                if B:
                    state = True
                    pS = [0, 0, r.random()]
                else:
                    state = False
                    pS = [0, 0, 1]
                
            sState.append(state)
            rotM = np.array(q2R(quat[0], quat[1], quat[2], quat[3]))
            pos = np.array(pS)
            # Get the position of the detected object in world coordinate
            pW = np.matmul(rotM, pos) + position
            row, col = self.getGridCoord(pW[0], pW[1], w, h, s)        
            sPoints.append([row, col])
        return sPosition, sState, sPoints
    
    def getGridCoord(self, xw, yw, h, w, s):
        """ Given a world coordinate, it returns it index on the grid """
        row = int(h / (2 * s) - m.floor(yw / s))
        col = int(m.ceil(xw / s) + w / (2 * s))
        return row, col
    
    def getPosition(self):
        _, rP = vrep.simxGetObjectPosition(self.clientID, self.pioneer, -1, vrep.simx_opmode_blocking)
        return rP
    
    def getOrientation(self):
        _, rOr = vrep.simxGetObjectOrientation(self.clientID, self.pioneer, -1, vrep.simx_opmode_blocking)
        return rOr
