#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 13:54:21 2019

@author: jash
"""

import sys
import numpy as np
import random as r
import math as m
import matplotlib.pyplot as plt
import vrep
import graph as g
import robot as rb
import pOccGrid as pog

def connect():
	""" Connect to vrep """
	vrep.simxFinish(-1) # just in case, close all opened connections
	clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # start a connection
	if clientID!=-1:
		print ('Connected to remote API server')
	else:
		print('Not connected to remote API server')
		sys.exit("No connection")
	return clientID


def loadObstacles(fPath, rRadio):
    """ Load the (x,y) coordinates of the obstacles and its radious of each. """
    obstacles = []
    with open(fPath, "r") as f:
        for line in f:
            l = line.split(" ")
            obstacles.append(l[0], l[1], l[2] + rRadio)
    return obstacles

def createRoadMap(g, N, obstacles, dMetric):
    """ Creates the roadmap of the scenario: 
        -g is the graph on which the obstacles are to be stored.
        -N the number of points.
        -obstacles is the obstacles' coordinates. 
        -dMetric: determines how near the points should be to one another.
    """
    for i in range(N):
        x = r.uniform(-7.5, 7.5)
        y = r.uniform(-7.5, 7.5)
        for obs in obstacles:
            if m.sqrt((obs[0] - x)**2 + (obs[1] - y)**2) > obs[2]:
                g.addVertex((x, y), dMetric)

def main():
    # Goal points
    endP = (3.0, 3.0) 
    Kv = 0.2
    Kh = 0.2
    # Robot constants
    L = 0.365
    r = 0.0975
    clientID = connect()
    pioneer = rb.robot(clientID, L, r)
    
    # Create occupancy grid, 0.5 -> don't know
    w, h = 60, 60
    s = 0.25
    row, col = int(w/s), int(h/s)
    og = pog.OccupancyGrid(row, col)

    # Probability of detection
    pFalse = 0.01
    pTrue = 0.01
    
    # Load the obstacles
    obstacles = loadObstacles("obstacle.txt", r + 0.001)
    
    # Generate the road map
    N = 500
    dMetric = 0.8
    createRoadMap(g, N, obstacles, dMetric)
    # Add initial and final points to the graph
    rP = pioneer.getPosition()
    initP = (rP[0], rP[1])
    g.addVertex(initP, dMetric)
    g.addVertex(endP, dMetric)
    S, parents = g.Dijkstra(initP)

    # Reconstruct route
    
    while    
    epsilon = 0.1
    
    error = 1
    while error > epsilon: