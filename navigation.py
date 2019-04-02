#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  1 13:54:21 2019

@author: jash
"""

import sys
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
            obstacles.append((float(l[0]), float(l[1]), float(l[2]) + rRadio))
    return obstacles

def createRoadMap(g, N, obstacles, dMetric):
    """ Creates the roadmap of the scenario: 
        -g is the graph on which the obstacles are to be stored.
        -N the number of points.
        -obstacles is the obstacles' coordinates. 
        -dMetric: determines how near the points should be to one another.
    """
    r.seed()
    for i in range(N):
        x = r.uniform(-7.0, 7.0)
        y = r.uniform(-7.0, 7.0)
        discard = False
        for obs in obstacles:
            if m.sqrt((obs[0] - x)**2 + (obs[1] - y)**2) <= obs[2]:
                discard = True
        if not discard:    
            g.addVertex((x, y), dMetric)

def main():
    # Goal points
    endP = (3.0, -3.0) 
    Kv = 0.2
    Kh = 0.2
    # Robot constants
    L = 0.365
    r = 0.0975
#    clientID = connect()
#    pioneer = rb.robot(clientID, L, r)
    
    # Create occupancy grid, 0.5 -> don't know
    w, h = 15, 15
    s = 0.25
    row, col = int(w/s), int(h/s)
    occgrid = pog.OccupancyGrid(row, col)
    w1, h1 = 15, 15
    s1 = 0.1
    rowW, colW = int(w1/s1), int(h1/s1)
    occgridworld = pog.OccupancyGrid(rowW, colW)

    # Probability of detection
    pFalse = 0.01
    pTrue = 0.01
    
    # Load the obstacles
    obstacles = loadObstacles("/Users/jash/Documents/obstacle.txt", r + 0.001)
    occgridworld.addObjects(obstacles, 0.001, h1, w1, s1)
    occgridworld.plotGrid()
    
    # Generate the road map
    roadmap = g.Graph()
    N = 1000
    dMetric = 0.9
    createRoadMap(roadmap, N, obstacles, dMetric)
    # Add initial and final points to the graph
#    rP = pioneer.getPosition()
    initP = (0, 0)#(rP[0], rP[1])
    roadmap.addVertex(initP, dMetric)
    roadmap.addVertex(endP, dMetric)
    vertex = roadmap.getVertex()
    edges = roadmap.getEdges()
    occgridworld.addPoints(vertex, edges, h1, w1, s1, 0.7)
    occgridworld.plotGrid()
    # Get the path to follow
    route = roadmap.Dijkstra(endP)
    occgridworld.addRoute(route, h1, w1, s1, 0.7)
    occgridworld.plotGrid()
#    
#    epsilon = 0.1
#    error = 1
#    route.reverse()
#    for point in route:
#        pioneer.setDestination(point[0], point[1])
#        while error > epsilon:
#            # Read sensors
#            sPosition, sState, sPoints = pioneer.getPoints(pTrue, pFalse, w, h, s)
#            # Update occupancy grid
#            occgrid.updateOccGrid(sState, sPosition, sPoints)
#            #
#            error = pioneer.setPointCtrl(Kv, Kh)
#        #
#        rOr = pioneer.getOrientation()
main()