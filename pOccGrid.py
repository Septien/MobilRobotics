#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 19 17:08:08 2019

@author: jash
"""
import numpy as np
import skimage.draw as skd
import matplotlib.pyplot as plt
import math as m

class OccupancyGrid:
    def __init__(self, row, col):
        # Init the each cell with probability 0.5
        self.occGrid = 0.5 * np.ones((row, col))
        self.pzm = 0.99
        self.pnznm = 0.99
        self.pnzm = 0.01
        self.pznm = 0.01

    def getSightLine(sPosition, sPoints):
        """ Get the points on the line of "sight" for each sensor """
        line = skd.line(sPosition[0], sPosition[1], sPoints[0], sPoints[1])
        return line
        
    def updateOccGrid(self, sState, sPosition, sPoints):
        """
            Updates the occupancy grid according to the state of the sensors (sState),
            its position, and the position of the sensor readings (sPoints)
        """
        for i in range(len(sState)):
            line = self.getSightLine(sPosition[i], sPoints[i])
            rows, cols = line[0], line[1]
            # Get the previous probability the cell
            pm = self.occGrid[rows[-1], cols[-1]]
            pnm = 1 - pm
            if sState[i]:
                # Update the cell with the probability that it is occupied
                p = (self.pzm * pm) / ((self.pzm * pm) + (self.pznm * pnm))
            else:
                p = (self.pnzm * pm) / ((self.pnzm * pm) + (self.pnznm * pnm))
            self.occGrid[rows[-1], cols[-1]] = p
            # Update the rest of the cells with the probability that the cell is not occupied
            for i in range(len(rows) - 1):
                pm = self.occGrid[rows[i], cols[i]]
                pnm = 1 - pm
                self.occGrid[rows[i], cols[i]] = (self.pnzm * pm) / ((self.pnzm * pm) + (self.pnznm * pnm))

    def plotGrid(self):
        plt.imshow(self.occGrid, "viridis")
        plt.show()
 
    def addObjects(self, obstacles, epsilon, h, w, s):
        """ Adds circles to the occupancy grid. obstacles is an array of circles """
        def getGridCoord(xw, yw, h, w, s):
            """ Given a world coordinate, it returns it index on the grid """
            row = int(h / (2 * s) - m.floor(yw / s))
            col = int(m.ceil(xw / s) + w / (2 * s))
            return row, col
        #
        for obj in obstacles:
            row, col = getGridCoord(obj[0], obj[1], h, w, s)
            r = obj[2] / s
            e = epsilon / s
            rr, cc = skd.circle(row, col, r - e)
            self.occGrid[rr, cc] = 0.8
            rr, cc = skd.circle(row, col, r)
            self.occGrid[rr, cc] = 1
            
    def addPoints(self, points, edges, h, w, s, color):
        """ """
        def getGridCoord(xw, yw, h, w, s):
            """ Given a world coordinate, it returns it index on the grid """
            row = int(h / (2 * s) - m.floor(yw / s))
            col = int(m.ceil(xw / s) + w / (2 * s))
            return row, col
        #
        for i in range(len(points)):
            row, col = getGridCoord(points[i][0], points[i][1], h, w, s)
            rr, cc = skd.circle(row, col, 2)
            self.occGrid[rr, cc] = color
            for e in edges[i]:
                row1, col1 = getGridCoord(points[e][0], points[e][1], h, w, s)
                rr, cc = skd.line(row, col, row1, col1)
                self.occGrid[rr, cc] = color
                
    def addRoute(self, route, h, w, s):
        """ """
        def getGridCoord(xw, yw, h, w, s):
            """ Given a world coordinate, it returns it index on the grid """
            row = int(h / (2 * s) - m.floor(yw / s))
            col = int(m.ceil(xw / s) + w / (2 * s))
            return row, col
        for i in range(len(route) - 1):
            pI = route[i]
            pF = route[i + 1]
            row, col = getGridCoord(pI[0], pI[1], h, w, s)
            row1, col1 = getGridCoord(pF[0], pF[1], h, w, s)
            rr, cc = skd.line(row, col, row1, col1)
            self.occGrid[rr, cc] = 0.5