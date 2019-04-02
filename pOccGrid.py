#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 19 17:08:08 2019

@author: jash
"""
import numpy as np
import skimage.draw as skd

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
