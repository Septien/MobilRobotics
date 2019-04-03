#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  3 12:50:39 2019

@author: jash
"""
import math as m

class Node:
    def __init__(self, xy):
        """ xy -> (x, y) """
        self.xy = xy
        self.p = -1
        self.d = m.inf
        self.h = m.inf
