# -*- coding: utf-8 -*-
"""
A priority queue

@author: Phantom
"""
import math as m

class PQueue:
    def __init__(self):
        self.queue = []
        self.len = 0
        
    def minimum(self):
        return self.queue[0]
        
    def extract_min(self):
        if len(self.queue) < 1:
            return
        minE = self.queue[0]
        self.queue[0] = self.queue[-1]
        self.len -= 1
        self.min_heapify(0)
        return minE

    def min_heapify(self, i):
        if self.len == 0:
            return
        l  = 2 * i
        r = (2 * i) + 1
        if l <= self.len and self.queue[l][1] > self.queue[r][1]:
            largest = l
        else:
            largest = r
        if r <= self.len and self.queue[r][1] > self.queue[largest][1]:
            largest = r
        if largest != i:
            aux = self.queue[i]
            self.queue[i] = self.queue[largest]
            self.queue[largest] = aux
            self.min_heapify(largest)
            
    def decrease_key(self, i, key):
        def parent(i):
            return int(i / 2)
        if key[1] > self.queue[i][1]:
            return
        self.queue[i] = key
        while i > 0 and self.queue[parent(i)][1] < self.queue[i][1]:
            aux = self.queue[i]
            self.queue[i] = self.queue[parent(i)]
            self.queue[parent(i)] = aux
            i = parent(i)

    def insert_key(self, key):
        self.len += 1
        self.queue.append(m.inf)
        self.decrease_key(self.len - 1, key)
    
    def isEmpty(self):
        return self.len == 0
