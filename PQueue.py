# -*- coding: utf-8 -*-
"""
A priority queue

@author: Phantom
"""
import math as m
import node as n

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
        self.queue.pop(-1)
        self.len -= 1
        self.min_heapify(0)
        return minE

    def min_heapify(self, i):
        if self.len == 0:
            return
        l  = (2 * i) + 1
        r = 2 * (i + 1)
        if l < self.len and self.queue[l][1].d < self.queue[i][1].d:
            least = l
        else:
            least = i
        if r < self.len and self.queue[r][1].d < self.queue[least][1].d:
            least = r
        if least != i:
            aux = self.queue[i]
            self.queue[i] = self.queue[least]
            self.queue[least] = aux
            self.min_heapify(least)
            
    def decrease_key(self, i, key):
        def parent(i):
            return int((i - 1) / 2)
        if key[1].d > self.queue[i][1].d:
            return
        self.queue[i] = key
        p = parent(i)
        while i > 0 and self.queue[p][1].d > self.queue[i][1].d:
            aux = self.queue[i]
            self.queue[i] = self.queue[parent(i)]
            self.queue[parent(i)] = aux
            i = parent(i)
            p = parent(i)

    def insert_key(self, key):
        self.len += 1
        self.queue.append((-1,n.Node((0, 0))))
        self.decrease_key(self.len - 1, key)
    
    def isEmpty(self):
        return self.len == 0
