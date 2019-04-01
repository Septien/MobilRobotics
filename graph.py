# -*- coding: utf-8 -*-
"""
A module for graphs. It will include basic facilities like adding and removing nodes from the graph, 
searching, and a facility for connected component

@author: Phantom
 """

import math as m
import queue as q

class Graph:
    def __init__(self):
        self.vertex = []
        self.edges = []
        
    def addVertex(self, vertex, dMetric):
        """ Add a vertex to the graph. 
                -dMetric is the metric distance for finding the adyacent nodes.
                -vertex: (x, y) coordinates.
        """
        # For the case it is the first node
        if len(self.vertex) == 0:
            self.vertex.append(vertex)
            self.edges.append([])
        else:
        # Find the nearest point and add it to its adyacency list
            nearestN = [len(self.edges)]
#            minD = m.inf
            minIdx = -1
            for i in range(len(self.vertex)):
                v = self.vertex[i]
                d = m.sqrt((v[0] - vertex[0])**2 + (v[1] - vertex[1])**2)
                if d < dMetric:
                    nearestN.append(i)
#                    if d < minD:
#                        minD = d
#                        minIdx = i
            self.vertex.append(vertex)
            if minIdx != -1:
                self.edges.append([minIdx])
                self.edges[minIdx].append(len(self.edges) - 1)
            else:
                self.edges.append([])
            # Perform a BFS for finding the connected components
            #conComp = self.BFS(vertex)
            for v in nearestN:
                self.edges[-1].append(v)
                self.edges[v].append(len(self.edges) - 1)
                #if v not in conComp:

    def BFS(self, vertex):
        """ Performs a breath first search starting at the node 'vertex'. """
        visited = []
        vertexInd = -1
        # Find the index of the node
        for i in range(len(self.vertex)):
            if self.eq(vertex[0], self.vertex[i][0]) and eq(vertex[1], self.vertex[i][1]):
                visited.append(1)
                vertexInd = i
            else:
                visited.append(0)
        if vertexInd == -1:
            print("Vertex not found: " + str(vertex))
            return
        
        search = []
        queue = []
        queue.append(vertexInd)
        while len(queue) > 0:
            v = queue.pop(0)
            visited[v] = 1
            search.append(v)
            for u in self.edges[v]:
                if visited[u] == 0:
                    queue.append(u)
        return search
        
    def Dijkstra(self, vertex):
        """ Implements the Dijkstra search on the graph """
        vertexInd = -1
        # Find index
        for i in range(len(self.vertex)):
            if self.eq(vertex[0], self.vertex[i][0]) and eq(vertex[1], self.vertex[i][1]):
                vertexInd = i
                break
        # initialize single source
        parents = []
        distance = []
        for i in range(len(self.vertex)):
            parents[i] = -1
            distance[i] = m.inf
        distance[vertexInd] = 0
        # Initialize the queue
        minQ = q.PQueue()
        minQ.insert_key((vertexInd, distance[vertexInd]))
        S = []
        while not minQ.isEmpty():
            u = minQ.extract_min()
            S.appen(u[0])
            for v in self.edges[u]:
                self.relax(u, v, distance, parents)
                if v not in S:
                    minQ.insert_key((v, distance[v]))
        return S, parents
    
    def relax(self, u, v, distance, parents):
        """ """
        def w(u, v, vertex):
            return m.sqrt((vertex[u][0] + vertex[v][0])**2 + (vertex[u][1] + vertex[v][1])**2)
        if distance[v] > distance[u] + w(u, v):
            distance[v] = distance[u] + w(u, v)
            parents[v] = u

    def getNodes(self, indexList):
        """ Returns a list of nodes with index from indexList """
        nodes = []
        for i in indexList:
            nodes.append(self.vertex[i])
        return nodes
        
    def eq(self, x1, x2):
            return m.fabs(x1 - x2) < 0.001

def main():
    g = Graph()
    g.addVertex((0, 0), 0.51)
    g.addVertex((1, 1), 0.51)
    g.addVertex((0.5, 0), 0.51)
    g.addVertex((1.5, 1.0), 0.51)
    g.addVertex((0.75, 0.75), 1.0)

main()