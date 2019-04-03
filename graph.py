# -*- coding: utf-8 -*-
"""
A module for graphs. It will include basic facilities like adding and removing nodes from the graph, 
searching, and a facility for connected component

@author: Phantom
 """

import math as m
import PQueue as q
import node as n

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
            self.vertex.append(n.Node(vertex))
            self.edges.append([])
        else:
            # Find the nearest point and add it to its adyacency list
            nearestN = []
#            minD = m.inf
            minIdx = -1
            for i in range(len(self.vertex)):
                v = self.vertex[i]
                d = m.sqrt((v.xy[0] - vertex[0])**2 + (v.xy[1] - vertex[1])**2)
                if d < dMetric:
                    nearestN.append(i)
#                    if d < minD:
#                        minD = d
#                        minIdx = i
            self.vertex.append(n.Node(vertex))
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
            if self.eq(vertex[0], self.vertex[i].xy[0]) and self.eq(vertex[1], self.vertex[i].xy[1]):
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
            if self.eq(vertex[0], self.vertex[i].xy[0]) and self.eq(vertex[1], self.vertex[i].xy[1]):
                vertexInd = i
                break
        # initialize single source
        for i in range(len(self.vertex)):
            self.vertex[i].p = -1
            self.vertex[i].d = m.inf
            self.vertex[i].h = m.inf
        self.vertex[vertexInd].d = 0
        # Initialize the queue
        minQ = q.PQueue()
        minQ.insert_key((vertexInd, self.vertex[vertexInd]))
        S = []
        qA = [vertexInd]
        while not minQ.isEmpty():
            u = minQ.extract_min()
            S.append(u[0])
            for v in self.edges[u[0]]:
                self.relax(u[0], v)
                if v not in qA:
                    minQ.insert_key((v, self.vertex[v]))
                    qA.append(v)

        return S, self.parents
    
    def relax(self, u, v):
        """ """
        def w(u, v, vertex):
            return m.sqrt((vertex[u].xy[0] + vertex[v].xy[0])**2 + (vertex[u].xy[1] + vertex[v].xy[1])**2)
        if self.vertex[v].d > self.vertex[u].d + w(u, v, self.vertex):
            self.vertex[v].d = self.vertex[u].d + w(u, v, self.vertex)
            self.vertex[v].p = u
    
    def Astar(self, initV, endV):
        """ """
        def distanceF(p1, p2):
            return m.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
        vertexInd = -1
        # Find index
        for i in range(len(self.vertex)):
            if self.eq(initV[0], self.vertex[i].xy[0]) and self.eq(initV[1], self.vertex[i].xy[1]):
                vertexInd = i
                break
        # initialize single source
        for i in range(len(self.vertex)):
            self.vertex[i].p = -1
            self.vertex[i].d = m.inf
            self.vertex[i].h = m.inf
        # Initialize distance arrays
        self.vertex[vertexInd].d = 0
        self.vertex[vertexInd].h = distanceF(initV, endV)
        # Initialize the queue
        minQ = q.PQueue()
        minQ.insert_key((vertexInd, self.vertex[vertexInd]))
        S = []
        qA = [vertexInd]
        while not minQ.isEmpty():
            u = minQ.extract_min()
            if self.eq(endV[0], self.vertex[u[0]].xy[0]) and self.eq(endV[1], self.vertex[u[0]].xy[1]):
                return self.getRoute(endV)
            S.append(u[0])
            for v in self.edges[u[0]]:
                d = self.vertex[u[0]].d + distanceF(self.vertex[u[0]].xy, self.vertex[v].xy)
                if v not in qA:
                    minQ.insert_key((v, self.vertex[v]))
                    qA.append(v)
                elif d >= self.vertex[v].d:
                    continue
                self.vertex[v].p = u[0]
                self.vertex[v].d = d
                self.vertex[v].h = d + distanceF(self.vertex[v].xy, endV)

    def getRoute(self, vertex):
        vertexInd = -1
        # Find index
        for i in range(len(self.vertex)):
            if self.eq(vertex[0], self.vertex[i].xy[0]) and self.eq(vertex[1], self.vertex[i].xy[1]):
                vertexInd = i
                break
        route = []
        endR = False
        while not endR:
            route.append(self.vertex[vertexInd].xy)
            if self.vertex[vertexInd].p == -1:
                endR = True
            vertexInd = self.vertex[vertexInd].p
        return route

    def getNodes(self, indexList):
        """ Returns a list of nodes with index from indexList """
        nodes = []
        for i in indexList:
            nodes.append(self.vertex[i])
        return nodes
        
    def eq(self, x1, x2):
            return m.fabs(x1 - x2) < 0.001

    def getVertex(self):
        v = [n.xy for n in self.vertex]
        return v
    
    def getEdges(self):
        return self.edges

#def main():
#    g = Graph()
#    g.addVertex((0, 0), 0.51)
#    g.addVertex((1, 1), 0.51)
#    g.addVertex((0.5, 0), 0.51)
#    g.addVertex((1.5, 1.0), 0.51)
#    g.addVertex((0.75, 0.75), 1.0)
#
#main()