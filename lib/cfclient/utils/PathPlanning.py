__author__ = 'Bonus'

import sys
import math
import numpy as np
import heapq

def Graph():
    dict = {}
    sx = [0, 1, 2, 3, 4, 5, 5, 6, 6, 7, 7]
    sy = [0, 1, 2, 3, 3, 4, 4, 5, 5, 5, 6]
    ex = [0, 24, 23, 22, 21, 20, 20, 19, 19, 18, 18]
    ey = [0, 18, 17, 16, 16, 15, 15, 14, 14, 14, 13]
    width = [0, 24, 22, 20, 18, 16, 16, 14, 14, 12, 12]
    height = [0, 18, 16, 14, 14, 12, 12, 10, 10, 10, 8]
    lz = 10
    for z in xrange(1, lz + 1):
        for y in xrange(sy[z], ey[z] + 1):
            for x in xrange(sx[z], ex[z] + 1):
                adjacent = []
                if (x == sx[z]) or (x == ex[z]) or (y == sy[z]) or (y == ey[z]):
                    sj, ej, sk, ek = -1, 2, -1, 2
                    if x == sx[z]: sk += 1
                    if x == ex[z]: ek -= 1
                    if y == sy[z]: sj += 1
                    if y == ey[z]: ej -= 1
                    for i in xrange(-1, 1):
                        for j in xrange(sj, ej):
                            for k in range(sk, ek):
                                if  (z + i > 0) and (z + i <= lz) and \
                                    (y + j >= sy[z]) and (y + j <= ey[z]) and \
                                    (x + k >= sx[z]) and (x + k <= ex[z]) and \
                                    not ((i == 0) and (j == 0) and (k == 0)):
                                    adjacent.append((x + k, y + j, z + i))
                    if (z < lz) and (width[z] == width[z + 1]):
                        if height[z] == height[z + 1]:
                            for j in xrange(sj, ej):
                                for k in range(sk, ek):
                                    if  (y + j >= sy[z]) and (y + j <= ey[z]) and \
                                        (x + k >= sx[z]) and (x + k <= ex[z]):
                                        adjacent.append((x + k, y + j, z + 1))
                        else:
                            if (y >= sy[z + 1]) and (y <= ey[z + 1]):
                                for j in xrange(sj, ej):
                                    for k in range(sk, ek):
                                        if  (y + j >= sy[z + 1]) and (y + j <= ey[z + 1]) and \
                                            (x + k >= sx[z + 1]) and (x + k <= ex[z + 1]):
                                            adjacent.append((x + k, y + j, z + 1))
                    elif (z < lz) and (height[z] == height[z + 1]):
                        if (x >= sx[z + 1]) and (x <= ex[z + 1]):
                            for j in xrange(sj, ej):
                                for k in range(sk, ek):
                                    if  (y + j >= sy[z + 1]) and (y + j <= ey[z + 1]) and \
                                        (x + k >= sx[z + 1]) and (x + k <= ex[z + 1]):
                                        adjacent.append((x + k, y + j, z + 1))

                elif (x == sx[z] + 1) or (x == ex[z] - 1) or (y == sy[z] + 1) or (y == ey[z] - 1):
                    for i in xrange(-1, 1):
                        for j in xrange(-1, 2):
                            for k in range(-1, 2):
                                if  (z + i > 0) and (z + i <= lz) and \
                                    (y + j >= sy[z]) and (y + j <= ey[z]) and \
                                    (x + k >= sx[z]) and (x + k <= ex[z]) and \
                                    not ((i == 0) and (j == 0) and (k == 0)):
                                    adjacent.append((x + k, y + j, z + i))
                    if (z < lz) and width[z] == width[z + 1]:
                        if height[z] == height[z + 1]:
                            for j in xrange(-1, 2):
                                for k in range(-1, 2):
                                    adjacent.append((x + k, y + j, z + 1))
                        else:
                            sj, ej = -1, 2
                            if y == sy[z + 1]: sj += 1
                            if y == ey[z + 1]: ej -= 1
                            for j in xrange(sj, ej):
                                for k in range(-1, 2):
                                    adjacent.append((x + k, y + j, z + 1))
                    elif (z < lz) and (height[z] == height[z + 1]):
                        sk, ek = -1, 2
                        if x == sx[z + 1]: sk += 1
                        if x == ex[z + 1]: ek -= 1
                        for j in xrange(-1, 2):
                            for k in range(sk, ek):
                                adjacent.append((x + k, y + j, z + 1))
                    elif z < lz:
                        sj, ej, sk, ek = -1, 2, -1, 2
                        if y == sy[z + 1]: sj += 1
                        if y == ey[z + 1]: ej -= 1
                        if x == sx[z + 1]: sk += 1
                        if x == ex[z + 1]: ek -= 1
                        for j in xrange(sj, ej):
                            for k in xrange(sk, ek):
                                adjacent.append((x + k, y + j, z + 1))
                else:
                    for i in xrange(-1, 2):
                        for j in xrange(-1, 2):
                            for k in range(-1, 2):
                                if  (z + i > 0) and (z + i <= lz) and \
                                    (y + j >= sy[z]) and (y + j <= ey[z]) and \
                                    (x + k >= sx[z]) and (x + k <= ex[z]) and \
                                    not ((i == 0) and (j == 0) and (k == 0)):
                                    adjacent.append((x + k, y + j, z + i))
                dict[(x, y, z)] = adjacent
    return dict

class PathFinding:

    def __init__(self, graph, start, goal, bound):
        self.start = start
        self.goal = goal
        self.graph = graph
        self.bound = bound

    def Distance(self, point1, point2):
        return (point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2 + (point2[2] - point1[2]) ** 2

    def Solve(self):
        return self.AStar()

    def AStar(self):
        openSet = set()
        closeSet = set()
        openHeap = []
        h = np.zeros((self.bound[2], self.bound[1], self.bound[0]))
        g = np.zeros((self.bound[2], self.bound[1], self.bound[0]))
        parent = {}
        for i in xrange(self.bound[2]):
            for j in xrange(self.bound[1]):
                for k in xrange(self.bound[0]):
                    h[i, j, k] = self.Distance((k, j, i), self.goal)

        def ReconstructPath(p):
            path = [p]
            while p != self.start:
                p = parent[p]
                path.append(p)
            path.reverse()
            return path

        current = self.start
        openSet.add(self.start)
        openHeap.append((0, self.start))
        while openSet:
            while current in closeSet:
                current = heapq.heappop(openHeap)[1]

            if current == self.goal:
                return ReconstructPath(current)
            openSet.remove(current)
            closeSet.add(current)
            for node in self.graph[current]:
                if node not in closeSet:
                    tentative = g[current[2], current[1], current[0]] + self.Distance(current, node)
                    if node not in openSet:
                        g[node[2], node[1], node[0]] = sys.maxint
                        openSet.add(node)
                    if tentative < g[node[2], node[1], node[0]]:
                        g[node[2], node[1], node[0]] = tentative
                        heapq.heappush(openHeap, (math.sqrt(h[node[2], node[1], node[0]]) + math.sqrt(g[node[2], node[1], node[0]]), node))
                        parent[node] = current
