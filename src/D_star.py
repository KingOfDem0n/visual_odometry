#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import math
from collections import deque
from priority_queue import PriorityQueue

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid

class Node(object):
    def __init__(self, val, g, rhs, x, y, bounds):
        self.val = val
        self.g = g
        self.rhs = rhs
        self.position = (y, x)
        self.neighbors = []
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                r = y + i
                c = x + j
                if (i != 0 or j != 0) and 0 <= r < bounds[0] and 0 <= c < bounds[1]:
                    self.neighbors.append((r,c))

class DStarLiteOptimized(object):
    """
        From original paper: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf
    """
    def __init__(self, map, start, goal):
        """
            map int8(n,m): An integer occupancy map
            start tuple(2): Tuple of size 2 containing (x,y)
            goal tuple(2): Tuple of size 2 containing (x,y)
        """
        self.start = start # tuple (y,x)
        self.goal = goal # tuple (y,x)
        self.position = self.start
        self.map = map.copy()
        self.km = 0
        self.U = PriorityQueue()
        self.nodes = np.empty(map.shape, dtype=object)

        # Populate self.nodes
        for y in range(map.shape[0]):
            for x in range(map.shape[1]):
                self.nodes[y,x] = Node(map[y, x], float('inf'), float('inf'), x, y, self.map.shape)
        self.nodes[self.goal].rhs = 0

        self.U.insert(self.nodes[self.goal], (self.h(self.nodes[self.start], self.nodes[self.goal]), 0))

    def h(self, a, b):
        y1, x1 = a.position
        y2, x2 = b.position

        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def c(self, a, b):
        if a.val == 1 or b.val == 1:
            return float('inf')
        y1, x1 = a.position
        y2, x2 = b.position

        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def calculate_rhs(self, u):
        value = []
        for (r,c) in u.neighbors:
            value.append(self.c(u, self.nodes[r,c])+self.nodes[r,c].g)

        return min(value)

    def successor(self, u):
        succ = []
        for (r,c) in u.neighbors:
            succ.append((r,c,self.c(u, self.nodes[r,c])+self.nodes[r,c].g))

        return min(succ, key= lambda x: x[2])[:2]

    def calculateKey(self, s):
        return (min([s.g, s.rhs]) + self.h(self.nodes[self.start],s) + self.km, min([s.g, s.rhs]))

    def updateVertex(self, u):
        open = list(self.U.__iter__())
        if u.g != u.rhs and open:
            self.U.update(u, self.calculateKey(u))
        elif u.g != u.rhs and (u not in open):
            self.U.insert(u, self.calculateKey(u))
        elif u.g == u.rhs and (u in open):
            self.U.delete(u)

    def computeShortestPath(self):
        while self.U.topKey() < self.calculateKey(self.nodes[self.position]) or self.nodes[self.position].rhs > self.nodes[self.position].g:
            k_old = self.U.topKey()
            u = self.U.top()
            k_new = self.calculateKey(u)

            if k_old < k_new:
                self.U.update(u, k_new)
            elif u.g > u.rhs:
                u.g = u.rhs
                self.U.delete(u)
                for n in u.neighbors:
                    if n != self.goal:
                        self.nodes[n].rhs = min([self.nodes[n].rhs, self.c(self.nodes[n],u) + u.g])
                    self.updateVertex(self.nodes[pred])
            else:
                g_old = u.g
                u.g = float('inf')
                for n in u.neighbors:
                    if self.nodes[n].rhs == self.c(self.nodes[n], u) + g_old:
                        if n != self.gaol:
                            self.nodes[n].rhs = self.calculate_rhs(self.nodes[n])
                    self.updateVertex(self.nodes[n])

    def scanForChange(self):
        occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        new_map = np.array(occupancy_grid.data, dtype=np.int8)
        new_map = map/100
        new_map[new_map < 0] = -1
        self.map = new_map

        return np.array(self.map != new_map, dtype=bool)

    def run(self):
        last_node = self.position
        path = [self.position]
        self.computeShortestPath()
        changes = []

        while self.position != self.goal:
            if self.nodes[self.position].rhs == float('inf'):
                raise Exception("Can not determine a path")

            self.position = self.successor(self.nodes[self.position])

            if count == 10:
                new_map = self.map.copy()
                new_map[7, 1:10] = 1
                changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
                self.map = new_map.copy()
                displayMap("Change 1", self.map, self.start, self.goal, path)
            elif count == 15:
                new_map = self.map.copy()
                new_map[:, :] = 0
                changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
                self.map = new_map.copy()
                displayMap("Change 2", self.map, self.start, self.goal, path)

            # # Scan graph for changed edge costs
            # changes = scanForChange()
            if np.any(changes):
                self.km += self.h(self.nodes[last_node], self.nodes[self.position])
                last_node = self.position

                for u in np.argwhere(changes==True):
                    for v in self.nodes[u].neighbors:
                        c_old = self.c(self.nodes[u], self.nodes[v])
                        self.nodes[u].val = self.map[u]
                        if c_old > self.c(self.nodes[u], self.nodes[v]):
                            if u != self.goal:
                                self.nodes[u] = min([self.nodes[u].rhs, self.c(self.nodes[u],self.nodes[v]) + v.g])

                    # for v in self.nodes[i].pred:
                    #     c_old = self.c(self.nodes[u], self.nodes[v])
                    #     self.nodes[u].val = self.map[u]
                    #     self.nodes[v].val = self.map[v]
                    #     if c_old > self.c(self.nodes[u], self.nodes[v]):
                    #         if u != self.goal:
                    #             self.nodes[u].rhs = min([self.nodes[u].rhs, self.c(self.nodes[u], self.nodes[v])+self.nodes[v].g])
                    #     elif self.nodes[u].rhs == c_old + self.nodes[v].g:
                    #         if u != self.goal:
                    #             self.nodes[u].rhs = min([self.c(self.nodes[u],self.nodes[succ])+self.nodes[succ].g for succ in self.nodes[u].succ])
                    #     self.updateVertex(self.nodes[u])
                self.computeShortestPath()
                changes = []

            path.append(self.position)

        return path

class DStarLiteCMU(object):
    """
        From CMU slide: https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf
    """
    def __init__(self, map, start, goal):
        """
            map int8(n,m): An integer occupancy map
            start tuple(2): Tuple of size 2 containing (x,y)
            goal tuple(2): Tuple of size 2 containing (x,y)
        """
        self.start = start # tuple (y,x)
        self.goal = goal # tuple (y,x)
        self.position = self.start
        self.map = map.copy()
        self.U = PriorityQueue()
        self.nodes = np.empty(map.shape, dtype=object)

        # Populate self.nodes
        for y in range(map.shape[0]):
            for x in range(map.shape[1]):
                self.nodes[y,x] = Node(map[y, x], float('inf'), float('inf'), x, y, self.map.shape)
        self.nodes[self.goal].rhs = 0

        self.U.insert(self.nodes[self.goal], self.calculateKey(self.nodes[self.goal]))

    def c(self, a, b):
        if a.val == 1 or b.val == 1:
            return float('inf')
        y1, x1 = a.position
        y2, x2 = b.position

        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def h(self, a, b):
        y1, x1 = a.position
        y2, x2 = b.position

        # return 0
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)

    def calculateKey(self, s):
        return (min([s.g, s.rhs]) + self.h(self.nodes[self.position],s), min([s.g, s.rhs]))

    def updateVertex(self, u):
        open = list(self.U.__iter__())
        if u is not self.nodes[self.goal]:
            u.rhs = self.calculate_rhs(u)
        if u in open:
            self.U.delete(u)
        if u.g != u.rhs:
            self.U.insert(u, self.calculateKey(u))

    def calculate_rhs(self, u):
        value = []
        for (r,c) in u.neighbors:
            value.append(self.c(u, self.nodes[r,c])+self.nodes[r,c].g)

        return min(value)

    def successor(self, u):
        succ = []
        for (r,c) in u.neighbors:
            succ.append((r,c,self.c(u, self.nodes[r,c])+self.nodes[r,c].g))

        return min(succ, key= lambda x: x[2])[:2]

    def computeShortestPath(self):
        it_count = 0
        while (not self.U.empty() and self.U.topKey() < self.calculateKey(self.nodes[self.position])) or self.nodes[self.position].rhs != self.nodes[self.position].g:
            it_count += 1
            u = self.U.top()
            if u.g > u.rhs:
                u.g = u.rhs
                for n in u.neighbors:
                    self.updateVertex(self.nodes[n])
            else:
                u.g = float('inf')
                for n in u.neighbors + [u.position]:
                    self.updateVertex(self.nodes[n])

    def scanForChange(self):
        occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
        new_map = np.array(occupancy_grid.data, dtype=np.int8)
        new_map = map/100
        new_map[new_map < 0] = -1
        changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
        self.map = new_map.copy()

        return changes

    def displayValueMap(self):
        str = ""
        for r in range(self.nodes.shape[0]):
            for c in range(self.nodes.shape[1]):
                str += "{} ".format(round(self.nodes[r,c].rhs, 2))
            print(str)
            str = ""
        print()

    def run(self):
        path = [self.position]
        self.computeShortestPath()
        count = 0
        changes = []
        while self.position != self.goal:
            if self.nodes[self.position].rhs == float('inf'):
                raise Exception("Can not determine a path")

            self.position = self.successor(self.nodes[self.position])

            if count == 10:
                new_map = self.map.copy()
                new_map[7, 1:10] = 1
                changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
                self.map = new_map.copy()
                displayMap("Change 1", self.map, self.start, self.goal, path)
            elif count == 15:
                new_map = self.map.copy()
                # new_map[6:9, 5] = 1
                new_map[:, :] = 0
                changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
                self.map = new_map.copy()
                displayMap("Change 2", self.map, self.start, self.goal, path)
            elif count == 16:
                new_map = self.map.copy()
                new_map[7, 1:10] = 1
                changes = np.array(self.map != new_map, dtype=bool).reshape(self.map.shape)
                self.map = new_map.copy()
                displayMap("Change 3", self.map, self.start, self.goal, path)

            if np.any(changes):
                for u in np.argwhere(changes==True):
                    u = tuple(u)
                    self.nodes[u].val = self.map[u]
                    self.updateVertex(self.nodes[u])

                    # Update the surrounding neighbors
                    for n in self.nodes[u].neighbors:
                        self.updateVertex(self.nodes[n])

                open = list(self.U.__iter__())
                for s in open:
                    self.U.update(s, self.calculateKey(s))
                self.computeShortestPath()
                changes = []

            path.append(self.position)
            count += 1
        return path

def displayMap(title, map, start, goal, path=[]):
    str = ""
    print(title)
    for r in range(map.shape[0]):
        for c in range(map.shape[1]):
            if (r,c) == start:
                str += "S "
            elif (r,c) in path:
                str += "X "
            elif (r,c) == goal:
                str += "G "
            else:
                str += "{} ".format(int(map[r,c]))
        print(str)
        str = ""
    print()

def main():
    # rospy.init_node('D_star')
    # occupancy_grid = rospy.wait_for_message("/map", OccupancyGrid)
    # map = np.array(occupancy_grid.data, dtype=np.int8)
    # map = map/100
    # map[map < 0] = -1

    start = (0,0)
    goal = (9,9)
    map = np.zeros((10,10))
    map[3, 0:9] = 1
    displayMap("Original Map", map, start, goal)

    dstar = DStarLiteCMU(map, start, goal)
    path = dstar.run()

    displayMap("Final Map", map, start, goal, path)

if __name__ == "__main__":
    main()
