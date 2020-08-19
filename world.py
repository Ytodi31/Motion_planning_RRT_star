import cv2
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from shapely.geometry.polygon import LinearRing

class polygon():
    def __init__(self, coords):
        self.vertex = coords

    def create(self):
        poly = Polygon(self.vertex)
        ring = LinearRing(self.vertex)
        x, y = ring.xy
        return poly, x, y

class circle():
    def __init__(self, radius, xc, yc):
        self.point = Point(xc, yc)
        self.r = radius

    def create(self):
        circ = self.point.buffer(self.r)
        x, y = circ.exterior.xy
        return circ, x, y

class map():
    def __init__(self, x, y):
        self.obstacles = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.x_lim = x
        self.y_lim = y

    def create_map(self):
        coords1 = [(1.75, 6.75), (2.25, 6.75), (2.25, 5.25), (1.75, 5.25)]
        poly1, x, y = polygon(coords1).create()
        self.ax.fill(x, y, "k")
        self.obstacles.append(poly1)

        coords2 = [(3.75, 8.75), (4.25, 8.75), (4.25, 7.25), (3.75, 7.25)]
        poly2, x, y = polygon(coords2).create()
        self.ax.fill(x, y, "k")
        self.obstacles.append(poly2)

        coords3 = [(8.25, 5.75), (9.75, 5.75), (9.75, 5.75), (8.25, 5.75)]
        poly3, x, y = polygon(coords3).create()
        self.ax.fill(x, y, "k")
        self.obstacles.append(poly3)

        circ1, x, y = circle(1, 7, 2).create()
        self.ax.fill(x, y, "k")
        self.obstacles.append(circ1)

    def obstacle_avoidance(self, p):
        for obstacle in self.obstacles:
            if p.within(obstacle):
                return True
        return False

    def show_map(self):
        self.ax.axis('equal')
        self.ax.plot(0,0, 'r', 3)
        self.ax.set_xlim([0, self.x_lim])
        self.ax.set_ylim([0, self.y_lim])
        self.ax.set_aspect(1)
        plt.show()
#
# world = map(5.5, 5.5)
# world.create_map()
# world.ax
# world.show_map()
# print(world.obstacle_avoidance(Point(5, 5)))

# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.plot([0], [0], 'ro')
# plt.show()