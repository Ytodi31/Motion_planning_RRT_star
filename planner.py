import numpy as np
import random
from shapely.geometry import Point
import world as w

step_size = 0.1
radius = 0.25

class node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0
        self.arr = np.array([self.x, self.y])
        self.parent = None

class rrt_planner():
    def __init__(self, iterations, map):
        self.K = iterations
        self.world = map
        self.goal_thresh = radius
        self.q_init = self.GetInput()
        self.q_goal, self.goal = self.GetGoal()
        self.nodes = [self.q_init]

    def GetInput(self):
        while True:
            x_start = int(input('Please enter the initial x-coordinate: '))
            y_start = int(input('Please enter the initial y-coordinate: '))
            if self.world.obstacle_avoidance(Point(x_start, y_start)):
                print("Coordinates in obstacle area, please enter again")
            elif x_start > self.world.x_lim or y_start > self.world.y_lim:
                print("Start Coordinates are out of tha map, please enter again")
            else:
                break
        q_init = node(x_start, y_start)
        q_init.parent = q_init
        return q_init

    def GetGoal(self):
        while True:
            x_goal = int(input('Please enter the goal x-coordinate: '))
            y_goal = int(input('Please enter the goal y-coordinate: '))
            if self.world.obstacle_avoidance(Point(x_goal, y_goal)):
                print("Coordinates in obstacle area, please enter again")
            elif x_goal > self.world.x_lim or y_goal > self.world.y_lim:
                print("Goal Coordinates are out of tha map, please enter again")
            else:
                break
        q_goal = node(x_goal, y_goal)
        goal, x, y = w.circle(self.goal_thresh, x_goal,y_goal).create()
        self.world.ax.fill(x, y, "r")
        return q_goal, goal

    def RandomPosition(self):
        x_new = random.random()*self.world.x_lim
        y_new = random.random()*self.world.y_lim
        random_node = node(x_new, y_new)
        return random_node

    def distance(self, point1, point2):
        return np.linalg.norm(point1.arr - point2.arr)

    def Nearest(self, x_rand):
        vertex = []
        for points in self.nodes:
            dist = self.distance(points, x_rand)
            vertex.append([dist, points])
        vertex.sort()
        return vertex[0][1]

    def NewConfig(self, X_new, X_nearest, step_size):
        D = self.distance(X_new, X_nearest)
        point_x = X_nearest.x + (step_size/D)*(X_new.x - X_nearest.x)
        point_y = X_nearest.y + (step_size/D)*(X_new.y - X_nearest.y)
        new_node = node(point_x, point_y)
        return new_node

    def AddParent(self, new_node, X_nearest):
        for node in self.nodes:
            if self.distance(node, new_node) <= radius:
                if node.cost + self.distance(node, new_node) < X_nearest.cost + self.distance(X_nearest, new_node):
                    X_nearest = node
            new_node.cost = X_nearest.cost + self.distance(X_nearest, new_node)
            new_node.parent = X_nearest
        return new_node, X_nearest

    def ReWire(self, new_node):
        for node in self.nodes:
            if node is not new_node.parent:
                if self.distance(new_node, node) < radius and new_node.cost + self.distance(new_node, node) < node.cost:
                    node.parent = new_node
                    node.cost = new_node.cost + self.distance(new_node, node)
        return

    def CheckGoal(self, node):
        p = Point(node.x, node.y)
        if p.within(self.goal):
            return True
        else:
            return False

    def BackTrack(self):
        temp_path = []
        path = []
        for n in self.nodes:
            dist = self.distance(self.q_goal, n)
            temp_path.append([dist, n])
        temp_path.sort()
        closest_node = temp_path[0][1]
        n = closest_node
        while n is not self.q_init:
            path.append(n)
            n = n.parent
        path.append(self.q_init)
        for n in path:
            self.world.ax.plot([n.x, n.parent.x], [n.y, n.parent.y], 'g')
        return path

world = w.map(10, 10)
world.create_map()
rrt = rrt_planner(5000, world)
# world.show_map()
for k in range(0, rrt.K):
    X_new = rrt.RandomPosition() # Finding a random configuration
    if world.obstacle_avoidance(Point(X_new.x, X_new.y)): #Checking if random node is in C-obs
        continue
    X_nearest = rrt.Nearest(X_new) # Finding nearest node
    new_node = rrt.NewConfig(X_new, X_nearest, step_size)
    if world.obstacle_avoidance(Point(new_node.x, new_node.y)):
        continue
    new_node, X_nearest = rrt.AddParent(new_node, X_nearest)
    rrt.nodes.append(new_node)
    rrt.ReWire(new_node)
    world.ax.plot([new_node.x, new_node.parent.x], [new_node.y, new_node.parent.y], 'r')
path = rrt.BackTrack()
world.show_map()