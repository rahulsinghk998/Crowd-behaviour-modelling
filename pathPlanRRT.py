import math
import copy
import random
import numpy as np
import numpy.random as rnd
import matplotlib.pyplot as plt


class RRT():
    """ Class for RRT Planning """

    def __init__(self, start, goal, obsList, randArea, expandDis=1.0, goalSampleRate=5, maxIter=500):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start          = Node(start[0], start[1])
        self.end            = Node(goal[0], goal[1])
        self.minrand        = randArea[0]
        self.maxrand        = randArea[1]
        self.expandDis      = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter        = maxIter
        self.obsList        = obsList

    def Planning(self, animation=True):
        """ Pathplanning animation: flag for animation on or off """
        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, self.obsList):
                continue

            self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break
                
            if animation:
                plt.pause(0.01)
                self.DrawGraph(rnd)

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x, node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self, rnd=None):
        #plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-g")

        self.PlotObstacles(self.obsList)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([0, 800, 0, 800])
        #plt.grid(True)
        #plt.pause(0.2)

    def PlotObstacles(self, obsList):
        for startpt, length, width in obsList:
            rectangle = plt.Rectangle(startpt, length, width, fc='r')
            plt.gca().add_patch(rectangle)

    def PlotCircle(self, x, y, size):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(math.radians(d)) for d in deg]
        yl = [y + size * math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obsList):
        for startpt, length_x, width_y in obsList: #calculating rectangles 4 vertices 
            (x1, y1) = (startpt[0], startpt[1])
            (x2, y2) = (startpt[0], startpt[1] + width_y)
            (x3, y3) = (startpt[0] + length_x, startpt[1] + width_y)
            (x4, y4) = (startpt[0] + length_x, startpt[1])

            p21 = (x2 - x1, y2 - y1)
            p41 = (x4 - x1, y4 - y1)
            p21magnitude_squared = p21[0]*p21[0] + p21[1]*p21[1]
            p41magnitude_squared = p41[0]*p41[0] + p41[1]*p41[1]
            p = (node.x - x1, node.y - y1)

            if 0 <= p[0] * p21[0] + p[1] * p21[1] <= p21magnitude_squared:
                if 0 <= p[0] * p41[0] + p[1] * p41[1] <= p41magnitude_squared:
                    return False
        return True

"""
RRT Node
"""
class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        
"""
Function: Find if the line segment intersects with a rectangle or not
Input Param: segment point 0, segment point 1, rectangle
"""
def LineCollisionCheck(pt0, pt1, obstacleList):
    for rectangle in obstacleList:
        # calculating rectangles 4 vertices
        startpt, length_x, width_y = rectangle    
        p1 = (x1, y1) = (startpt[0], startpt[1])
        p2 = (x2, y2) = (startpt[0], startpt[1] + width_y)
        p3 = (x3, y3) = (startpt[0] + length_x, startpt[1] + width_y)
        p4 = (x4, y4) = (startpt[0] + length_x, startpt[1])

        # Find intersection with the workspace boundary
        vertexList = np.array([p1, p2, p3, p4], dtype=np.float)
        for i in range(len(vertexList)):
            if findIntersection(vertexList[-1+i], vertexList[i], pt0, pt1):
                return False
        
    return True
    
    
"""
Function : Calculating intersection of two line segments: Line1->(p0,p1) and Line2->(p2,p3)
Help Link: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
"""
def findIntersection(p0, p1, p2, p3):
    s10_x = p1[0] - p0[0]
    s10_y = p1[1] - p0[1]
    s32_x = p3[0] - p2[0]
    s32_y = p3[1] - p2[1]

    denom = s10_x * s32_y - s32_x * s10_y
    if denom == 0 : return None # collinear

    denom_is_positive = denom > 0
    s02_x = p0[0] - p2[0]
    s02_y = p0[1] - p2[1]
    s_numer = s10_x * s02_y - s10_y * s02_x

    if (s_numer < 0) == denom_is_positive : return None # no collision
    t_numer = s32_x * s02_y - s32_y * s02_x
    if (t_numer < 0) == denom_is_positive : return None # no collision
    if (s_numer > denom) == denom_is_positive or (t_numer > denom) == denom_is_positive : return None # no collision
    
    # collision detected
    t = t_numer / denom
    intersection_point = [ p0[0] + (t * s10_x), p0[1] + (t * s10_y) ]

    return intersection_point


def GetPathLength(path):
    le = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
    return le


def GetTargetPoint(path, targetL):
    le = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        le += d
        if le >= targetL:
            ti = i - 1
            lastPairLen = d
            break

    #Writing (x,y) coordinates from the line length ratio and slope info
    partRatio = (le - targetL) / lastPairLen #  print(partRatio)  #  print((ti,len(path),path[ti],path[ti+1]))
    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))
    return [x, y, ti]

"""
Problem: Returns multiple same points on the path which becomes a problem for interpolation<< FIX IT
"""
def PathSmoothing(path, maxIter, obstacleList):
    #  print("PathSmoothing")
    le = GetPathLength(path)
    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, le), random.uniform(0, le)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        #  print(first)
        second = GetTargetPoint(path, pickPoints[1])
        #  print(second)

        if first[2] <= 0 or second[2] <= 0:
            continue
        if (second[2] + 1) > len(path):
            continue
        if second[2] == first[2]:
            continue

        # collision check
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        # Create New path
        newPath = []
        newPath.extend(path[:first[2] + 1])
        newPath.append([first[0], first[1]])
        newPath.append([second[0], second[1]])
        newPath.extend(path[second[2] + 1:])
        path = newPath
        le = GetPathLength(path)

    return path