
"""
1. A calibration process is required
2. there must be a higher distance limit for the obstacle scan 

Checking point inside a rectangle: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
"""
import math
import numpy as np
import matplotlib.pyplot as plt

from agentSimulation import agent
from pathPlanRRT import RRT


simulationRect = simOrigin, simWidth, simHeight = ((0, 0), 800, 800)
obstacleList = [
((0,0),     200, 125),
((300,0),   200, 125),
((0,250),   50,  400),
((150,250), 100, 400),
((350,250), 100, 400),
((550,250), 100, 400),
((750,250), 50,  400),
((0,750),   200, 50),
((300,750), 200, 50),
((600,750), 200, 50),
] #[(startPoint), length_x, width_y]

num_of_partition = 50
start  = np.array([650,1]) 
goal   = np.array([500, 400])
cur_vel= np.array([5,-5])

show_animation = False
rrt  = RRT(start=start, goal=goal, randArea=[0, 800], obsList=obstacleList, expandDis=10.0)
path = rrt.Planning(animation=show_animation)

path_x = [x for (x, y) in reversed(path)]
path_y = [y for (x, y) in reversed(path)]

for i in range(len(path_x)-1):
    #rrt.DrawGraph()
    plt.clf()
    plt.plot(path_x[0:i+1], path_y[0:i+1], '-r')
    for startpt, length, width in obstacleList:
        rectangle = plt.Rectangle(startpt, length, width, fc='r')
        plt.gca().add_patch(rectangle)
        
    start   = np.array([path_x[i], path_y[i]])
    cur_vel = np.array([path_x[i+1] - path_x[i], path_y[i+1] - path_y[i]])
    agent1  = agent(start, goal, cur_vel)
    obsMap  = agent1.obstacleMap(obstacleList)
    agent1.plotObstacleMap(obsMap.T[0])
    
    plt.pause(0.05)
    #plt.show()