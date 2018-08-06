"""
1. A calibration process is required
2. there must be a higher distance limit for the obstacle scan 

Checking point inside a rectangle: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
"""

"""
## Organize the "Thesis" ##
#  The approach is kind of similar to GMMs  # 
# Currently, don't know the perfect weights #

#Smooth the path and make into small fragments<<<<<<<<
#Make static human obstacle<< add them into environment


####################  CALIBRATION BASED. ######################
1. A calibration process is required -- it very important
need a controller for velocity
width of average human (in pixel value)
average speed of humna (in walk and brisk walk) - 
->how much each step will be in terms of distance wise which varies say for gaussian distribution way
width of walkway passage and the shelf where items are kept
33. Define the limit condition so as to have real-life cases
1. there must be a higher distance limit for the obstacle scan
2. Other functions to take care of: Detect the position and velocity of each actor at each time stamp

######################  FEATURE BASED. ########################
What are features to consider?
11. obstacle/people in front of if the obstacle is people then the velocity of approach 
-- currently not taking care of social interaction between 2 people
5. can we say goals of 2 types
    a. accurate
    b. directional (where people know in which direction to go don't know the exact path -> for new people or first timers)
"""

"""
Help Sites: Checking point inside a rectangle: https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not
"""




























"""
%matplotlib inline
import math
import pickle
import threading
import numpy as np
import matplotlib.pyplot as plt

import pathPlanRRT as rrt
import agentSimulation as agentsim
import probabilityConv as probconv

from IPython.display import clear_output

scan_map = []
num_of_partition = 50
simulationRect = simOrigin, simWidth, simHeight = ((0, 0), 800, 800)
sim_layout = [((0,0),     200, 125),
              ((300,0),   200, 125),
              ((0,250),   50,  400),
              ((150,250), 100, 400),
              ((350,250), 100, 400),
              ((550,250), 100, 400),
              ((750,250), 50,  400),
              ((0,750),   200, 50),
              ((300,750), 200, 50),
              ((600,750), 200, 50),] #[(startPoint), length_x, width_y]


agent1  = agentsim.agent(start= np.array(agentsim.getStartPos()), 
                         goal=np.array([500, 400]), 
                         cur_vel= np.array([5,-5]))
thread1 = threading.Thread(target=probconv.activateAgent, args=[agent1, sim_layout])
thread1.daemon = True
thread1.start()
while True:    
    if agent1.step == True:
        scan_map.append((agent1.cur_pos, agent1.cur_vel, agent1.obs_map))
        probconv.plotSimulation(agent1, sim_layout)
        agent1.step = False

#Save the scan data with obstacle distance and angle
#with open('scan_map', 'wb') as fp:
#    pickle.dump(scan_map, fp)
"""


"""
import math
import pickle
import threading
import numpy as np
import matplotlib.pyplot as plt

from agentSimulation import agent
from pathPlanRRT import RRT
import agentSimulation as agentsim
import probabilityConv as probconv

scan_map = []


simulationRect = simOrigin, simWidth, simHeight = ((0, 0), 800, 800)
obstacleList = [((0,0),     200, 125),
                ((300,0),   200, 125),
                ((0,250),   50,  400),
                ((150,250), 100, 400),
                ((350,250), 100, 400),
                ((550,250), 100, 400),
                ((750,250), 50,  400),
                ((0,750),   200, 50),
                ((300,750), 200, 50),
                ((600,750), 200, 50),] #[(startPoint), length_x, width_y]

num_of_partition = 50
#Add an agent
agent1  = agent(start= np.array(agentsim.getStartPos()), goal=np.array([500, 400]), cur_vel= np.array([5,-5]))
agent2  = agent(start= np.array(agentsim.getStartPos()), goal=np.array([700, 400]), cur_vel= np.array([5,-5]))

thread1 = threading.Thread(target=probconv.activateAgent, args=[agent1, obstacleList])
thread1.daemon = True
thread1.start()
thread2 = threading.Thread(target=probconv.activateAgent, args=[agent2, obstacleList])
thread2.daemon = True
thread2.start()

while True:    
    if agent1.step == True and agent2.step == True:
        #scan_map.append((agent1.cur_pos, agent1.cur_vel, agent1.obs_map))
        print('rahul')
        plt.clf()
        #plt.figure(num=None, figsize=(12, 12), dpi=80, facecolor='w', edgecolor='k')
        for startpt, length, width in obstacleList:
            rectangle = plt.Rectangle(startpt, length, width, fc='r')
            plt.gca().add_patch(rectangle)
            
        agent1.plotObstacleMap(agent1.obs_map.T[0])
        agent2.plotObstacleMap(agent2.obs_map.T[0])
        plt.plot(np.array(agent1.path).T[0], np.array(agent1.path).T[1], '-r')
        plt.plot(np.array(agent2.path).T[0], np.array(agent2.path).T[1], '-r')
        plt.pause(0.01)
        plt.show()
        probconv.plotSimulation(probconv.getObstacleToProbability(agent1.obs_map.T[1]),
                       probconv.getGoalToProbability(agent1.cur_vel, agent1.cur_pos, agent1.goal),
                       probconv.getVelocityToProbability(agent1.cur_vel))
        
        agent1.step = False
        agent2.step = False


#Save the scan data with obstacle distance and angle
with open('scan_map', 'wb') as fp:
    pickle.dump(scan_map, fp)

"""

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

"""