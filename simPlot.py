import math
import numpy as np
import matplotlib.pyplot as plt

import mathHelper as mh

"""
>>Make the function smarter currently its jugadu
>>Need some cleaning here

Mutliple Plots: https://jakevdp.github.io/PythonDataScienceHandbook/04.08-multiple-subplots.html
                https://www.python-course.eu/matplotlib_multiple_figures.php
Line segment plot: https://stackoverflow.com/questions/36470343/how-to-draw-a-line-with-matplotlib/36479941

Function: Plot the simulation
Input: Agent array, obstacle list, reference figure, probability weight values in 3x1 form
"""
def plotSimulation(agent, obstacleList, path1, path2 = None, human_path=None, agpath=None, cur_frame=None):
    #display.clear_output(wait=True)
    #Initialization of the figure << which i guess can be made a bit different
    sub = []
    X = [ (1,2,1), (3,2,2), (3,2,4), (3,2,6) ]
    fig = plt.figure(figsize=(18,9))
    for nrows, ncols, plot_number in X:
        f = fig.add_subplot(nrows, ncols, plot_number)
        sub.append(f)
        
    #Plot the obstacle environment of rectangles as static obstacles
    for fig, num in obstacleList.items():
        if fig=='rect':    
            for startpt, length, width in num:
                rectangle = plt.Rectangle(startpt, length, width)#, fc='r')
                sub[0].add_patch(rectangle)
        if fig=='circle':
            for centre, radius in num:
                circle = plt.Circle(centre, radius=radius)#, fc='y')
                sub[0].add_patch(circle)
        if fig=='line':
            for pt1, pt2, width in num:
                line = plt.Line2D((pt1[0], pt2[0]), (pt1[1], pt2[1]), lw=width)
                sub[0].add_line(line)
                
    # make the plot as (800 x 800)
    sub[0].set_xticks(np.arange(0, 800, 20))
    sub[0].set_yticks(np.arange(0, 800, 20))
    sub[0].set_xlim((0,800))    
    sub[0].set_ylim((0,800))
    sub[0].grid(True, linewidth=.2)
    
    # Plot unsimulated human paths #plotting the dynamic obstacles << need to see the function again
    if human_path is not None:
        for i in range(len(agpath)):
            if len(agpath[i])>0 and agpath[i][-1][1]==cur_frame: #x,y should be <800,be in frame and not equal to simulator agent
                human_path[i].append(agpath[i].pop()[2])         #store coord frame now
                xx = np.array(human_path[i]).T
                sub[0].text(xx[0][0], xx[1][0], str(i+1), fontsize=18)
                sub[0].plot(xx[0], xx[1], linewidth = 2)

    # Plot various types/forms of agent path
    sub[0].plot(np.array(agent.path).T[0], np.array(agent.path).T[1], '-r') # path simulated from algorithm
    sub[0].plot(np.array(path1).T[0], np.array(path1).T[1])         # path generated by RRT algorithm < check for real data
    
    if path2 is not None:
        sub[0].plot(np.array(path2).T[0], np.array(path2).T[1])     #smoothened (line segments) path obtained from RRT algorithm

    # Plot obstacle scan data i.e. like a lidar scan
    agent_angle     = mh.vectorToAngle(agent.cur_vel)
    [sub[0].plot([pt[0], agent.cur_pos[0]], 
                 [pt[1], agent.cur_pos[1]], 
                 color='k', linestyle='dashed', linewidth=0.3) for pt in agent.obs_map.T[0]]
    
    #Plot them probability map arrays
    sub[1].plot(agent.prob_obs_map)
    sub[2].plot(agent.prob_goal_map)
    sub[2].plot(agent.prob_vel_map)
    sub[3].plot(agent.prob_tot_map)
    
    plt.pause(0.01)
    plt.show()
    return 0


def plotTriangle(pathSeg, heightComp, baseComp):
    #get direction component from pathSeg
    (x1, y1) = (pathSeg[0][0], pathSeg[0][1])
    (x2, y2) = (pathSeg[1][0], pathSeg[1][1])
    theta = (y2 - y1)/(x2 - x1)
    pTheta= -1/theta

    #resize the size according to the passed parameters
    #baseWidth = baseComp/2
    #(x3, y3) = (baseWidth/math.sqrt(1.0 + pTheta*pTheta) + x1, y1 - pTheta*(x1 -  x3))
    #(x4, y4) = (-baseWidth/math.sqrt(1.0 + pTheta*pTheta) + x1, y1 - pTheta*(x1 -  x4))

    s=1/math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))
    (x3, y3) = (x2-(y1-y2), y2+(x1-x2))
    (x4, y4) = (x2+(y1-y2), y2-(x1-x2))
    #make an arrow towards the velocity vector
    #plot([x1, x2], [y1, y2], color='k', linestyle='-', linewidth=2)
    #draw a patch of triangle and pass the patch
    vtex = [[x1,y1], [x3,y3], [x4,y4]]
    t1 = plt.Polygon(vtex, color='blue')
    
    plt.gca().add_patch(t1)
    plt.plot([x for (x, y) in path_xy], [y for (x, y) in path_xy], '-r')
    plt.pause(.1)
    plt.show()
    
    return pTheta #need to decide


"""
Function: Introduce Random obstacles in the environment
Input: Number of obstacles and variance of radius
"""
def addCircularObstacles(obstacleList, num, radius, rad_std):
    pos = np.random.normal(radius, rad_std , 1)  # np.random.normal(centre_of_door, sigma = doorlen/6, 1), door_yaxis
    return circular_obs