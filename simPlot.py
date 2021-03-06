import cv2
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

import mathHelper as mh


"""
>>Make the function smarter currently its jugadu
>>Need some cleaning here
# Fix the ellipse color

Drawing ellipse in matplotlib: https://matplotlib.org/api/_as_gen/matplotlib.patches.Ellipse.html
Mutliple Plots: https://jakevdp.github.io/PythonDataScienceHandbook/04.08-multiple-subplots.html
                https://www.python-course.eu/matplotlib_multiple_figures.php
Arrow plot :    https://matplotlib.org/api/_as_gen/matplotlib.pyplot.arrow.html
Line segment plot: https://stackoverflow.com/questions/36470343/how-to-draw-a-line-with-matplotlib/36479941

Function: Plot the simulation
Input: Agent array, obstacle list, reference figure, probability weight values in 3x1 form
"""
def plotSimulation(agent, path1, warped):  #path1 can also be removed
    #display.clear_output(wait=True)
    # Initialization of the figure << which i guess can be made a bit different
    sub = []
    X = [ (2,2,1), (6,2,2), (6,2,4), (6,2,6), (2,2,3), (6,2,8), (6,2,10), (6,2,12)]
    fig = plt.figure(figsize=(18,18))
    for nrows, ncols, plot_number in X:
        f = fig.add_subplot(nrows, ncols, plot_number)
        sub.append(f)
        
    # Plot the obstacle environment of rectangles as static obstacles
    for fig, num in agent.simulation_env.items():
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
                
    # Make the plot as (800 x 800)
    sub[0].set_xticks(np.arange(0, 800, 20))
    sub[0].set_yticks(np.arange(0, 800, 20))
    sub[0].set_xlim((0,800))    
    sub[0].set_ylim((0,800))
    sub[0].grid(True, linewidth=.2)
    
    # Plot the path travelled by the humans in the environment
    # Iterate through travelled human path
    for comp_hum_path in agent.simulation_env['humanpath'][0]:
        if len(comp_hum_path)>0:
            path = np.array(comp_hum_path).T      #path is 3-D array # use print(path) to see
            sub[0].text(path[0][1][0], path[1][1][0], str(int(path[0][0][0])), fontsize=12)
            sub[0].plot(path[0][1], path[1][1], linewidth = 2)

    # Plot various types/forms of agent path
    sub[0].plot(np.array(agent.path).T[0], np.array(agent.path).T[1], '-r') # path simulated from algorithm
    sub[0].plot(np.array(path1).T[0], np.array(path1).T[1])         # path generated by RRT algorithm < check for real data
    
    # Plot the human as ellipse with their respective position and orientation
    hum_pos = agent.simulation_env['humanpos']
    tmp = np.array(hum_pos)
    posx, posy    = tmp.T[0][1], tmp.T[1][1]
    length, width = tmp.T[0][4], tmp.T[1][4]
    velx, vely    = tmp.T[0][2]/10.0, tmp.T[1][2]/10.0
    for i in range(len(hum_pos)):
        angle = mh.vectorToAngle((velx[i], vely[i])) + 90
        ellipse = Ellipse((posx[i], posy[i]), length[i], width[i], angle=angle)    
        ellipse.set_alpha(0.5)                    #e.set_clip_box(a.bbox)
        ellipse.set_facecolor(np.random.rand(3))
        sub[0].add_artist(ellipse)
        sub[0].arrow(posx[i], posy[i], velx[i], vely[i], head_width=10, head_length=10, fc='k', ec='k')
    
    # Plot the simulating agent 
    agent_pos = agent.simulation_env['agentpos']
    agent_ang = mh.vectorToAngle(agent_pos[2])
    ellipse = Ellipse(agent_pos[1], agent_pos[4][0], agent_pos[4][1], angle=agent_ang) 
    ellipse.set_alpha(0.5)                    #e.set_clip_box(a.bbox)
    ellipse.set_facecolor(np.random.rand(3))
    sub[0].add_artist(ellipse)
    sub[0].arrow(agent_pos[1][0], agent_pos[1][1], agent_pos[2][0]/10, agent_pos[2][1]/10, head_width=10, head_length=10, fc='k', ec='k')

    #if path2 is not None:
    #    sub[0].plot(np.array(path2).T[0], np.array(path2).T[1])     #smoothened (line segments) path obtained from RRT algorithm

    # Plot obstacle scan data i.e. like a lidar scan
    agent_angle     = mh.vectorToAngle(agent.cur_vel)
    [sub[0].plot([pt[0], agent.cur_pos[0]], 
                 [pt[1], agent.cur_pos[1]], 
                 color='k', linestyle='dashed', linewidth=0.3) for pt in agent.obs_map.T[0]]
    
    # Plot them probability map arrays
    sub[1].plot(agent.prob_obs_map)
    sub[2].plot(agent.prob_goal_map)
    sub[2].plot(agent.prob_vel_map)
    sub[3].plot(agent.prob_tot_map)
    sub[4].imshow(cv2.flip(warped, 1))
    
    #plt.pause(0.01)
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