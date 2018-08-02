import math
import numpy as np
import matplotlib.pyplot as plt

import pathPlanRRT as rrtplanner
import agentSimulation as agentsim
import mathHelper as mh


"""
Function: Create a gaussian kernel with side length l and a sigma of sig
Input: Length of kernel, standard deviation (sigma) and centre of gaussian kernel
"""
def gaussianKernel(length=51, sigma = 10., centre = 0):
    ax = np.arange(-length // 2 + 1., length // 2 + 1.)
    xx, = np.meshgrid(ax)
    kernel = np.exp(-((xx - centre)**2) / (2. * sigma**2))
    return kernel / np.sum(kernel)  # ensures that sum of all probability values is 1 


"""
Function: Probability map will give the direction
"""
def getObstacleToProbability(dist_map): #obsMap.T[1]
    dist_tot_sum = sum((dist_map))
    dist_prob_map = dist_map/dist_tot_sum
    return dist_prob_map


"""
Wiki Gaussian Kernel: https://en.wikipedia.org/wiki/Scale_space_implementation#The_discrete_Gaussian_kernel
Stackoverflow Code:   https://stackoverflow.com/questions/29731726/how-to-calculate-a-gaussian-kernel-matrix-efficiently-in-numpy

Function: Consider velocity probability in terms of gaussian function, 
similar to velocity step function convolving with gaussian kernel
Param: length of gaussian kernel and agent velocity
"""
def getVelocityToProbability(agent_vel, length = 51, sigma = 10.):
    # a discrete gaussian with mean in direction velocity
    vel_prob_map = gaussianKernel(length = length, sigma = sigma ) # sig = function of agent_vel
    return vel_prob_map


"""
Function: Get a probability distribution along the direction of goal
Param:  Current velocity (to get current orientation), current position, goal position
"""
def getGoalToProbability(cur_vel, cur_pos, goal_pos, length = 51, sigma = 10.):
    cur_dir   = mh.vectorToAngle(cur_vel)
    goal_vect = np.array([goal_pos[0]-cur_pos[0], goal_pos[1]-cur_pos[1]])
    goal_dir  = mh.vectorToAngle(goal_vect)
    
    #Find angular division which corresponds to the velocity
    del_dir   = cur_dir - goal_dir
    delta     = 180.0/50.0
    scan_num  = del_dir/delta 
    
    #print("Goal Angle Direction: ", goal_dir, "\nCurrent Velocity Direction: ", cur_dir)
    #print("Scan Position Number: ", scan_num)
    
    goal_prob_map = gaussianKernel(length = length, sigma = sigma, centre=scan_num)
    return goal_prob_map


"""
Function: To assign/give a behaivour to the agent. Behaviours are namely
1. Leader-follower
2. Goal-oriented
3. Companion/group based
4. Social comparison theory based - is a bit more higher level - i guess as it requires context information
## intrinsic and extrinsic parameters
"""
def getBehaviourProbability(behav_type, agent, obs_prob, goal_prob, vel_prob):
    if behav_type == 1:   #goal-oriented
        tot_prob = agent.behav_coef[0]*obs_prob + 2*agent.behav_coef[1]*goal_prob + agent.behav_coef[2]*vel_prob
    
    elif behav_type == 2: #leader-follower
        #check if some is walking in from  at a certain distance
        tot_prob = agent.behav_coef[0]*obs_prob + agent.behav_coef[1]*goal_prob + 2*agent.behav_coef[2]*vel_prob
    
    elif behav_type == 3: #companion/group
        tot_prob = agent.behav_coef[0]*obs_prob + agent.behav_coef[1]*goal_prob + agent.behav_coef[2]*vel_prob
    
    elif behav_type == 4: #social comparison
        tot_prob = agent.behav_coef[0]*obs_prob + agent.behav_coef[1]*goal_prob + agent.behav_coef[2]*vel_prob
    
    return tot_prob