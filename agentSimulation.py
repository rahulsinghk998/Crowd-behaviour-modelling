import math
import numpy as np
import matplotlib.pyplot as plt

import pathPlanRRT as rrt
import probabilityConv as probconv
import mathHelper as mh


"""
Store the agent trajectory, velocity, obstacle map, etccc < at each time stamp
"""
class agent():
    def __init__(self, start, goal, cur_vel = 1, avg_speed = 10, fov_partition = 50):
        # intrinsic parameters which remains constant throughout the journey
        self.start          = start   # [x,y]
        self.goal           = goal    # [x,y]
        self.path           = []
        self.avg_speed      = avg_speed
        self.step           = False   # Turn True if a step computation is completed
        
        # extrinsic parameters which are upadated at every time instant
        self.cur_vel        = cur_vel # x, y - a vector
        self.cur_pos        = start
        self.fov_partition  = fov_partition
        self.simulation_env = None
        self.obs_map        = None
        self.prob_tot_map   = []
        self.prob_obs_map   = []
        self.prob_vel_map   = []
        self.prob_goal_map  = []
        self.behav_coef     = [0, 0, 0]
        self.cur_behav      = 0
        
        #self.intpos        = [][behaviour]
        #self.nextpos       = 0
        #self.cur_behaviour = []
        #self.people_map    = [agent_num - [coord][velocity]]
        
        
    def updateVelocity(self):
        self.obs_map       = self.obstacleMap() 
        self.prob_obs_map  = probconv.getObstacleToProbability(self.obs_map.T[1])
        self.prob_obs_map  = self.prob_obs_map[::-1]                 #fliping the map as data was coming flipped<< need to check
        self.prob_goal_map = probconv.getGoalToProbability(self.cur_vel, self.cur_pos, self.goal)
        self.prob_vel_map  = probconv.getVelocityToProbability(self.cur_vel, sigma = 5)
        
        self.prob_tot_map  = self.prob_obs_map*self.prob_goal_map*self.prob_vel_map  #may require some changes
        
        # CALCULATE THE UPDATED POSITION
        #Also check for the feasible point as well before assigning the new position e.g. agents goes inside obstacles
        #Contraint the cur_position point can be a function as well <<
        max_arg              = 25 - np.argmax(self.prob_tot_map)
        new_vec_angle        = mh.vectorToAngle(self.cur_vel) + max_arg*180.0/50.0
        new_vel_vec          = mh.angleToUnitVector(new_vec_angle)
        self.cur_vel         = new_vel_vec*self.avg_speed
        return self.cur_vel


    """
    Problem:: when the agent lies on the boundary of workspace rectangle
    Function: Iterate through all the angles in the field of view and get the obstacle map
    Input: Obstacle Map and Angle Map
    """
    def obstacleMap(self):
        # the angle devision profile can be non linear as well i.e. more densed at centre and lightly packed on sidewards
        scan_angle_array = mh.angleMap(angle_dir = mh.vectorToAngle(self.cur_vel), 
                                       num_of_partition = self.fov_partition)
        obstacle_map = np.array([self.scanAngleObstacleDistance(angle) for angle in scan_angle_array])

        return obstacle_map


    """
    Concatination of lists: https://stackoverflow.com/questions/4344017/how-can-i-get-the-concatenation-of-two-lists-in-python-without-modifying-either

    Function: Scan in the direction of agent i.e. scan range = headingAngle - 90 to headingAngle + 90
    For each angular search, scan through all rectangles and find closest distance in particular direction
    Get Obstacle distance for a particular angular scan orientation
    Input: Obstacle List, Angular Scan Orientation and position of agent
    """
    def scanAngleObstacleDistance(self, angle):
        angle_vector = mh.angleToUnitVector(angle)
        obs_list1    = obs_list2 = obs_list3 = []
        #intersection with rectangle boundary i.e. outline which acts as reference for whole simulation
        ref_pt, ref_dist, ref_side = mh.rayIntersectionRect(self.simulation_env['outline'], self.cur_pos, angle_vector)  

        for fig, num in self.simulation_env.items():
            if fig=='rect':
                for rect in num:
                    pt, dist, side = mh.rayIntersectionRect(rect, self.cur_pos, angle_vector)
                    if pt is None: 
                        pt, dist = ref_pt, ref_dist
                    obs_list1.append((pt, dist, 1))

            if fig=='circle':
                for circle in num:
                    pt    = mh.lineSegmentCircleIntersection(circle, self.cur_pos, ref_pt)
                    if pt is None: 
                        pt, dist = ref_pt, ref_dist  # set to max value
                    else: 
                        dist = mh.euclidDist(self.cur_pos, circle[0]) 
                    obs_list2.append((pt, dist, 2))

            if fig=='line':
                for pt1, pt2, width in num:
                    pt = mh.rayIntersectionLineSegment(rayOrigin = self.cur_pos, rayDirection = angle_vector, 
                                                       point1 = pt1, point2 = pt2) 
                    if pt is None:
                        pt, dist = ref_pt, ref_dist  # set to max value
                    else: 
                        dist = mh.euclidDist(pt, self.cur_pos)
                    obs_list3.append((pt, dist, 3))

            if fig=='human':
                for human in num:
                    x = 0 #Do something with here

        concat_obs_list  = obs_list1 + obs_list2 + obs_list3
        concat_obs_array = np.array(concat_obs_list).T
        dist_argmin      = np.argmin(concat_obs_array[1])
        min_dist         = concat_obs_array[1][dist_argmin]
        min_point        = concat_obs_array[0][dist_argmin]
        min_type         = concat_obs_array[2][dist_argmin]

        return min_point, min_dist, min_type   # send the output as intersection point, intersect_dist, "intersection_type"