
################################################################################################################################
sim_layout = envsim.museumSimulationEnvironment()
agpath     = envsim.getActorTrajectory()
agent      = agentsim.agent(start= np.array(agentsim.getStartPos()), goal=np.array([500, 400]), cur_vel= np.array([5,-5]))

#Planner part which generates a smooth path for the agent
planner = rrt.RRT(start=agent.start, goal=agent.goal, randArea=[0, 800], obsList=sim_layout['rect'], expandDis=10.0)
path    = planner.Planning(animation=False)
path_x  = [x for (x, y) in reversed(path)]
path_y  = [y for (x, y) in reversed(path)]
PathS   = rrt.PathSmoothing(path, 500, sim_layout['rect'])
rep_pts = np.array(PathS, dtype=np.int16)
pts     = np.array(remove_duplicates(rep_pts))
pts     = pts[::-1]  # reverse the order

agent.avg_speed = 1                      # need to convert average distance to pixel space
agent.cur_pos   = agent.start = pts[0]   # where earlier pts was given by the planner
del_t           = 1                      # step movement = del_t * avg_speed
del_vec         = pts[1]-pts[0]
agent.cur_vel   = (del_vec/np.sqrt(sum(del_vec*del_vec.T)))*agent.avg_speed 

cur_frame  = 2
del_frame  = 2
human_path = [[] for i in range(8)]

for i in range(len(pts)-1):
    inter_dist   = agentsim.euclidDist(agent.cur_pos, pts[i+1])
    agent.goal  = pts[i+1]
    while inter_dist > 1.0:
        old_vel        = agent.cur_vel
        new_vel        = agent.updateVelocity(sim_layout)
        agent.cur_pos  = agent.cur_pos + agent.cur_vel*del_t
        inter_dist     = agentsim.euclidDist(agent.cur_pos, pts[i+1])
        agent.path.append(agent.cur_pos)
        probconv.plotSimulation(agent, sim_layout, path, PathS, human_path, agpath, cur_frame)
        print("Position:: ", agent.cur_pos, " :: velocity :: ", agent.cur_vel, " :: angle :: ", agentsim.vectorToAngle(agent.cur_vel))
        print("Start ::", agent.start, "Goal:: ", agent.goal)
        cur_frame = cur_frame + del_frame

################################################################################################################################
def obstacleMap(agent1, obstacleList):
    # the angle devision profile can be non linear as well i.e. more densed at centre and lightly packed on sidewards
    scan_angle_array = angleMap(agent_angle = vectorToAngle(agent1.cur_vel), 
                                num_of_partition = agent1.fov_partition)
    obstacle_map = np.array([scanAngleObstacleDistance(obstacleList, 
                                                       angleToUnitVector(angle), 
                                                       agent1.cur_pos) 
                             for angle in scan_angle_array])

    return obstacle_map
    
#put in agentsimulation.py
def scanAngleObstacleDistance(obstacle_list, scan_angle, agent_position):#intersect_point, distance, rect_intersect_side 
    for fig, num in obstacle_list.items():
        #calculate the maximum possible distance from the agent i.e. the simulation boundary
        pt2, dist,side = agentsim.rayIntersectionRect(simulationRect, agent_position, agent_direction)
        if fig=='rect':
            obs_list1 = []
            for rect in num:
                intersect_pt, dist, side = agentsim.rayIntersectionRect(rect, agent_position, scan_angle)
                if intersect_pt is None: 
                    euclid_distance = dist
                    intersect_pt    = pt2
                else:
                    euclid_distance = agentsim.euclidDist(agent_position, intersect_pt)
                obs_list1.append((intersect_pt, euclid_distance))
                
        if fig=='circle':
            obs_list2 = []
            for circle in num:
                intersect_pt    = agentsim.lineSegmentCircleIntersection(circle, agent_position, pt2)
                if intersect_pt is None: 
                    euclid_distance = dist  # set to max value
                    intersect_pt    = pt2
                else: 
                    euclid_distance = agentsim.euclidDist(agent_position, circle[0]) 
                obs_list2.append((intersect_pt, euclid_distance))
                
        if fig=='line':
            obs_list3 = []
            for pt1, pt2, width in num:
                intersect_pt = agentsim.rayIntersectionLineSegment(rayOrigin = agent_position, 
                                                          rayDirection = scan_angle, 
                                                          point1 = pt1, 
                                                          point2 = pt2)
                if intersect_pt is None:
                    euclid_distance = dist  # set to max value
                    intersect_pt    = pt2
                else:
                    intersect_pt = intersect_pt[0] #returns a list instead of np.array << make some change here# please<<
                    euclid_distance = agentsim.euclidDist(intersect_pt[0], agent_position)
                obs_list3.append((intersect_pt, euclid_distance))
                
    rect_dist_array = np.array(obs_list1).T[1]
    circ_dist_array = np.array(obs_list2).T[1]
    line_dist_array = np.array(obs_list3).T[1]    
    
    rect_min_dist = rect_dist_array[np.argmin(rect_dist_array)]
    circ_min_dist = circ_dist_array[np.argmin(circ_dist_array)]
    line_min_dist = line_dist_array[np.argmin(line_dist_array)]
    
    rect_min_pt = np.array(obs_list1).T[0][np.argmin(rect_dist_array)]
    circ_min_pt = np.array(obs_list2).T[0][np.argmin(circ_dist_array)]
    line_min_pt = np.array(obs_list3).T[0][np.argmin(line_dist_array)]
    
    print("obs list rect:", obs_list1)
    print("obs list circle: ", obs_list2)
    print("obs list line: ", obs_list3)
    print("rect:", rect_dist_array)
    print("circ:", circ_dist_array)
    print("line:", line_dist_array)
    print("rect:", rect_min_dist)
    print("circ:", circ_min_dist)
    print("line:", line_min_dist)
    print("combined dist:", np.array((rect_min_dist, circ_min_dist, line_min_dist)))
    print("combined pt:", np.array((rect_min_pt, circ_min_pt, line_min_pt)))
    
    min_dist_array = np.array((rect_min_dist, circ_min_dist, line_min_dist))
    min_pt_array   = np.array((rect_min_pt, circ_min_pt, line_min_pt))
    min_dist = min_dist_array[np.argmin(min_dist_array)]
    min_pt   = min_pt_array[np.argmin(min_dist_array)]
    return min_pt, min_dist, 0  # send the output as intersection, intersect_dist, intersection_type
    

"""
Function: For each agent, plots obstacle 'SCAN' data in form of line segments, similar to Lidar data
Input: obstacle map, agent angle and agent position
"""
def plotObstacleMap(agent):
    agent_angle     = agentsim.vectorToAngle(agent.cur_vel)
    agent_position  = agent.cur_pos
    [plt.plot([pt[0], agent_position[0]], 
              [pt[1], agent_position[1]], 
              color='k', linestyle='dashed') for pt in agent.obs_map.T[0]] #, marker='o'


#intersection(((0,0),12),(15,0), obstacle_list  = simulationRect
circle = ((80,80),50)
agent_direction= agentsim.angleToUnitVector(90)
agent_position = np.array((80,0.1))

temp = scanAngleObstacleDistance(museumSimulationEnvironment(), agent_direction, agent_position)

###################################################################################################
%matplotlib inline
import pickle
import threading
import numpy as np
import matplotlib.pyplot as plt

def getActorTrajectory():
    #load the stored trajectory
    with open ('agentpath_part1_278frames', 'rb') as fp:
        agentpath = pickle.load(fp)
        
    # load the perspective transformation matrix
    with open ('perspectiveTFmat', 'rb') as fp:
        tfmat = pickle.load(fp)
        
    actor_num = 8
    new_path  = [[] for i in range(actor_num)]
    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    for i in range(actor_num):
        tf_midpoint = []
        new = old   = []
        for a, b, c in agentpath[i]:
            new = a
            if new == old:   #correct for any resets during manual marking
                tf_midpoint.pop()
                new_path[i].pop()
            xy    = (x,y) = ((c[1][0] + c[0][0])/2, (c[1][1] + c[0][1])/2)
            tf_xy = (-1*sum(tfmat[0]*(x,y,1))/sum(tfmat[2]*(x,y,1)) + 800, #flip and shift
                     -1*sum(tfmat[1]*(x,y,1))/sum(tfmat[2]*(x,y,1)) + 800)
            tf_midpoint.append(tf_xy)
            new_path[i].append((a,tf_xy))
            old   = new
        
        new_path[i].reverse()
        xx = np.array(tf_midpoint).T
        plt.text(xx[0][0], xx[1][0], str(i+1), fontsize=18)
        plt.plot(xx[0], xx[1])

    plt.xlim((0, 800))
    plt.ylim((0, 800))
    plt.show()
    return new_path
    


agpath = getActorTrajectory()

cur_frame = 2
del_frame = 2

path = [[] for i in range(8)]
sum1 = sum([len(agpath[i]) for i in range(8)])

while sum1>0:
    plt.figure(num=None, figsize=(10, 10), dpi=80, facecolor='w', edgecolor='k')
    for i in range(8):
        if len(agpath[i])>0 and agpath[i][-1][0]==cur_frame: #and x,y should be <800 so as to be in the frame
            path[i].append(agpath[i].pop()[1])  #store coord frame now
            xx = np.array(path[i]).T
            plt.text(xx[0][0], xx[1][0], str(i+1), fontsize=18)
            plt.plot(xx[0], xx[1])
    cur_frame = cur_frame + del_frame
    sum1 = sum([len(agpath[i]) for i in range(8)])
    plt.xlim((0, 800))
    plt.ylim((0, 800))
    plt.show()
    plt.pause(0.2)
    
################################################################################################################################

# Line segment
#z1 = (0,0)
#z2 = (10, 10)
#r  = (-1, -1)
#d  = norm((1,1))
#print(lineRayIntersectionPoint(r,d,z1,z2))

#p0 = np.array([0,0])
#p1 = np.array([5,5])
#p2 = np.array([10,0])
#p3 = np.array([0,10.5])
#pt = np.array([5.0001,5.0001])
#checkPointOnLine(pt, p0, p1)

#rect = ((x,y), length, height) = ((0, 0), 100, 100)
#person_pos = (105, 95)
#person_vel = 5
#vel_angle  = 0 #degrees i.e. velocity = speed*cos(angle)*i + speed*sin(angle)*j

#rect1 = x,y,z = np.array(((0,0), 100, 100))
#print(rayIntersectionRect(rect1, np.array([180, 180]), np.array([1,.11])))

#print(angularObstacleDistance(obstacleList, np.array([180, 180]), np.array([1,1])))
#print(angleToUnitVector(45))
#print(vectorToAngle((-1,-1)))
#angleMap(agent_angle=45, num_of_partition=10)

#line = lines.Line2D([0.3,1.5],[0.9,0.3], linestyle='dashed',color='k')
#plt.gca().add_line(line)

#----------------------------------------------------------------------------#
#path_xy = np.load('temp.txt.npy') #end (path_xy[0]) --> start (path_xy[end])
#for x in range(len(path_xy), 1, -1):
#    clear_output()
#    plotTriangle(path_xy[x-2:x], 1, 1)

##plt.scatter(X[:, 0], X[:, 1], s = 0, color = Y[:])
#----------------------------------------------------------------------------#
#agent1.plotObstacleMap(agent1, agent1.obs_map.T[0])
#agent2.plotObstacleMap(agent2.obs_map.T[0])
#plt.plot(np.array(agent2.path).T[0], np.array(agent2.path).T[1], '-r')
#probconv.plotSimulation(probconv.getObstacleToProbability(agent1.obs_map.T[1]),
#                        probconv.getGoalToProbability(agent1.cur_vel, agent1.cur_pos, agent1.goal),
#                        probconv.getVelocityToProbability(agent1.cur_vel))

"""
from scipy.interpolate import interp1d

x, y = pts.T
i    = np.arange(len(pts))
interp_i = np.linspace(0, i.max(), 100)#5 * i.max())
xi = interp1d(i, x, kind='linear')(interp_i)
yi = interp1d(i, y, kind='linear')(interp_i)

plt.figure(num=None, figsize=(12, 12), dpi=80, facecolor='w', edgecolor='k')
plt.plot(xi, yi, 'ko')
plt.plot(x, y, 'ko')
plt.show()

#print("path:: ", path)
#print("Path Smooth", PathS)
#[print(xi[i], yi[i]) for i in range(len(xi))]
plt.figure(num=None, figsize=(12, 12), dpi=80, facecolor='w', edgecolor='k')
for startpt, length, width in sim_layout:
    rectangle = plt.Rectangle(startpt, length, width, fc='r')
    plt.gca().add_patch(rectangle)
    
plt.plot(np.array(path).T[0], np.array(path).T[1])
plt.plot(np.array(PathS).T[0], np.array(PathS).T[1])
plt.show()
"""

"""
pt2, dist,side = agentsim.rayIntersectionRect(simulationRect, agent_position, agent_direction)

print("circle:", circle, ": agent position :", agent_position, ": point 2 :", pt2)
inter = LineCollisionCheck((circle[0], circle[1]), agent_position, pt2)
print(inter)

circle = plt.Circle(circle[0], circle[1], fc='y')
plt.gca().add_patch(circle)
line = plt.Line2D((agent_position[0], pt2[0]), (agent_position[1], pt2[1]), lw=0.5)

plt.gca().add_line(line)
#plt.scatter(inter[0][0], inter[1][0], 2.5)
plt.xlim((0, 800))
plt.ylim((0, 800))
plt.show()"""


"""
agent1  = agentsim.agent(start   = np.array(agentsim.getStartPos()), 
                         goal    = np.array([500, 400]), 
                         cur_vel = np.array([5,-5]))

thread1 = threading.Thread(target=probconv.activateAgent, args=[agent1, sim_layout])
thread1.daemon = True
thread1.start()
while True:    
    if agent1.step == True:
        scan_map.append((agent1.cur_pos, agent1.cur_vel, agent1.obs_map))
        # Calculate the converted probability from various map arrays
        agent1.prob_obs_map  = probconv.getObstacleToProbability(agent1.obs_map.T[1])
        agent1.prob_goal_map = probconv.getGoalToProbability(agent1.cur_vel, agent1.cur_pos, agent1.goal)
        agent1.prob_vel_map  = probconv.getVelocityToProbability(agent1.cur_vel)
        agent1.prob_tot_map  = agent1.prob_obs_map*agent1.prob_goal_map*agent1.prob_vel_map
        probconv.plotSimulation(agent1, sim_layout)
        agent1.step = False
"""