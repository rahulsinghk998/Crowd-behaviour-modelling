import math
import numpy as np
import matplotlib.pyplot as plt

"""
An agent class

Intrinsic parameter - behaviour mode, age, avgerage velocity
Extrinsic parameter - start position, end goal, obstacle map, angular orientation, speed
"""


simulationRect = simOrigin, simWidth, simHeight = ((0, 0), 800, 800)

class agent():
    def __init__(self, start, goal, cur_vel, avg_speed = 10, fov_partition = 50):
        # intrinsic parameters which remains constant throughout the journey--<< need to look again about these
        self.start          = start  #[x,y]
        self.end            = goal   #[x,y]
        self.avg_speed      = avg_speed
        # extrinsic parameters which are upadated at every time instant
        self.obstacle_map   = None
        self.cur_vel        = cur_vel #x,y - a vector
        self.cur_pos        = start
        self.fov_partition  = fov_partition
        
        
    def updateVelocity(self, obstacleList):
        # calculate obstacle map
        agent_angle       = vectorToAngle(self.cur_vel)
        self.obstacle_map = obstacleMap(obstacleList, 
                                        agent_angle, 
                                        agent_position = self.cur_pos, 
                                        num_of_partition = self.fov_partition)
        #######################################
        # Do some probability based calcualtion 
        # based on current values and parameters
        #######################################
        
        # Update the position
        # self.cur_vel = new_velocity
        # self.cur_pos = new_position = self.cur_pos + new_velocity*delta_time
        return 0 #new_velocity
    
    
    """
    Problem: change the could of scan plot- currently its black
    
    Line segment plot: https://stackoverflow.com/questions/36470343/how-to-draw-a-line-with-matplotlib/36479941
    Function: For each agent, plots obstacle 'SCAN' data in form of line segments, similar to Lidar data
    Input: obstacle map, agent angle and agent position
    """
    def plotObstacleMap(self, obstacle_mapXY):
        agent_angle     = vectorToAngle(self.cur_vel)
        agent_position  = self.cur_pos
        [plt.plot([pt[0], agent_position[0]], 
                  [pt[1], agent_position[1]], 
                  color='k', linestyle='dashed') for pt in obstacle_mapXY] #, marker='o'
        
        #plot the agent direction line as well -- i.e. the centre line
        return 0
    

    """
    Problem:: when the agent lies on the boundary of workspace rectangle
    
    Function: Iterate through all the angles in the field of view and get the obstacle map
    Input: Obstacle Map and Angle Map
    """
    def obstacleMap(self, obstacleList):
        # the angle devision profile can be non linear as well i.e more densed at centre and lightly packed on sidewards
        scan_angle_array = angleMap(agent_angle = vectorToAngle(self.cur_vel), 
                                    num_of_partition = self.fov_partition)
        obstacle_map = np.array([scanAngleObstacleDistance(obstacleList, 
                                                           angleToUnitVector(angle), 
                                                           self.cur_pos) 
                                 for angle in scan_angle_array])

        return obstacle_map
    
    
    def calculateProbability(self, obstacleMap):
        
        return 0


"""
Calculate Euclidean Distance
Input: points - p0, p1
"""
def euclidDist(p0, p1):
    dist = math.sqrt((p0[0]-p1[0])*(p0[0]-p1[0]) + (p0[1]-p1[1])*(p0[1]-p1[1]))
    return dist


"""
Check if point lies on the line segment or Not
Inputs: Point - pt & Points - p0, p1
"""
def checkPointOnLine(pt, p0, p1):
    tolerence = 0.001
    del_dist = euclidDist(p0, pt) + euclidDist(pt, p1) - euclidDist(p0, p1)
    if del_dist > tolerence:
        return False
    else:
        return True

"""
Function: Get magnitude of a n-d vector
Input: n-d vector
"""    
def magnitude(vector):
    return np.sqrt(np.dot(np.array(vector),np.array(vector)))


"""
Function: Get normalized form of a vector
Input: n-d vector
"""
def norm(vector):
    return np.array(vector)/magnitude(np.array(vector))


"""
Function: Convert angle to unit vector in 2D space
Input: Angle (in degrees)
"""
def angleToUnitVector(angle):
    rad = angle*np.pi/180.0
    return np.array([math.cos(rad), math.sin(rad)])


"""
Function: Convert a vector to angle in 2D space
Input: Unit Vector
"""
def vectorToAngle(vector):
    x, y = vector
    rad  = math.atan2(y, x)
    angle= rad*180.0/np.pi
    return angle


"""
Take care when the line segment and the ray are parallel to each other
Help site: https://gist.github.com/danieljfarrell/faf7c4cafd683db13cbc
           https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
"""
def rayIntersectionLineSegment(rayOrigin, rayDirection, point1, point2):
    # Convert to numpy arrays
    rayOrigin = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(norm(rayDirection), dtype=np.float)
    point1 = np.array(point1, dtype=np.float)
    point2 = np.array(point2, dtype=np.float)
    # Ray-Line Segment Intersection Test in 2D
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return [rayOrigin + t1 * rayDirection]
    return None


"""
>>><<<Fix the closest side part of this function
>>>>>>THIS FUNCTION HAS LOTS OF ISSUES :: SOLVE AND DEBUG ONCE AGAIN

Function: Find intersection of ray with the workspace rectangle
Inputs: rectangle, ray start point, direction of ray
"""
def rayIntersectionRect(rectangle, rayOrigin, rayDirection):
    #Check if the rayOrigin is within confined space
    if not all(rayOrigin<simWidth) or not all(rayOrigin>0):
        return None, None, None
    
    #calculating rectangles 4 vertices
    startpt, length_x, width_y = rectangle    
    p1 = (x1, y1) = (startpt[0], startpt[1])
    p2 = (x2, y2) = (startpt[0], startpt[1] + width_y)
    p3 = (x3, y3) = (startpt[0] + length_x, startpt[1] + width_y)
    p4 = (x4, y4) = (startpt[0] + length_x, startpt[1])
    
    # Find intersection with the workspace boundary
    vertexList = np.array([p1, p2, p3, p4], dtype=np.float)
    intersectPoint = [rayIntersectionLineSegment(rayOrigin, rayDirection, vertexList[-1+i], vertexList[i]) 
                      for i in range(len(vertexList))]
    
    # If there is not intersection with rectangle then return the intersection with boundary of rectangular workspace
    intersectPoint = np.array(intersectPoint)
    if intersectPoint.any() == None:
        pt, dist, side = rayIntersectionRect(simulationRect, rayOrigin, rayDirection)
        return pt, dist, None
    
    # Simulated supermarket is rectangle with dimension 800X800 (length, height)
    # Sort out the intersection point: choose the closest one and store both distance and vertex information
    leastDist = 800*1.414 #maximum distance i.e square diagonal for x in xyz if x not in a
    closestSide = None
    for i in range(len(vertexList)):#####>>>COULD BE MORE SIMPLIFIED BY USING FOR LOOP AND IF TOGETHER<<<#####
        if intersectPoint[i] is not None:
            distPt = euclidDist(rayOrigin, intersectPoint[i][0])
            if distPt < leastDist:
                leastDist = distPt
                closestPt = intersectPoint[i]
                closestSide = i

    return closestPt[0], leastDist, closestSide  #(intersectionPoint, intersectionDist, intsectionRectSide)


"""
Function: Define a range of angles expanding from : angle - 90 to angle + 90: 
delta angle gives a field of view of angle in a discretized form which also sets the resolution of obstacle map
Input: Agent angular orientation (in degrees) and number of partitions of field of view
"""
def angleMap(agent_angle, num_of_partition):
    del_angle  = 180.0/num_of_partition
    angle_map  = np.zeros((6, 1))
    start_angle= agent_angle
    stop_angle = agent_angle + 180 #180 degree relates to the human horizontal field of view
    angle_map  = [del_angle*i + agent_angle for i in range(num_of_partition + 1)]
    
    angle_map = np.array(angle_map)
    x = angle_map >= 360.0
    angle_map[x] = angle_map[x] - 360.0
    return angle_map


"""
Function: Scan in the direction of agent i.e. scan range = headingAngle - 90 to headingAngle + 90
For each angular search, scan through all rectangles and find closest distance in particular direction
Get Obstacle distance for a particular angular scan orientation
Input: Obstacle List and Angular Scan Orientation
"""
def scanAngleObstacleDistance(obstacle_list, scan_angle, agent_position):#intersect_point, distance, rect_intersect_side 
    obs_list = [rayIntersectionRect(rect, agent_position, scan_angle) for rect in obstacle_list]
    obs_list = np.array(obs_list).T
    
    #search the closest rectangle to the 'agent' i.e. a point in 2D space
    nearest_rect_index  = np.argmin(obs_list[1])
    intersect_point     = obs_list[0][nearest_rect_index]
    intersect_distance  = obs_list[1][nearest_rect_index]
    intersect_side      = obs_list[2][nearest_rect_index]
    return intersect_point, intersect_distance, intersect_side


def getStartPos(fromDoor = True):
    pos = [int(np.random.normal(650, 50, 1)), 1] # np.random.normal(centre_of_door, sigma = doorlen/6, 1), door_yaxis
    return pos


def getIntermedPos(curPos, targetPos, step):
    x = np.linspace(curPos[0], targetPos[0], 200)
    y = np.linspace(curPos[1], targetPos[1], 200)
    intPos = [int(x[step]), int(y[step])]
    return intPos


def main():
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
    ]
    num_of_partition = 50
    start  = np.array(getStartPos()) 
    goal   = np.array([500, 400])
    cur_vel= np.array([5,-5])

    agent1 = agent(start, goal, cur_vel)
    obsMap = agent1.obstacleMap(obstacleList)
    agent1.plotObstacleMap(obsMap.T[0])
    plt.show()


if __name__ == '__main__':
    main()