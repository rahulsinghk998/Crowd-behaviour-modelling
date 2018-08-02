import math
import numpy as np


"""
Finding point of intersection between line and circle: https://stackoverflow.com/questions/30844482/what-is-most-efficient-way-to-find-the-intersection-of-a-line-and-a-circle-in-py
Circle and line segment intersection: https://stackoverflow.com/questions/22747702/finding-x-and-y-axis-line-intercept-points-of-a-circle-python

Function: Find the intersection between a circle and a line segment
Input: Circle center and two points of line segment
"""
def lineSegmentCircleIntersection(circle, pt1, pt2):
    x1, y1 = pt1[0], pt1[1]
    x2, y2 = pt2[0], pt2[1]
    try:
        a = y2 - y1
        b = -(x2 - x1)
        c = y2 * (x2 - x1) - x2 * (y2 - y1)
    except ZeroDivisionError:
        return None

    (ox, oy), size = circle
    d = abs(a * ox + b * oy + c) / (math.sqrt(a * a + b * b))
    if d <= (size):
        return circle[0]
    return None

    
"""
Function: Find intersection of ray with the workspace rectangle

2------c------3
|             |
|             |
b             d
|             |
|             |
1------a------4

Here: vertex: (1,2,3,4)
      sides : (a,b,c,d)
Inputs: rectangle, ray start point, direction of ray
Return: intersection Point, intersection Distance, intsection Rectangle Side
"""
def rayIntersectionRect(rectangle, rayOrigin, rayDirection):
    #calculating rectangles 4 vertices
    startpt, length_x, width_y = rectangle    
    p1 = (x1, y1) = (startpt[0], startpt[1])
    p2 = (x2, y2) = (startpt[0], startpt[1] + width_y)
    p3 = (x3, y3) = (startpt[0] + length_x, startpt[1] + width_y)
    p4 = (x4, y4) = (startpt[0] + length_x, startpt[1])
    vertexList = np.array([p1, p2, p3, p4], dtype=np.float)
    
    # Find intersection with the workspace boundary
    intersect_point = None
    closest_side    = None
    euclid_distance = None 
    for i in range(len(vertexList)):
        pt = rayIntersectionLineSegment(rayOrigin, rayDirection, vertexList[-1+i], vertexList[i])
        if pt is not None:
            dist = euclidDist(rayOrigin, pt)
            if euclid_distance is None:
                euclid_distance = dist
            if dist <= euclid_distance:
                intersect_point = pt
                euclid_distance = dist
                closest_side    = i+1
    
    return intersect_point, euclid_distance, closest_side


"""
Take care when the line segment and the ray are parallel to each other
Help site: https://gist.github.com/danieljfarrell/faf7c4cafd683db13cbc
           https://rootllama.wordpress.com/2014/06/20/ray-line-segment-intersection-test-in-2d/
"""
def rayIntersectionLineSegment(rayOrigin, rayDirection, point1, point2):
    # Convert to numpy arrays
    rayOrigin    = np.array(rayOrigin, dtype=np.float)
    rayDirection = np.array(norm(rayDirection), dtype=np.float)
    point1       = np.array(point1, dtype=np.float)
    point2       = np.array(point2, dtype=np.float)
    
    # Ray-Line Segment Intersection Test in 2D
    v1 = rayOrigin - point1
    v2 = point2 - point1
    v3 = np.array([-rayDirection[1], rayDirection[0]])
    if np.dot(v2, v3) == 0: 
        return None
    t1 = np.cross(v2, v1) / np.dot(v2, v3)
    t2 = np.dot(v1, v3) / np.dot(v2, v3)
    
    if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
        return rayOrigin + t1 * rayDirection
    return None


"""
Function: Define a range of angles expanding from : angle - 90 to angle + 90: 
delta angle gives a field of view of angle in a discretized form which also sets the resolution of obstacle map
Input: Agent angular orientation (in degrees) and number of partitions of field of view
"""
def angleMap(angle_dir, num_of_partition):
    del_angle  = 180.0/num_of_partition
    angle_map  = np.zeros((6, 1))
    start_angle= angle_dir - 90
    stop_angle = angle_dir + 90 #180 degree relates to the human horizontal field of view
    angle_map  = [del_angle*i + angle_dir - 90 for i in range(num_of_partition + 1)]
    
    angle_map = np.array(angle_map)
    x = angle_map >= 360.0
    angle_map[x] = angle_map[x] - 360.0
    return angle_map


"""
Function: Get normalized form of a vector
Input: n-d vector
"""
def norm(vector):
    return np.array(vector)/magnitude(np.array(vector))


"""
Function: Get magnitude of a n-d vector
Input: n-d vector
"""    
def magnitude(vector):
    return np.sqrt(np.dot(np.array(vector),np.array(vector)))


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