#! /usr/bin/env python3

import math

def myAtan2(x, y):
    return math.atan2(y, x)

# TODO
def iKine(point):
    '''Solve for the inverse kinematics of the RPP arm.
    Return a list of 3 elements containing [theta, vertical_distance, horizontal_distance].
    Please use myAtan2 defined above instead of math.atan2
    '''
    #return [0] * 3
    #for point in points:
    x_dist = point[0]
    y_dist = point[1]
    z_dist = point[2]
    
    theta  = myAtan2(x_dist,y_dist)

    vert_dist = z_dist + 0.535
    hor_dist = 1.5 - math.sqrt(x_dist**2 + y_dist**2)
    res = [theta , vert_dist , hor_dist]
    return res
    

