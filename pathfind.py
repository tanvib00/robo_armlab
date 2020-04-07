# import matplotlib
# import matplotlib.pyplot as plt
from waypoint import Waypoint
from math import sqrt, atan, pi
import time
from astar import astar
#from odometry_func import odometry


''' 
HELPER FUNCTIONS
'''


''' Generate the waypoint map '''
def generateMap():
    
    # Waypoints are (theta1, theta2)

    p00 = Waypoint(0, 0)
    p0 = Waypoint(47, 175)
    p1 = Waypoint(7, 151)
    p2 = Waypoint(147, 164)
    p3 = Waypoint(105, 123)
    p4 = Waypoint(166, 116)

    p5 = Waypoint(12, 89)
    p6 = Waypoint(58, 101)
    p7 = Waypoint(136, 76)
    p8 = Waypoint(77, 50)
    p9 = Waypoint(24, 29)

    p10 = Waypoint(155, 16)
    p11 = Waypoint(96, -1)
    p12 = Waypoint(51, -18)
    p13 = Waypoint(16, -38)
    p14 = Waypoint(62, -61)

    p15 = Waypoint(114, -45)
    p16 = Waypoint(172, -52)
    p17 = Waypoint(13, -106)
    p18 = Waypoint(87, -98)
    p19 = Waypoint(127, -82)

    p20 = Waypoint(130, -105)
    p21 = Waypoint(168, -109)
    p22 = Waypoint(62, -127)
    p23 = Waypoint(26, -171)
    p24 = Waypoint(78, -161)
    
    p25 = Waypoint(175, -150)
    p26 = Waypoint(175, -175)


    p00.neighbors = [p9,p13]
    p0.neighbors = [p1,p23,p24]
    p1.neighbors = [p23,p0,p5]
    p2.neighbors = [p26,p3,p4]
    p3.neighbors = [p2,p4,p6,p7]
    p4.neighbors = [p26,p2,p3,p7,p10]
    
    p5.neighbors = [p1,p6,p9]
    p6.neighbors = [p3,p5,p8]
    p7.neighbors = [p3,p4,p10]
    p8.neighbors = [p6,p11]
    p9.neighbors = [p00,p5,p12,p13]

    p10.neighbors = [p4,p7,p16]
    p11.neighbors = [p8,p15]
    p12.neighbors = [p9,p13,p14]
    p13.neighbors = [p00,p9,p12,p14,p17,p22]
    p14.neighbors = [p12,p13,p17,p18,p22]

    p15.neighbors = [p11,p19]
    p16.neighbors = [p10,p21]
    p17.neighbors = [p13,p14,p22,p23]
    p18.neighbors = [p14,p17,p19,p20,p22,p24]
    p19.neighbors = [p15,p18,p20]

    p20.neighbors = [p18,p19,p21]
    p21.neighbors = [p16,p20,p25]
    p22.neighbors = [p13,p14,p17,p18,p23,p24]
    p23.neighbors = [p17,p22,p24,p0,p1]
    p24.neighbors = [p18,p22,p23,p0]
    
    p25.neighbors = [p21,p26]
    p26.neighbors = [p25,p2,p4]


    return [p00,p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17, \
            p18,p19,p20,p21,p22,p23,p24,p25,p26]


''' Define when the robot can stop moving '''
def closeEnough(curr, end, radius=2.5):
    if (euclideanDistance(curr, end) < radius):
        return True
    else:
        return False
    

''' Euclidean Distance between two tuples/arrays '''
def euclideanDistance(a1,a2):
    x1 = a1[0]
    x2 = a2[0]
    y1 = a1[1]
    y2 = a2[1]
    xdist = x2 - x1
    ydist = abs(y1 - y2)
    if (ydist > 180):
        ydist = 360 - ydist
    return sqrt(pow(xdist,2) + pow(ydist,2))