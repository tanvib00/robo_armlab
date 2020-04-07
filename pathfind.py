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


    p0.neighbors = [p1,p23,p24]
    p1.neighbors = [p23,p0,p5]
    p2.neighbors = [p26,p3,p4]
    p3.neighbors = [p2,p4,p6,p7]
    p4.neighbors = [p26,p2,p3,p7,p10]
    
    p5.neighbors = [p1,p6,p9]
    p6.neighbors = [p3,p5,p8]
    p7.neighbors = [p3,p4,p10]
    p8.neighbors = [p6,p11]
    p9.neighbors = [p5,p12,p13]

    p10.neighbors = [p4,p7,p16]
    p11.neighbors = [p8,p15]
    p12.neighbors = [p9,p13,p14]
    p13.neighbors = [p9,p12,p14,p17,p22]
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


    return [p0,p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13,p14,p15,p16,p17, \
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


def calcPowers(posx, posy, goalx, goaly, theta):
    theta = theta * 180.0 / pi # convert to degrees
    desired_theta = theta

    # find distance units
    xdif = float(goalx - posx) # if -ve, go right. else go left
    ydif = float(goaly - posy) # if +ve, go up. else go down
    
    if xdif != 0: # take care of divide by zero error
        desired_theta = (atan(ydif/xdif)) * 180.0 / pi
    elif ydif > 0:
        desired_theta = 90.0
    else:
        desired_theta = - 90.0

    if (xdif < 0):
        desired_theta += 180.0
    
    desired_theta = desired_theta % 360.0
    theta = theta % 360.0

    d_theta = desired_theta - theta # if positive turn cw

    if (d_theta > 180):
        d_theta = d_theta - 360.0
    elif (d_theta < - 180):
        d_theta = 360.0 + d_theta

    if (abs(d_theta) > 6):
        leftPow = d_theta / abs(d_theta) * 25.0
        rightPow = leftPow * (-1.0) 
    else:
        Kp = 6.0 # proportional controller
        leftPow = 45.0 + Kp*d_theta
        rightPow = 45.0 - Kp*d_theta 
    
    return (leftPow, rightPow)


''' 
MAIN
'''

def main():
    p = input("Please enter the start and end position coordinates as: x1 y1 x2 y2\n")
    ps = p.split(" ")
    positions = [float(c) for c in ps] # assuming inputs are integers not floats
    xstart, ystart, xend, yend = positions[0], positions[1], positions[2], positions[3]

    endPos = tuple((xend, yend))

    course = generateMap() # generate map: list of waypoints
    path = astar(course, xstart, ystart, xend, yend) # get the path as a list of tuples
    
    robotPos = [xstart, ystart] # actual, encoder determined position of robot
    theta = pi / 2.0 # initial theta in rad
    index = 1 # where we are in the path list

    target_point = path[index]
    on_endpoint = False

    try:
        while (euclideanDistance(robotPos, path[index]) < 4.0 and not on_endpoint):
            path.pop(index)
            target_point = path[index]
            if index >= len(path):
                index = - 1
                target_point = list(path[-1])
                on_endpoint = True

        if (euclideanDistance(path[-1], path[-2]) < 4.0):
            path[-2] = path[-1]
            path.pop()
        
        print("Path points: ", path) # display path

        while (not closeEnough(tuple(robotPos), endPos)):

            start_time = time.time()

            if (closeEnough(tuple(robotPos), tuple(target_point))):
                index += 1
                target_point = path[index]
            
            leftPow, rightPow = calcPowers(robotPos[0], robotPos[1], target_point[0], target_point[1], theta)
            
            loop_time = time.time() - start_time
            robotPos[0], robotPos[1], theta = odometry((robotPos[0],robotPos[1],theta),BP, loop_time)
            
            BP.set_motor_power(BP.PORT_C, leftPow)
            BP.set_motor_power(BP.PORT_B, rightPow)

            

        BP.set_motor_power(BP.PORT_C, 0)
        BP.set_motor_power(BP.PORT_B, 0)
    
    except KeyboardInterrupt:
        BP.reset_all()

