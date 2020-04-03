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
    
    p1 = Waypoint(9,9)
    p2 = Waypoint(18,9)
    p3 = Waypoint(27,9)
    p4 = Waypoint(36,9)
    p5 = Waypoint(45,9)
    p6 = Waypoint(54,9)
    p7 = Waypoint(63,9)
    
    p8 = Waypoint(9,17)
    p9 = Waypoint(15,20)
    p10 = Waypoint(19,27)
    p11 = Waypoint(38,20)
    p12 = Waypoint(32,27)

    p13 = Waypoint(26.5,41)
    p13a = Waypoint(26.5,33)
    p14 = Waypoint(43,41)
    p15 = Waypoint(54,26)
    p16 = Waypoint(55,17)
    p17 = Waypoint(66,24)
    p18 = Waypoint(66,40)

    pa = Waypoint(9,48)
    pb = Waypoint(18,48)
    pc = Waypoint(27,48)
    pd = Waypoint(37,48)
    pe = Waypoint(45,48)
    pf = Waypoint(54,48)
    pg = Waypoint(63,48)
    pz = Waypoint(8,41)

    
    p1.neighbors = [p2,p8,p9]
    p2.neighbors = [p1,p9, p3]
    p3.neighbors = [p2,p4,p9,p11]
    p4.neighbors = [p3,p5,p11,p16]
    p5.neighbors = [p4,p6,p11,p16]
    p6.neighbors = [p5,p16,p7,p17]
    p7.neighbors = [p6,p16,p17]
    
    p8.neighbors = [p1,p9]
    p9.neighbors = [p8,p1,p2,p3,p10]
    p10.neighbors = [p9,p13a]
    p11.neighbors = [p3,p4,p5,p16,p12]
    p12.neighbors = [p11,p13a]
    p13a.neighbors = [p10,p12,p13]
    p13.neighbors = [p13a,pc]
    p14.neighbors = [p15,pd,pe]
    p15.neighbors = [p16,p14]
    p16.neighbors = [p11,p5,p4,p6,p7,p17]
    p17.neighbors = [p6,p7,p16,p18]
    p18.neighbors = [p17,pg]
    
    pa.neighbors = [pz,pb]
    pb.neighbors = [pa,pc]
    pc.neighbors = [pb,pd,p13]
    pd.neighbors = [pc,p14,pe]
    pe.neighbors = [pd,p14,pf]
    pf.neighbors = [pe,pg]
    pg.neighbors = [pf,p18]
    pz.neighbors = [pa]


    return [p1,p2,p3,p4,p5,p6,p7,p8,p9,p10,p11,p12,p13a,p13,p14,p15,p16,p17,p18,pa,pb,pc,pd,pe,pf,pg,pz]


''' Define when the robot can stop moving '''
def closeEnough(curr, end, radius=0.05):
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

