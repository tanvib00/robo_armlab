import time
from arm import AcrobotEnv
import sys
import math
import copy
import numpy as np
import pathfind #need to edit pathfind


#TODO: inverse kinematics
def doInverseKinematics(Ax, Ay, Bx, By, Cx, Cy):
    return (Ax, Ay, Bx, By, Cx, Cy)


#TODO: calculate the torques. can do PID in here mayb
def calcTorques(roboState, target)
    t1actual = roboState[0]
    t2actual = roboState[1]
    t1goal = target[0]
    t2goal = target[1]
    t1err = t1goal - t1actual
    t2err = t2goal - t2actual
    
    tor1 = t1err * 5
    tor2 = t2err * 5 # arbitrary number rn
    return(tor1, tor2)


if __name__ == '__main__':

    arm = AcrobotEnv() # set up an instance of the arm class
    
    timeStep = 0.02 # sec
    timeForEachMove = 1 # sec
    stepsForEachMove = round(timeForEachMove/timeStep)

    # Make configuration space
    # Insert you code or calls to functions here

    # Get three waypoints from the user
    Ax = int(input("Type Ax: "))
    Ay = int(input("Type Ay: "))
    Bx = int(input("Type Bx: "))
    By = int(input("Type By: "))
    Cx = int(input("Type Cx: "))
    Cy = int(input("Type Cy: "))

    arm.Ax = Ax*0.0254; # Simulaiton is in SI units
    arm.Ay = Ay*0.0254; # Simulaiton is in SI units
    arm.Bx = Bx*0.0254; # Simulaiton is in SI units
    arm.By = By*0.0254; # Simulaiton is in SI units
    arm.Cx = Cx*0.0254; # Simulaiton is in SI units
    arm.Cy = Cy*0.0254; # Simulaiton is in SI units

    # Plan a path

    course = pathfind.generateMap() # generate map: list of waypoints
    path1 = astar(course, 0, 0, Ax, Ay) # get the path as a list of tuples
    path2 = astar(course, Ax, Ay, Bx, By) # get the path as a list of tuples
    path3 = astar(course, Bx, By, Cx, Cy) # get the path as a list of tuples
    pth = [path1, path2, path3]
    path = [i for lst in pth for i in lst]
    numberOfWaypoints = len(path) # Change this based on your path
    print("Path points: ", path) # display path
    
    robotPos = [0,0]
    ax, ay, bx, by, cx, cy = doInverseKinematics(Ax, Ay, Bx, By, Cx, Cy)
    a = [ax,ay]
    b = [bx,by]
    c = [cx,cy]

    index = 1 # where we are in the path list
    target_point1 = path[index]
    on_point = False

    '''
    while (euclideanDistance(robotPos, path[index]) < 4.0 and not on_point):
    	path.pop(index)
    	target_point = path[index]
    	if index >= len(path):
	    index = - 1
	    target_point = list(path[-1])
            on_point = True

    # this got rid of last waypoint on path if it was close enough to the endpoint
    if (euclideanDistance(path[-1], path[-2]) < 4.0):
    	path[-2] = path[-1]
    	path.pop()

    while (not closeEnough(tuple(robotPos), endPos)):

    	start_time = time.time()

    	if (closeEnough(tuple(robotPos), tuple(target_point))):
            index += 1
	    target_point = path[index]
    
    '''
    
    arm.reset() # start simulation
    
    for idx in range(numberOfWaypoints):

        # Get current waypoint
        target = path[idx]

        for timeStep in range(stepsForEachMove):

            tic = time.perf_counter()

            # Control arm to reach this waypoint

	    # assuming actionHere1 means action for link1, etc
            actionHere1 = calcTorques(arm.state, target)[0] # N torque # Change this based on your controller
            actionHere2 = calcTorques(arm.state, target)[1] # N torque # Change this based on your controller
            
            arm.render() # Update rendering
            state, reward, terminal , __ = arm.step(actionHere1, actionHere2)
        
    print("Done")
    input("Press Enter to close...")
    arm.close()
