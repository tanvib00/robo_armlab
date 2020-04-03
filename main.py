import time
from arm import AcrobotEnv
import sys
import math
import copy
import numpy as np
import pathfind #need to edit pathfind

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
    # Insert your code or calls to functions here
    numberOfWaypoints = 10 # Change this based on your path
    
    arm.reset() # start simulation
    
    for waypoint in range(numberOfWaypoints):

        # Get current waypoint

        for timeStep in range(stepsForEachMove):

            tic = time.perf_counter()

            # Control arm to reach this waypoint

            actionHere1 = 0 # N torque # Change this based on your controller
            actionHere2 = 0 # N torque # Change this based on your controller
            
            arm.render() # Update rendering
            state, reward, terminal , __ = arm.step(actionHere1, actionHere2)
        
    print("Done")
    input("Press Enter to close...")
    arm.close()
