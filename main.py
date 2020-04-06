import time
from armNew import AcrobotEnv
import sys
import math
import copy
import numpy as np
import pathfind #need to edit pathfind
from pathfind import closeEnough
from astar import astar

r1 = 3.75 # length of the first arm link
r2 = 2.5 # length of the second arm link


def doInverseKinematics(Ax, Ay, Bx, By, Cx, Cy):
  A2 = math.acos((pow(Ax, 2) + pow(Ay, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r1))
  A1 = math.atan2(Ay, Ax) - math.atan2((r2 * math.sin(A2)), (r1 + r2 * math.cos(A2)))
  
  B2 = math.acos((pow(Bx, 2) + pow(By, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r1))
  B1 = math.atan2(By, Bx) - math.atan2((r2 * math.sin(B2)), (r1 + r2 * math.cos(B2)))
  
  C2 = math.acos((pow(Cx, 2) + pow(Cy, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r1))
  C1 = math.atan2(Cy, Cx) - math.atan2((r2 * math.sin(C2)), (r1 + r2 * math.cos(C2)))
  
  # NOTE: This only returns the solution with a positive theta2 value
  
  return (np.rad2deg(A1), np.rad2deg(A2), np.rad2deg(B1), np.rad2deg(B2), np.rad2deg(C1), np.rad2deg(C2))


#TODO: calculate the torques. can do PID in here maybe
def calcTorques(state, prev_state, prev_time, sums, target):
  offset = 0.000405
  
  # link 1 is 0.005 kg
  # link 2 is 0.001 kg
  Kp1 = 0.00001
  Kd1 = 0.000009
  Ki1 = 0.00001
  
  Kp2 = 0.01 # 0.00001
  Kd2 = 0.00003
  Ki2 = 0.00000#3
  
  Kg1 = 0.1 * (0.002336* math.cos(state[0]) + (0.00981 * (0.09525 * math.cos(state[0]) + 0.03175 * math.cos(state[0] + state[1]))))
  Kg2 = (0.0003114675 * math.cos(state[0] + state[1]))
  
  dt = time.time() - prev_time
  prev_t1actual = prev_state[0] / math.pi * 180.0
  prev_t2actual = prev_state[1] / math.pi * 180.0
  
  t1actual = state[0] / math.pi * 180.0
  t2actual = state[1] / math.pi * 180.0
  
  delta1 = prev_t1actual - t1actual
  delta2 = prev_t2actual - t2actual
  
  # Adjust for modular ambiguity
  if (delta1 > 180):
    delta1 = 360 - delta1
  elif (delta1 < -180):
    delta1 = 360 + delta1
    
  if (delta2 > 180):
    delta2 = 360 - delta2 
  elif (delta2 < -180):
    delta2 = 360 + delta2
    
  if (dt != 0):
    dt1 = delta1 / dt
    dt2 = delta2 / dt
  else:
    dt1 = 0
    dt2 = 0
  
  t1goal = target[0]
  t2goal = 135 #target[1]
  t1err = t1goal - t1actual
  t2err = t2goal - t2actual
  
  # Adjust for modular ambiguity
  if (t1err > 180):
    t1err = 360 - t1err
  elif (t1err < -180):
    t1err = 360 + t1err
    
  if (t2err > 180):
    t2err = 360 - t2err 
  elif (t2err < -180):
    t2err = 360 + t2err  
    
  sums[0] = 0.99 * (sums[0]) + (t1err * dt)
  sums[1] = 0.99 * (sums[1]) + (t2err * dt)
  print(t2actual)
  
  tor1 = Kp1 * t1err + Kd1 * dt1 + Kg1 + offset
  tor2 = Kp2 * t2err + Kd2 * dt2 + Kg2 + Ki2 * sums[1]# - (offset * (0.1 * (t2actual / 90.0)))
  #print(Kp2 * t2err, Kd2 * dt2, Kg2, Ki2 * sums[1])
  #print(np.deg2rad(Kp2 * t2err), np.deg2rad(Kd2 * dt2))
  return(0, np.deg2rad(Kp2 * t2err) + Kg2, sums)


if __name__ == '__main__':

  arm = AcrobotEnv() # set up an instance of the arm class
  
  timeStep = 0.02 # sec
  timeForEachMove = 3 # sec
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
  ax, ay, bx, by, cx, cy = doInverseKinematics(Ax, Ay, Bx, By, Cx, Cy)

  course = pathfind.generateMap() # generate map: list of waypoints # TODO
  path1 = astar(course, 0, 0, ax, ay) # get the path as a list of tuples
  path2 = astar(course, ax, ay, bx, by) # get the path as a list of tuples
  path3 = astar(course, bx, by, cx, cy) # get the path as a list of tuples
  pth = [path1, path2, path3]
  path = [i for lst in pth for i in lst]
  numberOfWaypoints = len(path) # Change this based on your path
  print("Path points: ", path) # display path
  
  robotPos = [0,0]
  
  a = [ax,ay]
  b = [bx,by]
  c = [cx,cy]

  index = 1 # where we are in the path list
  # target_point1 = path[index]
  # on_point = False

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
  
  start_time = time.time()
  prev_time = start_time
  prev_state = arm.state
  sums = [0, 0]
  
  for idx in range(1, numberOfWaypoints):

    # Get current waypoint
    target = path[idx]
    print(target)

    while (not closeEnough(robotPos, target)):
      # print(arm.state[0], arm.state[1])
      
        # assuming action1 means action for link1, etc
      actions = calcTorques(arm.state, prev_state, prev_time, sums, target) # N torque # Change this based on your controller
      sums = actions[2] 
      prev_time = time.time()
      prev_state = arm.state
      
      arm.render() # Update rendering
      state, reward, terminal , __ = arm.step(actions[0], actions[1])
      
      time.sleep(0.02)

      # if target is a, b, or c
      #     hold
      
  print("Done")
  input("Press Enter to close...")
  arm.close()
