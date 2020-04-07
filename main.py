import time
from armNew import AcrobotEnv
import sys
import math
import copy
import numpy as np
import pathfind #need to edit pathfind
from pathfind import closeEnough
from astar import astar
from show_path import show_path

r1 = 3.75 # length of the first arm link
r2 = 2.5 # length of the second arm link


def doInverseKinematics(Ax, Ay, Bx, By, Cx, Cy):
  A2 = math.acos((pow(Ax, 2) + pow(Ay, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r2))
  A1 = math.atan2(Ay, Ax) - math.atan2((r2 * math.sin(A2)), (r1 + r2 * math.cos(A2)))
  
  if (A1 > 180 or A1 < 0):
    A2 = -1 * A2
    A1 = math.atan2(Ay, Ax) - math.atan2((r2 * math.sin(A2)), (r1 + r2 * math.cos(A2)))
  
  B2 = math.acos((pow(Bx, 2) + pow(By, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r2))
  B1 = math.atan2(By, Bx) - math.atan2((r2 * math.sin(B2)), (r1 + r2 * math.cos(B2)))
  
  if (B1 > 180 or B1 < 0):
    B2 = -1 * B2
    B1 = math.atan2(By, Bx) - math.atan2((r2 * math.sin(B2)), (r1 + r2 * math.cos(B2)))
  
  C2 = math.acos((pow(Cx, 2) + pow(Cy, 2) - pow(r1, 2) - pow(r2, 2)) / (2 * r1 * r2))
  C1 = math.atan2(Cy, Cx) - math.atan2((r2 * math.sin(C2)), (r1 + r2 * math.cos(C2)))
  
  if (C1 > 180 or C1 < 0):
    C2 = -1 * C2
    C1 = math.atan2(Cy, Cx) - math.atan2((r2 * math.sin(C2)), (r1 + r2 * math.cos(C2)))
  
  return (np.rad2deg(A1), np.rad2deg(A2), np.rad2deg(B1), np.rad2deg(B2), np.rad2deg(C1), np.rad2deg(C2))


#TODO: calculate the torques. can do PID in here maybe
def calcTorques(state, prev_state, prev_time, sums, target):
  
  # link 1 is 0.005 kg
  # link 2 is 0.001 kg
  Kp1 = 0.05
  Kd1 = 0.001#1
  Ki1 = 0.035#5
  
  Kp2 = 0.01 # 0.00001
  Kd2 = 0.000125#1
  Ki2 = 0.005#1
  
  Kg1 = (0.002336* math.cos(state[0]) + (0.00981 * (0.09525 * math.cos(state[0]) + 0.03175 * math.cos(state[0] + state[1]))))
  Kg2 = (0.0003114675 * math.cos(state[0] + state[1]))
  
  dt = time.time() - prev_time
  
  # delta1 = prev_state[0] - state[0]
  # delta2 = prev_state[1] - state[1]
  
  # # Adjust for modular ambiguity    
  # if (delta2 > math.pi):
  #   delta2 = (2 * math.pi) - delta2 
  # elif (delta2 < (-1 * math.pi)):
  #   delta2 = (2 * math.pi) + delta2
    
  # dt1 = 0
  # dt2 = 0
  
  # if (dt != 0):
  #   dt1 = delta1 / dt
  #   dt2 = delta2 / dt
  
  t1err = np.deg2rad(target[0]) - state[0]
  t2err = np.deg2rad(target[1]) - state[1]
  
  # Adjust for modular ambiguity
  if (t2err > math.pi):
    t2err = -1 * ((2 * math.pi) - t2err)
  elif (t2err < (-1 * math.pi)):
    t2err = (2 * math.pi) + t2err
        
  sums[0] = sums[0] + (t1err * dt)
  sums[1] = sums[1] + (t2err * dt)
    
  tor1 = Kp1 * t1err - Kd1 * state[2] + Ki1 * sums[0] + Kg1
  tor2 = Kp2 * t2err - Kd2 * state[3] + Ki2 * sums[1] + Kg2 # - (offset * (t2actual / 90.0))
  
  #print(Kp2 * t2err, -1 * Kd2 * state[3])
  #print(np.deg2rad(Kp2 * t2err), np.deg2rad(Kd2 * dt2))
  return(tor1, tor2, sums)


def clearParents(waypoints):
    for w in waypoints:
        w.parent = None


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
  clearParents(course)
  
  path2 = astar(course, ax, ay, bx, by) # get the path as a list of tuples
  clearParents(course)
  
  path3 = astar(course, bx, by, cx, cy) # get the path as a list of tuples
  
  pth = [path1[0:-1], path2[0:-1], path3]
  path = [i for lst in pth for i in lst]
  numberOfWaypoints = len(path) # Change this based on your path
  
  show_path(path)
  '''
  oldPath = copy.deepcopy(path)
  
  for i in range(numberOfWaypoints - 1):
    midx = (oldPath[i][0] + oldPath[i+1][0]) / 2
    avgy = (oldPath[i][1] + oldPath[i+1][1]) / 2
    if(abs(oldPath[i][1] - oldPath[i+1][1]) > 180):
      if(avgy > 0):
        midy = avgy - 180
      else:
        midy = avgy + 180
    else:
      midy = avgy
      
    path.insert(2 * i + 1, (midx, midy))
    
  numberOfWaypoints = len(path)
    
  print("Path points: ", path) # display path
  '''
  
  robotPos = [0,0]
  
  a = (ax,ay)
  b = (bx,by)
  c = (cx,cy)

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

  prev_state = arm.state
  
  
  for idx in range(1, numberOfWaypoints):

    # Get current waypoint
    target = path[idx]
    print(target)
    sums = [0, 0]
    prev_time = time.time()
    

    while (((target == a or target == b or target == c) and not closeEnough(robotPos, target, 0.5)) or (target != a and target != b and target != c and not closeEnough(robotPos, target))):
      # print(arm.state[0], arm.state[1])
      
        # assuming action1 means action for link1, etc
      actions = calcTorques(arm.state, prev_state, prev_time, sums, target) # N torque # Change this based on your controller
      sums = actions[2]
      prev_time = time.time()
      prev_state = arm.state
      
      
      state, reward, terminal , __ = arm.step(actions[0], actions[1])
      
      arm.render() # Update rendering

      # if target is a, b, or c
      #     hold
      
      robotPos = [np.rad2deg(arm.state[0]), np.rad2deg(arm.state[1])]
    
    print(robotPos)
    if(target == a or target == b or target == c):
      print("WAYPOINT!")
      print(time.time() - start_time)
      time.sleep(1)
      
  print("Done")
  input("Press Enter to close...")
  arm.close()


