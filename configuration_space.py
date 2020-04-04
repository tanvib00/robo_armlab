#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 27 16:58:56 2020

@author: Andrew
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def calc_space():
  r1 = 3.75 # length of the first arm link
  r2 = 2.5 # length of the second arm link
  r_endeff = 0.2 # radius of the end-effector

  # 15 by 9 array encoding corners: from -7 to 7 on x, and 0 to 8 on y
  obs = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                  [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                  [0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


  #calculate pos for 0 to 180 for t1 and -180 to 180
  config = np.empty([360,180])
  
  for t1 in range(config.shape[1]): # iterate theta 1
    for t2 in range(config.shape[0]): # iterate over theta 2
      t2_real = t2 + t1 - 180
      # calculate x, y of endpoint
      x = r1 * np.cos(np.deg2rad(t1))
      y = r1 * np.sin(np.deg2rad(t1))
      
      x += r2 * np.cos(np.deg2rad(t2_real))
      y += r2 * np.sin(np.deg2rad(t2_real))

      # condition to make out of bounds not an obstacle
      if (y < 0 or y > 8 or x < -7 or x > 7):
        collision = 0
      else:
        collision = obs[int(np.floor(y)), int(np.floor(x)+7)] & \
                    obs[int(np.ceil(y)), int(np.floor(x)+7)] & \
                    obs[int(np.ceil(y)), int(np.ceil(x)+7)] & \
                    obs[int(np.floor(y)), int(np.ceil(x)+7)]

      for t_circ in range(0, 360, 45):
        if (collision == 0):
          x_c = x + r_endeff * np.cos(np.deg2rad(t_circ))
          y_c = y + r_endeff * np.sin(np.deg2rad(t_circ))

          if (y_c >= 0 and y_c <= 8 and x_c >= -7 and x_c <= 7):
            collision = obs[int(np.floor(y_c)), int(np.floor(x_c)+7)] & \
                        obs[int(np.ceil(y_c)), int(np.floor(x_c)+7)] & \
                        obs[int(np.ceil(y_c)), int(np.ceil(x_c)+7)] & \
                        obs[int(np.floor(y_c)), int(np.ceil(x_c)+7)]
                            
      config[t2, t1] = collision

  # flip array on y axis because 0,0 on image is top left           
  config = np.flip(config,0)

  return config


image = calc_space()

#config = calc_space()
#plt.imsave('./1.2_images/config_space.png', config)
#image = mpimg.imread('./1.2_images/config_space.png')

plt.imshow(image, extent=[0,180,-180,180])
plt.xlabel("Theta 1 (deg)")
plt.ylabel("Theta 2 (deg)")
plt.xticks(np.arange(0, 181, 45))
plt.yticks(np.arange(-180, 181, 45))
plt.show()
  













