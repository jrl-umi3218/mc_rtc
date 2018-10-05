#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import os
import sys
import tempfile

import eigen
import rbdyn
import mc_rbdyn

def load_log(fpath):
  data = {}
  if fpath.endswith('.bin'):
      tmpf = tempfile.mkstemp(suffix = '.log')[1]
      os.system("mc_bin_to_log {} {}".format(fpath, tmpf))
      return load_log(tmpf)
  else:
    with open(fpath) as fd:
      reader = csv.DictReader(fd, delimiter=';')
      for row in reader:
        for k in reader.fieldnames:
          try:
            value = float(row[k])
          except ValueError:
            value = None
          if not k in data:
            data[k] = []
          data[k].append(value)
    for k in data:
      data[k] = np.array(data[k])
  return data

def set_robot_config(robot, rjo, data, t):
  q = eigen.Quaterniond(data['ff_qw'][t], data['ff_qx'][t], data['ff_qy'][t], data['ff_qz'][t]).normalized().inverse()
  robot.mbc.q[0] = [
                    q.w(), q.x(), q.y(), q.z(),
                    data['ff_tx'][t], data['ff_ty'][t], data['ff_tz'][t]
                   ]
  for i,j in enumerate(rjo):
    jIndex = robot.jointIndexByName(j)
    robot.mbc.q[jIndex] = [data['qOut_{}'.format(i)][t]]
  robot.forwardKinematics()
  robot.forwardVelocity()
  return rbdyn.computeCoM(robot.mb, robot.mbc)

def update_figure(num, start_t, datas, plots):
  for d, p in zip(datas, plots):
    p.set_data(d[start_t + int(math.ceil(200/60.*num))])
  return plots

if __name__ == "__main__":
  assert(len(sys.argv) > 1)

  fig1 = plt.figure()

  triangle_plot, = plt.plot([], [], 'r-')
  com_plot, = plt.plot([], [], 'bo')
  com_target_plot, = plt.plot([], [], 'go')
  plt.xlabel('left <--> right')
  plt.ylabel('bwd <--> fwd')
  plt.title('CoM in polygon?')

  x_min = np.inf
  y_min = np.inf
  x_max = -np.inf
  y_max = -np.inf

  data = load_log(sys.argv[1])
  rm = mc_rbdyn.get_robot_module("HRP4")
  robots = mc_rbdyn.loadRobot(rm)
  hrp4 = robots.robot()
  lfSurf = hrp4.surface('LeftFoot')
  rfSurf = hrp4.surface('RightFoot')
  rhSurf = hrp4.surface('RightHand')
  nrEntries = len(data['t'])
  triangle_data = []
  com_data = []
  com_target_data = []
  start_t = None
  for t in range(nrEntries):
    com = set_robot_config(hrp4, rm.ref_joint_order(), data, t)
    com_data.append(np.array([com.x() , com.y()]).reshape(2,1))
    lfPos = lfSurf.X_0_s(hrp4).translation()
    rfPos = rfSurf.X_0_s(hrp4).translation()
    rhPos = rhSurf.X_0_s(hrp4).translation()
    x_min = np.min([x_min, com.x(), lfPos.x(), rfPos.x(), rhPos.x()])
    y_min = np.min([y_min, com.y(), lfPos.y(), rfPos.y(), rhPos.y()])
    x_max = np.max([x_max, com.x(), lfPos.x(), rfPos.x(), rhPos.x()])
    y_max = np.max([y_max, com.y(), lfPos.y(), rfPos.y(), rhPos.y()])
    com_target_x = data['com_hrp4_target_x'][t]
    com_target_y = data['com_hrp4_target_y'][t]
    if com_target_x:
      if start_t is None:
        start_t = t
      com_target_data.append(np.array([com_target_x, com_target_y]).reshape(2,1))
    else:
      com_target_data.append(np.array([]).reshape(2,-1))
    triangle_data.append(np.array([lfPos.x(), rfPos.x(), rhPos.x(), lfPos.x(), lfPos.y(), rfPos.y(), rhPos.y(), lfPos.y()]).reshape(2,-1))

  plt.xlim(x_min, x_max)
  plt.ylim(y_min, y_max)
  nrPoints = 60*(nrEntries - start_t)/200
  print "Generate animation with",nrPoints,"points"
  ani = animation.FuncAnimation(fig1, update_figure, nrPoints, fargs=(start_t, [triangle_data, com_data, com_target_data], [triangle_plot, com_plot, com_target_plot]),
                                         interval=1000*1/60., blit = True)

  ani.save('com.mp4')
  #plt.show()
