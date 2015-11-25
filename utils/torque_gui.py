#!/usr/bin/env python

import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/hrpsys/')
import OpenHRP
from OpenHRP import *

import rtm

import matplotlib.pyplot as plt
import numpy as np
import time
import threading

REF_JOINT_ORDER = [
    "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2", "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
    "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2", "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5",
    "CHEST_JOINT0", "CHEST_JOINT1", "HEAD_JOINT0", "HEAD_JOINT1",
    "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2", "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5", "RARM_JOINT6", "RARM_JOINT7",
    "LARM_JOINT0", "LARM_JOINT1", "LARM_JOINT2", "LARM_JOINT3", "LARM_JOINT4", "LARM_JOINT5", "LARM_JOINT6", "LARM_JOINT7",
    "RHAND_JOINT0", "RHAND_JOINT1", "RHAND_JOINT2", "RHAND_JOINT3", "RHAND_JOINT4",
    "LHAND_JOINT0", "LHAND_JOINT1", "LHAND_JOINT2", "LHAND_JOINT3", "LHAND_JOINT4"]

plt.ion()

import rtm
rtm.nshost = "hrp2001c"
#rtm.nshost = "localhost"
rtm.nsport = 2809

running = True

# Parameters for testing without the robot
#nrPoints = 100
#portname="rfsensor"
#monitoring = {"fx": 0, "fy": 1, "fz": 2}
#limits = {"fx": 10, "fy": 50, "fz": 200}

nrPoints = 300
portname = "ctau"
joints = ["RARM_JOINT5", "RARM_JOINT7", "LARM_JOINT5", "LARM_JOINT7"]
monitoring = {}
for j in joints:
  monitoring[j] = REF_JOINT_ORDER.index(j)
limits = {
  "RARM_JOINT5": 0.8*18.7,
  "RARM_JOINT7": 0.5*100,
  "LARM_JOINT5": 0.8*18.7,
  "LARM_JOINT7": 0.5*100
}
data = {}
lines = {}
limit_data = {}
limit_lines = {}
t = np.zeros(1)
legendEntries = []
legendText = []
for m in monitoring:
  data[m] = np.zeros(len(t))
  limit_data[m] = np.array([limits[m]])
  line, = plt.plot(t, data[m], label = m)
  lines[m] = line
  legendEntries.append(line)
  legendText.append(m)
  line, = plt.plot(t, limit_data[m], label = m + "_limit", linestyle='dashed')
  line.set_color(lines[m].get_color())
  line.set_linewidth(3.0)
  neg_line, = plt.plot(t, -1*limit_data[m], label = m + "_limit", linestyle='dashed')
  neg_line.set_color(lines[m].get_color())
  neg_line.set_linewidth(3.0)
  limit_lines[m] = [line, neg_line]
plt.legend(legendEntries, legendText, bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
plt.show()

def data_reader():
    global t, running
    prev_tm = None
    rtm.initCORBA()
    rh = rtm.findRTC("RobotHardware0")
    if rh is None:
      print "Getting data from simulation"
      rh = rtm.findRTC("HRP2DRCController(Robot)0")
    else:
      print "Getting data from robot hardware"
    while running:
        data_in = rtm.readDataPort(rh.port(portname))
        if data_in.tm != prev_tm:
            prev_tm = data_in.tm
            if len(t) < nrPoints:
                t = np.append(t, prev_tm.sec + prev_tm.nsec*1e-9)
            else:
                t[:-1] = t[1:]
                t[-1] = prev_tm.sec + prev_tm.nsec*1e-9
            for m in monitoring:
                if len(data[m]) < nrPoints:
                    data[m] = np.append(data[m], data_in.data[monitoring[m]])
                    limit_data[m] = np.append(limit_data[m], limits[m])
                else:
                    data[m][:-1] = data[m][1:]
                    data[m][-1] = data_in.data[monitoring[m]]
            lock = False

th = threading.Thread(target = data_reader)
th.start()

try:
        text = None
        while running:
            plt.xlim(xmin = t[0], xmax = t[-1])
            y_min = np.min([np.min(data[m]) for m in monitoring])
            if y_min > 0:
                y_min = 0.8*y_min
            else:
                y_min = 1.2*y_min
            y_max = np.max([np.max(data[m]) for m in monitoring])
            if y_max > 0:
                y_max = 1.2*y_max
            else:
                y_max = 0.8*y_max
            plt.ylim(ymin = y_min, ymax = y_max)
            emergency_text = []
            for m in monitoring:
                lines[m].set_data(t, data[m])
                limit_lines[m][0].set_data(t, limit_data[m])
                limit_lines[m][1].set_data(t, -1*limit_data[m])
                if abs(data[m][-1]) > limits[m]:
                  emergency_text.append('{0} over torque limit'.format(m))
            if len(emergency_text):
                text = plt.text(t[0] + 0.05*(t[-1] - t[0]), 0.92*y_max, '\n'.join(emergency_text),bbox={'facecolor': 'red', 'pad': 2})
            plt.pause(1/60.)
            if text:
                text.remove()
                text = None
except:
        running = False
        raise
