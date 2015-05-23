#!/usr/bin/env python

import itertools
import pygame

import signal
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/hrpsys/')

import rtm
rtm.nshost = "localhost"
#rtm.nshost = "hrp2001c"
rtm.nsport = 2809

import OpenHRP
from OpenHRP import *

def sig_handler(signal, frame):
  global run
  run = False

if __name__ == "__main__":
  global run
  run = True
  signal.signal(signal.SIGINT, sig_handler)
  #Get the service
  rtm.initCORBA()
  mc = rtm.findRTC("MCControl0")
  if mc is None:
    print "Failed to find MCControl0 RTC"
    sys.exit(1)
  mc_svc = rtm.narrow(mc.service("service0"), "MCControlService")
  if mc_svc is None:
    print "Failed to get service"
    sys.exit(1)
  print "[OK] Got service"

  efs = itertools.cycle(["RARM_LINK6", "LLEG_LINK5", "RLEG_LINK5"])
  def control_next_ef():
    ef = efs.next()
    mc_svc.change_ef(ef)
    print "Now controlling",ef
  control_next_ef()

  #Initialize the joystick
  pygame.init()
  clock = pygame.time.Clock()
  pygame.joystick.init()
  if pygame.joystick.get_count() == 0:
    print "No joystick plugged"
    sys.exit(1)

  joy = pygame.joystick.Joystick(0)
  joy.init()

  axes = ["LH", "LV", "LT", "RH", "RV", "RT"] # Left/Right Horizontal/Vertical/Trigger
  buttons = ["A","B","X","Y","L1","L2","Select","Start","Home","L3","R3"]

  while run:
    for event in pygame.event.get():
      if event.type == pygame.JOYBUTTONDOWN:
        button = buttons[event.button]
        if button == "R2":
          control_next_ef()
        pass
      if event.type == pygame.JOYBUTTONUP:
        button = buttons[event.button]
        pass
      if event.type == pygame.JOYAXISMOTION:
        axis = axes[event.axis]
        value = event.value
        pass
      if event.type == pygame.JOYHATMOTION:
        value = event.value
        pass

    clock.tick(60)
