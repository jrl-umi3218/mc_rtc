#!/usr/bin/env python

import itertools
import pygame

import signal
import sys
sys.path.insert(0, '/usr/local/lib/python2.7/dist-packages/hrpsys/')

import rtm
#rtm.nshost = "localhost"
rtm.nshost = "hrp2001c"
rtm.nsport = 2809

import OpenHRP
from OpenHRP import *

import eigen3
import spacevecalg as sva

def sig_handler(signal, frame):
  global run
  run = False

class EgressPhase(object):
  def __init__(self, mc_svc):
    self.mc_svc = mc_svc
  def handleButtonDown(self, button):
    pass
  def handleButtonUp(self, button):
    pass
  def handleAxisMotion(self, axis, value):
    pass
  def handleHatMotion(self, value):
    pass
  def run(self):
    pass

class MoveFootInsidePhase(EgressPhase):
  def __init__(self, mc_svc):
    super(MoveFootInsidePhase, self).__init__(mc_svc)
    self.x_speed = 0
    self.y_speed = 0
    self.z_speed = 0
    self.roll_speed = 0
    self.pitch_speed = 0
  def handleButtonDown(self, button):
    if button == "X":
      self.mc_svc.rotate_ef(0, 0., -0.05)
    if button == "B":
      self.mc_svc.rotate_ef(0, 0., 0.05)
  def handleAxisMotion(self, axis, value):
    if axis == "LH":
      self.x_speed = 0.001*value
    if axis == "RH":
      self.roll_speed = -0.01*value
    if axis == "LV":
      self.z_speed = -0.001*value
    if axis == "RV":
      self.pitch_speed = 0.01*value
  def handleHatMotion(self, value):
    self.y_speed = -1*value[0]*0.0005
  def run(self):
      self.mc_svc.translate_ef(self.x_speed, self.y_speed, self.z_speed)
      self.mc_svc.rotate_ef(self.roll_speed, self.pitch_speed, 0)

class LiftBodyPhase(EgressPhase):
  def __init__(self, mc_svc):
    super(LiftBodyPhase, self).__init__(mc_svc)
    self.comx_speed = 0
    self.comy_speed = 0
    self.comz_speed = 0
  def handleAxisMotion(self, axis, value):
    if axis == "LH":
      self.comy_speed = 0.0005*value
    if axis == "LV":
      self.comz_speed = -0.0005*value
    if axis == "RV":
      self.comx_speed = -0.0005*value
  def run(self):
    self.mc_svc.move_com(self.comx_speed, self.comy_speed, self.comz_speed)

class RotateBodyPhase(EgressPhase):
  def __init__(self, mc_svc):
    super(RotateBodyPhase, self).__init__(mc_svc)
    self.rotz_speed = 0
  def handleAxisMotion(self, axis, value):
    if axis == "RH":
      self.rotz_speed = 0.0005*value
  def run(self):
    self.mc_svc.rotate_ef(0, 0, self.rotz_speed)

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

  #phases = itertools.cycle(["MOVEFOOTINSIDE", "LIFTBODY", "ROTATEBODY"])
  phases = itertools.cycle(["LIFTBODY"])
  def nextPhase():
    phaseName = phases.next()
    if phaseName == "MOVEFOOTINSIDE":
      return MoveFootInsidePhase(mc_svc)
    if phaseName == "LIFTBODY":
      return LiftBodyPhase(mc_svc)
    elif phaseName == "ROTATEBODY":
      return RotateBodyPhase(mc_svc)
    else:
      return None
  phase = nextPhase()

  while run:
    for event in pygame.event.get():
      if event.type == pygame.JOYBUTTONDOWN:
        button = buttons[event.button]
        if button == "Home":
          if mc_svc.play_next_stance():
            phase = nextPhase()
        else:
          phase.handleButtonDown(button)
        pass
      if event.type == pygame.JOYBUTTONUP:
        button = buttons[event.button]
        phase.handleButtonUp(button)
        pass
      if event.type == pygame.JOYAXISMOTION:
        axis = axes[event.axis]
        value = event.value
        phase.handleAxisMotion(axis, value)
        pass
      if event.type == pygame.JOYHATMOTION:
        value = event.value
        phase.handleHatMotion(value)
        pass
    phase.run()

    clock.tick(60)
