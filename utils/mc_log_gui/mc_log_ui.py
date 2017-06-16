#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import csv
import numpy as np
import os
import re
import signal
import sys

from functools import partial

from PySide import QtCore, QtGui

from mc_log_main_ui import Ui_MainWindow
from mc_log_tab import MCLogTab

try:
  import mc_rbdyn
except ImportError:
  mc_rbdyn = None

class MCLogJointDialog(QtGui.QDialog):
  def __init__(self, parent, rm, y1_prefix, y2_prefix = None):
    super(MCLogJointDialog, self).__init__(parent)
    self.setModal(True)
    self.joints = []
    self.y1_prefix = y1_prefix
    self.y2_prefix = y2_prefix
    layout = QtGui.QGridLayout(self)
    self.setLayout(layout)
    row = 0
    col = 0
    for i, j in enumerate(rm.ref_joint_order()):
      cBox = QtGui.QCheckBox(j, self)
      cBox.stateChanged.connect(partial(self.checkboxChanged, j))
      layout.addWidget(cBox, row, col)
      col += 1
      if col == 4:
        col = 0
        row += 1
    row += 1
    col = 0
    okButton = QtGui.QPushButton("Ok", self)
    layout.addWidget(okButton, row, col, 1, 2)
    okButton.clicked.connect(self.okButton)
    cancelButton = QtGui.QPushButton("Cancel", self)
    layout.addWidget(cancelButton, row, col + 2, 1, 2)
    cancelButton.clicked.connect(self.reject)

  def okButton(self):
    if len(self.joints):
      self.parent().plot_joint_data(self.joints, self.y1_prefix, self.y2_prefix)
    self.accept()

  def checkboxChanged(self, item, state):
    if state:
      self.joints.append(item)
    else:
      self.joints.remove(item)

class MCLogUI(QtGui.QMainWindow):
  def __init__(self, parent = None):
    super(MCLogUI, self).__init__(parent)
    self.__init__ui = Ui_MainWindow()
    self.ui = Ui_MainWindow()

    self.ui.setupUi(self)

    self.activeRobotAction = None
    self.rm = None
    if mc_rbdyn is not None:
      rMenu = QtGui.QMenu("Robot", self.ui.menubar)
      rGroup = QtGui.QActionGroup(rMenu)
      for r in mc_rbdyn.RobotLoader.available_robots():
        rAct = QtGui.QAction(r, rGroup)
        rAct.setCheckable(True)
        rGroup.addAction(rAct)
      rMenu.addActions(rGroup.actions())
      rGroup.actions()[0].setChecked(True)
      self.activeRobotAction = rGroup.actions()[0]
      self.setRobot(rGroup.actions()[0])
      self.connect(rGroup, QtCore.SIGNAL("triggered(QAction *)"), self.setRobot)
      self.ui.menubar.addMenu(rMenu)

    self.tab_re = re.compile('^Plot [0-9]+$')

    self.data = {}

  def setRobot(self, action):
    try:
      self.rm = mc_rbdyn.RobotLoader.get_robot_module(action.text())
      self.activeRobotAction = action
      for i in range(self.ui.tabWidget.count() - 1):
        tab = self.ui.tabWidget.widget(i)
        assert(isinstance(tab, MCLogTab))
        tab.setRobotModule(self.rm)
    except RuntimeError:
      QtGui.QMessageBox.warning(self, "Failed to get RobotModule", "Could not retrieve Robot Module: {}{}Check your console for more details".format(action.text(), os.linesep))
      action.setChecked(False)
      self.activeRobotAction.setChecked(True)
      self.rm = None

  @QtCore.Slot()
  def on_actionLoad_triggered(self):
    fpath = QtGui.QFileDialog.getOpenFileName(self, "Log file")[0]
    if len(fpath):
      self.load_csv(fpath)

  @QtCore.Slot()
  def on_actionExit_triggered(self):
    QtGui.QApplication.quit()

  @QtCore.Slot(int)
  def on_tabWidget_currentChanged(self, idx):
    if idx == self.ui.tabWidget.count() - 1:
      plotW = MCLogTab()
      plotW.setData(self.data)
      plotW.setRobotModule(self.rm)
      j = 1
      for i in range(self.ui.tabWidget.count() -1):
        if self.tab_re.match(self.ui.tabWidget.tabText(i)):
          j += 1
      self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, "Plot {}".format(j))
      self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
      self.updateClosable()

  @QtCore.Slot(int)
  def on_tabWidget_tabCloseRequested(self, idx):
    self.ui.tabWidget.setCurrentIndex(abs(idx - 1))
    self.ui.tabWidget.removeTab(idx)
    j = 1
    for i in range(self.ui.tabWidget.count() - 1):
      if self.tab_re.match(self.ui.tabWidget.tabText(i)):
        self.ui.tabWidget.setTabText(i, "Plot {}".format(j))
        j += 1
    self.updateClosable()

  def updateClosable(self):
    has_closable = self.ui.tabWidget.count() > 2
    self.ui.tabWidget.setTabsClosable(has_closable)
    if has_closable:
      self.ui.tabWidget.tabBar().tabButton(self.ui.tabWidget.count() - 1, QtGui.QTabBar.RightSide).hide();

  def load_csv(self, fpath):
    self.data = {}
    with open(fpath) as fd:
      reader = csv.DictReader(fd, delimiter=';')
      for row in reader:
        for k in reader.fieldnames:
          try:
            value = float(row[k])
          except ValueError:
            value = None
          if not k in self.data:
            self.data[k] = []
          self.data[k].append(value)
    for k in self.data:
      self.data[k] = np.array(self.data[k])
    self.update_data()

  def update_data(self):
    self.update_menu()
    for i in range(self.ui.tabWidget.count() - 1):
      tab = self.ui.tabWidget.widget(i)
      assert(isinstance(tab, MCLogTab))
      tab.setData(self.data)
      tab.setRobotModule(self.rm)

  def update_menu(self):
    self.ui.menuCommonPlots.clear()
    menuEntries = [
        ("Encoders", "qIn", None),
        ("Commands", "qOut", None),
        ("Torques", "tau", None),
        ("Encoders/Commands", "qIn", "qOut"),
        ]
    menuEntries = [ (n, y1, y2) for n, y1, y2 in menuEntries if any([k.startswith(y1) or (y2 is not None and k.startswith(y2)) for k in self.data.keys()]) ]
    for n, y1, y2 in menuEntries:
      act = QtGui.QAction(n, self.ui.menuCommonPlots)
      act.triggered.connect(MCLogJointDialog(self, self.rm, y1, y2).exec_)
      self.ui.menuCommonPlots.addAction(act)
    fSensors = set()
    for k in self.data:
      if k.find('ForceSensor') != -1:
        fSensors.add(k[:k.find('ForceSensor')])
    if len(fSensors):
      fsMenu = QtGui.QMenu("Force sensors", self.ui.menuCommonPlots)
      for f in sorted(fSensors):
        act = QtGui.QAction(f, fsMenu)
        act.triggered.connect(partial(self.plot_force_sensor, f))
        fsMenu.addAction(act)
      self.ui.menuCommonPlots.addMenu(fsMenu)

  def plot_force_sensor(self, fs):
    plotW = MCLogTab.ForceSensorPlot(self, fs)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, "FS: {}".format(fs))
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()

  def plot_joint_data(self, joints, y1_prefix, y2_prefix = None):
    plotW = MCLogTab.JointPlot(self, joints, y1_prefix, y2_prefix)
    tabTitle = y1_prefix
    if y2_prefix:
      tabTitle += "/{}".format(y2_prefix)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, tabTitle)
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()

if __name__ == '__main__':
  app = QtGui.QApplication(sys.argv)
  gui = MCLogUI()
  if len(sys.argv) > 1:
    gui.load_csv(sys.argv[1])
  gui.showMaximized()

  def sigint_handler(*args):
    gui.close()
  signal.signal(signal.SIGINT, sigint_handler)

  sys.exit(app.exec_())
