#!/usr/bin/env python
# -*- coding: utf-8 -*-

import collections
import copy
import csv
import ctypes
import json
import numpy as np
import os
import re
import signal
import sys
import tempfile

from functools import partial

from PySide import QtCore, QtGui

from mc_log_main_ui import Ui_MainWindow
from mc_log_tab import MCLogTab

try:
  import mc_rbdyn
except ImportError:
  mc_rbdyn = None

UserPlot = collections.namedtuple('UserPlot', ['title', 'x', 'y1', 'y1d', 'y2', 'y2d'])

def safe_float(v):
    if len(v):
        return float(v)
    else:
        return None

def read_flat(f):
    def read_size(fd):
        return ctypes.c_size_t.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_size_t))).value
    def read_bool(fd):
        return ctypes.c_bool.from_buffer_copy(fd.read(ctypes.sizeof(ctypes.c_bool))).value
    def read_string(fd, size):
        return fd.read(size).decode('ascii')
    def read_array(fd, size):
        return np.frombuffer(fd.read(size * ctypes.sizeof(ctypes.c_double)), np.double)
    def read_string_array(fd, size):
        [ read_string(fd, read_size(fd)) for i in range(size) ]
    data = {}
    with open(f, 'rb') as fd:
        nrEntries = read_size(fd)
        for i in range(nrEntries):
            is_numeric = read_bool(fd)
            key = read_string(fd, read_size(fd))
            if is_numeric:
                data[key] = read_array(fd, read_size(fd))
            else:
                data[key] = read_string_array(fd, read_size(fd))
    return data

class MCLogJointDialog(QtGui.QDialog):
  def __init__(self, parent, rm, name, y1_prefix = None, y2_prefix = None, y1_diff_prefix = None, y2_diff_prefix = None):
    super(MCLogJointDialog, self).__init__(parent)
    self.setWindowTitle("Select plot joints")
    self.setModal(True)
    self.joints = []
    self.name = name
    self.y1_prefix = y1_prefix
    self.y2_prefix = y2_prefix
    self.y1_diff_prefix = y1_diff_prefix
    self.y2_diff_prefix = y2_diff_prefix
    layout = QtGui.QVBoxLayout(self)

    jointsBox = QtGui.QGroupBox("Joints", self)
    grid = QtGui.QGridLayout(jointsBox)
    margins = grid.contentsMargins()
    margins.setTop(20)
    grid.setContentsMargins(margins)
    self.jointsCBox = []
    row = 0
    col = 0
    if rm is not None:
        for i, j in enumerate(rm.ref_joint_order()):
          cBox = QtGui.QCheckBox(j, self)
          cBox.stateChanged.connect(partial(self.checkboxChanged, j))
          grid.addWidget(cBox, row, col)
          self.jointsCBox.append(cBox)
          col += 1
          if col == 4:
            col = 0
            row += 1
    layout.addWidget(jointsBox)

    optionsBox = QtGui.QGroupBox("Options", self)
    optionsLayout = QtGui.QHBoxLayout(optionsBox)
    margins = optionsLayout.contentsMargins()
    margins.setTop(20)
    optionsLayout.setContentsMargins(margins)
    self.selectAllBox = QtGui.QCheckBox("Select all", self)
    self.selectAllBox.stateChanged.connect(self.selectAllBoxChanged)
    optionsLayout.addWidget(self.selectAllBox)
    layout.addWidget(optionsBox)

    confirmLayout = QtGui.QHBoxLayout()
    okButton = QtGui.QPushButton("Ok", self)
    confirmLayout.addWidget(okButton)
    okButton.clicked.connect(self.okButton)
    cancelButton = QtGui.QPushButton("Cancel", self)
    confirmLayout.addWidget(cancelButton)
    cancelButton.clicked.connect(self.reject)
    layout.addLayout(confirmLayout)

  def okButton(self):
    if len(self.joints):
      self.parent().plot_joint_data(self.name, self.joints, self.y1_prefix, self.y2_prefix, self.y1_diff_prefix, self.y2_diff_prefix)
    self.accept()

  def checkboxChanged(self, item, state):
    if state:
      self.joints.append(item)
    else:
      self.joints.remove(item)

  def selectAllBoxChanged(self, state):
    for cBox in self.jointsCBox:
      cBox.setChecked(state)
    if state:
      self.selectAllBox.setText("Select none")
    else:
      self.selectAllBox.setText("Select all")

class MCLogUI(QtGui.QMainWindow):
  def __init__(self, parent = None):
    super(MCLogUI, self).__init__(parent)
    self.__init__ui = Ui_MainWindow()
    self.ui = Ui_MainWindow()

    self.ui.setupUi(self)

    self.userPlotList = []
    self.userPlotFile = os.path.expanduser("~") + "/.config/mc_log_ui/custom_plot.json"
    if os.path.exists(self.userPlotFile):
      with open(self.userPlotFile) as f:
        self.userPlotList = [UserPlot(*x) for x in json.load(f)]
    self.update_userplot_menu()

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

  def saveUserPlots(self):
    confDir = os.path.dirname(self.userPlotFile)
    if not os.path.exists(confDir):
      os.makedirs(confDir)
    with open(self.userPlotFile, 'w') as f:
      json.dump(self.userPlotList, f)
    self.update_userplot_menu()

  def update_userplot_menu(self):
    self.ui.menuUserPlots.clear()
    for p in self.userPlotList:
      act = QtGui.QAction(p.title, self.ui.menuUserPlots)
      act.triggered.connect(lambda plot=p: self.plot_userplot(plot))
      self.ui.menuUserPlots.addAction(act)
    act = QtGui.QAction("Save current plot", self.ui.menuUserPlots)
    act.triggered.connect(self.save_userplot)
    self.ui.menuUserPlots.addAction(act)
    if len(self.userPlotList):
      rmUserPlotMenu = QtGui.QMenu("Remove saved plots", self.ui.menuUserPlots)
      for p in self.userPlotList:
        act = QtGui.QAction(p.title, self.ui.menuUserPlots)
        act.triggered.connect(lambda plot=p: self.remove_userplot(plot))
        rmUserPlotMenu.addAction(act)
      self.ui.menuUserPlots.addMenu(rmUserPlotMenu)

  def save_userplot(self):
    tab = self.ui.tabWidget.currentWidget()
    canvas = tab.ui.canvas
    valid = len(canvas.axes_plots) != 0 or len(cavas.axes2_plots) != 0
    if not valid:
      err_diag = QtGui.QMessageBox(self)
      err_diag.setModal(True)
      err_diag.setText("Cannot save user plot if nothing is shown")
      err_diag.exec_()
      return
    title, ok = QtGui.QInputDialog.getText(self, "User plot", "Title of your plot:")
    if ok:
      y1 = filter(lambda k: k in self.data.keys(), canvas.axes_plots.keys())
      y2 = filter(lambda k: k in self.data.keys(), canvas.axes2_plots.keys())
      y1d = map(lambda sp: "{}_{}".format(sp.name, sp.id), filter(lambda sp: sp.idx == 0, tab.specials.values()))
      y2d = map(lambda sp: "{}_{}".format(sp.name, sp.id), filter(lambda sp: sp.idx == 1, tab.specials.values()))
      self.userPlotList.append(UserPlot(title, tab.x_data, y1, y1d, y2, y2d))
      self.saveUserPlots()

  def plot_userplot(self, p):
    valid = p.x in self.data.keys() and all([y in self.data.keys() for x in [p.y1, p.y2] for y in x])
    if not valid:
      missing_entries = ""
      if not p.x in self.data.keys():
        missing_entries += "- {}\n".format(p.x)
      for x in [p.y1, p.y1d, p.y2, p.y2d]:
        for y in x:
          if not y in self.data.keys():
            missing_entries += "- {}\n".format(y)
      missing_entries = missing_entries[:-1]
      err_diag = QtGui.QMessageBox(self)
      err_diag.setModal(True)
      err_diag.setText("Plot {} is not valid for this log file, some data is missing\nMissing entries:\n{}".format(p.title, missing_entries))
      err_diag.exec_()
      return
    plotW = MCLogTab.UserPlot(self, p)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, p.title)
    self.ui.tabWidget.setCurrentIndex(self.ui.tabWidget.count() - 2)
    self.updateClosable()

  def remove_userplot(self, p):
    self.userPlotList.remove(p)
    self.saveUserPlots()

  def setRobot(self, action):
    try:
      self.rm = mc_rbdyn.RobotLoader.get_robot_module(action.text())
      self.activeRobotAction = action
      for i in range(self.ui.tabWidget.count() - 1):
        tab = self.ui.tabWidget.widget(i)
        assert(isinstance(tab, MCLogTab))
        tab.setRobotModule(self.rm)
    except RuntimeError:
      #QtGui.QMessageBox.warning(self, "Failed to get RobotModule", "Could not retrieve Robot Module: {}{}Check your console for more details".format(action.text(), os.linesep))
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

  def load_csv(self, fpath, tmp = False):
    self.data = {}
    if fpath.endswith('.bin'):
      tmpf = tempfile.mkstemp(suffix = '.flat')[1]
      os.system("mc_bin_to_flat {} {}".format(fpath, tmpf))
      return self.load_csv(tmpf, True)
    elif fpath.endswith('.flat'):
      self.data = read_flat(fpath)
    else:
      with open(fpath) as fd:
        reader = csv.DictReader(fd, delimiter=';')
        for k in reader.fieldnames:
          self.data[k] = []
        for row in reader:
          for k in reader.fieldnames:
            self.data[k].append(safe_float(row[k]))
      for k in self.data:
        self.data[k] = np.array(self.data[k])
    i = 0
    while "qIn_{}".format(i) in self.data and "qOut_{}".format(i) in self.data:
      self.data["error_{}".format(i)] = self.data["qOut_{}".format(i)] - self.data["qIn_{}".format(i)]
      self.data["qIn_limits_lower_{}".format(i)] = np.full_like(self.data["qIn_{}".format(i)], 0)
      self.data["qIn_limits_upper_{}".format(i)] = np.full_like(self.data["qIn_{}".format(i)], 0)
      self.data["qOut_limits_lower_{}".format(i)] = self.data["qIn_limits_lower_{}".format(i)]
      self.data["qOut_limits_upper_{}".format(i)] = self.data["qIn_limits_upper_{}".format(i)]
      i += 1
    i = 0
    while "tauIn_{}".format(i) in self.data:
      self.data["tauIn_limits_lower_{}".format(i)] = np.full_like(self.data["tauIn_{}".format(i)], 0)
      self.data["tauIn_limits_upper_{}".format(i)] = np.full_like(self.data["tauIn_{}".format(i)], 0)
      i += 1
    self.update_data()
    if tmp:
      os.unlink(fpath)

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
        ("Encoders", "qIn", None, None, None),
        ("Commands", "qOut", None, None, None),
        ("Error", "error", None, None, None),
        ("Torques", "tauIn", None, None, None),
        ("Encoders/Commands", "qIn", "qOut", None, None),
        ("Error/Torque", "error", "tauIn", None, None),
        ("Encoders velocity", None, None, "qIn", None),
        ("Command velocity", None, None, "qOut", None),
        ("Encoders/Commands velocity", None, None, "qIn", "qOut"),
        ("Encoders/Encoders velocity", "qIn", None, None, "qIn"),
        ("Command/Command velocity", "qOut", None, None, "qOut"),
        ]
    def validEntry(y):
      return any([ y is None or (y is not None and k.startswith(y)) for k in self.data.keys() ])
    menuEntries = [ (n, y1, y2, y1d, y2d) for n, y1, y2, y1d, y2d in menuEntries if all([validEntry(y) for y in [y1, y2, y1d, y2d]]) ]
    for n, y1, y2, y1d, y2d in menuEntries:
      act = QtGui.QAction(n, self.ui.menuCommonPlots)
      act.triggered.connect(MCLogJointDialog(self, self.rm, n, y1, y2, y1d, y2d).exec_)
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

  def plot_joint_data(self, name, joints, y1_prefix = None, y2_prefix = None,
                                    y1_diff_prefix = None, y2_diff_prefix = None):
    plotW = MCLogTab.JointPlot(self, joints, y1_prefix, y2_prefix, y1_diff_prefix, y2_diff_prefix)
    self.ui.tabWidget.insertTab(self.ui.tabWidget.count() - 1, plotW, name)
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
