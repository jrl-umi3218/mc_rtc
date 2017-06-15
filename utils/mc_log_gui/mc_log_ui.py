#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
import csv
import numpy as np
import os
import re
import signal
import sys

from PySide import QtCore, QtGui

from mc_log_main_ui import Ui_MainWindow
from mc_log_tab import MCLogTab

class MCLogUI(QtGui.QMainWindow):
  def __init__(self, parent = None):
    super(MCLogUI, self).__init__(parent)
    self.__init__ui = Ui_MainWindow()
    self.ui = Ui_MainWindow()

    self.ui.setupUi(self)

    self.tab_re = re.compile('^Plot [0-9]+$')

    self.data = {}

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
    for i in range(self.ui.tabWidget.count() - 1):
      tab = self.ui.tabWidget.widget(i)
      assert(isinstance(tab, MCLogTab))
      tab.setData(self.data)

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
