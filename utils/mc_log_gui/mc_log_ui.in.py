#!/usr/bin/env @MC_LOG_UI_PYTHON_EXECUTABLE@
# -*- coding: utf-8 -*-

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtGui, QtWidgets
from mc_log_ui import MCLogUI, get_icon

import signal
import sys

if __name__ == '__main__':
  app = QtWidgets.QApplication(sys.argv)
  app.setWindowIcon(QtGui.QIcon(get_icon()))
  gui = MCLogUI()
  if len(sys.argv) > 1:
    for fpath in sys.argv[1:]:
      gui.load_csv(fpath, False)
  gui.showMaximized()

  def sigint_handler(*args):
    gui.close()
  signal.signal(signal.SIGINT, sigint_handler)

  sys.exit(app.exec_())
