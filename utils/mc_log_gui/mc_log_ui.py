#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtWidgets
from mc_log_ui import MCLogUI

import signal
import sys

if __name__ == '__main__':
  app = QtWidgets.QApplication(sys.argv)
  gui = MCLogUI()
  if len(sys.argv) > 1:
    gui.load_csv(sys.argv[1])
  gui.showMaximized()

  def sigint_handler(*args):
    gui.close()
  signal.signal(signal.SIGINT, sigint_handler)

  sys.exit(app.exec_())
