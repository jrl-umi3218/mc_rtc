#!/usr/bin/env python2

from PySide import QtGui

import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas,\
                                                NavigationToolbar2QT as NavigationToolbar

from PySide.QtGui import QWidget, QVBoxLayout

from matplotlib.figure import Figure

import numpy as np

class PlotCanvasWithToolbar(QWidget):
    def __init__(self, parent=None):
      QWidget.__init__(self, parent)
      self.canvas = PlotCanvas(self)
      self.toolbar = NavigationToolbar(self.canvas, self)

      vbox = QVBoxLayout()
      vbox.addWidget(self.canvas)
      vbox.addWidget(self.toolbar)
      self.setLayout(vbox)

    def plot(self, data, x, y1, y2, ydiff):
      self.canvas.plot(data, x, y1, y2, ydiff)

    def clear(self):
      self.canvas.clear()

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
      fig = Figure(figsize=(width, height), dpi=dpi)
      self.axes = fig.add_subplot(111)
      self.axes2 = self.axes.twinx()

      FigureCanvas.__init__(self, fig)
      self.setParent(parent)

      FigureCanvas.setSizePolicy(self,
                                 QtGui.QSizePolicy.Expanding,
                                 QtGui.QSizePolicy.Expanding)
      FigureCanvas.updateGeometry(self)

    def plot(self, data, x, y1, y2, ydiff):
      self.clear()
      [ self.axes.plot(data[x], data[y], label = y) for y in y1 ]
      [ self.axes2.plot(data[x], data[y], label = y) for y in y2 ]
      dt = data[x][1] - data[x][0]
      [ self.axes.plot(data[x][1:], np.diff(data[y])/dt, label = '{}_dot'.format(y)) for y in ydiff[0] ]
      [ self.axes2.plot(data[x][1:], np.diff(data[y])/dt, label = '{}_dot'.format(y)) for y in ydiff[1] ]
      self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=4, mode="expand", borderaxespad=0.)
      self.axes2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=4, mode="expand", borderaxespad=0.)
      self.draw()

    def clear(self):
      self.axes.clear()
      self.axes2.clear()
      self.draw()
