#!/usr/bin/env python2

from PySide import QtGui

import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas,\
                                                NavigationToolbar2QT as NavigationToolbar

from PySide.QtGui import QWidget, QVBoxLayout

from matplotlib.figure import Figure

import itertools
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

    def plot(self, data, x, y, ydiff, y_label, ydiff_label):
      self.canvas.plot(data, x, y, ydiff, y_label, ydiff_label)

    def clear(self):
      self.canvas.clear()

    def title(self, title):
      self.canvas.fig.suptitle(title)

    def y1_label(self, label):
      self.canvas.axes.set_ylabel(label)

    def y2_label(self, label):
      self.canvas.axes2.set_ylabel(label)

class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
      self.fig = Figure(figsize=(width, height), dpi=dpi)
      self.axes = self.fig.add_subplot(111)
      self.axes2 = self.axes.twinx()

      FigureCanvas.__init__(self, self.fig)
      self.setParent(parent)

      FigureCanvas.setSizePolicy(self,
                                 QtGui.QSizePolicy.Expanding,
                                 QtGui.QSizePolicy.Expanding)
      FigureCanvas.updateGeometry(self)

    def plot(self, data, x, y_, ydiff, y_labels, ydiff_labels):
      assert(len(y_) == len(y_labels))
      assert(len(ydiff) == len(ydiff_labels))
      color_cycler = itertools.cycle(['r','g','b','y','k','c','m','orange'])
      self.clear()
      [ self.axes.plot(data[x], data[y], label = y_label, color = color_cycler.next()) for y, y_label in zip(y_[0], y_labels[0]) ]
      [ self.axes2.plot(data[x], data[y], label = y_label, color = color_cycler.next()) for y, y_label in zip(y_[1], y_labels[1]) ]
      dt = data[x][1] - data[x][0]
      [ self.axes.plot(data[x][1:], np.diff(data[y])/dt, label = '{}'.format(y_label), color = color_cycler.next()) for y, y_label in zip(ydiff[0], ydiff_labels[0]) ]
      [ self.axes2.plot(data[x][1:], np.diff(data[y])/dt, label = '{}'.format(y_label), color = color_cycler.next()) for y, y_label in zip(ydiff[1], ydiff_labels[1]) ]
      self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=3, mode="expand", borderaxespad=0.5, fontsize = 10.0)
      self.axes2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=3, mode="expand", borderaxespad=-0.5, fontsize = 10.0)
      self.draw()

    def clear(self):
      self.axes.clear()
      self.axes2.clear()
      self.draw()
