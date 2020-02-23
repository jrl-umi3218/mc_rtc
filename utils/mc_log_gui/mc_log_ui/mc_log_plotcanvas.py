#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from PyQt5 import QtCore, QtGui, QtWidgets

from PyQt5.QtWidgets import QWidget, QVBoxLayout

import copy
import math
import matplotlib
import numpy as np
matplotlib.use('Qt5Agg')
import matplotlib.pyplot

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas,\
                                               NavigationToolbar2QT as NavigationToolbar

from collections import OrderedDict
from math import asin, atan2

from mc_log_types import LineStyle, PlotSide


def rpyFromMat(E):
    """Same as mc_rbdyn::rpyFromMat."""
    roll = atan2(E[1][2], E[2][2]);
    pitch = -asin(E[0][2]);
    yaw = atan2(E[0][1], E[0][0]);
    return [roll, pitch, yaw]


def rpyFromQuat(quat):
    """Same as mc_rbdyn::rpyFromQuat."""
    import eigen
    return rpyFromMat(list(eigen.Quaterniond(*quat).toRotationMatrix()))

class PlotYAxis(object):
  def __init__(self, parent, x_axis = None):
    self.figure = parent
    if x_axis is None:
      self.is_left = True
      self._axis = parent.fig.add_subplot(111)
      self._axis.autoscale(enable = True, axis = 'both', tight = False)
      self._x_axis = self._axis
    else:
      self.is_left = False
      self._axis = x_axis.twinx()
      self._x_axis = x_axis
    self._axis.autoscale_view(False,True,True)
    self._axis.format_coord = parent.format_coord
    self.grid = LineStyle(linestyle = '--')
    self.plots = OrderedDict()
    self._label_fontsize = 10
    self._legend_ncol = 3

  def __len__(self):
    return len(self.plots)

  def _data(self):
    return self.figure.data

  def _legend_fontsize(self):
    return self.figure._legend_fontsize

  def _labelpad(self):
    return self.figure._labelpad

  def _x_label_fontsize(self):
    return self.figure._x_label_fontsize

  def axis(self):
    return self._axis

  def drawGrid(self):
    if len(self.plots):
      self._axis.grid(color = self.grid.color, linestyle = self.grid.linestyle, linewidth = self.grid.linewidth, visible = self.grid.visible, which = 'both')

  def legend(self):
    if not len(self.plots):
      return
    if self.is_left:
      loc = 3
      top_anchor = 1.02
    else:
      loc = 2
      top_anchor = -0.125
      if len(self.figure.x_label()):
        top_anchor = -0.175
    self._axis.legend(bbox_to_anchor=(0., top_anchor, 1., .102), loc=loc, ncol=self._legend_ncol, mode="expand", borderaxespad=0.5, fontsize=self._legend_fontsize())

  def legendNCol(self, n = None):
    if n is None:
      return self._legend_ncol
    self._legend_ncol = n
    self.legend()

  def legendRows(self):
    return math.ceil(len(self.plots) / float(self._legend_ncol))

  def legendOffset(self, offset, sign):
    if self.legendRows() > 3:
      offset = offset + sign * 0.015 * (self.legendRows() - 3)
    return offset

  # If this axis is empty but the other is not, fix my own limits, in the
  # opposite condition call the opposite function
  def fixLimits(self, axis):
    self._axis.get_yaxis().set_visible(len(self) != 0)
    axis._axis.get_yaxis().set_visible(len(axis) != 0)
    if len(axis) != 0 and len(self) == 0:
      point = (axis.axis().dataLim.get_points()[1] + axis.axis().dataLim.get_points()[0])/2
      plt, = self._axis.plot([point[0]], [point[1]], visible = False)
      plt.remove()
      del plt
      self._axis.relim()
    elif len(self) != 0 and len(axis) == 0:
      axis.fixLimits(self)

  def setLimits(self, xlim = None, ylim = None):
    if not len(self):
      return xlim
    dataLim = self._axis.dataLim.get_points()
    if xlim is not None:
      x_min = xlim[0]
      x_max = xlim[1]
    else:
      dataLim = self._axis.dataLim.get_points()
      x_range = dataLim[1][0] - dataLim[0][0]
      x_min = dataLim[0][0] - x_range*0.01
      x_max = dataLim[1][0] + x_range*0.01
    self._x_axis.set_xlim([x_min, x_max])
    if ylim is None:
      y_range = dataLim[1][1] - dataLim[0][1]
      self._axis.set_ylim([dataLim[0][1] - y_range*0.01, dataLim[1][1] + y_range*0.01])
    else:
      self._axis.set_ylim(ylim)
    return x_min, x_max

  def _label(self, get_label, set_label, l, size):
    if l is None:
      l = get_label()
    set_label(l, fontsize = size, labelpad = self._labelpad())

  def _label_property(self, get_label, set_label, l = None):
    if l is None:
      return get_label()
    set_label(l)

  def _x_label(self, l = None):
    self._label(self.x_label, self._x_axis.set_xlabel, l, self._x_label_fontsize())

  def x_label(self, l = None):
    return self._label_property(self._x_axis.get_xlabel, self._x_label, l)

  def _y_label(self, l = None):
    self._label(self.y_label, self._axis.set_ylabel, l, self._label_fontsize)

  def y_label(self, l = None):
    return self._label_property(self._axis.get_ylabel, self._y_label, l)

  def y_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._label_fontsize
    self._label_fontsize = fontsize
    self._y_label()

  def _plot(self, x, y, y_label, style = None):
    if style is None:
      return self._plot(x, y, y_label, LineStyle(color = self.figure._next_color()))
    if y_label in self.plots:
      return False
    self.plots[y_label] = self._axis.plot(x, y, label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)[0]
    self.legend()
    return True

  def add_plot(self, x, y, y_label, style = None):
    return self._plot(self._data()[x], self._data()[y], y_label, style)

  def add_diff_plot(self, x, y, y_label):
    dt = self._data()[x][1] - self._data()[x][0]
    return self._plot(self._data()[x][1:], np.diff(self._data()[y])//dt, y_label)

  def _add_rpy_plot(self, x_label, y, idx):
    assert (idx >= 0 and idx <= 2),"index must be 0, 1 or 2"
    rpy_label = ['roll', 'pitch', 'yaw']
    y_label = "{}_{}".format(y, rpy_label[idx])
    fmt = ""
    if "{}_qw".format(y) in self._data().keys():
      fmt = "q"
    qw, qx, qy, qz = [ self._data()[k] for k in [ "{}_{}{}".format(y, fmt, ax) for ax in ["w", "x", "y", "z"] ] ]
    data = [ rpyFromQuat([w, x, y, z])[idx] for w, x, y, z in zip(qw, qx, qy, qz) ]
    return self._plot(self._data()[x_label], data, y_label)

  def add_roll_plot(self, x, y):
    return self._add_rpy_plot(x, y, 0)

  def add_pitch_plot(self, x, y):
    return self._add_rpy_plot(x, y, 1)

  def add_yaw_plot(self, x, y):
    return self._add_rpy_plot(x, y, 2)

  def add_rpy_plot(self, x, y):
    r = self.add_roll_plot(x, y)
    p = self.add_pitch_plot(x, y)
    y = self.add_yaw_plot(x, y)
    return r or p or y

  def remove_plot(self, y):
    if y not in self.plots:
      return
    self.plots[y].remove()
    del self.plots[y]
    if len(self.plots):
      self._axis.relim()
      self.legend()
    else:
      self._axis.clear()

  def clear(self):
    self.plots = {}
    self._axis.clear()

  # Get or set the style of a given plot
  def style(self, y, style = None):
    if y not in self.plots:
      raise KeyError("No plot named {}".format(y))
    plt = self.plots[y]
    if style is None:
      return LineStyle(plt.get_color(), plt.get_linestyle(), plt.get_linewidth(), label = plt.get_label())
    plt.set_color(style.color)
    plt.set_linestyle(style.linestyle)
    plt.set_linewidth(style.linewidth)
    if len(style.label):
      plt.set_label(style.label)

class PlotFigure(object):
  def __init__(self):
    self.fig = matplotlib.pyplot.figure(figsize=(5, 4), dpi=100)
    self.axes = {}
    self.axes[PlotSide.LEFT] = PlotYAxis(self)
    self.axes[PlotSide.RIGHT] = PlotYAxis(self, self._left().axis())

    self._title_fontsize = 12
    self._x_label_fontsize = 10
    self._labelpad = 10
    self._tick_labelsize = 10
    self._legend_fontsize = 10
    self._top_offset = 0.9
    self._bottom_offset = 0.1

    self.data = None
    self.computed_data = {}

    self.color = 0
    cm = matplotlib.cm.Set1
    self.Ncolor = min(cm.N, 12)
    cm2rgb = (np.array(cm(x)[0:3]) for x in np.linspace(0, 1, self.Ncolor))
    self.colors = ['#%02x%02x%02x' % tuple((255 * rgb).astype(int)) for rgb in cm2rgb]

    self.x_data = 't'

  # Helper function to call something on all axes, call expects an axis argument
  def _axes(self, call):
    call(self._left())
    call(self._right())

  # Shortcut to the left axis
  def _left(self):
    return self.axes[PlotSide.LEFT]

  # Shortcut to the right axis
  def _right(self):
    return self.axes[PlotSide.RIGHT]

  def _drawGrid(self):
    self._axes(lambda axis: axis.drawGrid())

  def _legend(self):
    self._axes(lambda axis: axis.legend())

  def draw(self, x_limits = None, y1_limits = None, y2_limits = None):
    self._left().fixLimits(self._right())
    x_limits = self._left().setLimits(x_limits, y1_limits)
    self._right().setLimits(x_limits, y2_limits)
    self._legend()
    self._drawGrid()
    top_offset = self._left().legendOffset(self._top_offset, -1)
    bottom_offset = self._left().legendOffset(self._bottom_offset, 1)
    self.fig.subplots_adjust(top = top_offset, bottom = bottom_offset)

  def setData(self, data):
    self.data = data

  def show(self):
    self.fig.show()

  def top_offset(self, off = None):
    if off is None:
      return self._top_offset
    else:
      self._top_offset = off

  def bottom_offset(self, off = None):
    if off is None:
      return self._bottom_offset
    else:
      self._bottom_offset = off

  def title(self, title = None):
    if title is None:
      if self.fig._suptitle is None:
        return ""
      return self.fig._suptitle.get_text()
    self.fig.suptitle(title)

  def title_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._title_fontsize
    self._title_fontsize = fontsize
    self.fig.suptitle(self.title(), fontsize = self._title_fontsize)

  def tick_fontsize(self, size = None):
    if size is None:
      return self._tick_labelsize
    self._tick_labelsize = size
    self._axes(lambda a: a.axis().tick_params(labelsize = self._tick_labelsize))

  def labelpad(self, pad = None):
    if pad is None:
      return self._labelpad
    self._labelpad = pad
    self._left()._x_label()
    self._axes(lambda axis: axis._y_label())

  def _x_label(self, label = None):
    self._left()._x_label(label)

  def _y1_label(self, label = None):
    self._left()._y_label(label)

  def _y2_label(self, label = None):
    self._right()._y_label(label)

  def x_label(self, label = None):
    return self._left().x_label(label)

  def x_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._x_label_fontsize
    self._x_label_fontsize = fontsize
    self._x_label()

  def y1_label(self, label = None):
    return self._left().y_label(label)

  def y1_label_fontsize(self, fontsize = None):
    return self._left().y_label_fontsize(fontsize)

  def y2_label(self, label = None):
    return self._right().y_label(label)

  def y2_label_fontsize(self, fontsize = None):
    return self._right().y_label_fontsize(fontsize)

  def _next_color(self):
    self.color += 1
    return self.colors[ (self.color - 1) % self.Ncolor ]

  def legend_fontsize(self, size = None):
    if size is None:
      return self._legend_fontsize
    self._legend_fontsize = size
    self._legend()

  def y1_legend_ncol(self, n = None):
    return self._left().legendNCol(n)

  def y2_legend_ncol(self, n = None):
    return self._right().legendNCol(n)

  def add_plot_left(self, x, y, y_label, style = None):
    return self._left().add_plot(x, y, y_label, style)

  def add_plot_right(self, x, y, y_label, style = None):
    return self._right().add_plot(x, y, y_label, style)

  def add_diff_plot_left(self, x, y, y_label):
    return self._left().add_diff_plot(x, y, y_label)

  def add_diff_plot_right(self, x, y, y_label):
    return self._right().add_diff_plot(x, y, y_label)

  def add_roll_plot_left(self, x, y):
    return self._left().add_roll_plot(x, y)

  def add_pitch_plot_left(self, x, y):
    return self._left().add_pitch_plot(x, y)

  def add_yaw_plot_left(self, x, y):
    return self._left().add_yaw_plot(x, y)

  def add_roll_plot_right(self, x, y):
    return self._right().add_roll_plot(x, y)

  def add_pitch_plot_right(self, x, y):
    return self._right().add_pitch_plot(x, y)

  def add_yaw_plot_right(self, x, y):
    return self._right().add_yaw_plot(x, y)

  def add_rpy_plot_left(self, x, y):
    return self._left().add_rpy_plot(x, y)

  def add_rpy_plot_right(self, x, y):
    return self._right().add_rpy_plot(x, y)

  def _remove_plot(self, SIDE, y_label):
    self.axes[SIDE].remove_plot(y_label)
    if len(self._left()) == 0 and len(self._right()) == 0:
      self.color = 0

  def remove_plot_left(self, y_label):
    self._remove_plot(PlotSide.LEFT, y_label)

  def remove_plot_right(self, y_label):
    self._remove_plot(PlotSide.RIGHT, y_label)

  def format_coord(self, x, y):
    display_coord = self.axes[PlotSide.RIGHT].axis().transData.transform((x,y))
    inv = self.axes[PlotSide.LEFT].axis().transData.inverted()
    ax_coord = inv.transform(display_coord)
    if len(self._left()) and len(self._right()):
      return "x: {:.3f}    y1: {:.3f}    y2: {:.3f}".format(x, ax_coord[1], y)
    elif len(self._left()):
      return "x: {:.3f}    y1: {:.3f}".format(x, ax_coord[1])
    elif len(self._right()):
      return "x: {:.3f}    y2: {:.3f}".format(x, y)
    else:
      return "x: {:.3f}".format(x)

  def clear_all(self):
    self.color = 0
    self._axes(lambda a: a.clear())

  def style_left(self, y, styleIn = None):
    return self._left().style(y, styleIn)

  def style_right(self, y, styleIn = None):
    return self._right().style(y, styleIn)

class SimpleAxesDialog(QtWidgets.QDialog):
  def __init__(self, parent):
    QtWidgets.QDialog.__init__(self, parent)
    self.setWindowTitle('Edit axes limits')
    self.setModal(True)
    self.layout = QtWidgets.QGridLayout(self)
    self.layout.addWidget(QtWidgets.QLabel("Min"), 0, 1)
    self.layout.addWidget(QtWidgets.QLabel("Max"), 0, 2)

    self.layout.addWidget(QtWidgets.QLabel("X"), 1, 0)
    x_limits = parent.x_limits
    if x_limits is None:
      x_limits = parent._left().axis().get_xlim()
    self.x_min = QtWidgets.QLineEdit(str(x_limits[0]))
    self.x_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.x_min, 1, 1)
    self.x_max = QtWidgets.QLineEdit(str(x_limits[1]))
    self.x_max.setValidator(QtGui.QDoubleValidator())
    self.x_init = [float(self.x_min.text()), float(self.x_max.text())]
    self.layout.addWidget(self.x_max, 1, 2)

    self.layout.addWidget(QtWidgets.QLabel("Y1"), 2, 0)
    y1_limits = parent.y1_limits
    if y1_limits is None:
      y1_limits = parent._left().axis().get_ylim()
    self.y1_min = QtWidgets.QLineEdit(str(y1_limits[0]))
    self.y1_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y1_min, 2, 1)
    self.y1_max = QtWidgets.QLineEdit(str(y1_limits[1]))
    self.y1_max.setValidator(QtGui.QDoubleValidator())
    self.y1_init = [float(self.y1_min.text()), float(self.y1_max.text())]
    self.layout.addWidget(self.y1_max, 2, 2)

    self.layout.addWidget(QtWidgets.QLabel("Y2"), 3, 0)
    y2_limits = parent.y2_limits
    if y2_limits is None:
      y2_limits = parent._right().axis().get_ylim()
    self.y2_min = QtWidgets.QLineEdit(str(y2_limits[0]))
    self.y2_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y2_min, 3, 1)
    self.y2_max = QtWidgets.QLineEdit(str(y2_limits[1]))
    self.y2_max.setValidator(QtGui.QDoubleValidator())
    self.y2_init = [float(self.y2_min.text()), float(self.y2_max.text())]
    self.layout.addWidget(self.y2_max, 3, 2)

    confirmLayout = QtWidgets.QHBoxLayout()
    okButton = QtWidgets.QPushButton("Ok", self)
    confirmLayout.addWidget(okButton)
    okButton.clicked.connect(self.accept)
    applyButton = QtWidgets.QPushButton("Apply", self)
    confirmLayout.addWidget(applyButton)
    applyButton.clicked.connect(self.apply)
    cancelButton = QtWidgets.QPushButton("Cancel", self)
    confirmLayout.addWidget(cancelButton)
    cancelButton.clicked.connect(self.reject)
    self.layout.addLayout(confirmLayout, 4, 0, 1, 3)

  def apply(self):
    changed = False
    x_limits = [float(self.x_min.text()), float(self.x_max.text())]
    if x_limits != self.x_init:
      changed = True
      self.parent().x_locked.setChecked(True)
      self.parent().x_limits = x_limits
    y1_limits = [float(self.y1_min.text()), float(self.y1_max.text())]
    if y1_limits != self.y1_init:
      changed = True
      self.parent().y1_locked.setChecked(True)
      self.parent().y1_limits = y1_limits
    y2_limits = [float(self.y2_min.text()), float(self.y2_max.text())]
    if y2_limits != self.y2_init:
      changed = True
      self.parent().y2_locked.setChecked(True)
      self.parent().y2_limits = y2_limits
    if changed:
      self.parent().draw()

  def accept(self):
    QtWidgets.QDialog.accept(self)
    self.apply()

class PlotCanvasWithToolbar(PlotFigure, QWidget):
  def __init__(self, parent = None):
    PlotFigure.__init__(self)
    QWidget.__init__(self, parent)

    self.canvas = FigureCanvas(self.fig)
    self.canvas.mpl_connect('draw_event', self.on_draw)
    self.toolbar = NavigationToolbar(self.canvas, self)

    vbox = QVBoxLayout(self)
    vbox.addWidget(self.canvas)
    vbox.addWidget(self.toolbar)
    self.setLayout(vbox)

  def setupLockButtons(self, layout):
    self.x_locked = QtWidgets.QPushButton(u"🔒X", self)
    self.x_locked.setCheckable(True)
    layout.addWidget(self.x_locked)
    self.x_locked.toggled.connect(self.x_locked_changed)
    self.x_limits = None

    self.y1_locked = QtWidgets.QPushButton(u"🔒 Y1", self)
    self.y1_locked.setCheckable(True)
    layout.addWidget(self.y1_locked)
    self.y1_locked.toggled.connect(self.y1_locked_changed)
    self.y1_limits = None

    self.y2_locked = QtWidgets.QPushButton(u"🔒 Y2", self)
    self.y2_locked.setCheckable(True)
    layout.addWidget(self.y2_locked)
    self.y2_locked.toggled.connect(self.y2_locked_changed)
    self.y2_limits = None

  def axesDialog(self):
    SimpleAxesDialog(self).exec_()

  def on_draw(self, event):
    if self.x_limits is not None:
      self.x_limits = self._left().axis().get_xlim()
    if self.y1_limits is not None:
      self.y1_limits = self._left().axis().get_ylim()
    if self.y2_limits is not None:
      self.y2_limits = self._right().axis().get_ylim()

  def draw(self):
    PlotFigure.draw(self, self.x_limits, self.y1_limits, self.y2_limits)
    self.canvas.draw()

  def _y_lock_changed(self, name, cbox, get_lim):
    if cbox.isChecked():
      cbox.setText(u"🔓 {}".format(name))
      return get_lim()
    else:
      cbox.setText(u"🔒{}".format(name))
      return None

  def x_locked_changed(self, status):
    self.x_limits = self._y_lock_changed("X", self.x_locked, self._left().axis().get_xlim)
    if self.x_limits is None:
      self.draw()

  def y1_locked_changed(self, status):
    self.y1_limits = self._y_lock_changed("Y1", self.y1_locked, self._left().axis().get_ylim)
    if self.y1_limits is None:
      self.draw()

  def y2_locked_changed(self, status):
    self.y2_limits = self._y_lock_changed("Y2", self.y2_locked, self._right().axis().get_ylim)
    if self.y2_limits is None:
      self.draw()
