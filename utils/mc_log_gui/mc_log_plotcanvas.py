#!/usr/bin/env python
# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

from PySide.QtGui import QWidget, QVBoxLayout

import copy
import math
import matplotlib
import numpy as np
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'
import matplotlib.pyplot

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas,\
                                               NavigationToolbar2QT as NavigationToolbar

from collections import OrderedDict
from math import asin, atan2

from mc_log_types import LineStyle


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

class PlotFigure(object):
  def __init__(self):
    self.fig = matplotlib.pyplot.figure(figsize=(5, 4), dpi=100)
    self.axes = self.fig.add_subplot(111)
    self.axes.autoscale(enable = True, axis = 'both', tight = False)
    self.axes.autoscale_view(False,True,True)
    self.axes2 = self.axes.twinx()
    self.axes2.autoscale_view(False,True,True)
    self.axes_format_coord = self.axes.format_coord
    self.axes2_format_coord = self.axes2.format_coord
    self.axes.format_coord = self.format_coord
    self.axes2.format_coord = self.format_coord

    self.grid = LineStyle()
    self.grid2 = LineStyle()

    self._x_label_fontsize = 10
    self._y1_label_fontsize = 10
    self._y2_label_fontsize = 10

    self.data = None
    self.computed_data = {}
    self.axes_plots = OrderedDict()
    self.axes2_plots = OrderedDict()

    self.color = 0
    self.Ncolor = 12
    # self.colors = ['r', 'g', 'b', 'y', 'k', 'cyan', 'magenta', 'orange']
    cm = matplotlib.cm.Set1
    cm2rgb = (np.array(cm(x)[0:3]) for x in np.linspace(0, 1, self.Ncolor))
    self.colors = ['#%02x%02x%02x' % tuple(255 * rgb) for rgb in cm2rgb]

    self.x_data = 't'

  def _drawGrid(self):
    def draw(axes, style):
      axes.grid(color = style.color, linestyle = style.linestyle, linewidth = style.linewidth, visible = style.visible, which = 'both')
      axes.set_axisbelow(True)
    if len(self.axes_plots) > 0:
      draw(self.axes, self.grid)
    if len(self.axes2_plots) > 0:
      draw(self.axes2, self.grid2)

  def draw(self, y1_limits = None, y2_limits = None):
    def fix_axes_limits(axes, axes2):
      point = (axes2.dataLim.get_points()[1] + axes2.dataLim.get_points()[0])/2
      plt, = axes.plot([point[0]], [point[1]], visible = False)
      plt.remove()
      del plt
      axes.relim()
    if len(self.axes_plots) == 0 and len(self.axes2_plots) != 0:
      fix_axes_limits(self.axes, self.axes2)
    if len(self.axes2_plots) == 0 and len(self.axes_plots) != 0:
      fix_axes_limits(self.axes2, self.axes)
    def set_axes_limits(axes, min_x = float("inf"), max_x = float("-inf"), ylim = None):
      dataLim = axes.dataLim.get_points()
      x_range = (dataLim[1][0] - dataLim[0][0])/2
      y_range = (dataLim[1][1] - dataLim[0][1])/2
      x_min = min(min_x, dataLim[0][0] - x_range*0.05)
      x_max = max(max_x, dataLim[1][0] + x_range*0.05)
      axes.set_xlim([x_min, x_max])
      if ylim is None:
        axes.set_ylim([dataLim[0][1] - y_range*0.05, dataLim[1][1] + y_range*0.05])
      else:
        axes.set_ylim(ylim)
      return x_min, x_max
    min_x, max_x = set_axes_limits(self.axes, ylim = y1_limits)
    set_axes_limits(self.axes2, min_x, max_x, ylim = y2_limits)
    self._legend_left()
    self._legend_right()
    self._drawGrid()
    top_offset = 0.9
    top_legend_rows = math.ceil(len(self.axes_plots.keys()) / 3.)
    if top_legend_rows > 3:
      top_offset = top_offset - 0.015 * (top_legend_rows - 3)
    bottom_offset = 0.1
    bottom_legend_rows = math.ceil(len(self.axes2_plots.keys()) / 3.)
    if bottom_legend_rows > 3:
      bottom_offset = bottom_offset + 0.015 * (bottom_legend_rows - 3)
    self.fig.subplots_adjust(top = top_offset, bottom = bottom_offset)

  def setData(self, data):
    self.data = data

  def show(self):
    self.fig.show()

  def title(self, title = None):
    if title is None:
      if self.fig._suptitle is None:
        return ""
      return self.fig._suptitle.get_text()
    else:
      self.fig.suptitle(title)

  def title_fontsize(self, fontsize = None):
    if fontsize is None:
      if self.fig._suptitle is None:
        return 12
      return self.fig._suptitle.get_fontsize()
    else:
      self.fig.suptitle(self.title(), fontsize = fontsize)

  def x_label(self, label = None):
    if label is None:
      return self.axes.get_xlabel()
    self.axes.set_xlabel(label)

  def x_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._x_label_fontsize
    else:
      self._x_label_fontsize = fontsize
      self.axes.set_xlabel(self.x_label(), fontsize = self._x_label_fontsize)

  def y1_label(self, label = None):
    if label is None:
      return self.axes.get_ylabel()
    self.axes.set_ylabel(label)

  def y1_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._y1_label_fontsize
    else:
      self._y1_label_fontsize = fontsize
      self.axes.set_ylabel(self.y1_label(), fontsize = self._y1_label_fontsize)

  def y2_label(self, label = None):
    if label is None:
      return self.axes2.get_ylabel()
    self.axes2.set_ylabel(label)

  def y2_label_fontsize(self, fontsize = None):
    if fontsize is None:
      return self._y2_label_fontsize
    else:
      self._y2_label_fontsize = fontsize
      self.axes2.set_ylabel(self.y2_label(), fontsize = self._y2_label_fontsize)

  def _next_color(self):
    self.color += 1
    return self.colors[ (self.color - 1) % self.Ncolor ]

  def _legend_left(self):
    self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=3, mode="expand", borderaxespad=0.5, fontsize = 10.0)

  def _legend_right(self):
    top_anchor = -0.125
    if len(self.x_label()):
      top_anchor = -0.175
    self.axes2.legend(bbox_to_anchor=(0., top_anchor, 1., .102), loc=2, ncol=3, mode="expand", borderaxespad=0.5, fontsize = 10.0)

  def _plot(self, axe, update_legend_fn, x, y, y_label, style = None):
    if style is None:
      plt = axe.plot(x, y, label = y_label, color = self._next_color())
    else:
      plt = axe.plot(x, y, label = y_label, color = style.color, linestyle = style.linestyle, linewidth = style.linewidth)
    update_legend_fn()
    return plt[0]

  def add_plot_left(self, x, y, y_label, style = None):
    if y_label in self.axes_plots:
      return False
    self.axes_plots[y] = self._plot(self.axes, self._legend_left, self.data[x], self.data[y], y_label, style)
    return True

  def add_plot_right(self, x, y, y_label, style = None):
    if y_label in self.axes2_plots:
      return False
    self.axes2_plots[y] = self._plot(self.axes2, self._legend_right, self.data[x], self.data[y], y_label, style)
    return True

  def _add_diff_plot(self, axes, legend, x, y, y_label):
    dt = self.data[x][1] - self.data[x][0]
    return self._plot(axes, legend, self.data[x][1:], np.diff(self.data[y])/dt, y_label)

  def add_diff_plot_left(self, x, y, y_label):
    if y_label in self.axes_plots:
      return False
    self.axes_plots[y_label] = self._add_diff_plot(self.axes, self._legend_left, x, y, y_label)
    return True
  def add_diff_plot_right(self, x, y, y_label):
    if y_label in self.axes2_plots:
      return False
    self.axes2_plots[y_label] = self._add_diff_plot(self.axes2, self._legend_right, x, y, y_label)
    return True

  def _add_roll_plot(self, axes, legend, x_label, base):
    if "{}_qw".format(base) in self.data.keys():
      fmt = "q"
    else:
      fmt = ""
    keys = [ "{}_{}{}".format(base, fmt, ax) for ax in ["w", "x", "y", "z"] ]
    qw, qx, qy, qz = [ self.data[k] for k in keys ]
    rpys = [ rpyFromQuat([w, x, y, z]) for w, x, y, z in zip(qw, qx, qy, qz) ]
    r = [ rpy[0] for rpy in rpys ]
    return self._plot(axes, legend, self.data[x_label], r, "{}_roll".format(base))

  def add_roll_plot_left(self, x, y):
    if "{}_roll".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_roll".format(y)] = self._add_roll_plot(self.axes, self._legend_left, x, y)
    return True

  def add_roll_plot_right(self, x, y):
    if "{}_roll".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_roll".format(y)] = self._add_roll_plot(self.axes2, self._legend_right, x, y)
    return True

  def _add_pitch_plot(self, axes, legend, x_label, base):
    if "{}_qw".format(base) in self.data.keys():
      fmt = "q"
    else:
      fmt = ""
    keys = [ "{}_{}{}".format(base, fmt, ax) for ax in ["w", "x", "y", "z"] ]
    qw, qx, qy, qz = [ self.data[k] for k in keys ]
    rpys = [ rpyFromQuat([w, x, y, z]) for w, x, y, z in zip(qw, qx, qy, qz) ]
    p = [ rpy[1] for rpy in rpys ]
    return self._plot(axes, legend, self.data[x_label], p, "{}_pitch".format(base))

  def add_pitch_plot_left(self, x, y):
    if "{}_pitch".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_pitch".format(y)] = self._add_pitch_plot(self.axes, self._legend_left, x, y)
    return True

  def add_pitch_plot_right(self, x, y):
    if "{}_pitch".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_pitch".format(y)] = self._add_pitch_plot(self.axes2, self._legend_right, x, y)
    return True

  def _add_yaw_plot(self, axes, legend, x_label, base):
    if "{}_qw".format(base) in self.data.keys():
      fmt = "q"
    else:
      fmt = ""
    keys = [ "{}_{}{}".format(base, fmt, ax) for ax in ["w", "x", "y", "z"] ]
    qw, qx, qy, qz = [ self.data[k] for k in keys ]
    rpys = [ rpyFromQuat([w, x, y, z]) for w, x, y, z in zip(qw, qx, qy, qz) ]
    y = [ rpy[2] for rpy in rpys ]
    return self._plot(axes, legend, self.data[x_label], y, "{}_yaw".format(base))

  def add_yaw_plot_left(self, x, y):
    if "{}_yaw".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_yaw".format(y)] = self._add_yaw_plot(self.axes, self._legend_left, x, y)
    return True

  def add_yaw_plot_right(self, x, y):
    if "{}_yaw".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_yaw".format(y)] = self._add_yaw_plot(self.axes2, self._legend_right, x, y)
    return True

  def add_rpy_plot_left(self, x, y):
    has_roll = "{}_roll".format(y) in self.axes2_plots
    has_pitch = "{}_pitch".format(y) in self.axes2_plots
    has_yaw = "{}_yaw".format(y) in self.axes2_plots
    if has_roll and has_pitch and has_yaw:
        return False
    if not has_roll:
      self.add_roll_plot_left(x, y)
    if not has_pitch:
      self.add_pitch_plot_left(x, y)
    if not has_yaw:
      self.add_yaw_plot_left(x, y)
    return True

  def add_rpy_plot_right(self, x, y):
    has_roll = "{}_roll".format(y) in self.axes2_plots
    has_pitch = "{}_pitch".format(y) in self.axes2_plots
    has_yaw = "{}_yaw".format(y) in self.axes2_plots
    if has_roll and has_pitch and has_yaw:
        return False
    if not has_roll:
      self.add_roll_plot_right(x, y)
    if not has_pitch:
      self.add_pitch_plot_right(x, y)
    if not has_yaw:
      self.add_yaw_plot_right(x, y)
    return True

  def remove_plot_left(self, y_label):
    if y_label not in self.axes_plots:
      return
    self.axes_plots[y_label].remove()
    del self.axes_plots[y_label]
    if len(self.axes_plots):
      self.axes.relim()
      self._legend_left()
    else:
      self.axes.clear()
    if len(self.axes_plots) == 0 and len(self.axes2_plots) == 0:
      self.color = 0

  def remove_plot_right(self, y_label):
    if y_label not in self.axes2_plots:
      return
    self.axes2_plots[y_label].remove()
    del self.axes2_plots[y_label]
    if len(self.axes2_plots):
      self.axes2.relim()
      self._legend_right()
    else:
      self.axes2.clear()
    if len(self.axes_plots) == 0 and len(self.axes2_plots) == 0:
      self.color = 0

  def format_coord(self, x, y):
    display_coord = self.axes2.transData.transform((x,y))
    inv = self.axes.transData.inverted()
    ax_coord = inv.transform(display_coord)
    if len(self.axes2_plots) and len(self.axes_plots):
      return "x: {:.3f}    y1: {:.3f}    y2: {:.3f}".format(x, ax_coord[1], y)
    elif len(self.axes_plots):
      return "x: {:.3f}    y1: {:.3f}".format(x, ax_coord[1])
    elif len(self.axes2_plots):
      return "x: {:.3f}    y2: {:.3f}".format(x, y)
    else:
      return "x: {:.3f}".format(x)

  def clear_all(self):
    self.color = 0
    self.axes_plots = {}
    self.axes2_plots = {}
    self.axes.clear()
    self.axes2.clear()

  def _style(self, plots, y, styleIn = None):
    if not y in plots:
      raise KeyError("No plot named {}".format(y))
    plt = plots[y]
    if styleIn is None:
      return LineStyle(plt.get_color(), plt.get_linestyle(), plt.get_linewidth(), label = plt.get_label())
    else:
      plt.set_color(styleIn.color)
      plt.set_linestyle(styleIn.linestyle)
      plt.set_linewidth(styleIn.linewidth)
      if len(styleIn.label):
        plt.set_label(styleIn.label)

  def style_left(self, y, styleIn = None):
    return self._style(self.axes_plots, y, styleIn)

  def style_right(self, y, styleIn = None):
    return self._style(self.axes2_plots, y, styleIn)

class SimpleAxesDialog(QtGui.QDialog):
  def __init__(self, parent):
    QtGui.QDialog.__init__(self, parent)
    self.setWindowTitle('Edit axes limits')
    self.setModal(True)
    self.layout = QtGui.QGridLayout(self)
    self.layout.addWidget(QtGui.QLabel("Min"), 0, 1)
    self.layout.addWidget(QtGui.QLabel("Max"), 0, 2)
    self.layout.addWidget(QtGui.QLabel("Y1"), 1, 0)
    y1_limits = parent.y1_limits
    if y1_limits is None:
      y1_limits = parent.axes.get_ylim()
    self.y1_min = QtGui.QLineEdit(str(y1_limits[0]))
    self.y1_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y1_min, 1, 1)
    self.y1_max = QtGui.QLineEdit(str(y1_limits[1]))
    self.y1_max.setValidator(QtGui.QDoubleValidator())
    self.y1_init = [float(self.y1_min.text()), float(self.y1_max.text())]
    self.layout.addWidget(self.y1_max, 1, 2)
    self.layout.addWidget(QtGui.QLabel("Y2"), 2, 0)
    y2_limits = parent.y2_limits
    if y2_limits is None:
      y2_limits = parent.axes2.get_ylim()
    self.y2_min = QtGui.QLineEdit(str(y2_limits[0]))
    self.y2_min.setValidator(QtGui.QDoubleValidator())
    self.layout.addWidget(self.y2_min, 2, 1)
    self.y2_max = QtGui.QLineEdit(str(y2_limits[1]))
    self.y2_max.setValidator(QtGui.QDoubleValidator())
    self.y2_init = [float(self.y2_min.text()), float(self.y2_max.text())]
    self.layout.addWidget(self.y2_max, 2, 2)

    confirmLayout = QtGui.QHBoxLayout()
    okButton = QtGui.QPushButton("Ok", self)
    confirmLayout.addWidget(okButton)
    okButton.clicked.connect(self.accept)
    applyButton = QtGui.QPushButton("Apply", self)
    confirmLayout.addWidget(applyButton)
    applyButton.clicked.connect(self.apply)
    cancelButton = QtGui.QPushButton("Cancel", self)
    confirmLayout.addWidget(cancelButton)
    cancelButton.clicked.connect(self.reject)
    self.layout.addLayout(confirmLayout, 3, 0, 1, 3)

  def apply(self):
    changed = False
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
    QtGui.QDialog.accept(self)
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
    self.y1_locked = QtGui.QPushButton(u"🔒 Y1", self)
    self.y1_locked.setCheckable(True)
    layout.addWidget(self.y1_locked)
    self.y1_locked.toggled.connect(self.y1_locked_changed)
    self.y1_limits = None

    self.y2_locked = QtGui.QPushButton(u"🔒 Y2", self)
    self.y2_locked.setCheckable(True)
    layout.addWidget(self.y2_locked)
    self.y2_locked.toggled.connect(self.y2_locked_changed)
    self.y2_limits = None

  def axesDialog(self):
    SimpleAxesDialog(self).exec_()

  def on_draw(self, event):
    if self.y1_limits is not None:
      self.y1_limits = self.axes.get_ylim()
    if self.y2_limits is not None:
      self.y2_limits = self.axes2.get_ylim()

  def draw(self):
    PlotFigure.draw(self, self.y1_limits, self.y2_limits)
    self.canvas.draw()

  def _y_lock_changed(self, name, cbox, axes):
    if cbox.isChecked():
      cbox.setText(u"🔓 {}".format(name))
      return axes.get_ylim()
    else:
      cbox.setText(u"🔒{}".format(name))
      return None

  def y1_locked_changed(self, status):
    self.y1_limits = self._y_lock_changed("Y1", self.y1_locked, self.axes)
    if self.y1_limits is None:
      self.draw()

  def y2_locked_changed(self, status):
    self.y2_limits = self._y_lock_changed("Y2", self.y2_locked, self.axes2)
    if self.y2_limits is None:
      self.draw()
