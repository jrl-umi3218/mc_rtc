#!/usr/bin/env python2

from PySide import QtCore, QtGui

from PySide.QtGui import QWidget, QVBoxLayout

import copy
import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')
matplotlib.rcParams['backend.qt4'] = 'PySide'

from matplotlib.figure import Figure
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas,\
                                               NavigationToolbar2QT as NavigationToolbar

from math import asin, atan2

def rpy_from_quat(quat):
    """
    Roll-pitch-yaw angles of a quaternion.

    Parameters
    ----------
    quat : (4,) array
        Quaternion in `[w x y z]` format.

    Returns
    -------
    rpy : (3,) array
        Array of roll-pitch-yaw angles, in [rad].

    Notes
    -----
    Roll-pitch-yaw are Euler angles corresponding to the sequence (1, 2, 3).
    """
    roll = atan2(
        2 * quat[2] * quat[3] + 2 * quat[0] * quat[1],
        quat[3] ** 2 - quat[2] ** 2 - quat[1] ** 2 + quat[0] ** 2)
    pitch = -asin(
        2 * quat[1] * quat[3] - 2 * quat[0] * quat[2])
    yaw = atan2(
        2 * quat[1] * quat[2] + 2 * quat[0] * quat[3],
        quat[1] ** 2 + quat[0] ** 2 - quat[3] ** 2 - quat[2] ** 2)
    return [roll, pitch, yaw]

class PlotCanvasWithToolbar(QWidget):
  def __init__(self, parent = None):
    super(PlotCanvasWithToolbar, self).__init__(parent)

    self.fig = Figure(figsize=(5, 4), dpi=100)
    self.axes = self.fig.add_subplot(111)
    self.axes.autoscale(enable = True, axis = 'both', tight = False)
    self.axes2 = self.axes.twinx()
    self.canvas = FigureCanvas(self.fig)
    self.toolbar = NavigationToolbar(self.canvas, self)
    self.axes_format_coord = self.axes.format_coord
    self.axes2_format_coord = self.axes2.format_coord
    self.axes.format_coord = self.format_coord
    self.axes2.format_coord = self.format_coord

    vbox = QVBoxLayout(self)
    vbox.addWidget(self.canvas)
    vbox.addWidget(self.toolbar)
    self.setLayout(vbox)

    self.data = None
    self.computed_data = {}
    self.axes_plots = {}
    self.axes2_plots = {}

    self.color = 0
    self.Ncolor = 12
    self.color_space = np.linspace(0, 1, self.Ncolor)
    self.cm = matplotlib.cm.Set1

    self.x_data = 't'

  def draw(self):
    self.canvas.draw()

  def setData(self, data):
    self.data = data

  def title(self, title):
    self.fig.suptitle(title)

  def y1_label(self, label):
    self.axes.set_ylabel(label)

  def y2_label(self, label):
    self.axes2.set_ylabel(label)

  def _next_color(self):
    self.color += 1
    return self.cm(self.color_space[ (self.color - 1) % self.Ncolor ])

  def _legend_left(self):
    self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=3, mode="expand", borderaxespad=0.5, fontsize = 10.0)

  def _legend_right(self):
    self.axes2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=3, mode="expand", borderaxespad=-0.5, fontsize = 10.0)

  def _plot(self, axe, update_legend_fn, x, y, y_label):
    plt = axe.plot(x, y, label = y_label, color = self._next_color())
    update_legend_fn()
    return plt[0]

  def add_plot_left(self, x, y, y_label):
    self.axes_plots[y_label] = self._plot(self.axes, self._legend_left, self.data[x], self.data[y], y_label)

  def add_plot_right(self, x, y, y_label):
    self.axes2_plots[y_label] = self._plot(self.axes2, self._legend_right, self.data[x], self.data[y], y_label)

  def _add_diff_plot(self, axes, legend, x, y, y_label):
    dt = self.data[x][1] - self.data[x][0]
    return self._plot(axes, legend, self.data[x][1:], np.diff(self.data[y])/dt, y_label)

  def add_diff_plot_left(self, x, y, y_label):
    self.axes_plots[y_label] = self._add_diff_plot(self.axes, self._legend_left, x, y, y_label)
  def add_diff_plot_right(self, x, y, y_label):
    self.axes2_plots[y_label] = self._add_diff_plot(self.axes2, self._legend_right, x, y, y_label)

  def _add_rpy_plot(self, axes, legend, x_label, base):
    if "{}_qw".format(base) in self.data.keys():
      fmt = "q"
    else:
      fmt = ""
    keys = [ "{}_{}{}".format(base, fmt, ax) for ax in ["w", "x", "y", "z"] ]
    qw, qx, qy, qz = [ self.data[k] for k in keys ]
    rpys = [ rpy_from_quat([w, x, y, z]) for w, x, y, z in zip(qw, qx, qy, qz) ]
    r = [ rpy[0] for rpy in rpys ]
    p = [ rpy[1] for rpy in rpys ]
    y = [ rpy[2] for rpy in rpys ]
    return self._plot(axes, legend, self.data[x_label], r, "{}_r".format(base)),\
           self._plot(axes, legend, self.data[x_label], p, "{}_p".format(base)),\
           self._plot(axes, legend, self.data[x_label], y, "{}_y".format(base))

  def add_rpy_plot_left(self, x, y):
    self.axes_plots["{}_r".format(y)],\
    self.axes_plots["{}_p".format(y)],\
    self.axes_plots["{}_y".format(y)] = self._add_rpy_plot(self.axes, self._legend_left, x, y)

  def add_rpy_plot_right(self, x, y):
    self.axes2_plots["{}_r".format(y)],\
    self.axes2_plots["{}_p".format(y)],\
    self.axes2_plots["{}_y".format(y)] = self._add_rpy_plot(self.axes2.vb, self._legend_right, x, y)

  def remove_plot_left(self, y_label):
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
    if len(self.axes2_plots):
      return "x: {:.3f}, y1: {:.3f}, y2: {:.3f}".format(x, ax_coord[1], y)
    else:
      return "x: {:.3f}, y1: {:.3f}".format(x, ax_coord[1])

  def clear_all(self):
    self.color = 0
    self.axes_plots = {}
    self.axes2_plots = {}
    self.axes.clear()
    self.axes2.clear()
    self.canvas.clear()
