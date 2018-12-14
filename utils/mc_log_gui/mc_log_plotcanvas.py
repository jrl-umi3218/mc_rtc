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


class PlotCanvasWithToolbar(QWidget):
  def __init__(self, parent = None):
    super(PlotCanvasWithToolbar, self).__init__(parent)

    self.fig = Figure(figsize=(5, 4), dpi=100)
    self.axes = self.fig.add_subplot(111)
    self.axes.autoscale(enable = True, axis = 'both', tight = False)
    self.axes.autoscale_view(False,True,True)
    self.axes2 = self.axes.twinx()
    self.axes2.autoscale_view(False,True,True)
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
    # self.colors = ['r', 'g', 'b', 'y', 'k', 'cyan', 'magenta', 'orange']
    cm = matplotlib.cm.Set1
    cm2rgb = (np.array(cm(x)[0:3]) for x in np.linspace(0, 1, self.Ncolor))
    self.colors = ['#%02x%02x%02x' % tuple(255 * rgb) for rgb in cm2rgb]

    self.x_data = 't'

  def draw(self):
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
    def set_axes_limits(axes, min_x = float("inf"), max_x = float("-inf")):
      dataLim = axes.dataLim.get_points()
      x_range = (dataLim[1][0] - dataLim[0][0])/2
      y_range = (dataLim[1][1] - dataLim[0][1])/2
      x_min = min(min_x, dataLim[0][0] - x_range*0.05)
      x_max = max(max_x, dataLim[1][0] + x_range*0.05)
      axes.set_xlim([x_min, x_max])
      axes.set_ylim([dataLim[0][1] - y_range*0.05, dataLim[1][1] + y_range*0.05])
      return x_min, x_max
    min_x, max_x = set_axes_limits(self.axes)
    set_axes_limits(self.axes2, min_x, max_x)
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
    return self.colors[ (self.color - 1) % self.Ncolor ]

  def _legend_left(self):
    self.axes.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=3, mode="expand", borderaxespad=0.5, fontsize = 10.0)

  def _legend_right(self):
    self.axes2.legend(bbox_to_anchor=(0., -.1, 1., -1.02), loc=3, ncol=3, mode="expand", borderaxespad=-0.5, fontsize = 10.0)

  def _plot(self, axe, update_legend_fn, x, y, y_label):
    plt = axe.plot(x, y, label = y_label, color = self._next_color())
    update_legend_fn()
    return plt[0]

  def add_plot_left(self, x, y, y_label):
    if y_label in self.axes_plots:
      return False
    self.axes_plots[y] = self._plot(self.axes, self._legend_left, self.data[x], self.data[y], y_label)
    return True

  def add_plot_right(self, x, y, y_label):
    if y_label in self.axes2_plots:
      return False
    self.axes2_plots[y] = self._plot(self.axes2, self._legend_right, self.data[x], self.data[y], y_label)
    return True

  def _add_diff_plot(self, axes, legend, x, y, y_label):
    dt = self.data[x][1] - self.data[x][0]
    return self._plot(axes, legend, self.data[x][1:], np.diff(self.data[y])/dt, y_label)

  def add_diff_plot_left(self, x, y, y_label):
    if y_label in self.axes_plots:
      return False
    self.axes_plots[y] = self._add_diff_plot(self.axes, self._legend_left, x, y, y_label)
    return True
  def add_diff_plot_right(self, x, y, y_label):
    if y_label in self.axes2_plots:
      return False
    self.axes2_plots[y] = self._add_diff_plot(self.axes2, self._legend_right, x, y, y_label)
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
    return self._plot(axes, legend, self.data[x_label], r, "{}_r".format(base))

  def add_roll_plot_left(self, x, y):
    if "{}_r".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_r".format(y)] = self._add_roll_plot(self.axes, self._legend_left, x, y)
    return True

  def add_roll_plot_right(self, x, y):
    if "{}_r".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_r".format(y)] = self._add_roll_plot(self.axes2, self._legend_right, x, y)
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
    return self._plot(axes, legend, self.data[x_label], p, "{}_p".format(base))

  def add_pitch_plot_left(self, x, y):
    if "{}_p".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_p".format(y)] = self._add_pitch_plot(self.axes, self._legend_left, x, y)
    return True

  def add_pitch_plot_right(self, x, y):
    if "{}_p".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_p".format(y)] = self._add_pitch_plot(self.axes2, self._legend_right, x, y)
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
    return self._plot(axes, legend, self.data[x_label], y, "{}_y".format(base))

  def add_yaw_plot_left(self, x, y):
    if "{}_y".format(y) in self.axes_plots:
      return False
    self.axes_plots["{}_y".format(y)] = self._add_yaw_plot(self.axes, self._legend_left, x, y)
    return True

  def add_yaw_plot_right(self, x, y):
    if "{}_y".format(y) in self.axes2_plots:
      return False
    self.axes2_plots["{}_y".format(y)] = self._add_yaw_plot(self.axes2, self._legend_right, x, y)
    return True

  def add_rpy_plot_left(self, x, y):
    has_roll = "{}_r".format(y) in self.axes2_plots
    has_pitch = "{}_p".format(y) in self.axes2_plots
    has_yaw = "{}_y".format(y) in self.axes2_plots
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
    has_roll = "{}_r".format(y) in self.axes2_plots
    has_pitch = "{}_p".format(y) in self.axes2_plots
    has_yaw = "{}_y".format(y) in self.axes2_plots
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
