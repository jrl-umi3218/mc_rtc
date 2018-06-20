#!/usr/bin/env python2

from PySide import QtCore, QtGui

from PySide.QtGui import QWidget, QVBoxLayout

import itertools
import numpy as np
import pyqtgraph as pg

## Switch to using white background and black foreground
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

# Note: directly taken from pyqtgraph source
class ItemSample(pg.GraphicsWidget):
    """ Class responsible for drawing a single item in a LegendItem (sans label).

    This may be subclassed to draw custom graphics in a Legend.
    """
    ## Todo: make this more generic; let each item decide how it should be represented.
    def __init__(self, item):
        pg.GraphicsWidget.__init__(self)
        self.item = item

    def boundingRect(self):
        return QtCore.QRectF(0, 0, 20, 20)

    def paint(self, p, *args):
        #p.setRenderHint(p.Antialiasing)  # only if the data is antialiased.
        opts = self.item.opts

        if opts.get('fillLevel',None) is not None and opts.get('fillBrush',None) is not None:
            p.setBrush(pg.functions.mkBrush(opts['fillBrush']))
            p.setPen(pg.functions.mkPen(None))
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(2,18), QtCore.QPointF(18,2), QtCore.QPointF(18,18)]))

        if not isinstance(self.item, pg.ScatterPlotItem):
            p.setPen(pg.functions.mkPen(opts['pen'], width = 5))
            p.drawLine(2, 18, 18, 2)

        symbol = opts.get('symbol', None)
        if symbol is not None:
            if isinstance(self.item, pg.PlotDataItem):
                opts = self.item.scatter.opts

            pen = pg.functions.mkPen(opts['pen'])
            brush = pg.functions.mkBrush(opts['brush'])
            size = opts['size']

            p.translate(10,10)
            path = drawSymbol(p, symbol, size, pen, brush)

class HorizontalLegendItem(pg.LegendItem):
  def __init__(self, **kwds):
    super(HorizontalLegendItem, self).__init__(**kwds)
    self.setLayout(self.layout)
    self.col = 0
    self.row = 0
  def addItem(self, item, name):
    label = pg.LabelItem(name)
    if isinstance(item, ItemSample):
        sample = item
    else:
        sample = ItemSample(item)
    self.items.append((sample, label))
    self.layout.addItem(self.items[-1][0], self.row, self.col)
    self.layout.addItem(self.items[-1][1], self.row, self.col + 1)
    self.col += 2
    if self.col % 14 == 0:
      self.col = 0
      self.row += 1
    self.updateSize()
  def removeItem(self, name):
    while self.layout.count():
      self.layout.removeAt(0)
    for item in self.items:
      if item[1].text == name:
        item[0].deleteLater()
        item[1].deleteLater()
        self.items.remove(item)
        break
    self.col = 0
    self.row = 0
    for sample, label in self.items:
      self.layout.addItem(sample, self.row, self.col)
      self.layout.addItem(sample, self.row, self.col + 1)
      self.col += 2
      if self.col % 14 == 0:
        self.col = 0
        self.row += 1
    self.updateSize()
  def updateSize(self):
    size = self.boundingRect()
    height = 50
    if len(self.items) != 0:
      size.setHeight((self.row + 1)*height + 5)
    else:
      size.setHeight(0)
    self.setGeometry(size)
    self.parentItem().setMaximumHeight(self.boundingRect().size().height())

class PlotCanvasWithToolbar(pg.GraphicsLayoutWidget):
    def __init__(self, parent = None):
      super(PlotCanvasWithToolbar, self).__init__(parent)

      self.data = None
      self.computed_data = {}
      self.axes_plots = {}
      self.axes2_plots = {}

      self.color = 0

      self.axes_legend_box = self.addViewBox(col = 0, row = 0, enableMouse = False)
      self.axes_legend_box.setMaximumHeight(0)
      self.axes_legend = HorizontalLegendItem()
      self.axes_legend.setParentItem(self.axes_legend_box)
      self.axes_legend.anchor((0,0), (0,0))

      self.axes = self.addPlot(col = 0, row = 1)
      self.axes.vb.setMouseMode(self.axes.vb.RectMode)
      self.axes.vb.setAutoVisible(y = True)

      self.axes2_legend_box = self.addViewBox(col = 0, row = 2, enableMouse = False)
      self.axes2_legend_box.setMaximumHeight(0)
      self.axes2_legend = HorizontalLegendItem()
      self.axes2_legend.setParentItem(self.axes2_legend_box)
      self.axes2_legend.anchor((0,0), (0,0))

      self.axes2 = pg.ViewBox()
      self.axes.showAxis('right')
      self.axes.scene().addItem(self.axes2)
      self.axes.getAxis('right').linkToView(self.axes2)
      self.axes2.setXLink(self.axes)
      self.axes2.setAutoVisible(y = True)

      self.setSizePolicy(QtGui.QSizePolicy.Expanding,
                         QtGui.QSizePolicy.Expanding)
      self.updateViews()
      self.axes.vb.sigResized.connect(self.updateViews)
      self.axes2_item = []

    def setData(self, data):
      self.data = data

    def updateViews(self):
      self.axes2.setGeometry(self.axes.vb.sceneBoundingRect())
      self.axes2.linkedViewChanged(self.axes.vb, self.axes2.XAxis)

    def title(self, title):
      self.axes.setTitle(title)

    def y1_label(self, label):
      self.axes.setLabels(left = label)

    def y2_label(self, label):
      self.axes.setLabels(right = label)

    def _next_color(self):
      self.color += 1
      return self.color - 1

    def _plot(self, axe, legend, x, y, y_label):
      plt = pg.PlotCurveItem(x, y, name = y_label, pen = self._next_color(), autoDownsample = True)
      axe.addItem(plt)
      legend.addItem(plt, y_label)
      return plt

    def add_plot_left(self, x, y, y_label):
      self.axes_plots[y_label] = self._plot(self.axes.vb, self.axes_legend, self.data[x], self.data[y], y_label)

    def add_plot_right(self, x, y, y_label):
      self.axes2_plots[y_label] = self._plot(self.axes2, self.axes2_legend, self.data[x], self.data[y], y_label)

    def _add_diff_plot(self, axes, legend, x, y, y_label):
      dt = self.data[x][1] - self.data[x][0]
      return self._plot(axes, legend, self.data[x][1:], np.diff(self.data[y])/dt, y_label)

    def add_diff_plot_left(self, x, y, y_label):
      self.axes_plots[y_label] = self._add_diff_plot(self.axes.vb, self.axes_legend, x, y, y_label)
    def add_diff_plot_right(self, x, y, y_label):
      self.axes2_plots[y_label] = self._add_diff_plot(self.axes2, self.axes2_legend, x, y, y_label)

    def remove_plot_left(self, y_label):
      self.axes.vb.removeItem(self.axes_plots[y_label])
      del self.axes_plots[y_label]
      self.axes_legend.removeItem(y_label)
    def remove_plot_right(self, y_label):
      self.axes2.removeItem(self.axes_plot[y_label])
      del self.axes2_plots[y_label]
      self.axes2_legend.removeItem(y_label)

    def clear_all(self):
      self.color = 0
      self.axes_plots = {}
      self.axes_legend.close()
      self.axes_legend = HorizontalLegendItem()
      self.axes_legend.setParentItem(self.axes_legend_box)
      self.axes_legend.anchor((0,0), (0,0))
      self.axes_legend.updateSize()
      self.axes_plots = {}
      self.axes2_legend.close()
      self.axes2_legend = HorizontalLegendItem()
      self.axes2_legend.setParentItem(self.axes2_legend_box)
      self.axes2_legend.anchor((0,0), (0,0))
      self.axes2_legend.updateSize()
      self.axes.clear()
      self.axes2.clear()
