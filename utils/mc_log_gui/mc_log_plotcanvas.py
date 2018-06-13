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
    self.col = 0
    self.row = 0
  def addItem(self, item, name):
    label = pg.LabelItem(name)
    if isinstance(item, ItemSample):
        sample = item
    else:
        sample = ItemSample(item)
    if self.col % 14 == 0:
      self.col = 0
      self.row += 1
    self.items.append((sample, label))
    self.layout.addItem(sample, self.row, self.col)
    self.layout.addItem(label, self.row, self.col + 1)
    self.col += 2
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

class PlotCanvasWithToolbar(QWidget):
    def __init__(self, parent=None):
      QWidget.__init__(self, parent)
      self.canvas = PlotCanvas(self)

      vbox = QVBoxLayout()
      vbox.addWidget(self.canvas)
      self.setLayout(vbox)

    def plot(self, data, x, y, ydiff, y_label, ydiff_label):
      self.canvas.plot(data, x, y, ydiff, y_label, ydiff_label)

    def clear(self):
      self.canvas.clear()

    def title(self, title):
      self.canvas.title(title)

    def y1_label(self, label):
      self.canvas.set_y1_label(label)

    def y2_label(self, label):
      self.canvas.set_y2_label(label)

class PlotCanvas(pg.GraphicsLayoutWidget):
    def __init__(self, parent = None):
      super(PlotCanvas, self).__init__(parent)
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

    def updateViews(self):
      self.axes2.setGeometry(self.axes.vb.sceneBoundingRect())
      self.axes2.linkedViewChanged(self.axes.vb, self.axes2.XAxis)

    def title(self, title):
      self.axes.setTitle(title)

    def set_y1_label(self, label):
      self.axes.setLabels(left = label)

    def set_y2_label(self, label):
      self.axes.setLabels(right = label)

    def next_color(self):
      self.color += 1
      return self.color - 1

    def plot(self, data, x, y_, ydiff, y_labels, ydiff_labels):
      assert(len(y_) == len(y_labels))
      assert(len(ydiff) == len(ydiff_labels))
      self.clear_axes()
      dt = data[x][1] - data[x][0]

      for y, y_label in zip(y_[0], y_labels[0]):
        plt = self.axes.plot(data[x], data[y], name = y_label, pen = self.next_color(), autoDownsample = True)
        self.axes_legend.addItem(plt, y_label)
      for y, y_label in zip(ydiff[0], ydiff_labels[0]):
        plt = self.axes.plot(data[x][1:], np.diff(data[y])/dt, name = y_label, pen = self.next_color(), autoDownsample = True)
        self.axes_legend.addItem(plt, y_label)

      for y, y_label in zip(y_[1], y_labels[1]):
        plt = pg.PlotCurveItem(data[x], data[y], name = y_label, pen = self.next_color(), autoDownsample = True)
        self.axes2.addItem(plt)
        self.axes2_legend.addItem(plt, y_label)
      for y, y_label in zip(ydiff[1], ydiff_labels[1]):
        plt = pg.PlotCurveItem(data[x][1:], np.diff(data[y])/dt, name = y_label, pen = self.next_color(), autoDownsample = True)
        self.axes2.addItem(plt)
        self.axes2_legend.addItem(plt, y_label)

    def clear_axes(self):
      self.color = 0
      self.axes_legend.close()
      self.axes_legend = HorizontalLegendItem()
      self.axes_legend.setParentItem(self.axes_legend_box)
      self.axes_legend.anchor((0,0), (0,0))
      self.axes_legend.updateSize()
      self.axes2_legend.close()
      self.axes2_legend = HorizontalLegendItem()
      self.axes2_legend.setParentItem(self.axes2_legend_box)
      self.axes2_legend.anchor((0,0), (0,0))
      self.axes2_legend.updateSize()
      self.axes.clear()
      self.axes2.clear()
