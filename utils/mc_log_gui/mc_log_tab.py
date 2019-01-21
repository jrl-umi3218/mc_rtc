# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

from mc_log_tab_ui import Ui_MCLogTab
from mc_log_types import LineStyle
from mc_log_plotcanvas import PlotFigure

from functools import partial

import copy
import re

class MCLogTreeWidgetItem(QtGui.QTreeWidgetItem):
  def __init__(self, parent, displayText, actualText, hasData):
    super(MCLogTreeWidgetItem, self).__init__(parent, [displayText])
    self._displayText = displayText
    self.originalText = displayText
    self.actualText = actualText
    self.hasData = hasData
  @property
  def displayText(self):
    return self._displayText
  @displayText.setter
  def displayText(self, value):
    self._displayText = value
    self.setText(0, self._displayText)

class TreeView(object):
  def __init__(self, name = None, parent = None, dataName = None):
    self.name = name
    if dataName is None:
      if parent is not None and parent.dataName is not None:
        self.dataName = parent.dataName + "_" + self.name
      else:
        self.dataName = self.name
    else:
      self.dataName = dataName
    self.hasData = False
    self.leafs = []
    self.parent = parent
    self.modelIdxs = []
    self.widgets = []
  def leaf(self, name):
    for l in self.leafs:
      if l.name == name:
        return l
    self.leafs.append(TreeView(name, self))
    return self.leafs[-1]
  def add(self, key):
    if len(key) == 0:
      self.hasData = True
      return
    self.leaf(key[0]).add(key[1:])
  def simplify(self):
    while len(self.leafs) == 1 and not(self.hasData):
      self.name = self.name + '_' + self.leafs[0].name
      self.dataName = self.leafs[0].dataName
      self.hasData = self.leafs[0].hasData
      self.leafs = self.leafs[0].leafs
      for l in self.leafs:
        l.parent = self
    for l in self.leafs:
      l.simplify()
  def update_y_selector(self, ySelector, parent, baseModelIdx = None):
    row = 0
    needExpand = False
    if all([l.name.isdigit() for l in self.leafs]):
      self.leafs.sort(lambda l,r: cmp(int(l.name), int(r.name)))
    for l in self.leafs:
      l.widgets.append(MCLogTreeWidgetItem(parent, l.name, l.dataName, l.hasData))
      if baseModelIdx is not None:
        l.modelIdxs.append(ySelector.model().index(row, 0, baseModelIdx))
      else:
        l.modelIdxs.append(ySelector.model().index(row, 0))
      l.update_y_selector(ySelector, l.widgets[-1], l.modelIdxs[-1])
      row += 1
  def select(self, name, ySelector, idx, fullName = ""):
    if name == fullName:
      selection = ySelector.selectionModel()
      selection.select(self.modelIdxs[idx], QtGui.QItemSelectionModel.Select)
      ySelector.setSelectionModel(selection)
      parent = self.parent
      while parent is not None and idx < len(parent.widgets):
        parent.widgets[idx].setExpanded(True)
        parent = parent.parent
    else:
      for l in self.leafs:
        if len(fullName):
          fName = fullName + "_" + l.name
        else:
          fName = l.name
        if name.startswith(fName):
          l.select(name, ySelector, idx, fName)
  def __print(self, indent):
    ret = "\n"
    if self.name is not None:
      ret = " "*indent + "| " + self.name + '\n'
    for l in self.leafs:
      ret += l.__print(indent + 1)
    return ret
  def __str__(self):
    return self.__print(-1)

class FilterRightClick(QtCore.QObject):
  def __init__(self, parent):
    super(FilterRightClick, self).__init__(parent)

  def eventFilter(self, obj, event):
    if event.type() == QtCore.QEvent.MouseButtonPress:
      if event.button() == QtCore.Qt.RightButton:
        return True
    return False

class SpecialPlot(object):
  def __init__(self, name, figure, idx, special_id):
    self.figure = figure
    self.idx = idx
    self.name = name
    self.id = special_id
    self.added = []
    if idx == 0:
      self.remove = self.figure.remove_plot_left
    else:
      self.remove = self.figure.remove_plot_right
    if special_id == "diff":
      self.__plot = self.__add_diff
    elif special_id == "rpy":
      self.__plot = self.__add_rpy
    elif special_id == "r":
      self.__plot = self.__add_roll
    elif special_id == "p":
      self.__plot = self.__add_pitch
    elif special_id == "y":
      self.__plot = self.__add_yaw
    else:
      print "Cannot handle this special plot:", special_id
    self.plot()
  def __add_diff(self):
    added = filter(lambda x: re.match("{}($|_.*$)".format(self.name), x) is not None, self.figure.data.keys())
    if self.idx == 0:
      add_fn = self.figure.add_diff_plot_left
    else:
      add_fn = self.figure.add_diff_plot_right
    for a in added:
      label = "{}_diff".format(a)
      if add_fn(self.figure.x_data, a, label):
        self.added.append(label)
  def __add_rpy(self):
    if self.idx == 0:
      add_fn = self.figure.add_rpy_plot_left
    else:
      add_fn = self.figure.add_rpy_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, s) for s in ["roll", "pitch", "yaw"] ]
  def __add_roll(self):
    if self.idx == 0:
      add_fn = self.figure.add_roll_plot_left
    else:
      add_fn = self.figure.add_roll_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "roll") ]
  def __add_pitch(self):
    if self.idx == 0:
      add_fn = self.figure.add_pitch_plot_left
    else:
      add_fn = self.figure.add_pitch_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "pitch") ]
  def __add_yaw(self):
    if self.idx == 0:
      add_fn = self.figure.add_yaw_plot_left
    else:
      add_fn = self.figure.add_yaw_plot_right
    if add_fn(self.figure.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, "yaw") ]
  def plot(self):
    self.__plot()

class RemoveSpecialPlotButton(SpecialPlot, QtGui.QPushButton):
  def __init__(self, name, logtab, idx, special_id):
    self.logtab = logtab
    SpecialPlot.__init__(self, name, logtab.ui.canvas, idx, special_id)
    QtGui.QPushButton.__init__(self, u"Remove {} {} plot".format(name, special_id), logtab)
    self.clicked.connect(self.on_clicked)
    if idx == 0:
      self.layout = logtab.ui.y1SelectorLayout
    else:
      self.layout = logtab.ui.y2SelectorLayout
    self.layout.addWidget(self)
    if len(self.added) == 0:
      self.deleteLater()
    else:
      self.logtab.specials["{}_{}".format(name, special_id)] = self
  def plot(self):
    SpecialPlot.plot(self)
    self.logtab.ui.canvas.draw()
  def on_clicked(self):
    for added in self.added:
      self.remove(added)
    self.logtab.ui.canvas.draw()
    del self.logtab.specials["{}_{}".format(self.name, self.id)]
    self.deleteLater()

class MCLogTab(QtGui.QWidget):
  canvas_need_update = QtCore.Signal()
  def __init__(self, parent = None):
    super(MCLogTab, self).__init__(parent)
    self.ui = Ui_MCLogTab()
    self.ui.setupUi(self)
    self.ui.canvas.setupLockButtons(self.ui.selectorLayout)
    if parent is not None:
      self.ui.canvas.grid = parent.gridStyles['left']
      self.ui.canvas.grid2 = parent.gridStyles['right']
    def setupSelector(ySelector):
      ySelector.setHeaderLabels(["Data"])
      ySelector.header().setResizeMode(QtGui.QHeaderView.ResizeMode.Fixed)
      ySelector.viewport().installEventFilter(FilterRightClick(ySelector))
    setupSelector(self.ui.y1Selector)
    setupSelector(self.ui.y2Selector)
    self.ui.y1Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    self.ui.y2Selector.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
    self.y1Selected = []
    self.y2Selected = []

    self.data = None
    self.rm = None
    self.ui.canvas.x_data = 't'
    self.x_data = 't'

    self.specials = {}

  def setData(self, data):
    self.data = data
    self.ui.canvas.setData(data)
    self.update_x_selector()
    self.update_y_selectors()

  def setGridStyles(self, gridStyles):
    self.ui.canvas.grid = copy.deepcopy(gridStyles['left'])
    self.ui.canvas.grid2 = copy.deepcopy(gridStyles['right'])

  def setRobotModule(self, rm):
    self.rm = rm
    if self.rm is None:
      return
    def setQNames(ySelector):
      qList = ySelector.findItems("q", QtCore.Qt.MatchFlag.MatchStartsWith)
      qList += ySelector.findItems("alpha", QtCore.Qt.MatchFlag.MatchStartsWith)
      qList += ySelector.findItems("error", QtCore.Qt.MatchFlag.MatchStartsWith)
      qList += ySelector.findItems("tau", QtCore.Qt.MatchFlag.MatchStartsWith)
      def update_child_display(items):
        for itm in items:
          cCount = itm.childCount()
          if cCount == 0:
            if itm.originalText.isdigit():
              jIndex = int(itm.originalText)
              if jIndex < len(self.rm.ref_joint_order()):
                itm.displayText = self.rm.ref_joint_order()[jIndex]
          else:
            update_child_display([itm.child(i) for i in range(cCount)])
      update_child_display(qList)
    setQNames(self.ui.y1Selector)
    setQNames(self.ui.y2Selector)
    bounds = self.rm.bounds()
    if self.data is None:
      return
    for i, jn in enumerate(self.rm.ref_joint_order()):
      if "qIn_limits_lower_{}".format(i) in self.data:
        self.data["qIn_limits_lower_{}".format(i)].fill(bounds[0][jn][0])
        self.data["qIn_limits_upper_{}".format(i)].fill(bounds[1][jn][0])
        self.data["qOut_limits_lower_{}".format(i)].fill(bounds[0][jn][0])
        self.data["qOut_limits_upper_{}".format(i)].fill(bounds[1][jn][0])
      if "tauIn_limits_lower_{}".format(i) in self.data:
        self.data["tauIn_limits_lower_{}".format(i)].fill(bounds[4][jn][0])
        self.data["tauIn_limits_upper_{}".format(i)].fill(bounds[5][jn][0])


  @QtCore.Slot(str)
  def on_xSelector_activated(self, k):
    self.x_data = k
    self.ui.canvas.clear_all()
    self.ui.canvas.x_data = k
    self.y1Selected = []
    self.itemSelectionChanged(self.ui.y1Selector, self.y1Selected, 0)
    self.y2Selected = []
    self.itemSelectionChanged(self.ui.y2Selector, self.y2Selected, 1)
    for _,s in self.specials.iteritems():
      s.plot()

  @QtCore.Slot(QtGui.QTreeWidgetItem, int)
  def on_y1Selector_itemClicked(self, item, col):
    self.y1Selected = self.itemSelectionChanged(self.ui.y1Selector, self.y1Selected, 0)

  @QtCore.Slot(QtGui.QTreeWidgetItem, int)
  def on_y2Selector_itemClicked(self, item, col):
    self.y2Selected = self.itemSelectionChanged(self.ui.y2Selector, self.y2Selected, 1)

  @QtCore.Slot(QtCore.QPoint)
  def on_y1Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y1Selector, point, 0)

  @QtCore.Slot(QtCore.QPoint)
  def on_y2Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y2Selector, point, 1)

  def itemSelectionChanged(self, ySelector, prevSelected, idx):
    if idx == 0:
      add_fn = self.ui.canvas.add_plot_left
    else:
      add_fn = self.ui.canvas.add_plot_right
    if idx == 0:
      remove_fn = self.ui.canvas.remove_plot_left
    else:
      remove_fn = self.ui.canvas.remove_plot_right
    selected_items = [(i.actualText,i.hasData) for i in ySelector.selectedItems()]
    def is_selected(s, x):
      if s[1]:
        return x == s[0]
      return re.match("{}($|_.*$)".format(s[0]), x) is not None
    selected = sorted(filter(lambda x: any([is_selected(s, x) for s in selected_items]), self.data.keys()))
    def find_item(s):
      for itm in [it.value() for it in QtGui.QTreeWidgetItemIterator(ySelector)]:
        if itm.actualText == s:
          return itm
      return None
    legends = [itm.actualText.replace(itm.originalText, itm.displayText) for itm in [ find_item(s) for s in selected ] ]
    for s,l in zip(selected, legends):
      if s not in prevSelected:
        add_fn(self.x_data, s, l)
    for s in prevSelected:
      if s not in selected:
        remove_fn(s)
    self.ui.canvas.draw()
    return selected

  def update_x_selector(self):
    self.ui.xSelector.clear()
    self.ui.xSelector.addItems(sorted(self.data.keys()))
    idx = self.ui.xSelector.findText(self.x_data)
    if idx != -1:
      self.ui.xSelector.setCurrentIndex(idx)

  def update_y_selectors(self):
    self.ui.y1Selector.clear()
    self.ui.y2Selector.clear()
    self.tree_view = TreeView()
    for k in sorted(self.data.keys()):
      self.tree_view.add(k.split('_'))
    self.tree_view.simplify()
    def update_y_selector(ySelector):
      self.tree_view.update_y_selector(ySelector, ySelector)
      ySelector.resizeColumnToContents(0)
      cWidth = ySelector.sizeHintForColumn(0)
      ySelector.setMaximumWidth(cWidth + 75)
    update_y_selector(self.ui.y1Selector)
    update_y_selector(self.ui.y2Selector)

  def showCustomMenu(self, ySelector, point, idx):
    item = ySelector.itemAt(point)
    if item is None:
      return
    menu = QtGui.QMenu(ySelector)
    addedAction = False
    action = QtGui.QAction(u"Plot diff".format(item.actualText), menu)
    action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText, self, idx, "diff"))
    menu.addAction(action)
    s = re.match('^(.*)_q?[wxyz]$', item.actualText)
    if s is not None:
      for item_label, axis_label in [("RPY angles", "rpy"), ("ROLL angle", "r"), ("PITCH angle", "p"), ("YAW angle", "y")]:
        action = QtGui.QAction(u"Plot {}".format(item_label, item.actualText), menu)
        action.triggered.connect(lambda label=axis_label: RemoveSpecialPlotButton(s.group(1), self, idx, label))
        menu.addAction(action)
    else:
      quat_childs = filter(lambda x: x is not None, [ re.match('{}((_.+)*)_q?w$'.format(item.actualText), x) for x in self.data.keys() ])
      for qc in quat_childs:
        for item_label, axis_label in [("RPY angles", "rpy"), ("ROLL angle", "r"), ("PITCH angle", "p"), ("YAW angle", "y")]:
          if len(qc.group(1)):
            action_text = u"Plot {} {}".format(qc.group(1)[1:], item_label)
          else:
            action_text = u"Plot {}".format(item_label)
          action = QtGui.QAction(action_text, menu)
          plot_name = item.actualText + qc.group(1)
          action.triggered.connect(lambda name=plot_name, label=axis_label: RemoveSpecialPlotButton(name, self, idx, label))
          menu.addAction(action)
    menu.exec_(ySelector.viewport().mapToGlobal(point))

  @staticmethod
  def MakeFigure(data, x, y1, y2, y1_label = None, y2_label = None, figure = None):
    if y1_label is None:
      return MCLogTab.MakeFigure(data, x, y1, y2, y1, y2_label, figure)
    if y2_label is None:
      return MCLogTab.MakeFigure(data, x, y1, y2, y1_label, y2, figure)
    if figure is None:
      return MCLogTab.MakeFigure(data, x, y1, y2, y1_label, y2_label, PlotFigure())
    figure.setData(data)
    for y,yl in zip(y1, y1_label):
      figure.add_plot_left(x, y, yl)
    for y,yl in zip(y2, y2_label):
      figure.add_plot_right(x, y, yl)
    return figure

  @staticmethod
  def MakePlot(parent, x_data, y1, y2, y1_label = None, y2_label = None):
    if y1_label is None:
      return MCLogTab.MakePlot(parent, x_data, y1, y2, y1, y2_label)
    if y2_label is None:
      return MCLogTab.MakePlot(parent, x_data, y1, y2, y1_label, y2)
    tab = MCLogTab(parent)
    tab.x_data = x_data
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
    for y,yl in zip(y1, y1_label):
      tab.tree_view.select(y, tab.ui.y1Selector, 0)
    for y,yl in zip(y2, y2_label):
      tab.tree_view.select(y, tab.ui.y2Selector, 1)
    tab.y1Selected = y1
    tab.y2Selected = y2
    MCLogTab.MakeFigure(parent.data, x_data, y1, y2, y1_label, y2_label, tab.ui.canvas)
    tab.ui.canvas.x_data = x_data
    return tab

  @staticmethod
  def UserFigure(data, p, figure = None, special = None):
    if figure is None:
      return MCLogTab.UserFigure(data, p, MCLogTab.MakeFigure(data, p.x, p.y1, p.y2), special)
    if special is None:
      return MCLogTab.UserFigure(data, p, figure, lambda y, idx, id_: UserPlot(y, figure, idx, id_))
    def set_label(label_fn, label_size_fn, label):
      if len(label.text):
        label_fn(label.text)
        label_size_fn(label.fontsize)
    set_label(figure.title, figure.title_fontsize, p.graph_labels.title)
    set_label(figure.x_label, figure.x_label_fontsize, p.graph_labels.x_label)
    set_label(figure.y1_label, figure.y1_label_fontsize, p.graph_labels.y1_label)
    set_label(figure.y2_label, figure.y2_label_fontsize, p.graph_labels.y2_label)
    def handle_yd(yds, idx):
      for yd in yds:
        match = re.match("(.*)_(.*)$", yd)
        if match is None:
          special(yd, idx, "diff")
        elif match.group(2) in ["rpy", "r", "p", "y"]:
          special(match.group(1), idx, match.group(2))
        else:
          special(match.group(1), idx, "diff")
    handle_yd(p.y1d, 0)
    handle_yd(p.y2d, 1)
    if not isinstance(p.grid1, LineStyle):
      figure.grid = LineStyle(**p.grid1)
    else:
      figure.grid = p.grid1
    if not isinstance(p.grid2, LineStyle):
      figure.grid2 = LineStyle(**p.grid2)
    else:
      figure.grid2 = p.grid2
    for y,s in p.style.iteritems():
      figure.style_left(y, s)
    for y,s in p.style2.iteritems():
      figure.style_right(y, s)
    return figure

  @staticmethod
  def UserPlot(parent, p):
    tab = MCLogTab.MakePlot(parent, p.x, p.y1, p.y2)
    MCLogTab.UserFigure(parent.data, p, tab.ui.canvas, lambda y, idx, id_: RemoveSpecialPlotButton(y, tab, idx, id_))
    tab.ui.canvas.draw()
    return tab

  @staticmethod
  def ForceSensorPlot(parent, fs):
    tab = MCLogTab.MakePlot(parent, 't', ['{}ForceSensor_f{}'.format(fs, ax) for ax in ['x', 'y', 'z']], ['{}ForceSensor_c{}'.format(fs, ax) for ax in ['x', 'y', 'z']])
    tab.ui.canvas.title('Force sensor: {}'.format(fs))
    tab.ui.canvas.y1_label('Force')
    tab.ui.canvas.y2_label('Moment')
    tab.ui.canvas.draw()
    return tab

  @staticmethod
  def JointPlot(parent, joints, y1_prefix, y2_prefix, y1_diff_prefix, y2_diff_prefix, plot_limits = False):
    def prefix_to_label(joints, prefix, diff):
      suffix = ''
      if diff:
        suffix += '_velocity'
      if prefix == "qIn":
        return "encoder" + suffix
      if prefix == "qOut":
        return "command" + suffix
      if prefix == "tauIn":
        return "torque" + suffix
      if prefix == "error":
        return "error"+suffix
      return prefix
    y1_label = prefix_to_label(joints, y1_prefix, False)
    y2_label = prefix_to_label(joints, y2_prefix, False)
    y1_diff_label = prefix_to_label(joints, y1_diff_prefix, True)
    y2_diff_label = prefix_to_label(joints, y2_diff_prefix, True)
    rjo = parent.rm.ref_joint_order()
    y_data = [[], []]
    y_data_labels = [[], []]
    y_diff_data = [[], []]
    y_diff_data_labels = [[], []]
    for j in joints:
      jIndex = rjo.index(j)
      if y1_prefix:
        y_data[0] += [ '{}_{}'.format(y1_prefix, jIndex) ]
        y_data_labels[0] += [ '{}_{}'.format(y1_label, j) ]
        if y1_prefix != "error" and plot_limits:
          y_data[0] += [ '{}_limits_lower_{}'.format(y1_prefix, jIndex) ]
          y_data_labels[0] += [ '{}_limits_lower_{}'.format(y1_label, j) ]
          y_data[0] += [ '{}_limits_upper_{}'.format(y1_prefix, jIndex) ]
          y_data_labels[0] += [ '{}_limits_upper_{}'.format(y1_label, j) ]
      if y2_prefix:
        y_data[1] += [ '{}_{}'.format(y2_prefix, jIndex) ]
        y_data_labels[1] += [ '{}_{}'.format(y2_label, j) ]
        if y2_prefix != "error" and plot_limits:
          y_data[1] += [ '{}_limits_lower_{}'.format(y2_prefix, jIndex) ]
          y_data_labels[1] += [ '{}_limits_lower_{}'.format(y2_label, j) ]
          y_data[1] += [ '{}_limits_upper_{}'.format(y2_prefix, jIndex) ]
          y_data_labels[1] += [ '{}_limits_upper_{}'.format(y2_label, j) ]
      if y1_diff_prefix:
        y_diff_data[0] += [ '{}_{}'.format(y1_diff_prefix, jIndex) ]
        y_diff_data_labels[0] += [ '{}_{}'.format(y1_diff_label, j) ]
      if y2_diff_prefix:
        y_diff_data[1] += [ '{}_{}'.format(y2_diff_prefix, jIndex) ]
        y_diff_data_labels[1] += [ '{}_{}'.format(y2_diff_label, j) ]
    tab = MCLogTab.MakePlot(parent, 't', y_data[0], y_data[1], y_data_labels[0], y_data_labels[1])
    for y, y_label in zip(y_diff_data[0], y_diff_data_labels[0]):
      tab.ui.canvas.add_diff_plot_left(tab.x_data, y, y_label)
    for y, y_label in zip(y_diff_data[1], y_diff_data_labels[1]):
      tab.ui.canvas.add_diff_plot_right(tab.x_data, y, y_label)
    class nonlocal: pass
    nonlocal.title = ''
    def updateTitle(nTitle):
      if len(nonlocal.title):
        nonlocal.title += ' / '
      nTitle = nTitle.replace('_', ' ')
      nonlocal.title += nTitle
    if y1_label:
      updateTitle(y1_label.title())
      tab.ui.canvas.y1_label(y1_label)
    if y2_label:
      updateTitle(y2_label.title())
      tab.ui.canvas.y2_label(y2_label)
    if y1_diff_label:
      updateTitle(y1_diff_label.title())
      tab.ui.canvas.y1_label(y1_diff_label)
    if y2_diff_label:
      updateTitle(y2_diff_label.title())
      tab.ui.canvas.y2_label(y2_diff_label)
    tab.ui.canvas.title(nonlocal.title)
    tab.ui.canvas.draw()
    return tab
