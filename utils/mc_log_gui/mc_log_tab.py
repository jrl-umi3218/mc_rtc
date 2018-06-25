# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

from mc_log_tab_ui import Ui_MCLogTab

from functools import partial

import copy
import re

class MCLogTreeWidgetItem(QtGui.QTreeWidgetItem):
  def __init__(self, parent, displayText, actualText):
    super(MCLogTreeWidgetItem, self).__init__(parent, [displayText])
    self._displayText = displayText
    self.actualText = actualText
  @property
  def displayText(self):
    return self._displayText
  @displayText.setter
  def displayText(self, value):
    self._displayText = value
    self.setText(0, self._displayText)

class TreeView(object):
  def __init__(self, name = None):
    self.name = name
    self.leafs = []
  def leaf(self, name):
    for l in self.leafs:
      if l.name == name:
        return l
    self.leafs.append(TreeView(name))
    return self.leafs[-1]
  def add(self, key):
    if len(key) == 0:
      return
    self.leaf(key[0]).add(key[1:])
  def simplify(self):
    while len(self.leafs) == 1:
      self.name = self.name + '_' + self.leafs[0].name
      self.leafs = self.leafs[0].leafs
    for l in self.leafs:
      l.simplify()
  def update_y_selector(self, ySelector, parent, baseModelIdx = None, baseName = ""):
    row = 0
    needExpand = False
    for l in self.leafs:
      fullName = baseName
      if len(fullName):
        fullName += "_"
      fullName += l.name
      base = MCLogTreeWidgetItem(parent, l.name, fullName)
      if baseModelIdx is not None:
        newModelIdx = ySelector.model().index(row, 0, baseModelIdx)
      else:
        newModelIdx = ySelector.model().index(row, 0)
      #if fullName in selected_data:
      #  selection = ySelector.selectionModel()
      #  selection.select(newModelIdx, QtGui.QItemSelectionModel.Select)
      #  ySelector.setSelectionModel(selection)
      #  needExpand = True
      if l.update_y_selector(ySelector, base, newModelIdx, fullName):
        base.setExpanded(True)
      row += 1
    return needExpand
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

class RemoveSpecialPlotButton(QtGui.QPushButton):
  def __init__(self, name, logtab, idx, special_id):
    self.logtab = logtab
    if idx == 0:
      self.layout = logtab.ui.y1SelectorLayout
    else:
      self.layout = logtab.ui.y2SelectorLayout
    super(RemoveSpecialPlotButton, self).__init__(u"Remove {} {} plot".format(name, special_id), logtab)
    self.layout.addWidget(self)
    self.idx = idx
    self.name = name
    self.id = special_id
    self.added = []
    if idx == 0:
      self.remove = self.logtab.ui.canvas.remove_plot_left
    else:
      self.remove = self.logtab.ui.canvas.remove_plot_right
    self.clicked.connect(self.on_clicked)
    if special_id == "diff":
      self.__plot = self.__add_diff
    elif special_id == "rpy":
      self.__plot = self.__add_rpy
    else:
      print "Cannot handle this special plot:", special_id
    self.plot()
    if len(self.added) == 0:
      self.deleteLater()
  def __add_diff(self):
    added = filter(lambda x: re.match("{}($|_.*$)".format(self.name), x) is not None, self.logtab.data.keys())
    if self.idx == 0:
      add_fn = self.logtab.ui.canvas.add_diff_plot_left
    else:
      add_fn = self.logtab.ui.canvas.add_diff_plot_right
    for a in added:
      label = "{}_diff".format(a)
      if add_fn(self.logtab.x_data, a, label):
        self.added.append(label)
  def __add_rpy(self):
    if self.idx == 0:
      add_fn = self.logtab.ui.canvas.add_rpy_plot_left
    else:
      add_fn = self.logtab.ui.canvas.add_rpy_plot_right
    if add_fn(self.logtab.x_data, self.name):
      self.added = [ "{}_{}".format(self.name, s) for s in ["r", "p", "y"] ]
  def plot(self):
    self.__plot()
    self.logtab.ui.canvas.draw()

  def on_clicked(self):
    for added in self.added:
      self.remove(added)
    self.logtab.ui.canvas.draw()
    self.deleteLater()

class MCLogTab(QtGui.QWidget):
  canvas_need_update = QtCore.Signal()
  def __init__(self, parent = None):
    super(MCLogTab, self).__init__(parent)
    self.ui = Ui_MCLogTab()
    self.ui.setupUi(self)
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
    self.x_data_trigger = False
    self.x_data = 't'

  def setData(self, data):
    self.data = data
    self.ui.canvas.setData(data)
    self.update_x_selector()
    self.update_y_selectors()

  def setRobotModule(self, rm):
    self.rm = rm
    if self.rm is None:
      return
    def setQNames(ySelector):
      qList = ySelector.findItems("q", QtCore.Qt.MatchFlag.MatchStartsWith)
      qList += ySelector.findItems("error", QtCore.Qt.MatchFlag.MatchStartsWith)
      for qIn in qList:
        cCount = qIn.childCount()
        for i in range(cCount):
          c = qIn.child(i)
          if c.actualText.isdigit():
            jIndex = int(c.actualText)
            if jIndex < self.rm.mb.nrJoints():
              try:
                jName = self.rm.mb.joint(jIndex + 1).name()
                c.displayText = self.rm.ref_joint_order()[self.rm.ref_joint_order().index(jName)]
              except ValueError,IndexError:
                pass
    setQNames(self.ui.y1Selector)
    setQNames(self.ui.y2Selector)


  @QtCore.Slot(str)
  def on_xSelector_activated(self, k):
    self.x_data = k
    self.ui.canvas.clear_all()
    # FIXME SHOULD REPLOT ALL ACCORDING TO NEW X

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
    selected_items = [i.actualText for i in ySelector.selectedItems()]
    def is_selected(s, x):
      return re.match("{}($|_.*$)".format(s), x) is not None
    selected = sorted(filter(lambda x: any([is_selected(s, x) for s in selected_items]), self.data.keys()))
    for s in selected:
      if s not in prevSelected:
        add_fn(self.x_data, s, s)
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
    tree_view = TreeView()
    for k in sorted(self.data.keys()):
      tree_view.add(k.split('_'))
    tree_view.simplify()
    def update_y_selector(ySelector):
      tree_view.update_y_selector(ySelector, ySelector)
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
      action = QtGui.QAction(u"Plot as RPY angles".format(item.actualText), menu)
      action.triggered.connect(lambda: RemoveSpecialPlotButton(s.group(1), self, idx, "rpy"))
      menu.addAction(action)
    else:
      quat_childs = filter(lambda x: x is not None, [ re.match('{}((_.+)*)_q?w$'.format(item.actualText), x) for x in self.data.keys() ])
      for qc in quat_childs:
        if len(qc.group(1)):
          action_text = u"Plot {} as RPY angles".format(qc.group(1)[1:])
        else:
          action_text = u"Plot as RPY angles"
        action = QtGui.QAction(action_text, menu)
        action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText + qc.group(1), self, idx, "rpy"))
        menu.addAction(action)
    menu.exec_(ySelector.viewport().mapToGlobal(point))

  @staticmethod
  def UserPlot(parent, p):
    tab = MCLogTab(parent)
    tab.x_data = p.x
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
    for y1 in p.y1:
      tab.ui.canvas.add_plot_left(tab.x_data, y1, y1)
    for y2 in p.y2:
      tab.ui.canvas.add_plot_right(tab.x_data, y2, y2)
    for yd in p.y1d:
      RemoveSpecialPlotButton(yd, tab, 0, "diff")
    for yd in p.y2d:
      RemoveSpecialPlotButton(yd, tab, 1, "diff")
    tab.ui.canvas.draw()
    return tab

  @staticmethod
  def ForceSensorPlot(parent, fs):
    tab = MCLogTab(parent)
    tab.x_data = 't'
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
    tab.ui.canvas.title('Force sensor: {}'.format(fs))
    tab.ui.canvas.y1_label('Force')
    tab.ui.canvas.y2_label('Moment')
    for y in [ '{}ForceSensor_f{}'.format(fs, ax) for ax in ['x', 'y', 'z'] ]:
      tab.ui.canvas.add_plot_left(tab.x_data, y, y)
    for y in [ '{}ForceSensor_c{}'.format(fs, ax) for ax in ['x', 'y', 'z'] ]:
      tab.ui.canvas.add_plot_right(tab.x_data, y, y)
    tab.ui.canvas.draw()
    return tab

  @staticmethod
  def JointPlot(parent, joints, y1_prefix, y2_prefix, y1_diff_prefix, y2_diff_prefix):
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
    tab = MCLogTab(parent)
    tab.x_data = 't'
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
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
      if y2_prefix:
        y_data[1] += [ '{}_{}'.format(y2_prefix, jIndex) ]
        y_data_labels[1] += [ '{}_{}'.format(y2_label, j) ]
      if y1_diff_prefix:
        y_diff_data[0] += [ '{}_{}'.format(y1_diff_prefix, jIndex) ]
        y_diff_data_labels[0] += [ '{}_{}'.format(y1_diff_label, j) ]
      if y2_diff_prefix:
        y_diff_data[1] += [ '{}_{}'.format(y2_diff_prefix, jIndex) ]
        y_diff_data_labels[1] += [ '{}_{}'.format(y2_diff_label, j) ]
    for y, y_label in zip(y_data[0], y_data_labels[0]):
      tab.ui.canvas.add_plot_left(tab.x_data, y, y_label)
    for y, y_label in zip(y_diff_data[0], y_diff_data_labels[0]):
      tab.ui.canvas.add_diff_plot_left(tab.x_data, y, y_label)
    for y, y_label in zip(y_data[1], y_data_labels[1]):
      tab.ui.canvas.add_plot_right(tab.x_data, y, y_label)
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
