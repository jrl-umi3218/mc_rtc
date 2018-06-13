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
  def update_y_selector(self, ySelector, selected_data, parent, baseModelIdx = None, baseName = ""):
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
      if fullName in selected_data:
        selection = ySelector.selectionModel()
        selection.select(newModelIdx, QtGui.QItemSelectionModel.Select)
        ySelector.setSelectionModel(selection)
        needExpand = True
      if l.update_y_selector(ySelector, selected_data, base, newModelIdx, fullName):
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
  def __init__(self, name, logtab, idx, special_id, button_only = False):
    self.logtab = logtab
    if idx == 0:
      self.layout = logtab.ui.y1SelectorLayout
    else:
      self.layout = logtab.ui.y2SelectorLayout
    super(RemoveSpecialPlotButton, self).__init__(u"Remove {} {} plot".format(name, special_id), logtab)
    self.idx = idx
    self.layout.addWidget(self)
    self.clicked.connect(self.on_clicked)
    self.added = sorted(filter(lambda x: x.startswith(name), self.logtab.data.keys()))
    self.added_labels = [ "{}_{}".format(l, special_id) for l in self.added]
    if not button_only:
        self.logtab.y_diff_data[idx] += self.added
        self.logtab.y_diff_data_labels[idx] += self.added_labels
        self.logtab.canvas_need_update.emit()
  def on_clicked(self):
    self.logtab.y_diff_data[self.idx] = filter(lambda x: x not in self.added, self.logtab.y_diff_data[self.idx])
    self.logtab.y_diff_data_labels[self.idx] = filter(lambda x: x not in self.added_labels, self.logtab.y_diff_data_labels[self.idx])
    self.logtab.canvas_need_update.emit()
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
    self.canvas_need_update.connect(self.update_canvas)

    self.data = None
    self.rm = None
    self.x_data_trigger = False
    self.x_data = 't'
    self.y_data = [[], []]
    self.y_data_labels = [[], []]
    self.y_diff_data = [[], []]
    self.y_diff_data_labels = [[], []]

  def setData(self, data):
    self.data = data
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
    self.update_canvas()

  @QtCore.Slot()
  def on_y1Selector_itemClicked(self):
    self.itemSelectionChanged(self.ui.y1Selector, 0)

  @QtCore.Slot()
  def on_y2Selector_itemClicked(self):
    self.itemSelectionChanged(self.ui.y2Selector, 1)

  @QtCore.Slot(QtCore.QPoint)
  def on_y1Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y1Selector, point, 0)

  @QtCore.Slot(QtCore.QPoint)
  def on_y2Selector_customContextMenuRequested(self, point):
    self.showCustomMenu(self.ui.y2Selector, point, 1)

  def update_canvas(self):
    self.ui.canvas.plot(self.data, self.x_data, self.y_data, self.y_diff_data, self.y_data_labels, self.y_diff_data_labels)

  def itemSelectionChanged(self, ySelector, idx):
    self.y_data[idx] = []
    self.y_data_labels[idx] = []
    selected = ySelector.selectedItems()
    for item in selected:
      self.y_data[idx] += sorted(filter(lambda x: x.startswith(item.actualText), self.data.keys()))
    self.y_data_labels[idx] = self.y_data[idx]
    self.update_canvas()

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
    def update_y_selector(ySelector, selected_data):
      tree_view.update_y_selector(ySelector, selected_data, ySelector)
      ySelector.resizeColumnToContents(0)
      cWidth = ySelector.sizeHintForColumn(0)
      ySelector.setMaximumWidth(cWidth + 75)
    update_y_selector(self.ui.y1Selector, self.y_data[0])
    update_y_selector(self.ui.y2Selector, self.y_data[1])

  def showCustomMenu(self, ySelector, point, idx):
    item = ySelector.itemAt(point)
    if item is None:
      return
    menu = QtGui.QMenu(ySelector)
    action = QtGui.QAction(u"Plot {} diff".format(item.actualText), menu)
    action.triggered.connect(lambda: RemoveSpecialPlotButton(item.actualText, self, idx, "diff"))
    menu.addAction(action)
    menu.exec_(ySelector.viewport().mapToGlobal(point))

  @staticmethod
  def UserPlot(parent, p):
    tab = MCLogTab(parent)
    tab.x_data = p.x
    tab.y_data[0] = copy.deepcopy(p.y1)
    tab.y_data_labels[0] = copy.deepcopy(p.y1)
    tab.y_data[1] = copy.deepcopy(p.y2)
    tab.y_data_labels[1] = copy.deepcopy(p.y2)
    tab.y_diff_data[0] = copy.deepcopy(p.y1d)
    tab.y_diff_data_labels[0] = [ l + '_diff' for l in tab.y_diff_data[0] ]
    tab.y_diff_data[1] = copy.deepcopy(p.y2d)
    tab.y_diff_data_labels[1] = [ l + '_diff' for l in tab.y_diff_data[1] ]
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
    tab.update_canvas()
    for yd in tab.y_diff_data[0]:
      RemoveSpecialPlotButton(yd, tab, 0, "diff", button_only = True)
    for yd in tab.y_diff_data[1]:
      RemoveSpecialPlotButton(yd, tab, 1, "diff", button_only = True)
    return tab

  @staticmethod
  def ForceSensorPlot(parent, fs):
    tab = MCLogTab(parent)
    tab.x_data = 't'
    tab.y_data[0] = [ '{}ForceSensor_f{}'.format(fs, ax) for ax in ['x', 'y', 'z' ] ]
    tab.y_data_labels[0] = tab.y_data[0]
    tab.y_data[1] = [ '{}ForceSensor_c{}'.format(fs, ax) for ax in ['x', 'y', 'z' ] ]
    tab.y_data_labels[1] = tab.y_data[1]
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
    tab.ui.canvas.title('Force sensor: {}'.format(fs))
    tab.ui.canvas.y1_label('Force')
    tab.ui.canvas.y2_label('Moment')
    tab.update_canvas()
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
    rjo = parent.rm.ref_joint_order()
    for j in joints:
      jIndex = rjo.index(j)
      if y1_prefix:
        tab.y_data[0] += [ '{}_{}'.format(y1_prefix, jIndex) ]
        tab.y_data_labels[0] += [ '{}_{}'.format(y1_label, j) ]
      if y2_prefix:
        tab.y_data[1] += [ '{}_{}'.format(y2_prefix, jIndex) ]
        tab.y_data_labels[1] += [ '{}_{}'.format(y2_label, j) ]
      if y1_diff_prefix:
        tab.y_diff_data[0] += [ '{}_{}'.format(y1_diff_prefix, jIndex) ]
        tab.y_diff_data_labels[0] += [ '{}_{}'.format(y1_diff_label, j) ]
      if y2_diff_prefix:
        tab.y_diff_data[1] += [ '{}_{}'.format(y2_diff_prefix, jIndex) ]
        tab.y_diff_data_labels[1] += [ '{}_{}'.format(y2_diff_label, j) ]
    tab.setData(parent.data)
    tab.setRobotModule(parent.rm)
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
    tab.update_canvas()
    return tab
