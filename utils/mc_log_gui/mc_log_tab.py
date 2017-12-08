# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

from mc_log_tab_ui import Ui_MCLogTab

from functools import partial

import re

def get_full_text(item):
  if item.childCount():
    return [ (t,d) for i in range(item.childCount()) for t,d in get_full_text(item.child(i)) ]
  else:
    base_text = item.actualText
    display_text = item.displayText
    parent = item.parent()
    while parent is not None:
      base_text = parent.actualText + "_" + base_text
      display_text = parent.displayText + "_" + display_text
      parent = parent.parent()
    return [ (base_text, display_text) ]

class MCLogTreeWidgetItem(QtGui.QTreeWidgetItem):
  def __init__(self, parent, values):
    super(MCLogTreeWidgetItem, self).__init__(parent, values)
    self._displayText = values[0]
    self.actualText = values[0]
  @property
  def displayText(self):
    return self._displayText
  @displayText.setter
  def displayText(self, value):
    self._displayText = value
    self.setText(0, self._displayText)

class MCLogTab(QtGui.QWidget):
  def __init__(self, parent = None):
    super(MCLogTab, self).__init__(parent)
    self.ui = Ui_MCLogTab()
    self.ui.setupUi(self)
    def setupSelector(ySelector):
      ySelector.setHeaderLabels(["Data", "Diff"])
      ySelector.header().setResizeMode(QtGui.QHeaderView.ResizeMode.Fixed)
    setupSelector(self.ui.y1Selector)
    setupSelector(self.ui.y2Selector)

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

  def update_canvas(self):
    self.ui.canvas.plot(self.data, self.x_data, self.y_data, self.y_diff_data, self.y_data_labels, self.y_diff_data_labels)

  def itemSelectionChanged(self, ySelector, idx):
    self.y_data[idx] = []
    self.y_data_labels[idx] = []
    selected = ySelector.selectedItems()
    for item in selected:
      for t,d in get_full_text(item):
        self.y_data[idx] += [t.encode(u'ascii')]
        self.y_data_labels[idx] += [d.encode(u'ascii')]
    self.update_canvas()

  def checkboxChanged(self, item, idx, state):
    y_diff_data, y_diff_data_labels = [list(t) for t in zip(*get_full_text(item))]
    if state:
      self.y_diff_data[idx] += y_diff_data
      self.y_diff_data_labels[idx] += [ l + "_dot" for l in y_diff_data_labels ]
    else:
      for e in y_diff_data:
        self.y_diff_data[idx].remove(e)
      for e in y_diff_data_labels:
        self.y_diff_data_labels[idx].remove(e + "_dot")
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
    tree_view = {}
    for k in sorted(self.data.keys()):
      idx = k.rfind('_')
      if idx != -1 and idx + 1 < len(k):
        base = k[:idx]
        leaf = k[idx+1:]
        if not base in tree_view:
            tree_view[base] = []
        tree_view[base] += [leaf]
      else:
        assert(not k in tree_view)
        tree_view[k] = []
    def update_y_selector(ySelector, idx):
      for k, values in sorted(tree_view.items()):
        base = MCLogTreeWidgetItem(ySelector, [k])
        box = QtGui.QCheckBox(ySelector)

        if len(values) == 0:
          if k in self.y_data[idx]:
            base.setSelected(True)
        if k in self.y_diff_data[idx]:
          box.setChecked(True)
        box.stateChanged.connect(partial(self.checkboxChanged, base, idx))
        ySelector.setItemWidget(base, 1, box)

        needExpand = False
        for v in values:
          item = MCLogTreeWidgetItem(base, [v])
          if "{}_{}".format(k, v) in self.y_data[idx]:
            item.setSelected(True)
            needExpand = True
          box = QtGui.QCheckBox(ySelector)
          if "{}_{}".format(k, v) in self.y_diff_data[idx]:
            box.setChecked(True)
          box.stateChanged.connect(partial(self.checkboxChanged, item, idx))
          ySelector.setItemWidget(item, 1, box)
        base.setExpanded(needExpand)

      ySelector.resizeColumnToContents(0)
      ySelector.resizeColumnToContents(1)
      cWidth = min(200, ySelector.sizeHintForColumn(0))
      ySelector.setColumnWidth(0, cWidth)
      ySelector.setColumnWidth(1, 75)
      ySelector.setMaximumWidth(cWidth + 75)
    update_y_selector(self.ui.y1Selector, 0)
    update_y_selector(self.ui.y2Selector, 1)

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
