# -*- coding: utf-8 -*-

from PySide import QtCore, QtGui

from mc_log_tab_ui import Ui_MCLogTab

from functools import partial

def get_full_text(item):
  if item.childCount():
    return [ t for i in range(item.childCount()) for t in get_full_text(item.child(i)) ]
  else:
    base_text = item.text(0)
    parent = item.parent()
    while parent is not None:
      base_text = parent.text(0) + "_" + base_text
      parent = parent.parent()
    return [ base_text ]

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
    self.x_data_trigger = False
    self.x_data = 't'
    self.y1_data = []
    self.y2_data = []
    self.y_diff_data = [[], []]

  def setData(self, data):
    self.data = data
    self.update_x_selector()
    self.update_y_selectors()

  @QtCore.Slot(str)
  def on_xSelector_activated(self, k):
    self.x_data = k
    self.update_canvas()

  @QtCore.Slot()
  def on_y1Selector_itemSelectionChanged(self):
    self.y1_data = self.itemSelectionChanged(self.ui.y1Selector)
    self.update_canvas()

  @QtCore.Slot()
  def on_y2Selector_itemSelectionChanged(self):
    self.y2_data = self.itemSelectionChanged(self.ui.y2Selector)
    self.update_canvas()

  def update_canvas(self):
    self.ui.canvas.plot(self.data, self.x_data, self.y1_data, self.y2_data, self.y_diff_data)

  def itemSelectionChanged(self, ySelector):
    selected = ySelector.selectedItems()
    return [ t.encode(u'ascii') for item in selected for t in get_full_text(item) ]

  def checkboxChanged(self, entries, idx, state):
    if state:
      self.y_diff_data[idx] += entries
    else:
      for e in entries:
        self.y_diff_data[idx].remove(e)
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
        base = QtGui.QTreeWidgetItem(ySelector, [k])
        box = QtGui.QCheckBox(ySelector)

        entries = [ "{}_{}".format(k, v) for v in values ]
        if len(entries) == 0:
          entries = [k]
        box.stateChanged.connect(partial(self.checkboxChanged, entries, idx))
        ySelector.setItemWidget(base, 1, box)

        for v in values:
          item = QtGui.QTreeWidgetItem(base, [v])
          entries = ["{}_{}".format(k, v)]
          box = QtGui.QCheckBox(ySelector)
          box.stateChanged.connect(partial(self.checkboxChanged, entries, idx))
          ySelector.setItemWidget(item, 1, box)

        if len(values) < 6:
          base.setExpanded(True)
      ySelector.resizeColumnToContents(0)
      ySelector.resizeColumnToContents(1)
      ySelector.setMaximumWidth(ySelector.sizeHintForColumn(0) + 75)
    update_y_selector(self.ui.y1Selector, 0)
    update_y_selector(self.ui.y2Selector, 1)
