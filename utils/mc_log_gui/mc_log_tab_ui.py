# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mc_log_tab_ui.ui'
#
# Created: Fri Jun 16 10:16:24 2017
#      by: pyside-uic 0.2.15 running on PySide 1.2.1
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_MCLogTab(object):
    def setupUi(self, MCLogTab):
        MCLogTab.setObjectName("MCLogTab")
        MCLogTab.resize(803, 538)
        self.gridLayout_2 = QtGui.QGridLayout(MCLogTab)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtGui.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.canvas = PlotCanvasWithToolbar(MCLogTab)
        self.canvas.setObjectName("canvas")
        self.gridLayout.addWidget(self.canvas, 0, 1, 1, 1)
        self.y1Selector = QtGui.QTreeWidget(MCLogTab)
        self.y1Selector.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.y1Selector.setSelectionBehavior(QtGui.QAbstractItemView.SelectItems)
        self.y1Selector.setColumnCount(2)
        self.y1Selector.setObjectName("y1Selector")
        self.y1Selector.headerItem().setText(0, "1")
        self.y1Selector.headerItem().setText(1, "2")
        self.y1Selector.header().setVisible(True)
        self.gridLayout.addWidget(self.y1Selector, 0, 0, 1, 1)
        self.y2Selector = QtGui.QTreeWidget(MCLogTab)
        self.y2Selector.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.y2Selector.setColumnCount(2)
        self.y2Selector.setObjectName("y2Selector")
        self.y2Selector.headerItem().setText(0, "1")
        self.y2Selector.headerItem().setText(1, "2")
        self.y2Selector.header().setVisible(True)
        self.gridLayout.addWidget(self.y2Selector, 0, 2, 1, 1)
        self.xSelector = QtGui.QComboBox(MCLogTab)
        self.xSelector.setObjectName("xSelector")
        self.gridLayout.addWidget(self.xSelector, 1, 1, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)

        self.retranslateUi(MCLogTab)
        QtCore.QMetaObject.connectSlotsByName(MCLogTab)

    def retranslateUi(self, MCLogTab):
        MCLogTab.setWindowTitle(QtGui.QApplication.translate("MCLogTab", "MCLogTab", None, QtGui.QApplication.UnicodeUTF8))

from mc_log_plotcanvas import PlotCanvasWithToolbar
