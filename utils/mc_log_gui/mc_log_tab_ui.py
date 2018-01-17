# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mc_log_tab_ui.ui'
#
# Created: Wed Jan 17 15:52:09 2018
#      by: pyside-uic 0.2.15 running on PySide 1.2.2
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_MCLogTab(object):
    def setupUi(self, MCLogTab):
        MCLogTab.setObjectName("MCLogTab")
        MCLogTab.resize(803, 538)
        self.horizontalLayout = QtGui.QHBoxLayout(MCLogTab)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.splitter = QtGui.QSplitter(MCLogTab)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setObjectName("splitter")
        self.y1Selector = QtGui.QTreeWidget(self.splitter)
        self.y1Selector.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.y1Selector.setSelectionBehavior(QtGui.QAbstractItemView.SelectItems)
        self.y1Selector.setColumnCount(2)
        self.y1Selector.setObjectName("y1Selector")
        self.y1Selector.headerItem().setText(0, "1")
        self.y1Selector.headerItem().setText(1, "2")
        self.y1Selector.header().setVisible(True)
        self.widget = QtGui.QWidget(self.splitter)
        self.widget.setObjectName("widget")
        self.verticalLayout = QtGui.QVBoxLayout(self.widget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.canvas = PlotCanvasWithToolbar(self.widget)
        self.canvas.setObjectName("canvas")
        self.verticalLayout.addWidget(self.canvas)
        self.xSelector = QtGui.QComboBox(self.widget)
        self.xSelector.setObjectName("xSelector")
        self.verticalLayout.addWidget(self.xSelector)
        self.y2Selector = QtGui.QTreeWidget(self.splitter)
        self.y2Selector.setSelectionMode(QtGui.QAbstractItemView.ExtendedSelection)
        self.y2Selector.setColumnCount(2)
        self.y2Selector.setObjectName("y2Selector")
        self.y2Selector.headerItem().setText(0, "1")
        self.y2Selector.headerItem().setText(1, "2")
        self.y2Selector.header().setVisible(True)
        self.horizontalLayout.addWidget(self.splitter)

        self.retranslateUi(MCLogTab)
        QtCore.QMetaObject.connectSlotsByName(MCLogTab)

    def retranslateUi(self, MCLogTab):
        MCLogTab.setWindowTitle(QtGui.QApplication.translate("MCLogTab", "MCLogTab", None, QtGui.QApplication.UnicodeUTF8))

from mc_log_plotcanvas import PlotCanvasWithToolbar
