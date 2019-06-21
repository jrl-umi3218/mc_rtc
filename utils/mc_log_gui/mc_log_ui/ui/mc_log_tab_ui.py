# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mc_log_tab_ui.ui'
#
# Created: Fri Jun 21 10:19:11 2019
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
        self.y1SelectorLayout = QtGui.QVBoxLayout()
        self.y1SelectorLayout.setObjectName("y1SelectorLayout")
        self.y1Selector = QtGui.QTreeWidget(MCLogTab)
        self.y1Selector.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)
        self.y1Selector.setSelectionBehavior(QtGui.QAbstractItemView.SelectItems)
        self.y1Selector.setColumnCount(1)
        self.y1Selector.setObjectName("y1Selector")
        self.y1Selector.headerItem().setText(0, "1")
        self.y1Selector.header().setVisible(True)
        self.y1SelectorLayout.addWidget(self.y1Selector)
        self.horizontalLayout.addLayout(self.y1SelectorLayout)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.canvas = PlotCanvasWithToolbar(MCLogTab)
        self.canvas.setObjectName("canvas")
        self.verticalLayout.addWidget(self.canvas)
        self.selectorLayout = QtGui.QHBoxLayout()
        self.selectorLayout.setObjectName("selectorLayout")
        self.xSelector = QtGui.QComboBox(MCLogTab)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.xSelector.sizePolicy().hasHeightForWidth())
        self.xSelector.setSizePolicy(sizePolicy)
        self.xSelector.setObjectName("xSelector")
        self.selectorLayout.addWidget(self.xSelector)
        self.verticalLayout.addLayout(self.selectorLayout)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.y2SelectorLayout = QtGui.QVBoxLayout()
        self.y2SelectorLayout.setObjectName("y2SelectorLayout")
        self.y2Selector = QtGui.QTreeWidget(MCLogTab)
        self.y2Selector.setSelectionMode(QtGui.QAbstractItemView.MultiSelection)
        self.y2Selector.setColumnCount(1)
        self.y2Selector.setObjectName("y2Selector")
        self.y2Selector.headerItem().setText(0, "1")
        self.y2Selector.header().setVisible(True)
        self.y2SelectorLayout.addWidget(self.y2Selector)
        self.horizontalLayout.addLayout(self.y2SelectorLayout)

        self.retranslateUi(MCLogTab)
        QtCore.QMetaObject.connectSlotsByName(MCLogTab)

    def retranslateUi(self, MCLogTab):
        MCLogTab.setWindowTitle(QtGui.QApplication.translate("MCLogTab", "MCLogTab", None, QtGui.QApplication.UnicodeUTF8))

from mc_log_ui.mc_log_plotcanvas import PlotCanvasWithToolbar
