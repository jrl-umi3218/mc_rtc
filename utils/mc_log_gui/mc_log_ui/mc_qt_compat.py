# -*- coding: utf-8 -*-

#
# Copyright 2015-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

import warnings

try:
    from PyQt6 import QtCore, QtGui, QtWidgets

    QT_API = "PyQt6"
    QT_VERSION = 6
except ImportError:
    from PyQt5 import QtCore, QtGui, QtWidgets

    QT_API = "PyQt5"
    QT_VERSION = 5


def get_matplotlib_backend():
    return "Qt6Agg" if QT_VERSION == 6 else "Qt5Agg"


def configure_matplotlib():
    import matplotlib

    try:
        matplotlib.use(get_matplotlib_backend())
    except (ImportError, ValueError) as exc:
        warnings.warn(
            "Failed to configure matplotlib backend {} (non-fatal, mc_log_ui can keep running): {}".format(
                get_matplotlib_backend(),
                exc,
            )
        )


if QT_VERSION == 6:
    _qt_aliases = {
        "LeftButton": QtCore.Qt.MouseButton.LeftButton,
        "RightButton": QtCore.Qt.MouseButton.RightButton,
        "MiddleButton": QtCore.Qt.MouseButton.MiddleButton,
        "CustomContextMenu": QtCore.Qt.ContextMenuPolicy.CustomContextMenu,
        "MatchStartsWith": QtCore.Qt.MatchFlag.MatchStartsWith,
        "MatchRecursive": QtCore.Qt.MatchFlag.MatchRecursive,
        "MatchWildcard": QtCore.Qt.MatchFlag.MatchWildcard,
        "DisplayRole": QtCore.Qt.ItemDataRole.DisplayRole,
        "ApplicationShortcut": QtCore.Qt.ShortcutContext.ApplicationShortcut,
        "LeftToRight": QtCore.Qt.LayoutDirection.LeftToRight,
        "CTRL": QtCore.Qt.KeyboardModifier.ControlModifier,
        "Key_O": QtCore.Qt.Key.Key_O,
        "Key_W": QtCore.Qt.Key.Key_W,
        "Key_PageDown": QtCore.Qt.Key.Key_PageDown,
        "Key_PageUp": QtCore.Qt.Key.Key_PageUp,
        "Key_T": QtCore.Qt.Key.Key_T,
        "Key_S": QtCore.Qt.Key.Key_S,
        "Key_A": QtCore.Qt.Key.Key_A,
    }
    for _name, _value in _qt_aliases.items():
        # Keep compatibility with both native PyQt6 enums and potential pre-existing aliases.
        if not hasattr(QtCore.Qt, _name):
            setattr(QtCore.Qt, _name, _value)

    _qwidget_aliases = [
        (
            QtWidgets.QAbstractItemView,
            {
                "MultiSelection": QtWidgets.QAbstractItemView.SelectionMode.MultiSelection,
                "ExtendedSelection": QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection,
                "SelectItems": QtWidgets.QAbstractItemView.SelectionBehavior.SelectItems,
            },
        ),
        (
            QtWidgets.QHeaderView,
            {"Fixed": QtWidgets.QHeaderView.ResizeMode.Fixed},
        ),
        (
            QtWidgets.QSizePolicy,
            {
                "Maximum": QtWidgets.QSizePolicy.Policy.Maximum,
                "Expanding": QtWidgets.QSizePolicy.Policy.Expanding,
                "Fixed": QtWidgets.QSizePolicy.Policy.Fixed,
            },
        ),
        (
            QtWidgets.QMessageBox,
            {
                "Yes": QtWidgets.QMessageBox.StandardButton.Yes,
                "No": QtWidgets.QMessageBox.StandardButton.No,
                "Critical": QtWidgets.QMessageBox.Icon.Critical,
            },
        ),
        (
            QtWidgets.QTabBar,
            {"RightSide": QtWidgets.QTabBar.ButtonPosition.RightSide},
        ),
    ]
    for _klass, _aliases in _qwidget_aliases:
        for _name, _value in _aliases.items():
            if not hasattr(_klass, _name):
                setattr(_klass, _name, _value)

    for _klass in [
        QtWidgets.QApplication,
        QtWidgets.QDialog,
        QtWidgets.QMenu,
        QtWidgets.QMessageBox,
    ]:
        if not hasattr(_klass, "exec_") and hasattr(_klass, "exec"):
            setattr(_klass, "exec_", _klass.exec)


__all__ = [
    "QtCore",
    "QtGui",
    "QtWidgets",
    "QT_API",
    "QT_VERSION",
    "get_matplotlib_backend",
    "configure_matplotlib",
]
