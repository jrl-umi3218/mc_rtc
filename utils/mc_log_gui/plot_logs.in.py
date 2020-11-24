#!/usr/bin/env @MC_LOG_UI_PYTHON_EXECUTABLE@
# -*- coding: utf-8 -*-

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

import json
import sys

from mc_log_ui import *
UserPlot.__new__.__defaults__ = (LineStyle(), LineStyle(), {}, {}, GraphLabels(), {}, PlotType(0))

import matplotlib.pyplot as plt
plt.rcParams['svg.fonttype'] = 'none'

def usage():
    print "{} [log] [plots] [format=png]".format(sys.argv[0])
    sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        usage()
    data = read_log(sys.argv[1])
    plots = load_UserPlots(sys.argv[2])
    format_ = "png"
    if len(sys.argv) > 3:
        format_ = sys.argv[3]
    for plot in plots:
        figure = MCLogTab.UserFigure(data, plot)
        figure.fig.set_size_inches(30, 20)
        figure.fig.savefig("{}.{}".format(plot.title, format_))
        plt.close(figure.fig)
