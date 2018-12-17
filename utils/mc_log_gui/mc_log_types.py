# -*- coding: utf-8 -*-

class LineStyle(object):
  def __init__(self, color = 'black', linestyle = '--', linewidth = 0.5, visible = False):
    self.color = color
    self.linestyle = linestyle
    self.linewidth = linewidth
    self.visible = visible
  def __repr__(self):
    return "color: {}, linestyle: {}, linewidth: {}, visible: {}".format(self.color, self.linestyle, self.linewidth, self.visible)
