# -*- coding: utf-8 -*-

class LineStyle(object):
  def __init__(self, color = 'black', linestyle = '--', linewidth = 0.5, visible = False, label = ""):
    self.color = color
    self.linestyle = linestyle
    self.linewidth = linewidth
    self.visible = visible
    self.label = label
  def __repr__(self):
    return "color: {}, linestyle: {}, linewidth: {}, visible: {}, label: {}".format(self.color, self.linestyle, self.linewidth, self.visible, self.label)
