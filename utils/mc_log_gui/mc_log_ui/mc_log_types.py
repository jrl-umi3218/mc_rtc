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

class TextWithFontSize(object):
  def __init__(self, text = "", fontsize = 10):
    self.text = text
    self.fontsize = fontsize

class GraphLabels(object):
  def __init__(self, title = TextWithFontSize(fontsize = 12), x_label = TextWithFontSize(), y1_label = TextWithFontSize(), y2_label = TextWithFontSize()):
    self.title = title
    self.x_label = x_label
    self.y1_label = y1_label
    self.y2_label = y2_label
