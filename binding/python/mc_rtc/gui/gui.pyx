# distutils: language = c++

cimport c_gui

cimport mc_rtc.mc_rtc as mc_rtc
cimport mc_rtc.c_mc_rtc as c_mc_rtc

from cython.operator cimport dereference as deref

from libcpp.vector cimport vector
from libcpp.string cimport string

cdef c_mc_rtc.Configuration python_to_conf(get_fn) with gil:
  return deref((<mc_rtc.Configuration>mc_rtc.Configuration.from_(get_fn())).impl)

cdef void conf_to_python(set_fn, type t, c_mc_rtc.Configuration & data) with gil:
  config = mc_rtc.ConfigurationFromRef(data)
  set_fn(config.to(t))

cdef void conf_to_python_list(set_fn, object t, c_mc_rtc.Configuration & data) with gil:
  config = mc_rtc.ConfigurationFromRef(data)
  set_fn(config.to(t))

cdef void void_cb(set_fn) with gil:
  set_fn()

cdef vector[string] py2cpp(values):
  return values

cdef class Element(object):
  cdef c_gui.Element * base
  def name(self):
    return self.base.name()

cdef class Label(Element):
  cdef c_gui.LabelImpl[c_gui.get_fn] impl
  cdef _get_fn
  def __cinit__(self, name, get_fn):
    self._get_fn = get_fn
    self.impl = c_gui.Label[c_gui.get_fn](name, c_gui.make_getter(python_to_conf, get_fn))
    self.base = &self.impl

cdef class ArrayLabel(Element):
  cdef c_gui.ArrayLabelImpl[c_gui.get_fn] impl
  cdef _get_fn
  def __cinit__(self, name, get_fn, labels = []):
    self._get_fn = get_fn
    self.impl = c_gui.ArrayLabel[c_gui.get_fn](name, py2cpp(labels), c_gui.make_getter(python_to_conf, get_fn))
    self.base = &self.impl

cdef class Button(Element):
  cdef c_gui.ButtonImpl[c_gui.void_cb] impl
  cdef _cb
  def __cinit__(self, name, cb):
    self._cb = cb
    self.impl = c_gui.Button[c_gui.void_cb](name, c_gui.make_void_cb(void_cb, cb))
    self.base = &self.impl

cdef class Checkbox(Element):
  cdef c_gui.CheckboxImpl[c_gui.get_fn,c_gui.void_cb] impl
  cdef _get_fn
  cdef _cb
  def __cinit__(self, name, get_fn, cb):
    self._get_fn = get_fn
    self._cb = cb
    self.impl = c_gui.Checkbox[c_gui.get_fn,c_gui.void_cb](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_void_cb(void_cb, cb))
    self.base = &self.impl

cdef class StringInput(Element):
  cdef c_gui.StringInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, get_fn, set_fn):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.StringInput[c_gui.get_fn, c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, str))
    self.base = &self.impl

cdef class IntegerInput(Element):
  cdef c_gui.IntegerInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, get_fn, set_fn):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.IntegerInput[c_gui.get_fn, c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, int))
    self.base = &self.impl

cdef class NumberInput(Element):
  cdef c_gui.NumberInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, get_fn, set_fn):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.NumberInput[c_gui.get_fn, c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, float))
    self.base = &self.impl

cdef class NumberSlider(Element):
  cdef c_gui.NumberSliderImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, get_fn, set_fn, min_, max_):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.NumberSlider[c_gui.get_fn, c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, float), min_, max_)
    self.base = &self.impl

cdef class ArrayInput(Element):
  cdef c_gui.ArrayInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  cdef _cb_ret_type
  def __cinit__(self, name, get_fn, set_fn, labels = []):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self._cb_ret_type = [float]
    self.impl = c_gui.ArrayInput[c_gui.get_fn, c_gui.set_fn](name, py2cpp(labels), c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python_list, set_fn, self._cb_ret_type))
    self.base = &self.impl

cdef class ComboInput(Element):
  cdef c_gui.ComboInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, values, get_fn, set_fn):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.ComboInput[c_gui.get_fn, c_gui.set_fn](name, py2cpp(values), c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, str))
    self.base = &self.impl

cdef class DataComboInput(Element):
  cdef c_gui.DataComboInputImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef _get_fn
  cdef _set_fn
  def __cinit__(self, name, ref, get_fn, set_fn):
    self._get_fn = get_fn
    self._set_fn = set_fn
    self.impl = c_gui.DataComboInput[c_gui.get_fn, c_gui.set_fn](name, py2cpp(ref), c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python, set_fn, str))
    self.base = &self.impl

cdef class Point3D(Element):
  cdef c_gui.Point3DROImpl[c_gui.get_fn] ro_impl
  cdef c_gui.Point3DImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef is_ro
  cdef _get_fn
  cdef _set_fn
  cdef _cb_ret_type
  def __cinit__(self, name, get_fn, set_fn = None):
    self.is_ro = set_fn is None
    self._get_fn = get_fn
    self._set_fn = set_fn
    if self.is_ro:
      self.ro_impl = c_gui.Point3DRO[c_gui.get_fn](name, c_gui.make_getter(python_to_conf, get_fn))
      self.base = &self.ro_impl
    else:
      self._cb_ret_type = [float]
      self.impl = c_gui.Point3D[c_gui.get_fn,c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python_list, set_fn, self._cb_ret_type))
      self.base = &self.impl

cdef class Rotation(Element):
  cdef c_gui.RotationROImpl[c_gui.get_fn] ro_impl
  cdef c_gui.RotationImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef is_ro
  cdef _get_fn
  cdef _set_fn
  cdef _cb_ret_type
  def __cinit__(self, name, get_fn, set_fn = None):
    self.is_ro = set_fn is None
    self._get_fn = get_fn
    self._set_fn = set_fn
    if self.is_ro:
      self.ro_impl = c_gui.RotationRO[c_gui.get_fn](name, c_gui.make_getter(python_to_conf, get_fn))
      self.base = &self.ro_impl
    else:
      self._cb_ret_type = [float]
      self.impl = c_gui.Rotation[c_gui.get_fn,c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python_list, set_fn, self._cb_ret_type))
      self.base = &self.impl

cdef class Transform(Element):
  cdef c_gui.TransformROImpl[c_gui.get_fn] ro_impl
  cdef c_gui.TransformImpl[c_gui.get_fn, c_gui.set_fn] impl
  cdef is_ro
  cdef _get_fn
  cdef _set_fn
  cdef _cb_ret_type
  def __cinit__(self, name, get_fn, set_fn = None):
    self.is_ro = set_fn is None
    self._get_fn = get_fn
    self._set_fn = set_fn
    if self.is_ro:
      self.ro_impl = c_gui.TransformRO[c_gui.get_fn](name, c_gui.make_getter(python_to_conf, get_fn))
      self.base = &self.ro_impl
    else:
      self._cb_ret_type = [float]
      self.impl = c_gui.Transform[c_gui.get_fn,c_gui.set_fn](name, c_gui.make_getter(python_to_conf, get_fn), c_gui.make_setter(conf_to_python_list, set_fn, self._cb_ret_type))
      self.base = &self.impl

cdef class FormCheckbox(object):
  cdef c_gui.FormCheckbox impl
  def __cinit__(self, name, required, default = None):
    if default is None:
      self.impl = c_gui.FormCheckbox(name, required)
    else:
      self.impl = c_gui.FormCheckbox(name, required, default)

cdef class FormIntegerInput(object):
  cdef c_gui.FormIntegerInput impl
  def __cinit__(self, name, required, default = None):
    if default is None:
      self.impl = c_gui.FormIntegerInput(name, required)
    else:
      self.impl = c_gui.FormIntegerInput(name, required, default)

cdef class FormNumberInput(object):
  cdef c_gui.FormNumberInput impl
  def __cinit__(self, name, required, default = None):
    if default is None:
      self.impl = c_gui.FormNumberInput(name, required)
    else:
      self.impl = c_gui.FormNumberInput(name, required, default)

cdef class FormStringInput(object):
  cdef c_gui.FormStringInput impl
  def __cinit__(self, name, required, default = None):
    if default is None:
      self.impl = c_gui.FormStringInput(name, required)
    else:
      self.impl = c_gui.FormStringInput(name, required, default)

cdef class FormNumberArrayInput(object):
  cdef c_gui.FormArrayInput[vector[double]] impl
  def __cinit__(self, name, required, default = None, fixed_size = None):
    if default is None:
      if fixed_size is None:
        fixed_size = False
      self.impl = c_gui.FormArrayInput[vector[double]](name, required, fixed_size)
    else:
      if fixed_size is None:
        fixed_size = True
      self.impl = c_gui.FormArrayInput[vector[double]](name, required, default, fixed_size)

cdef class FormStringArrayInput(object):
  cdef c_gui.FormArrayInput[vector[string]] impl
  def __cinit__(self, name, required, default = None, fixed_size = None):
    if default is None:
      if fixed_size is None:
        fixed_size = False
      self.impl = c_gui.FormArrayInput[vector[string]](name, required, fixed_size)
    else:
      if fixed_size is None:
        fixed_size = True
      self.impl = c_gui.FormArrayInput[vector[string]](name, required, py2cpp(default), fixed_size)

cdef class FormComboInput(object):
  cdef c_gui.FormComboInput impl
  def __cinit__(self, name, required, values, send_index = False):
    self.impl = c_gui.FormComboInput(name, required, py2cpp(values), send_index)

cdef class FormDataComboInput(object):
  cdef c_gui.FormDataComboInput impl
  def __cinit__(self, name, required, values, send_index = False):
    self.impl = c_gui.FormDataComboInput(name, required, py2cpp(values), send_index)

cdef class Form(Element):
  cdef c_gui.FormImpl[c_gui.set_fn] impl
  cdef _cb
  cdef _elements
  def __cinit__(self, name, cb, *args):
    self.impl = c_gui.Form[c_gui.set_fn](name, c_gui.make_setter(conf_to_python, cb, mc_rtc.Configuration))
    self.base = &self.impl
    self._cb = cb
    self._elements= []
    for arg in args:
      t = type(arg)
      self._elements.append(arg)
      if t is FormCheckbox:
        self.impl.addElement((<FormCheckbox>arg).impl)
      elif t is FormIntegerInput:
        self.impl.addElement((<FormIntegerInput>arg).impl)
      elif t is FormNumberInput:
        self.impl.addElement((<FormNumberInput>arg).impl)
      elif t is FormStringInput:
        self.impl.addElement((<FormStringInput>arg).impl)
      elif t is FormNumberArrayInput:
        self.impl.addElement((<FormNumberArrayInput>arg).impl)
      elif t is FormStringArrayInput:
        self.impl.addElement((<FormStringArrayInput>arg).impl)
      elif t is FormComboInput:
        self.impl.addElement((<FormComboInput>arg).impl)
      elif t is FormDataComboInput:
        self.impl.addElement((<FormDataComboInput>arg).impl)
      else:
        self._elements.remove(arg)
        print "Unsupported type {}, cannot be added to the form".format(t)


cdef class StateBuilder(object):
  ELEMENTS = {}
  def addElement(self, category, element, *args):
    assert(self.impl.get() != NULL)
    t = type(element)
    StateBuilder.ELEMENTS.setdefault("/".join(category), []).append(element)
    if t is Label:
      self.impl.get().addElement[c_gui.LabelImpl[c_gui.get_fn]](py2cpp(category), (<Label>element).impl)
    elif t is ArrayLabel:
      self.impl.get().addElement[c_gui.ArrayLabelImpl[c_gui.get_fn]](py2cpp(category), (<ArrayLabel>element).impl)
    elif t is Button:
      self.impl.get().addElement[c_gui.ButtonImpl[c_gui.void_cb]](py2cpp(category), (<Button>element).impl)
    elif t is Checkbox:
      self.impl.get().addElement[c_gui.CheckboxImpl[c_gui.get_fn,c_gui.void_cb]](py2cpp(category), (<Checkbox>element).impl)
    elif t is StringInput:
      self.impl.get().addElement[c_gui.StringInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<StringInput>element).impl)
    elif t is IntegerInput:
      self.impl.get().addElement[c_gui.IntegerInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<IntegerInput>element).impl)
    elif t is NumberInput:
      self.impl.get().addElement[c_gui.NumberInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<NumberInput>element).impl)
    elif t is NumberSlider:
      self.impl.get().addElement[c_gui.NumberSliderImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<NumberSlider>element).impl)
    elif t is ArrayInput:
      self.impl.get().addElement[c_gui.ArrayInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<ArrayInput>element).impl)
    elif t is ComboInput:
      self.impl.get().addElement[c_gui.ComboInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<ComboInput>element).impl)
    elif t is DataComboInput:
      self.impl.get().addElement[c_gui.DataComboInputImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<DataComboInput>element).impl)
    elif t is Point3D:
      is_ro = (<Point3D>element).is_ro
      if is_ro:
        self.impl.get().addElement[c_gui.Point3DROImpl[c_gui.get_fn]](py2cpp(category), (<Point3D>element).ro_impl)
      else:
        self.impl.get().addElement[c_gui.Point3DImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<Point3D>element).impl)
    elif t is Rotation:
      is_ro = (<Rotation>element).is_ro
      if is_ro:
        self.impl.get().addElement[c_gui.RotationROImpl[c_gui.get_fn]](py2cpp(category), (<Rotation>element).ro_impl)
      else:
        self.impl.get().addElement[c_gui.RotationImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<Rotation>element).impl)
    elif t is Transform:
      is_ro = (<Transform>element).is_ro
      if is_ro:
        self.impl.get().addElement[c_gui.TransformROImpl[c_gui.get_fn]](py2cpp(category), (<Transform>element).ro_impl)
      else:
        self.impl.get().addElement[c_gui.TransformImpl[c_gui.get_fn, c_gui.set_fn]](py2cpp(category), (<Transform>element).impl)
    elif t is Form:
      self.impl.get().addElement[c_gui.FormImpl[c_gui.set_fn]](py2cpp(category), (<Form>element).impl)
    else:
      StateBuilder.ELEMENTS["/".join(category)].remove(element)
      print "Unsupported type {}, this element will not be added".format(t)
    if len(args):
      self.addElement(category, args[0], *args[1:])
  def reset(self):
    self.impl.get().reset()
  def removeCategory(self, category):
    self.impl.get().removeCategory(py2cpp(category))
    StateBuilder.ELEMENTS.pop("/".join(category), None)
  def removeElement(self, category, name):
    self.impl.get().removeElement(py2cpp(category), name)
    for i, el in enumerate(StateBuilder.ELEMENTS.get("/".join(category), [])):
      if el.name() == name:
        StateBuilder.ELEMENTS["/".join(category)].pop(i)
        break
  def data(self):
    return mc_rtc.ConfigurationFromValue(self.impl.get().data())

cdef StateBuilder StateBuilderFromShPtr(c_gui.shared_ptr[c_gui.StateBuilder] ptr):
  cdef StateBuilder ret = StateBuilder()
  ret.impl = ptr
  return ret
