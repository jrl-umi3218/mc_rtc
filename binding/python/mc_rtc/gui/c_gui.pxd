#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_rtc.c_mc_rtc as c_mc_rtc

from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp.map cimport map as cppmap  # Careful map is a python built-in
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr()
    shared_ptr(const shared_ptr[T]&)
    T * get()

cdef extern from "<mc_rtc/gui.h>" namespace "mc_rtc::gui":

  cdef cppclass Element:
    Element()
    const string& name()

  cdef cppclass LabelImpl[T](Element):
    pass
  cdef LabelImpl[T] Label[T](const string&, T)

  cdef cppclass ArrayLabelImpl[T](Element):
    pass
  cdef ArrayLabelImpl[T] ArrayLabel[T](const string&, T)
  cdef ArrayLabelImpl[T] ArrayLabel[T](const string&, const vector[string]&, T)

  cdef cppclass ButtonImpl[T](Element):
    pass
  cdef ButtonImpl[T] Button[T](const string&, T)

  cdef cppclass CheckboxImpl[GetT, SetT](Element):
    pass
  cdef CheckboxImpl[GetT, SetT] Checkbox[GetT, SetT](const string&, GetT, SetT)

  cdef cppclass StringInputImpl[GetT, SetT](Element):
    pass
  cdef StringInputImpl[GetT, SetT] StringInput[GetT, SetT](const string&, GetT, SetT)

  cdef cppclass IntegerInputImpl[GetT, SetT](Element):
    pass
  cdef IntegerInputImpl[GetT, SetT] IntegerInput[GetT, SetT](const string&, GetT, SetT)

  cdef cppclass NumberInputImpl[GetT, SetT](Element):
    pass
  cdef NumberInputImpl[GetT, SetT] NumberInput[GetT, SetT](const string&, GetT, SetT)

  cdef cppclass NumberSliderImpl[GetT, SetT](Element):
    pass
  cdef NumberSliderImpl[GetT, SetT] NumberSlider[GetT, SetT](const string&, GetT, SetT, double, double)

  cdef cppclass ArrayInputImpl[GetT, SetT](Element):
    pass
  cdef ArrayInputImpl[GetT, SetT] ArrayInput[GetT, SetT](const string&, GetT, SetT)
  cdef ArrayInputImpl[GetT, SetT] ArrayInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef cppclass ComboInputImpl[GetT, SetT](Element):
    pass
  cdef ComboInputImpl[GetT, SetT] ComboInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef cppclass DataComboInputImpl[GetT, SetT](Element):
    pass
  cdef DataComboInputImpl[GetT, SetT] DataComboInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef cppclass Point3DROImpl[GetT](Element):
    pass
  cdef Point3DROImpl[GetT] Point3DRO"mc_rtc::gui::Point3D"[GetT](const string&, GetT)
  cdef cppclass Point3DImpl[GetT, SetT](Element):
    pass
  cdef Point3DImpl[GetT,SetT] Point3D[GetT,SetT](const string&, GetT, SetT)

  cdef cppclass RotationROImpl[GetT](Element):
    pass
  cdef RotationROImpl[GetT] RotationRO"mc_rtc::gui::Rotation"[GetT](const string&, GetT)
  cdef cppclass RotationImpl[GetT, SetT](Element):
    pass
  cdef RotationImpl[GetT,SetT] Rotation[GetT,SetT](const string&, GetT, SetT)

  cdef cppclass TransformROImpl[GetT](Element):
    pass
  cdef TransformROImpl[GetT] TransformRO"mc_rtc::gui::Transform"[GetT](const string&, GetT)
  cdef cppclass TransformImpl[GetT, SetT](Element):
    pass
  cdef TransformImpl[GetT,SetT] Transform[GetT,SetT](const string&, GetT, SetT)

  cdef cppclass FormComboInput:
    FormComboInput()
    FormComboInput(const string&, cppbool, const vector[string]&, cppbool)

  cdef cppclass FormDataComboInput:
    FormDataComboInput()
    FormDataComboInput(const string&, cppbool, const vector[string]&, cppbool)

  cdef cppclass FormImpl[Callback](Element):
    # C++ method is a template method but this does not work well here
    void addElement(...)
  cdef FormImpl[Callback] Form[Callback](const string&, Callback)

  cdef cppclass StateBuilder:
    void addElement[T](const vector[string]&, T)
    void reset()
    void removeCategory(const vector[string]&)
    void removeElement(const vector[string]&, const string&)
    c_mc_rtc.Configuration data()

cdef extern from "mc_rtc_gui_wrapper.hpp":
  cdef cppclass get_fn:
    pass
  cdef cppclass set_fn:
    pass
  cdef cppclass void_cb:
    pass

  get_fn make_getter[T,U](T,U)
  set_fn make_setter[T,Cb,t](T, Cb, t)
  void_cb make_void_cb[T, Cb](T, Cb)

  void add_form_checkbox[Callback](FormImpl[Callback] & form, const string & name, cppbool required)
  void add_form_checkbox[Callback](FormImpl[Callback] & form, const string & name, cppbool required, cppbool value)

  void add_form_integer[Callback](FormImpl[Callback] & form, const string & name, cppbool required)
  void add_form_integer[Callback](FormImpl[Callback] & form, const string & name, cppbool required, int value)

  void add_form_number[Callback](FormImpl[Callback] & form, const string & name, cppbool required)
  void add_form_number[Callback](FormImpl[Callback] & form, const string & name, cppbool required, double value)

  void add_form_string[Callback](FormImpl[Callback] & form, const string & name, cppbool required)
  void add_form_string[Callback](FormImpl[Callback] & form, const string & name, cppbool required, const string & value)

  void add_form_number_array[Callback](FormImpl[Callback] & form, const string & name, cppbool required, cppbool fixed_size)
  void add_form_number_array[Callback](FormImpl[Callback] & form, const string & name, cppbool required, const vector[double] & value, cppbool fixed_size)

  void add_form_string_array[Callback](FormImpl[Callback] & form, const string & name, cppbool required, cppbool fixed_size)
  void add_form_string_array[Callback](FormImpl[Callback] & form, const string & name, cppbool required, const vector[string] & value, cppbool fixed_size)
