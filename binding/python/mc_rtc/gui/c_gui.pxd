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

cdef extern from "<cstddef>" namespace "std":
  cdef cppclass nullptr_t:
    pass

cdef extern from "<mc_rtc/gui.h>" namespace "mc_rtc::gui":
  cdef cppclass Element:
    Element()
    const string& name()

cdef extern from "<mc_rtc/gui.h>" namespace "mc_rtc::gui::details":
  cdef cppclass ButtonImpl[T](Element):
    pass

  cdef cppclass CheckboxImpl[GetT, SetT](Element):
    pass

  cdef cppclass LabelImpl[T](Element):
    pass

  cdef cppclass ArrayLabelImpl[T](Element):
    pass

  cdef cppclass IntegerInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass NumberInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass NumberSliderImpl[GetT, SetT](Element):
    pass

  cdef cppclass StringInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass ArrayInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass ComboInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass DataComboInputImpl[GetT, SetT](Element):
    pass

  cdef cppclass Point3DImpl[GetT, SetT](Element):
    pass

  cdef cppclass RotationImpl[GetT, SetT](Element):
    pass

  cdef cppclass TransformImpl[GetT, SetT](Element):
    pass

  cdef cppclass FormImpl[Callback](Element):
    # C++ method is a template method but this does not work well here
    void addElement(...)

cdef extern from "<mc_rtc/gui.h>" namespace "mc_rtc::gui":
  cdef LabelImpl[T] Label[T](const string&, T)

  cdef ArrayLabelImpl[T] ArrayLabel[T](const string&, T)
  cdef ArrayLabelImpl[T] ArrayLabel[T](const string&, const vector[string]&, T)

  cdef ButtonImpl[T] Button[T](const string&, T)

  cdef CheckboxImpl[GetT, SetT] Checkbox[GetT, SetT](const string&, GetT, SetT)

  cdef StringInputImpl[GetT, SetT] StringInput[GetT, SetT](const string&, GetT, SetT)

  cdef IntegerInputImpl[GetT, SetT] IntegerInput[GetT, SetT](const string&, GetT, SetT)

  cdef NumberInputImpl[GetT, SetT] NumberInput[GetT, SetT](const string&, GetT, SetT)

  cdef NumberSliderImpl[GetT, SetT] NumberSlider[GetT, SetT](const string&, GetT, SetT, double, double)

  cdef ArrayInputImpl[GetT, SetT] ArrayInput[GetT, SetT](const string&, GetT, SetT)
  cdef ArrayInputImpl[GetT, SetT] ArrayInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef ComboInputImpl[GetT, SetT] ComboInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef DataComboInputImpl[GetT, SetT] DataComboInput[GetT, SetT](const string&, const vector[string]&, GetT, SetT)

  cdef Point3DImpl[GetT, nullptr_t] Point3DRO"mc_rtc::gui::Point3D"[GetT, nullptr_t](const string&, GetT, void *)
  cdef Point3DImpl[GetT,SetT] Point3D[GetT,SetT](const string&, GetT, SetT)

  cdef RotationImpl[GetT, nullptr_t] RotationRO"mc_rtc::gui::Rotation"[GetT, nullptr_t](const string&, GetT)
  cdef RotationImpl[GetT,SetT] Rotation[GetT,SetT](const string&, GetT, SetT)

  cdef TransformImpl[GetT, nullptr_t] TransformRO"mc_rtc::gui::Transform"[GetT, nullptr_t](const string&, GetT)
  cdef TransformImpl[GetT,SetT] Transform[GetT,SetT](const string&, GetT, SetT)

  cdef cppclass FormComboInput:
    FormComboInput()
    FormComboInput(const string&, cppbool, const vector[string]&, cppbool)

  cdef cppclass FormDataComboInput:
    FormDataComboInput()
    FormDataComboInput(const string&, cppbool, const vector[string]&, cppbool)

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
