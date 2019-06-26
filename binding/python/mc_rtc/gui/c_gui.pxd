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

cdef extern from "<mc_rtc/GUIState.h>" namespace "mc_rtc::gui":

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

  cdef cppclass FormCheckbox:
    FormCheckbox()
    FormCheckbox(const string&, cppbool)
    FormCheckbox(const string&, cppbool, cppbool)

  cdef cppclass FormIntegerInput:
    FormIntegerInput()
    FormIntegerInput(const string&, cppbool)
    FormIntegerInput(const string&, cppbool, const int&)

  cdef cppclass FormNumberInput:
    FormNumberInput()
    FormNumberInput(const string&, cppbool)
    FormNumberInput(const string&, cppbool, const double&)

  cdef cppclass FormStringInput:
    FormStringInput()
    FormStringInput(const string&, cppbool)
    FormStringInput(const string&, cppbool, const string&)

  cdef cppclass FormArrayInput[T]:
    FormArrayInput()
    FormArrayInput(const string&, cppbool, const T&, cppbool)
    FormArrayInput(const string&, cppbool, cppbool)

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
