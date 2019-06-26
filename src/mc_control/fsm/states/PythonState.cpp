/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/PythonState.h>

// clang-format off

extern "C"
{
#include "Python.h"
}
#include "mc_control/fsm/fsm.h"

extern "C"
{

#include <stddef.h> // offsetof

FSM_STATE_API void MC_RTC_FSM_STATE(std::vector<std::string> & names)
{
  names = {"@PYTHON_NAME@"};
}

FSM_STATE_API void destroy(mc_control::fsm::State * ptr)
{
  auto gstate = PyGILState_Ensure();
  Py_Finalize();
}

FSM_STATE_API void LOAD_GLOBAL() {}

FSM_STATE_API mc_control::fsm::State * create(const std::string &, const std::string & module)
{
  /** TODO */
  if(!Py_IsInitialized())
  {
    Py_Initialize();
    PyEval_SaveThread();
  }
  auto gstate = PyGILState_Ensure();
  PySys_SetArgvEx(0, {}, 0);

  std::string mod_name = "mc_python_states";
  std::string s_name = module;
  auto dot_pos = module.rfind('.');
  if(dot_pos != std::string::npos)
  {
    mod_name = module.substr(0, dot_pos);
    s_name = module.substr(dot_pos + 1);
  }
  auto mod = PyImport_ImportModule(mod_name.c_str());
  PyErr_Print();

  auto s_class = PyObject_GetAttrString(mod, s_name.c_str());
  PyErr_Print();
  auto s_obj = PyObject_CallObject(s_class, nullptr);
  PyErr_Print();

  PyGILState_Release(gstate);
  auto res = reinterpret_cast<PythonStateObject *>(s_obj);
  return res->impl;
}

}

// clang-format on
