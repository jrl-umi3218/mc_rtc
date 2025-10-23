/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "python_controller.h"

#include <mc_rtc/mc_rtc_python.h>

#include <mc_control/mc_python_controller.h>

#include "mc_control/mc_control.h"
#include "mc_rbdyn/mc_rbdyn.h"

extern "C"
{
  void MC_RTC_CONTROLLER(std::vector<std::string> & names)
  {
    names = {"@PYTHON_NAME@"};
  }
  void destroy(mc_control::MCController * ptr)
  {
    // The Python object should be garbage collected, so we only take care of
    // the C++ memory we allocated
    delete ptr;
    mc_rtc::python::Finalize();
  }
  mc_control::MCController * create(const std::string &,
                                    const std::string & controller_name,
                                    const std::shared_ptr<mc_rbdyn::RobotModule> & robot,
                                    const double & dt,
                                    const mc_control::Configuration &)
  {
    mc_rtc::log::info("[PythonController] Running with Python {}.{}.{}", PY_MAJOR_VERSION, PY_MINOR_VERSION,
                      PY_MICRO_VERSION);
    mc_rtc::python::Initialize();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    auto gstate = PyGILState_Ensure();
#pragma GCC diagnostic pop
    PySys_SetArgvEx(0, {}, 0);

    auto sys_path_object = PySys_GetObject("path");
    PyErr_Print();
    auto sys_path_repr = PyObject_Repr(sys_path_object);
    PyErr_Print();
#if PY_MAJOR_VERSION > 2
    auto sys_path_str = PyUnicode_AsUTF8String(sys_path_repr);
    PyErr_Print();
    auto sys_path = PyBytes_AsString(sys_path_str);
#else
    auto sys_path = PyString_AsString(sys_path_repr);
#endif
    PyErr_Print();
    mc_rtc::log::info("[PythonController] sys.path: {}", sys_path);

    auto signal_mod = PyImport_ImportModule("signal");
    PyErr_Print();
    auto signal_signal = PyObject_GetAttrString(signal_mod, "signal");
    PyErr_Print();
    auto signal_getsignal = PyObject_GetAttrString(signal_mod, "getsignal");
    PyErr_Print();
    auto signal_default_int_handler = PyObject_GetAttrString(signal_mod, "default_int_handler");
    PyErr_Print();
    auto signal_SIGINT = PyObject_GetAttrString(signal_mod, "SIGINT");
    PyErr_Print();
    auto signal_SIG_DFL = PyObject_GetAttrString(signal_mod, "SIG_DFL");
    PyErr_Print();
    if(signal_signal && signal_SIGINT && signal_SIG_DFL && signal_getsignal && signal_default_int_handler)
    {
      auto handler = PyObject_CallFunctionObjArgs(signal_getsignal, signal_SIGINT, NULL);
      if(handler == signal_default_int_handler)
      {
        PyObject_CallFunctionObjArgs(signal_signal, signal_SIGINT, signal_SIG_DFL, NULL);
      }
    }
    else
    {
      mc_rtc::log::error(
          "Failed to get signal.signal, signal.getsignal, signal.default_int_handler, signal.SIGINT "
          "and/or signal.SIG_DFL");
    }

    auto mc_rbdyn_mod = PyImport_ImportModule("mc_rbdyn");
    PyErr_Print();

    std::string mod_name = "mc_python_controllers";
    std::string c_name = controller_name;
    auto dot_pos = controller_name.find('.');
    if(dot_pos != std::string::npos)
    {
      mod_name = controller_name.substr(0, dot_pos);
      c_name = controller_name.substr(dot_pos + 1);
    }
    PyObject * mc_python_controllers_mod = PyImport_ImportModule(mod_name.c_str());
    PyErr_Print();

    auto mc_controller_obj_class = PyObject_GetAttrString(mc_rbdyn_mod, "RobotModule");
    RobotModuleObject * python_robot = (RobotModuleObject *)(PyObject_CallObject(mc_controller_obj_class, NULL));
    python_robot->impl = robot;

    auto cc_class = PyObject_GetAttrString(mc_python_controllers_mod, c_name.c_str());
    PyErr_Print();
    auto cc_obj = PyObject_GetAttrString(cc_class, "create");
    PyErr_Print();
    auto args = Py_BuildValue("Od", (PyObject *)python_robot, dt);
    auto c_obj = PyObject_CallObject(cc_obj, args);
    PyErr_Print();

    PyGILState_Release(gstate);
    MCPythonControllerObject * res = (MCPythonControllerObject *)(c_obj);
    res->impl->handle_python_error = []() -> bool
    {
      auto gstate = PyGILState_Ensure();
      auto error = PyErr_Occurred();
      if(error)
      {
        mc_rtc::log::error("[PythonController] Fatal error in Python module");
        PyErr_Print();
      }
      PyGILState_Release(gstate);
      return error != nullptr;
    };
    return res->impl;
  }
  void LOAD_GLOBAL() {}
}
