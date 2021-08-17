/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "python_controller.h"

#pragma GCC diagnostic push
#ifdef __clang__
#  pragma GCC diagnostic ignored "-Wdeprecated-register"
#endif
#include <Python.h>
#pragma GCC diagnostic pop

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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    auto gstate = PyGILState_Ensure();
#pragma GCC diagnostic pop
    Py_Finalize();
  }
  mc_control::MCController * create(const std::string &,
                                    const std::string & controller_name,
                                    const std::shared_ptr<mc_rbdyn::RobotModule> & robot,
                                    const double & dt,
                                    const mc_control::Configuration &)
  {
    if(!Py_IsInitialized())
    {
      Py_Initialize();
      PyEval_SaveThread();
    }
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    auto gstate = PyGILState_Ensure();
#pragma GCC diagnostic pop
    PySys_SetArgvEx(0, {}, 0);

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
      mc_rtc::log::error("Failed to get signal.signal, signal.getsignal, signal.default_int_handler, signal.SIGINT "
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
    MCControllerObject * res = (MCControllerObject *)(c_obj);
    return res->base;
  }
  void LOAD_GLOBAL() {}
}
