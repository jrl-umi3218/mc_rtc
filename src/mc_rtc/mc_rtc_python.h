/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/** Safely initialize and finalize the Python interpreter multiple times */

#pragma GCC diagnostic push
#ifdef __clang__
#  pragma GCC diagnostic ignored "-Wdeprecated-register"
#endif
#pragma GCC diagnostic ignored "-Wregister"
#include "Python.h"
#pragma GCC diagnostic pop

namespace mc_rtc::python
{

inline static int PYTHON_INIT_COUNT = 0;

inline static void Initialize()
{
  if(!Py_IsInitialized())
  {
    PYTHON_INIT_COUNT = 1;
    Py_Initialize();
    PyEval_SaveThread();
  }
  else
  {
    PYTHON_INIT_COUNT += 1;
  }
}

inline static void Finalize()
{
  PYTHON_INIT_COUNT -= 1;
  if(PYTHON_INIT_COUNT <= 0)
  {
    PYTHON_INIT_COUNT = 0;
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
    auto gstate = PyGILState_Ensure();
#pragma GCC diagnostic pop
    Py_Finalize();
  }
}

} // namespace mc_rtc::python
