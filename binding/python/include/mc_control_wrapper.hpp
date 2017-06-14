#pragma once

#include <Eigen/Core>
#include <mc_control/mc_python_controller.h>

#include <functional>
#include <memory>
#include <sstream>

namespace mc_control
{

ControllerResetData& const_cast_crd(const ControllerResetData & in)
{
  return const_cast<ControllerResetData&>(in);
}

typedef bool (*run_callback_t)(void*);
typedef void (*reset_callback_t)(const ControllerResetData&, void*);
typedef bool (*read_msg_callback_t)(std::string&, void*);
typedef PythonRWCallback (*read_write_msg_callback_t)(std::string&, void*);
typedef std::string (*log_header_callback_t)(void*);
typedef std::string (*log_data_callback_t)(void*);

void set_run_callback(MCPythonController & ctl, run_callback_t fn, void * data)
{
  ctl.run_callback = std::bind(fn, data);
}

void set_reset_callback(MCPythonController & ctl, reset_callback_t fn, void * data)
{
  ctl.reset_callback = std::bind(fn, std::placeholders::_1, data);
}

void set_read_msg_callback(MCPythonController & ctl, read_msg_callback_t fn, void * data)
{
  ctl.read_msg_callback = std::bind(fn, std::placeholders::_1, data);
}

void set_read_write_msg_callback(MCPythonController & ctl, read_write_msg_callback_t fn, void * data)
{
  ctl.read_write_msg_callback = std::bind(fn, std::placeholders::_1, data);
}

void set_log_header_callback(MCPythonController & ctl, log_header_callback_t fn, void * data)
{
  ctl.log_header_callback = std::bind(fn, data);
}

void set_log_data_callback(MCPythonController & ctl, log_data_callback_t fn, void * data)
{
  ctl.log_data_callback = std::bind(fn, data);
}

}
