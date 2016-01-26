#include <mc_control/mc_python_controller.h>

namespace mc_control
{

MCPythonController::MCPythonController(const std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> & robots, double dt)
: MCController(robots, dt)
{
}

void MCPythonController::reset(const ControllerResetData& reset_data)
{
  MCController::reset(reset_data);
  if(reset_callback)
  {
    reset_callback(reset_data);
  }
}

bool MCPythonController::run()
{
  bool ret = MCController::run();
  if(ret)
  {
    if(run_callback)
    {
      ret = ret && run_callback();
    }
  }
  return ret;
}

bool MCPythonController::read_msg(std::string & msg)
{
  if(read_msg_callback)
  {
    return read_msg_callback(msg);
  }
  return false;
}

bool MCPythonController::read_write_msg(std::string & msg, std::string & out)
{
  if(read_write_msg_callback)
  {
    auto res = read_write_msg_callback(msg);
    out = res.out;
    return res.success;
  }
  return false;
}

std::ostream& MCPythonController::log_header(std::ostream & os)
{
  if(log_header_callback)
  {
    os << ";" << log_header_callback();
  }
  return os;
}

std::ostream& MCPythonController::log_data(std::ostream & os)
{
  if(log_data_callback)
  {
    os << ";" << log_data_callback();
  }
  return os;
}

}
