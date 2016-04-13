#include "MCControlTCP.h"

#include <tcp_control/hrp4_data.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char **argv)
{
  std::string conf_file = mc_rtc::CONF_PATH;
  std::string host = "hrp4005c";
  po::options_description desc("MCControlTCP options");
  desc.add_options()
    ("help", "display help message")
    ("host,h", po::value<std::string>(&host)->default_value("hrp4005c"), "connection host")
    ("conf,f", po::value<std::string>(&conf_file)->default_value(mc_rtc::CONF_PATH), "configuration file");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  mc_control::MCGlobalController controller(conf_file);
  if(controller.robot().name() != "hrp4")
  {
    std::cerr << "This program can only handle hrp4 at the moment" << std::endl;
    return 1;
  }

  MCControlTCP nodeWrapper(host, controller);
  nodeWrapper.initialize();
  nodeWrapper.start<HRP4OpenHRPSensors, HRP4OpenHRPControl>();

  return 0;
}
