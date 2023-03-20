#include <mc_control/Ticker.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  mc_control::Ticker::Configuration config;
  {
    bool no_replay_outputs = false;
    bool only_gui_inputs = false;
    po::options_description desc("mc_rtc_ticker options");
    // clang-format off
    desc.add_options()
      ("help", "Show this help message")
      ("mc-config,f", po::value<std::string>(&config.mc_rtc_configuration), "Configuration given to mc_rtc")
      ("step-by-step,S", po::bool_switch(&config.step_by_step), "Start the ticker in step-by-step mode")
      ("run-for", po::value<double>(&config.run_for), "Run for the specified time (seconds)")
      ("no-sync,s", po::bool_switch(&config.no_sync), "Synchronize ticker time with real time")
      ("replay-log,l", po::value<std::string>(&config.replay_configuration.log), "Log to replay")
      ("datastore-mapping,m", po::value<std::string>(&config.replay_configuration.with_datastore_config), "Mapping of log keys to datastore")
      ("replay-gui-inputs-only,g", po::bool_switch(&only_gui_inputs), "Only replay the GUI inputs")
      ("replay-no-outputs", po::bool_switch(&no_replay_outputs), "Disable outputs replay");
    // clang-format on
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
      std::cout << desc << "\n";
      return 0;
    }
    if(no_replay_outputs)
    {
      config.replay_configuration.with_outputs = false;
    }
    if(only_gui_inputs)
    {
      if(no_replay_outputs)
      {
        mc_rtc::log::warning("--replay-no-outputs is redudant with --replay-gui-inputs-only");
      }
      config.replay_configuration.with_inputs = false;
      config.replay_configuration.with_outputs = false;
    }
  }
  mc_control::Ticker ticker(config);
  ticker.run();
  return 0;
}
