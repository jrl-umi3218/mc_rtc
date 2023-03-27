#include <mc_control/Ticker.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char * argv[])
{
  mc_control::Ticker::Configuration config;
  {
    bool replay_outputs = false;
    bool only_gui_inputs = false;
    po::options_description desc("mc_rtc_ticker options");
    // clang-format off
    desc.add_options()
      ("help", "Show this help message")
      ("mc-config,f", po::value<std::string>(&config.mc_rtc_configuration), "Configuration given to mc_rtc")
      ("step-by-step,S", po::bool_switch(&config.step_by_step), "Start the ticker in step-by-step mode")
      ("run-for", po::value<double>(&config.run_for), "Run for the specified time (seconds)")
      ("no-sync,s", po::bool_switch(&config.no_sync), "Synchronize ticker time with real time")
      ("sync-ratio,r", po::value<double>(&config.sync_ratio), "Sim/real ratio for synchronization purpose")
      ("replay-log,l", po::value<std::string>(&config.replay_configuration.log), "Log to replay")
      ("datastore-mapping,m", po::value<std::string>(&config.replay_configuration.with_datastore_config), "Mapping of log keys to datastore")
      ("replay-gui-inputs-only,g", po::bool_switch(&only_gui_inputs), "Only replay the GUI inputs")
      ("exit-after-replay,e", po::bool_switch(&config.replay_configuration.exit_after_log), "Exit after log replay")
      ("replay-outputs", po::bool_switch(&replay_outputs), "Enable outputs replay (override controller)");
    // clang-format on
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
      std::cout << desc << "\n";
      return 0;
    }
    if(replay_outputs)
    {
      config.replay_configuration.with_outputs = true;
    }
    if(only_gui_inputs)
    {
      if(replay_outputs)
      {
        mc_rtc::log::error_and_throw("--replay-outputs is contradictory with --replay-gui-inputs-only");
      }
      config.replay_configuration.with_inputs = false;
      config.replay_configuration.with_outputs = false;
    }
    if(config.sync_ratio <= 0)
    {
      mc_rtc::log::error_and_throw("sync-ratio must be strictly positive");
    }
  }
  mc_control::Ticker ticker(config);
  ticker.run();
  return 0;
}
