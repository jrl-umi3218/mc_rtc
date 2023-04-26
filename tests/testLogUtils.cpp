#include "utils.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>

struct LogData
{
private:
  /** Some data members supported by the logger */
  bool b = random_bool();
  double d = rnd();
  std::string s = random_string();
  mutable Eigen::Vector2d v2d = Eigen::Vector2d::Random();
  mutable Eigen::Vector3d v3d = Eigen::Vector3d::Random();
  mutable Eigen::Vector6d v6d = Eigen::Vector6d::Random();
  mutable Eigen::VectorXd vxd = Eigen::VectorXd::Random(static_cast<Eigen::DenseIndex>(random_size()));
  Eigen::Quaterniond q = random_quat();
  sva::PTransformd pt = random_pt();
  sva::ForceVecd fv = random_fv();
  sva::MotionVecd mv = random_mv();
  std::vector<double> v = random_vector();

public:
  /** Change data */
  void refresh() { *this = LogData{}; }
  void addToLogger(mc_rtc::Logger & logger)
  {
#define ADD_LOG_ENTRY(NAME, MEMBER) logger.addLogEntry(NAME, this, [this]() { return MEMBER; })
    ADD_LOG_ENTRY("bool", b);
    ADD_LOG_ENTRY("double", d);
    ADD_LOG_ENTRY("std::string", s);
    ADD_LOG_ENTRY("Eigen::Vector2d", v2d);
    ADD_LOG_ENTRY("Eigen::Vector3d", v3d);
    ADD_LOG_ENTRY("Eigen::Vector6d", v6d);
    ADD_LOG_ENTRY("Eigen::VectorXd", vxd);
    ADD_LOG_ENTRY("Eigen::Quaterniond", q);
    ADD_LOG_ENTRY("sva::PTransformd", pt);
    ADD_LOG_ENTRY("sva::ForceVecd", fv);
    ADD_LOG_ENTRY("sva::MotionVecd", mv);
    ADD_LOG_ENTRY("std::vector<double>", v);
#undef ADD_LOG_ENTRY
  }
  void removeFromLogger(mc_rtc::Logger & logger) { logger.removeLogEntries(this); }
};

std::string make_log_ref()
{
  using Policy = mc_rtc::Logger::Policy;
  mc_rtc::Logger logger(Policy::NON_THREADED, bfs::temp_directory_path().string(), "mc-rtc-test");
  logger.start("log-utils", 0.001);
  auto log_s = [&](size_t sec)
  {
    size_t n_events = 1;
    for(size_t i = 0; i < sec * 1000; ++i)
    {
      if(i % 247 == 0)
      {
        mc_rtc::Configuration data;
        for(size_t j = 0; j < n_events; ++j)
        {
          data.add("data", i + j);
          logger.addGUIEvent(mc_rtc::Logger::GUIEvent{{"My", "Category"}, "MyObject", data("data")});
        }
        n_events += 1;
      }
      logger.log();
    }
  };
  LogData data;
  data.addToLogger(logger);
  /** Log for 10 seconds */
  log_s(10);
  /** Add some data for the next second */
  logger.addLogEntry("data", []() { return 42.42; });
  /** Log for 5 seconds */
  log_s(5);
  /** Remove the data */
  logger.removeLogEntry("data");
  /** Log for 10 more seconds */
  log_s(10);
  auto latest = bfs::temp_directory_path() / "mc-rtc-test-log-utils-latest.bin";
  if(bfs::exists(latest)) { bfs::remove(latest); }
  return logger.path();
}

void do_cleanup(const std::string & path)
{
  if(bfs::exists(path)) { bfs::remove(path); }
}

bool check_split(const std::string & path)
{
  auto out = fmt::format("{}/mc-rtc-test-log-utils-split", bfs::temp_directory_path().string());
  auto split_cmd = fmt::format("{} split {} {} 2", MC_BIN_UTILS, path, out);
  int err = system(split_cmd.c_str());
  if(err != 0)
  {
    mc_rtc::log::critical("Execution failed: {}", split_cmd);
    return false;
  }
  auto path_1 = out + "_1.bin";
  auto path_2 = out + "_2.bin";
  if(!bfs::exists(path_1) || !bfs::exists(path_2))
  {
    mc_rtc::log::critical("No split files found at {} or {}", path_1, path_2);
    return false;
  }
  auto flat = mc_rtc::log::FlatLog(path);
  auto flat_1 = mc_rtc::log::FlatLog(path_1);
  auto flat_2 = mc_rtc::log::FlatLog(path_2);
  auto do_return = [&](bool res)
  {
    do_cleanup(path_1);
    do_cleanup(path_2);
    return res;
  };
  if(flat.size() != flat_1.size() + flat_2.size())
  {
    mc_rtc::log::critical("Split files size not equal to total log size ({} != {} + {})", flat.size(), flat_1.size(),
                          flat_2.size());
    return do_return(false);
  }
  auto last_t1 = flat_1.get<double>("t", flat_1.size() - 1, 0.0);
  auto first_t2 = flat_2.get<double>("t", 0, 0.0);
  if(last_t1 != first_t2 - 0.001)
  {
    mc_rtc::log::critical("Part 2 does not start where part 1 ends");
    return do_return(false);
  }
  // Note: this works because the log should be split around half-time
  if(flat_1.entries() != flat_2.entries())
  {
    mc_rtc::log::critical("We expect the split to share the same keys");
    return do_return(false);
  }
  return do_return(true);
}

bool check_extract_time(const std::string & path)
{
  auto out = fmt::format("{}/mc-rtc-test-log-utils-extract-time", bfs::temp_directory_path().string());
  auto extract_cmd = fmt::format("{} extract {} {} --from 5 --to 15", MC_BIN_UTILS, path, out);
  int err = system(extract_cmd.c_str());
  if(err != 0)
  {
    mc_rtc::log::critical("Execution failed: {}", extract_cmd);
    return false;
  }
  auto path_out = out + "_from_5_to_15.bin";
  if(!bfs::exists(path_out))
  {
    mc_rtc::log::critical("No output file after command: {}", extract_cmd);
    return false;
  }
  auto do_return = [&](bool ret)
  {
    do_cleanup(path_out);
    return ret;
  };
  auto flat = mc_rtc::log::FlatLog(path_out);
  if(flat.size() != 10 * 1000 + 1)
  {
    mc_rtc::log::critical("Flat size different from expected (expected: {}, actual: {})", 10 * 1000 + 1, flat.size());
    return do_return(false);
  }
  auto t_0 = flat.get<double>("t", 0, 0.0);
  if(std::fabs(t_0 - 5.0) > 1e-6)
  {
    mc_rtc::log::critical("First time entry different from expected 5.0 (got {})", t_0);
    return do_return(false);
  }
  auto t_final = flat.get<double>("t", flat.size() - 1, 0.0);
  if(std::fabs(t_final - 15.0) > 1e-6)
  {
    mc_rtc::log::critical("Final time entry different from expected 15.0 (got {})", t_final);
    return do_return(false);
  }
  return do_return(true);
}

bool check_extract_key(const std::string & path)
{
  auto out = fmt::format("{}/mc-rtc-test-log-utils-extract-key", bfs::temp_directory_path().string());
  auto extract_cmd = fmt::format("{} extract {} {} --key data", MC_BIN_UTILS, path, out);
  int err = system(extract_cmd.c_str());
  if(err != 0)
  {
    mc_rtc::log::critical("Execution failed: {}", extract_cmd);
    return false;
  }
  auto path_out = out + ".bin";
  if(!bfs::exists(path_out))
  {
    mc_rtc::log::critical("No output file after command: {}", extract_cmd);
    return false;
  }
  auto do_return = [&](bool ret)
  {
    do_cleanup(path_out);
    return ret;
  };
  auto flat = mc_rtc::log::FlatLog(path_out);
  if(!flat.entries().count("data"))
  {
    mc_rtc::log::critical("No data key in extracted log");
    return do_return(false);
  }
  for(auto data_ptr : flat.getRaw<double>("data"))
  {
    if(!data_ptr)
    {
      mc_rtc::log::critical("Extracted log is missing data for the data key");
      return do_return(false);
    }
  }
  return do_return(true);
}

bool check_extract_keys(const std::string & path)
{
  auto out = fmt::format("{}/mc-rtc-test-log-utils-extract-keys", bfs::temp_directory_path().string());
  auto extract_cmd = fmt::format("{} extract {} {} --keys Eigen::*", MC_BIN_UTILS, path, out);
  int err = system(extract_cmd.c_str());
  if(err != 0) { mc_rtc::log::critical("Execution failed: {}", extract_cmd); }
  auto path_out = out + ".bin";
  if(!bfs::exists(path_out))
  {
    mc_rtc::log::critical("No output file after command: {}", extract_cmd);
    return false;
  }
  auto do_return = [&](bool ret)
  {
    do_cleanup(path_out);
    return ret;
  };
  auto flat = mc_rtc::log::FlatLog(path_out);
  auto expected_entries = std::set<std::string>{
      "t", "Eigen::Quaterniond", "Eigen::Vector2d", "Eigen::Vector3d", "Eigen::Vector6d", "Eigen::VectorXd"};
  if(flat.entries() != expected_entries)
  {
    auto print_entries = [](const std::set<std::string> & entries)
    {
      std::string out = "{";
      size_t i = 0;
      for(const auto & e : entries)
      {
        if(i++ != 0) { out = fmt::format("{}, ", out); }
        out = fmt::format("{}{}", out, e);
      }
      out += "}";
      return out;
    };
    mc_rtc::log::critical("Output log does not have the expected entries (expected: {}, got: {})",
                          print_entries(expected_entries), print_entries(flat.entries()));
    return do_return(false);
  }
  return do_return(true);
}

bool check_extract_events(const std::string & path)
{
  auto out = fmt::format("{}/mc-rtc-test-log-utils-extract-events", bfs::temp_directory_path().string());
  auto extract_cmd = fmt::format("{} extract --events {} {}", MC_BIN_UTILS, path, out);
  int err = system(extract_cmd.c_str());
  if(err != 0)
  {
    mc_rtc::log::critical("Execution failed: {}", extract_cmd);
    return false;
  }
  auto path_out = out + ".bin";
  if(!bfs::exists(path_out)) { mc_rtc::log::critical("No output file after command: {}", extract_cmd); }
  bool ret = false;
  auto flat_in = mc_rtc::log::FlatLog(path);
  auto flat_out = mc_rtc::log::FlatLog(path_out);
  if(flat_in.size() != flat_out.size())
  {
    mc_rtc::log::critical("Flat size of output ({}) is different from the input ({})", flat_out.size(), flat_in.size());
    goto do_cleanup_extract_events;
  }
  if(flat_out.entries() != std::set<std::string>{"t"})
  {
    mc_rtc::log::critical("Output log has more entries than expected");
    goto do_cleanup_extract_events;
  }
  for(size_t i = 0; i < flat_out.size(); ++i)
  {
    if(flat_out.guiEvents()[i].size() != flat_in.guiEvents()[i].size())
    {
      mc_rtc::log::critical("Different events at iteration {} between input and output", i);
      goto do_cleanup_extract_events;
    }
    for(size_t j = 0; j < flat_out.guiEvents()[i].size(); ++j)
    {
      const auto & lhs = flat_out.guiEvents()[i][j];
      const auto & rhs = flat_in.guiEvents()[i][j];
      if(lhs.category != rhs.category || lhs.name != rhs.name
         || lhs.data.operator size_t() != rhs.data.operator size_t())
      {
        mc_rtc::log::critical("Different events at iteration {} between intput and output");
      }
    }
  }
  ret = true;
do_cleanup_extract_events:
  bfs::remove(path_out);
  return ret;
}

int main()
{
  mc_rtc::log::info("mc_bin_utils at {}", MC_BIN_UTILS);
  auto log = make_log_ref();
  auto do_check = [&](auto && callback)
  {
    if(!callback(log))
    {
      do_cleanup(log);
      std::exit(1);
    }
  };
  do_check(check_split);
  do_check(check_extract_time);
  do_check(check_extract_key);
  do_check(check_extract_keys);
  do_check(check_extract_events);
  do_cleanup(log);
  return 0;
}
