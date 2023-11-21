/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/** This utility is able to perform some simple operations on a binary log file:
 * - Display some information about the log
 * - Split the file into N parts
 * - Extract the part(s) where a given entry was recorded
 * - Convert to csv/flat/bag format
 */

#include <mc_rtc/config.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/log/iterate_binary_log.h>

#include <boost/algorithm/string/predicate.hpp>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "mc_bin_to_flat.h"
#include "mc_bin_to_log.h"

#include <bitset>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "../src/mc_rtc/internals/LogEntry.h"

namespace
{
static bfs::path MC_BIN_TO_ROSBAG = "@CMAKE_INSTALL_PREFIX@/bin/mc_bin_to_rosbag@CMAKE_EXECUTABLE_SUFFIX@";

struct TypedKey
{
  std::string key;
  mc_rtc::log::LogType type;

  friend inline bool operator==(const TypedKey & lhs, const TypedKey & rhs)
  {
    return lhs.type == rhs.type && lhs.key == rhs.key;
  }

  friend inline bool operator!=(const TypedKey & lhs, const TypedKey & rhs) { return !(lhs == rhs); }
};

void addToLogger(const TypedKey & key, const mc_rtc::log::FlatLog & log, mc_rtc::Logger & logger, size_t & idx)
{
#define HANDLE_CASE(LOGT, CPPT)                                                                                   \
  case mc_rtc::log::LogType::LOGT:                                                                                \
    logger.addLogEntry(key.key, [&idx, &log, key]() -> const CPPT & { return *log.getRaw<CPPT>(key.key, idx); }); \
    break
  switch(key.type)
  {
    HANDLE_CASE(Bool, bool);
    HANDLE_CASE(Int8_t, int8_t);
    HANDLE_CASE(Int16_t, int16_t);
    HANDLE_CASE(Int32_t, int32_t);
    HANDLE_CASE(Int64_t, int64_t);
    HANDLE_CASE(Uint8_t, uint8_t);
    HANDLE_CASE(Uint16_t, uint16_t);
    HANDLE_CASE(Uint32_t, uint32_t);
    HANDLE_CASE(Uint64_t, uint64_t);
    HANDLE_CASE(Float, float);
    HANDLE_CASE(Double, double);
    HANDLE_CASE(String, std::string);
    HANDLE_CASE(Vector2d, Eigen::Vector2d);
    HANDLE_CASE(Vector3d, Eigen::Vector3d);
    HANDLE_CASE(Vector6d, Eigen::Vector6d);
    HANDLE_CASE(VectorXd, Eigen::VectorXd);
    HANDLE_CASE(Quaterniond, Eigen::Quaterniond);
    HANDLE_CASE(PTransformd, sva::PTransformd);
    HANDLE_CASE(ForceVecd, sva::ForceVecd);
    HANDLE_CASE(MotionVecd, sva::MotionVecd);
    HANDLE_CASE(VectorDouble, std::vector<double>);
    default:
      return;
  }
#undef HANDLE_CASE
}

void print_string_vector(const std::vector<std::string> & v)
{
  for(size_t i = 0; i < v.size(); ++i)
  {
    if(i == 0) { std::cout << "["; }
    std::cout << " " << v[i];
    if(i == v.size() - 1) { std::cout << " ]"; }
    else { std::cout << ","; }
  }
}

} // namespace

void usage()
{
  std::cout << "mc_bin_utils is a command-line tool to work with mc_rtc bin logs\n\n";
  std::cout << "Available commands:\n";
  std::cout << "    show      Display information about the log\n";
  std::cout << "    split     Split a log into N part\n";
  std::cout << "    extract   Extra part of a log\n";
  std::cout << "    convert   Convert binary logs to various formats\n";
  std::cout << "\nUse mc_bin_utils <command> --help for usage of each command\n";
}

int show(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils show options");
  bool print_calib = false;
  bool print_events = false;
  bool print_init = false;
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("print-init", po::bool_switch(&print_init), "Show initial positions/configurations of the robots")
    ("print-calib", po::bool_switch(&print_calib), "Show the stored force sensor calibration")
    ("print-events", po::bool_switch(&print_events), "Show GUI events in the log")
    ("in", po::value<std::string>(), "Input file");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);

  if(!vm.count("in") || vm.count("help"))
  {
    std::cout << "Usage: mc_bin_utils show [in]\n\n";
    std::cout << tool << "\n";
    return !vm.count("help");
  }
  std::string in = vm["in"].as<std::string>();
  std::set<std::pair<std::string, mc_rtc::log::LogType>> keys;
  double start_t = 0;
  double end_t = 0;
  size_t n = 0;
  size_t n_events = 0;
  double dt = 0;
  std::vector<std::vector<mc_rtc::Logger::GUIEvent>> events;
  std::optional<mc_rtc::Logger::Meta> meta;
  auto callback = [&](mc_rtc::log::IterateBinaryLogData data)
  {
    if(n++ == 0) { start_t = *data.time; }
    end_t = *data.time;
    if(n == 2) { dt = end_t - start_t; }
    if(data.keys.size())
    {
      for(size_t i = 0; i < data.keys.size(); ++i)
      {
        const std::string & k = data.keys[i];
        const mc_rtc::log::FlatLog::record & r = data.records[i];
        keys.insert(std::make_pair(k, r.type));
      }
    }
    if(!meta && data.meta) { meta = data.meta; }
    if(print_events) { events.push_back(data.gui_events); }
    n_events += data.gui_events.size();
    return true;
  };
  if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::iterate_binary_log_callback(callback), false)) { return 1; }
  std::cout << in << " summary\n";
  if(!meta) { std::cout << "(no meta information)\n"; }
  else
  {
    std::cout << "Timestep: " << meta->timestep << "\n";
    std::cout << "MainRobot: " << meta->main_robot << "\n";
    std::cout << "MainRobotParams: " << mc_rtc::io::to_string(meta->main_robot_module) << "\n";
    if(print_init)
    {
      for(const auto & [r, init_pos] : meta->init)
      {
        std::cout << r << " initial pose:\n";
        std::cout << "  translation: " << init_pos.translation().transpose() << "\n";
        const auto & q = Eigen::Quaterniond(init_pos.rotation());
        std::cout << "  rotation: [" << mc_rtc::io::to_string(std::array{q.w(), q.x(), q.y(), q.z()}) << "]\n";
      }
      for(const auto & [r, init_q] : meta->init_q)
      {
        std::cout << r << " initial configuration:\n";
        std::cout << "["
                  << mc_rtc::io::to_string(init_q,
                                           [](const auto & qi) { return "[" + mc_rtc::io::to_string(qi) + "]"; })
                  << "]\n";
      }
    }
    if(print_calib)
    {
      for(const auto & [r, calibs] : meta->calibs)
      {
        if(calibs.empty())
        {
          std::cout << r << ": No calibration data\n";
          continue;
        }
        for(const auto & [sensor, calib] : calibs)
        {
          std::cout << r << "::" << sensor << ":\n";
          std::cout << calib.dump(true, true) << "\n";
        }
      }
    }
  }
  std::cout << "Entries: " << keys.size() << "\n";
  std::cout << "GUI events: " << n_events << "\n";
  if(start_t != end_t)
  {
    std::cout << "Start time: " << start_t << "s\n";
    std::cout << "End time: " << end_t << "s\n";
    std::cout << "Duration: " << (end_t - start_t) << "s\n";
    std::cout << "Timestep: " << dt << "s\n";
  }
  std::cout << "Entry size: " << n << "\n";
  std::cout << "Available entries:\n";
  for(const auto & e : keys) { std::cout << "- " << e.first << " (" << mc_rtc::log::LogTypeName(e.second) << ")\n"; }
  if(print_events)
  {
    for(size_t i = 0; i < events.size(); ++i)
    {
      if(events[i].size() == 0) { continue; }
      std::cout << "Events at t = " << (static_cast<double>(i) * dt) << ":\n";
      for(const auto & e : events[i])
      {
        std::cout << "- category: ";
        print_string_vector(e.category);
        std::cout << "\n";
        std::cout << "  name: " << e.name << "\n";
        std::cout << "  data: " << e.data.dump(true, true) << "\n";
      }
    }
  }
  return 0;
}

void write_magic(std::ofstream & ofs)
{
  using Logger = mc_rtc::Logger;
  ofs.write((const char *)&Logger::magic, sizeof(Logger::magic) - sizeof(uint8_t));
  const char version = static_cast<uint8_t>(Logger::magic[3] + Logger::version);
  ofs.write(&version, sizeof(uint8_t));
}

int split(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils split options");
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input file")
    ("out", po::value<std::string>(), "Output template")
    ("parts", po::value<unsigned int>(), "Number of parts");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);
  pos.add("parts", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);
  if(vm.count("help") || !vm.count("in") || !vm.count("out") || !vm.count("parts"))
  {
    std::cout << "Usage: mc_bin_utils split [in] [out] [parts]\n\n";
    std::cout << tool << "\n";
    return !vm.count("help");
  }
  auto in = vm["in"].as<std::string>();
  auto out = vm["out"].as<std::string>();
  auto parts = vm["parts"].as<unsigned int>();
  if(parts < 2)
  {
    std::cerr << "Should split in 2 parts at least\n";
    return 1;
  }
  bfs::path in_p(in);
  if(!bfs::exists(in_p) || !bfs::is_regular_file(in_p))
  {
    std::cerr << in << " does not exist or is not a file, aborting...\n";
    return 1;
  }
  auto size = bfs::file_size(in_p);
  auto part_size = size / parts;
  if(part_size < 10 * 1024 * 1024)
  {
    std::cerr << "Cannot split in < 10Mb parts\n";
    return 1;
  }
  auto width = static_cast<int>(std::to_string(parts).size());
  std::vector<std::string> keys;
  size_t written = 0;
  size_t part = 0;
  size_t desired_size = part_size;
  std::ofstream ofs;
  auto callback = [&](const std::vector<std::string> & ks, const std::vector<mc_rtc::log::FlatLog::record> &, double,
                      const mc_rtc::log::copy_callback & copy, const char * data, uint64_t dataSize)
  {
    // Start a new part if no data has been written
    if(written == 0)
    {
      if(ofs.is_open()) { ofs.close(); }
      std::stringstream ss;
      ss << out << "_" << std::setfill('0') << std::setw(static_cast<int>(width)) << ++part << ".bin";
      if(part == parts) { desired_size = size; }
      ofs.open(ss.str(), std::ofstream::binary);
      if(!ofs)
      {
        mc_rtc::log::error("Failed to open {} for writing", ss.str());
        return false;
      }
      write_magic(ofs);
    }
    if(ks.size()) { keys = ks; }
    else if(written == 0) // Started a new part but split on an entry with no keys
    {
      std::vector<char> data;
      mc_rtc::MessagePackBuilder builder(data);
      copy(builder, keys);
      uint64_t s = builder.finish();
      ofs.write((char *)(&s), sizeof(uint64_t));
      ofs.write(data.data(), static_cast<int>(s));
      written += sizeof(uint64_t) + s;
      return true;
    }
    ofs.write((char *)&dataSize, sizeof(uint64_t));
    ofs.write(data, static_cast<int>(dataSize * sizeof(char)));
    written += sizeof(uint64_t) + dataSize * sizeof(char);
    if(written >= desired_size) { written = 0; }
    return true;
  };
  if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback), false)) { return 1; }
  return 0;
}

int extract(int argc, char * argv[])
{
  std::string in = "";
  std::string out = "";
  std::string key = "";
  std::vector<std::string> extract_keys = {};
  double from = 0;
  double to = std::numeric_limits<double>::infinity();
  bool extract_events = false;
  po::variables_map vm;
  po::options_description tool("mc_bin_utils extract options");
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(&in), "Input file")
    ("out", po::value<std::string>(&out), "Output template")
    ("key", po::value<std::string>(&key)->default_value(""), "Extract parts of the log where the given key is present")
    ("keys", po::value<std::vector<std::string>>(&extract_keys)->multitoken(), "Extract the given keys from the log")
    ("events", po::bool_switch(&extract_events), "Extract events from the log")
    ("from", po::value<double>(&from)->default_value(0), "Start time")
    ("to", po::value<double>(&to)->default_value(std::numeric_limits<double>::infinity()), "End time");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);
  auto extract_usage = [&tool]()
  {
    std::cout << "Usage: mc_bin_utils extract [in] [out]\n\n";
    std::cout << tool << "\n\n";
    std::cout << "Examples:\n\n";
    std::cout << "  # Extract 10 seconds of in.bin\n";
    std::cout << "  mc_bin_utils extract in.bin out --from 50 --to 60\n\n";
    std::cout << "  # Extract parts of the log where MyKey appears\n";
    std::cout << "  mc_bin_utils extract in.bin out --key MyKey\n\n";
    std::cout << "  # Extract the provided keys and create a new log\n";
    std::cout << "  mc_bin_utils extract in.bin out --keys LeftFootForceSensor RightFootForceSensor\n";
    std::cout << "  # Extract the GUI events and nothing else\n";
    std::cout << "  mc_bin_utils extract in.bin out --events\n";
  };
  std::bitset<4> options;
  options[0] = key.size() != 0;
  options[1] = extract_keys.size() != 0;
  options[2] = (from != 0 || to != std::numeric_limits<double>::infinity());
  options[3] = extract_events;
  if(vm.count("help") || !vm.count("in") || !vm.count("out") || options.count() == 0)
  {
    extract_usage();
    return !vm.count("help");
  }
  if(options.count() != 1)
  {
    std::cout << "events, key, keys and from/to options are mutually exclusive\n";
    return 1;
  }
  if(from < 0)
  {
    std::cout << "Starting time must be positive, acting as if you provided 0\n";
    from = 0;
  }
  if(to <= from)
  {
    std::cout << "End time must be greater than starting time, acting as if you provided infinity\n";
    to = std::numeric_limits<double>::infinity();
  }
  size_t width = 0;
  size_t n = 0;
  auto rename_all = [&out](size_t prev_w, size_t w)
  {
    if(prev_w == 0)
    {
      bfs::path old(out + ".bin");
      bfs::path new_(out + "_1.bin");
      bfs::rename(old, new_);
      return;
    }
    size_t upper = static_cast<size_t>(std::pow(10.0, static_cast<double>(prev_w)));
    for(size_t i = 0; i < upper; ++i)
    {
      std::stringstream ss_old;
      ss_old << out << "_" << std::setfill('0') << std::setw(static_cast<int>(prev_w)) << (i + 1) << ".bin";
      std::stringstream ss_new;
      ss_new << out << "_" << std::setfill('0') << std::setw(static_cast<int>(w)) << (i + 1) << ".bin";
      bfs::rename({ss_old.str()}, {ss_new.str()});
    }
  };
  auto out_name = [&](size_t i)
  {
    if(i == 0) { return out + ".bin"; }
    auto prev_width = width;
    width = std::to_string(i + 1).size();
    if(width != prev_width) { rename_all(prev_width, width); }
    std::stringstream ss;
    ss << out << "_" << std::setfill('0') << std::setw(static_cast<int>(width)) << (i + 1) << ".bin";
    return ss.str();
  };
  if(out.size() > 4 && out.substr(out.size() - 4) == ".bin") { out = out.substr(0, out.size() - 4); }
  std::ofstream ofs;
  bool key_present = false;
  auto callback_extract_key = [&](const std::vector<std::string> & ks,
                                  const std::vector<mc_rtc::log::FlatLog::record> &, double,
                                  const mc_rtc::log::copy_callback & copy, const char * data, uint64_t dataSize)
  {
    if(ks.size())
    {
      bool key_was_present = key_present;
      key_present = std::find_if(ks.begin(), ks.end(), [&key](const std::string & k) { return k == key; }) != ks.end();
      if(key_present && !key_was_present)
      {
        std::string nfile = out_name(n);
        ofs.open(nfile, std::ofstream::binary);
        if(!ofs)
        {
          mc_rtc::log::error("Failed to open {} for writing", nfile);
          return false;
        }
        write_magic(ofs);
        {
          std::vector<char> data;
          mc_rtc::MessagePackBuilder builder(data);
          copy(builder, ks);
          uint64_t s = builder.finish();
          ofs.write((char *)(&s), sizeof(uint64_t));
          ofs.write(data.data(), static_cast<int>(s));
          n++;
          return true;
        }
      }
      if(key_was_present && !key_present) { ofs.close(); }
    }
    if(key_present)
    {
      ofs.write((char *)&dataSize, sizeof(uint64_t));
      ofs.write(data, static_cast<int>(dataSize * sizeof(char)));
    }
    return true;
  };
  std::vector<std::string> keys;
  double final_t = 0;
  auto callback_extract_from_to = [&](const std::vector<std::string> & ks,
                                      const std::vector<mc_rtc::log::FlatLog::record> &, double t,
                                      const mc_rtc::log::copy_callback & copy, const char * data, uint64_t dataSize)
  {
    final_t = t;
    if(ks.size()) { keys = ks; }
    if(t >= from && t <= to)
    {
      if(!ofs.is_open())
      {
        std::stringstream ss;
        ss << out << "_from_" << from << "_to_";
        if(to == std::numeric_limits<double>::infinity()) { ss << "end"; }
        else { ss << to; }
        ss << ".bin";
        ofs.open(ss.str(), std::ofstream::binary);
        write_magic(ofs);
        {
          std::vector<char> data;
          mc_rtc::MessagePackBuilder builder(data);
          copy(builder, keys);
          uint64_t s = builder.finish();
          ofs.write((char *)(&s), sizeof(uint64_t));
          ofs.write(data.data(), static_cast<int>(s));
          return true;
        }
      }
      ofs.write((char *)&dataSize, sizeof(uint64_t));
      ofs.write(data, static_cast<int>(dataSize * sizeof(char)));
    }
    return true;
  };
  if(key.size())
  {
    if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback_extract_key), false))
    {
      return 1;
    }
    if(n == 0) { std::cout << "No key " << key << " in this log file\n"; }
  }
  if(from != 0 || to != std::numeric_limits<double>::infinity())
  {
    if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback_extract_from_to), false))
    {
      return 1;
    }
    if(!ofs.is_open()) { std::cout << "Provided start time is higher than last time recorded: " << final_t << "\n"; }
  }
  if(extract_keys.size())
  {
    auto log = mc_rtc::log::FlatLog{in};
    if(log.size() <= 1)
    {
      std::cout << in << " is empty or has only one entry\n";
      return 1;
    }
    // Remove "t" from the extract_keys as we will implicitly extract it
    {
      auto it = std::find(extract_keys.begin(), extract_keys.end(), "t");
      if(it != extract_keys.end()) { extract_keys.erase(it); }
    }
    std::vector<std::string> wildcards;
    for(auto it = extract_keys.begin(); it != extract_keys.end();)
    {
      const auto & key = *it;
      if(key.empty())
      {
        it = extract_keys.erase(it);
        continue;
      }
      if(!log.has(key))
      {
        if(key.back() == '*') { wildcards.push_back(std::string{key, 0, key.size() - 1}); }
        else { std::cout << *it << " is not in " << in << "\n"; }
        it = extract_keys.erase(it);
      }
      else { ++it; }
    }
    for(const auto & key : wildcards)
    {
      auto size_before = extract_keys.size();
      for(const auto & k : log.entries())
      {
        if(boost::algorithm::starts_with(k, key)) { extract_keys.push_back(k); }
      }
      if(extract_keys.size() == size_before) { std::cout << "No match for wildcard " << key << "* in " << in << "\n"; }
    }
    if(extract_keys.empty())
    {
      std::cout << "All the keys you asked to extract are not in this log (You cannot extract \"t\" only)\n";
      return 1;
    }
    // Returns the keys in the log at iteration i
    auto get_keys_in_log = [&](size_t i)
    {
      std::vector<TypedKey> out;
      for(const auto & k : extract_keys)
      {
        auto type = log.type(k, i);
        if(type != mc_rtc::log::LogType::None) { out.push_back({k, type}); }
      }
      return out;
    };
    std::vector<TypedKey> prev_keys_in_log;
    mc_rtc::Logger logger(mc_rtc::Logger::Policy::NON_THREADED, "", "");
    double timestep = *log.getRaw<double>("t", 1) - *log.getRaw<double>("t", 0);
    for(size_t i = 0; i < log.size(); ++i)
    {
      auto keys_in_log = get_keys_in_log(i);
      if(keys_in_log != prev_keys_in_log)
      {
        if(prev_keys_in_log.empty())
        {
          double start_t = *log.getRaw<double>("t", i);
          std::string file = out_name(n++);
          logger.open(file, timestep, start_t);
        }
        for(const auto & k : prev_keys_in_log) { logger.removeLogEntry(k.key); }
        for(const auto & k : keys_in_log) { addToLogger(k, log, logger, i); }
        prev_keys_in_log = keys_in_log;
      }
      if(keys_in_log.size()) { logger.log(); }
    }
  }
  if(extract_events)
  {
    auto log = mc_rtc::log::FlatLog{in};
    if(log.size() < 1)
    {
      std::cout << in << " is empty\n";
      return 1;
    }
    double dt = 0;
    if(!log.meta())
    {
      if(log.size() == 1) { dt = 0.005; }
      else { dt = *log.getRaw<double>("t", 1) - *log.getRaw<double>("t", 0); }
    }
    else { dt = log.meta()->timestep; }
    mc_rtc::Logger logger(mc_rtc::Logger::Policy::NON_THREADED, "", "");
    if(log.meta()) { logger.meta() = *log.meta(); }
    logger.open(out_name(0), dt, 0);
    for(const auto & events : log.guiEvents())
    {
      for(const auto & e : events) { logger.addGUIEvent(mc_rtc::Logger::GUIEvent{e}); }
      logger.log();
    }
  }
  return 0;
}

int convert(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils convert options");
  double dt = 0.005;
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(), "Input file")
    ("out", po::value<std::string>(), "Output file or template")
    ("format", po::value<std::string>(), "Log format (csv|flat|bag), can be deduced from [out]")
    ("entries", po::value<std::vector<std::string>>()->multitoken(), "Name of entries to log (all if ommitted)")
    ("dt", po::value<double>(&dt), "Log timestep (only for bag conversion)");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);
  if(vm.count("help") || !vm.count("in") || !vm.count("out"))
  {
    std::cout << "Usage: mc_bin_utils convert [in] [out]\n\n";
    std::cout << tool << "\n";
    return !vm.count("help");
  }
  auto in = vm["in"].as<std::string>();
  auto out = vm["out"].as<std::string>();
  bfs::path in_p(in);
  if(!bfs::exists(in_p) || !bfs::is_regular_file(in_p))
  {
    std::cerr << in << " does not exist or is not file, aborting...\n";
    return 1;
  }
  bfs::path out_p(out);
  auto ext = out_p.extension().string();
  std::string format = "";
  if(vm.count("format"))
  {
    format = vm["format"].as<std::string>();
    if(format[0] != '.') { format = "." + format; }
    if(format != ".bag" && format != ".csv" && format != ".flat")
    {
      mc_rtc::log::error("Unsupported format {}", format);
      format = "";
    }
  }
  if(ext == ".bag")
  {
    if(format.size() && format != ext)
    {
      mc_rtc::log::warning("Command-line specified format clashes with file extension, trusting the provided format");
    }
    else { format = ext; }
  }
  else if(ext == ".log" || ext == ".csv")
  {
    if(format.size() && format != ext)
    {
      mc_rtc::log::warning("Command-line specified format clashes with file extension, trusting the provided format");
    }
    else { format = ext; }
  }
  else if(ext == ".flat")
  {
    if(format.size() && format != ext)
    {
      mc_rtc::log::warning("Command-line specified format clashes with file extension, trusting the provided format");
    }
    else { format = ext; }
  }
  else
  {
    if(ext.size() == 0 && format.size())
    {
      out_p.replace_extension(format);
      mc_rtc::log::info("Conversion will be output to {}", out_p.string());
    }
  }
  if(!format.size())
  {
    mc_rtc::log::error("Could not deduce the desired output format");
    return 1;
  }
  std::vector<std::string> entries;
  if(vm.count("entries")) { entries = vm["entries"].as<std::vector<std::string>>(); }

  if(format == ".flat") { mc_bin_to_flat(in, out_p.string(), entries); }
  else if(format == ".csv" || format == ".log") { mc_bin_to_log(in, out_p.string(), entries); }
  else if(format == ".bag")
  {
    if(bfs::exists(MC_BIN_TO_ROSBAG))
    {
      if(vm.count("dt")) { dt = vm["dt"].as<double>(); }
      if(vm.count("entries"))
      {
        mc_rtc::log::critical(
            "--entries option is not supported for ROSBAG format, please enquire with mc_rtc maintainers ;)");
        exit(1);
      }
      std::string cmd = MC_BIN_TO_ROSBAG.string() + " " + in + " " + out_p.string() + " " + std::to_string(dt);
      if(system(cmd.c_str()) != 0) { mc_rtc::log::error("The following conversion call failed: {}", cmd); }
    }
    else { mc_rtc::log::error("mc_rtc is not build with ROS support, bag conversion is not available"); }
  }
  return 0;
}

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    usage();
    return 0;
  }
  std::string tool = argv[1];
  argc = argc - 1;
  argv = &argv[1];
  if(tool == "show") { return show(argc, argv); }
  else if(tool == "split") { return split(argc, argv); }
  else if(tool == "extract") { return extract(argc, argv); }
  else if(tool == "convert") { return convert(argc, argv); }
  else
  {
    usage();
    std::cout << "\n" << tool << " is not a valid command\n";
    return 1;
  }

  return 0;
}
