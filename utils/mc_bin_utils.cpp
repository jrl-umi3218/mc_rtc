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
#include <mc_rtc/log/FlatLog.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "mc_bin_to_flat.h"
#include "mc_bin_to_log.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#ifdef MC_RTC_HAS_ROS
#  include "mc_bin_to_rosbag.h"
#endif

#include "../src/mc_rtc/internals/LogEntry.h"

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

void fill_options(int argc,
                  char * argv[],
                  po::variables_map & vm,
                  const po::options_description & tool,
                  const std::vector<std::string> & extra)
{
  po::options_description desc("mc_bin_utils options");
  desc.add_options()("help", "Produce this message")("tool", po::value<std::string>(), "Select a tool");
  desc.add(tool);
  auto parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();

  po::positional_options_description pos;
  auto positional_if_not_provided = [&](const std::string & e) {
    auto it = std::find_if(parsed.options.begin(), parsed.options.end(),
                           [&e](const po::option & o) { return o.string_key == e; });
    if(it == parsed.options.end())
    {
      pos.add(e.c_str(), 1);
    }
  };
  positional_if_not_provided("tool");
  for(const auto & e : extra)
  {
    positional_if_not_provided(e);
  }
  if(extra.size() == 0)
  {
    pos.add("misc", -1);
    desc.add_options()("misc", po::value<std::vector<std::string>>(), "Tool options");
  }
  po::store(po::command_line_parser(argc, argv).options(desc).positional(pos).allow_unregistered().run(), vm);
}

int show(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils show options");
  tool.add_options()("in", po::value<std::string>(), "Input file");
  fill_options(argc, argv, vm, tool, {"in"});

  if(!vm.count("in") || vm.count("help"))
  {
    std::cout << "Usage: mc_bin_utils show [in]\n\n";
    std::cout << tool << "\n";
    return 1;
  }
  std::string in = vm["in"].as<std::string>();
  bfs::path in_p(in);
  if(!bfs::exists(in_p) || !bfs::is_regular_file(in_p))
  {
    std::cerr << in << " does not exist or is not a file, aborting...\n";
    return 1;
  }
  std::ifstream ifs(in, std::ifstream::binary);
  std::vector<char> buffer(1024);
  std::set<std::pair<std::string, mc_rtc::log::LogType>> keys;
  double start_t = 0;
  double end_t = 0;
  size_t n = 0;
  bool has_t = false;
  size_t t_index = 0;
  while(ifs)
  {
    size_t entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(size_t));
    if(!ifs)
    {
      break;
    }
    while(buffer.size() < static_cast<size_t>(entrySize))
    {
      buffer.resize(2 * buffer.size());
    }
    ifs.read(buffer.data(), entrySize);
    if(!ifs)
    {
      break;
    }
    mc_rtc::log::internal::LogEntry log(buffer, entrySize, false);
    if(!log.valid())
    {
      return 1;
    }
    if(log.keys().size())
    {
      has_t = false;
      for(size_t i = 0; i < log.keys().size(); ++i)
      {
        const auto & k = log.keys()[i];
        if(k == "t")
        {
          has_t = true;
          t_index = i;
        }
        keys.insert(std::make_pair(log.keys()[i], log.records()[i].type));
      }
    }
    if(has_t && n == 0)
    {
      start_t = log.getTime(t_index);
    }
    else
    {
      end_t = log.getTime(t_index);
    }
    n++;
  }
  std::cout << in << " summary\n";
  std::cout << "Entries: " << keys.size() << "\n";
  if(start_t != end_t)
  {
    std::cout << "Start time: " << start_t << "s\n";
    std::cout << "End time: " << end_t << "s\n";
    std::cout << "Duration: " << (end_t - start_t) << "s\n";
  }
  std::cout << "Entry size: " << n << "\n";
  std::cout << "Available entries:\n";
  for(const auto & e : keys)
  {
    std::cout << "- " << e.first << " (" << mc_rtc::log::LogTypeName(e.second) << ")\n";
  }
  return 0;
}

int split(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils split options");
  tool.add_options()("in", po::value<std::string>(), "Input file")("out", po::value<std::string>(), "Output template")(
      "parts", po::value<unsigned int>(), "Number of parts");
  fill_options(argc, argv, vm, tool, {"in", "out", "parts"});
  if(vm.count("help") || !vm.count("in") || !vm.count("out") || !vm.count("parts"))
  {
    std::cout << "Usage: mc_bin_utils split [in] [out] [parts]\n\n";
    std::cout << tool << "\n";
    return 1;
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
  std::ifstream ifs(in, std::ifstream::binary);
  std::vector<char> buffer(1024);
  std::vector<std::string> keys;
  auto split_file = [&](unsigned int n, size_t desired_size) {
    std::stringstream ss;
    ss << out << "_" << std::setfill('0') << std::setw(static_cast<int>(width)) << (n + 1) << ".bin";
    std::ofstream ofs(ss.str(), std::ofstream::binary);
    if(!ofs)
    {
      LOG_ERROR("Failed to open " << ss.str() << " for writing")
      return false;
    }
    size_t written = 0;
    while(ifs && written < desired_size)
    {
      int entrySize = 0;
      ifs.read((char *)&entrySize, sizeof(int));
      if(!ifs)
      {
        break;
      }
      while(buffer.size() < static_cast<size_t>(entrySize))
      {
        buffer.resize(2 * buffer.size());
      }
      ifs.read(buffer.data(), entrySize);
      if(!ifs)
      {
        break;
      }
      /** Update current keys */
      auto log = mc_rtc::log::internal::LogEntry(buffer, entrySize, false);
      if(log.keys().size())
      {
        keys = log.keys();
      }
      else if(written == 0)
      {
        std::vector<char> data;
        mc_rtc::MessagePackBuilder builder(data);
        log.copy(builder, keys);
        size_t s = builder.finish();
        ofs.write((char *)(&s), sizeof(size_t));
        ofs.write(data.data(), s);
        written += sizeof(size_t) + s;
        continue;
      }
      ofs.write((char *)(&entrySize), sizeof(size_t));
      ofs.write(buffer.data(), entrySize);
      written += sizeof(size_t) + entrySize;
    }
    return true;
  };
  for(unsigned int i = 0; i < parts; ++i)
  {
    if(!split_file(i, i != parts - 1 ? part_size : size))
    {
      return 1;
    }
  }
  return 0;
}

int extract(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils extract options");
  tool.add_options()("in", po::value<std::string>(), "Input file")("out", po::value<std::string>(), "Output template")(
      "key", po::value<std::string>(), "Key to extract");
  fill_options(argc, argv, vm, tool, {"in", "out", "key"});
  if(vm.count("help") || !vm.count("in") || !vm.count("out") || !vm.count("key"))
  {
    std::cout << "Usage: mc_bin_utils extract [in] [out] [key]\n\n";
    std::cout << tool << "\n";
    return 1;
  }
  auto in = vm["in"].as<std::string>();
  auto out = vm["out"].as<std::string>();
  auto key = vm["key"].as<std::string>();
  bfs::path in_p(in);
  if(!bfs::exists(in_p) || !bfs::is_regular_file(in_p))
  {
    std::cerr << in << " does not exist or is not file, aborting...\n";
    return 1;
  }
  std::ifstream ifs(in, std::ifstream::binary);
  size_t width = 0;
  size_t n = 0;
  auto rename_all = [&out](size_t prev_w, size_t w) {
    if(prev_w == 0)
    {
      bfs::path old(out + ".bin");
      bfs::path new_(out + "_1.bin");
      bfs::rename(old, new_);
      return;
    }
    for(size_t i = 0; i < std::pow(10, prev_w); ++i)
    {
      std::stringstream ss_old;
      ss_old << out << "_" << std::setfill('0') << std::setw(static_cast<int>(prev_w)) << (i + 1) << ".bin";
      std::stringstream ss_new;
      ss_new << out << "_" << std::setfill('0') << std::setw(static_cast<int>(w)) << (i + 1) << ".bin";
      bfs::rename({ss_old.str()}, {ss_new.str()});
    }
  };
  auto out_name = [&](size_t i) {
    if(i == 0)
    {
      return out + ".bin";
    }
    auto prev_width = width;
    width = std::to_string(i + 1).size();
    if(width != prev_width)
    {
      rename_all(prev_width, width);
    }
    std::stringstream ss;
    ss << out << "_" << std::setfill('0') << std::setw(static_cast<int>(width)) << (i + 1) << ".bin";
    return ss.str();
  };
  std::vector<char> buffer(1024);
  std::ofstream ofs;
  bool key_present = false;
  while(ifs)
  {
    size_t entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(size_t));
    if(!ifs)
    {
      break;
    }
    while(buffer.size() < static_cast<size_t>(entrySize))
    {
      buffer.resize(2 * buffer.size());
    }
    ifs.read(buffer.data(), entrySize);
    if(!ifs)
    {
      break;
    }
    auto log = mc_rtc::log::internal::LogEntry(buffer, entrySize);
    if(log.keys().size())
    {
      bool key_was_present = key_present;
      key_present =
          std::find_if(log.keys().begin(), log.keys().end(), [&key](const std::string & k) { return k == key; })
          != log.keys().end();
      if(key_present && !key_was_present)
      {
        ofs.open(out_name(n));
        n++;
      }
      if(key_was_present && !key_present)
      {
        ofs.close();
      }
    }
    if(key_present)
    {
      ofs.write((char *)&entrySize, sizeof(size_t));
      ofs.write(buffer.data(), entrySize);
    }
  }
  if(n == 0)
  {
    std::cout << "No key " << key << " in this log file\n";
  }
  return 0;
}

int convert(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils convert options");
  double dt = 0.005;
  tool.add_options()("in", po::value<std::string>(), "Input file")("out", po::value<std::string>(),
                                                                   "Output file or template")(
      "format", po::value<std::string>(), "Log format (csv|flat|bag), can be deduced from [out]")(
      "dt", po::value<double>(&dt), "Log timestep (only for bag conversion)");
  fill_options(argc, argv, vm, tool, {"in", "out", "format", "dt"});
  if(vm.count("help") || !vm.count("in") || !vm.count("out"))
  {
    std::cout << "Usage: mc_bin_utils convert [in] [out] ([format] [dt])\n\n";
    std::cout << tool << "\n";
    return 1;
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
    if(format[0] != '.')
    {
      format = "." + format;
    }
    if(format != ".bag" && format != ".csv" && format != ".flat")
    {
      LOG_ERROR("Unsupported format " << format)
      format = "";
    }
  }
  if(ext == ".bag")
  {
    if(format.size() && format != ext)
    {
      LOG_WARNING("Command-line specified format clashes with file extension, trusting the provided format")
    }
    else
    {
      format = ext;
    }
  }
  else if(ext == ".log" || ext == ".csv")
  {
    if(format.size() && format != ext)
    {
      LOG_WARNING("Command-line specified format clashes with file extension, trusting the provided format")
    }
    else
    {
      format = ext;
    }
  }
  else if(ext == ".flat")
  {
    if(format.size() && format != ext)
    {
      LOG_WARNING("Command-line specified format clashes with file extension, trusting the provided format")
    }
    else
    {
      format = ext;
    }
  }
  else
  {
    if(ext.size() == 0 && format.size())
    {
      out_p.replace_extension(format);
      LOG_INFO("Conversion will be output to " << out_p)
    }
  }
  if(!format.size())
  {
    LOG_ERROR("Could not deduce the desired output format")
    return 1;
  }
  if(format == ".flat")
  {
    mc_bin_to_flat(in, out_p.string());
  }
  else if(format == ".csv" || format == ".log")
  {
    mc_bin_to_flat(in, out_p.string());
  }
  else if(format == ".bag")
  {
#ifdef MC_RTC_HAS_ROS
    if(vm.count("dt"))
    {
      dt = vm["dt"].as<double>();
    }
    mc_bin_to_rosbag(in, out_p.string(), dt);
#else
    LOG_ERROR("mc_rtc is not build with ROS support, bag conversion is not available")
#endif
  }
  return 0;
}

int main(int argc, char * argv[])
{
  po::variables_map vm;
  fill_options(argc, argv, vm, {}, {});

  if(vm.count("help") || !vm.count("tool"))
  {
    usage();
    return 0;
  }

  auto tool = vm["tool"].as<std::string>();
  if(tool == "show")
  {
    return show(argc, argv);
  }
  else if(tool == "split")
  {
    return split(argc, argv);
  }
  else if(tool == "extract")
  {
    return extract(argc, argv);
  }
  else if(tool == "convert")
  {
    return convert(argc, argv);
  }
  else
  {
    usage();
    std::cout << "\n" << tool << " is not a valid command\n";
    return 1;
  }

  return 0;
}
