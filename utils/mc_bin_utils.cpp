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
#include <mc_rtc/log/iterate_binary_log.h>
#include <mc_rtc/log/Logger.h>

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
    return !vm.count("help");
  }
  std::string in = vm["in"].as<std::string>();
  std::set<std::pair<std::string, mc_rtc::log::LogType>> keys;
  double start_t = 0;
  double end_t = 0;
  size_t n = 0;
  auto callback = [&](const std::vector<std::string> & ks,
                      const std::vector<mc_rtc::log::FlatLog::record> & records,
                      double t)
  {
    if(n++ == 0)
    {
      start_t = t;
    }
    end_t = t;
    if(ks.size())
    {
      for(size_t i = 0; i < ks.size(); ++i)
      {
        const std::string & k = ks[i];
        const mc_rtc::log::FlatLog::record & r = records[i];
        keys.insert(std::make_pair(k, r.type));
      }
    }
    return true;
  };
  if(!mc_rtc::log::iterate_binary_log(in, callback, false))
  {
    return 1;
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
  auto callback = [&](const std::vector<std::string> & ks,
                      const std::vector<mc_rtc::log::FlatLog::record> &,
                      double,
                      const mc_rtc::log::copy_callback & copy,
                      const char * data,
                      size_t dataSize)
  {
    // Start a new part if no data has been written
    if(written == 0)
    {
      if(ofs.is_open())
      {
        ofs.close();
      }
      std::stringstream ss;
      ss << out << "_" << std::setfill('0') << std::setw(static_cast<int>(width)) << ++part << ".bin";
      if(part == parts)
      {
        desired_size = size;
      }
      ofs.open(ss.str(), std::ofstream::binary);
      if(!ofs)
      {
        LOG_ERROR("Failed to open " << ss.str() << " for writing")
        return false;
      }
      ofs.write((const char *)&mc_rtc::Logger::magic, sizeof(mc_rtc::Logger::magic));
    }
    if(ks.size())
    {
      keys = ks;
    }
    else if(written == 0) // Started a new part but split on an entry with no keys
    {
      std::vector<char> data;
      mc_rtc::MessagePackBuilder builder(data);
      copy(builder, keys);
      size_t s = builder.finish();
      ofs.write((char *)(&s), sizeof(size_t));
      ofs.write(data.data(), s);
      written += sizeof(size_t) + s;
      return true;
    }
    ofs.write((char *)&dataSize, sizeof(size_t));
    ofs.write(data, dataSize * sizeof(char));
    written += sizeof(size_t) + dataSize * sizeof(char);
    if(written >= desired_size)
    {
      written = 0;
    }
    return true;
  };
  if(!mc_rtc::log::iterate_binary_log(in, callback, false))
  {
    return 1;
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
    return !vm.count("help");
  }
  auto in = vm["in"].as<std::string>();
  auto out = vm["out"].as<std::string>();
  auto key = vm["key"].as<std::string>();
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
  std::ofstream ofs;
  bool key_present = false;
  auto callback = [&](const std::vector<std::string> & ks,
                      const std::vector<mc_rtc::log::FlatLog::record> &,
                      double,
                      const mc_rtc::log::copy_callback &,
                      const char * data,
                      size_t dataSize)
  {
   if(ks.size())
   {
     bool key_was_present = key_present;
     key_present = std::find_if(ks.begin(), ks.end(), [&key](const std::string & k) { return k == key; }) != ks.end();
     if(key_present && !key_was_present)
     {
       std::string nfile = out_name(n);
       ofs.open(nfile);
       if(!ofs)
       {
         LOG_ERROR("Failed to open " << nfile << "for writing")
         return false;
       }
       ofs.write((const char *)&mc_rtc::Logger::magic, sizeof(mc_rtc::Logger::magic));
       n++;
     }
     if(key_was_present && !key_present)
     {
       ofs.close();
     }
   }
   if(key_present)
   {
    ofs.write((char *)&dataSize, sizeof(size_t));
    ofs.write(data, dataSize * sizeof(char));
   }
   return true;
  };
  if(!mc_rtc::log::iterate_binary_log(in, callback, false))
  {
    return 1;
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

  if(!vm.count("tool"))
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
