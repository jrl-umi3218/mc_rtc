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
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/log/iterate_binary_log.h>

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

int show(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_utils show options");
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
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
  auto callback = [&](const std::vector<std::string> & ks, const std::vector<mc_rtc::log::FlatLog::record> & records,
                      double t) {
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
  if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_callback(callback), false))
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
                      const mc_rtc::log::copy_callback & copy, const char * data, size_t dataSize) {
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
  if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback), false))
  {
    return 1;
  }
  return 0;
}

int extract(int argc, char * argv[])
{
  std::string in = "";
  std::string out = "";
  std::string key = "";
  double from = 0;
  double to = std::numeric_limits<double>::infinity();
  po::variables_map vm;
  po::options_description tool("mc_bin_utils extract options");
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("in", po::value<std::string>(&in), "Input file")
    ("out", po::value<std::string>(&out), "Output template")
    ("key", po::value<std::string>(&key)->default_value(""), "Key to extract")
    ("from", po::value<double>(&from)->default_value(0), "Start time")
    ("to", po::value<double>(&to)->default_value(std::numeric_limits<double>::infinity()), "End time");
  // clang-format on
  po::positional_options_description pos;
  pos.add("in", 1);
  pos.add("out", 1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);
  auto extract_usage = [&tool]() {
    std::cout << "Usage: mc_bin_utils extract [in] [out]\n\n";
    std::cout << tool << "\n\n";
    std::cout << "Examples:\n\n";
    std::cout << "  #Extract 10 seconds of in.bin\n";
    std::cout << "  mc_bin_utils extract in.bin out --from 50 --to 60\n\n";
    std::cout << "  #Extract parts of the log where MyKey appears\n";
    std::cout << "  mc_bin_utils extract in.bin out --key MyKey\n";
  };
  if(vm.count("help") || !vm.count("in") || !vm.count("out")
     || (!key.size() && from == 0 && to == std::numeric_limits<double>::infinity()))
  {
    extract_usage();
    return !vm.count("help");
  }
  if(key.size() && (from != 0 || to != std::numeric_limits<double>::infinity()))
  {
    std::cout << "key options and from/to options are exclusive\n";
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
  auto callback_extract_key = [&](const std::vector<std::string> & ks,
                                  const std::vector<mc_rtc::log::FlatLog::record> &, double,
                                  const mc_rtc::log::copy_callback &, const char * data, size_t dataSize) {
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
  std::vector<std::string> keys;
  double final_t = 0;
  auto callback_extract_from_to = [&](const std::vector<std::string> & ks,
                                      const std::vector<mc_rtc::log::FlatLog::record> &, double t,
                                      const mc_rtc::log::copy_callback & copy, const char * data, size_t dataSize) {
    final_t = t;
    if(ks.size())
    {
      keys = ks;
    }
    if(t >= from && t <= to)
    {
      if(!ofs.is_open())
      {
        std::stringstream ss;
        ss << out << "_from_" << from << "_to_";
        if(to == std::numeric_limits<double>::infinity())
        {
          ss << "end";
        }
        else
        {
          ss << to;
        }
        ss << ".bin";
        ofs.open(ss.str(), std::ofstream::binary);
        ofs.write((const char *)&mc_rtc::Logger::magic, sizeof(mc_rtc::Logger::magic));
        if(!ks.size())
        {
          std::vector<char> data;
          mc_rtc::MessagePackBuilder builder(data);
          copy(builder, keys);
          size_t s = builder.finish();
          ofs.write((char *)(&s), sizeof(size_t));
          ofs.write(data.data(), s);
          return true;
        }
      }
      ofs.write((char *)&dataSize, sizeof(size_t));
      ofs.write(data, dataSize * sizeof(char));
    }
    return true;
  };
  if(key.size())
  {
    if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback_extract_key), false))
    {
      return 1;
    }
    if(n == 0)
    {
      std::cout << "No key " << key << " in this log file\n";
    }
  }
  if(from != 0 || to != std::numeric_limits<double>::infinity())
  {
    if(!mc_rtc::log::iterate_binary_log(in, mc_rtc::log::binary_log_copy_callback(callback_extract_from_to), false))
    {
      return 1;
    }
    if(!ofs.is_open())
    {
      std::cout << "Provided start time is higher than last time recorded: " << final_t << "\n";
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
  if(argc < 2)
  {
    usage();
    return 0;
  }
  std::string tool = argv[1];
  argc = argc - 1;
  argv = &argv[1];
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
