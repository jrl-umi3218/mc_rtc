/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_bin_utils.h"
#include <fstream>

struct SizedType
{
  mc_rtc::log::LogType type;
  size_t size;
};

using SizedEntries = std::map<std::string, SizedType>;

SizedEntries write_header(std::ofstream & ofs, const mc_rtc::log::FlatLog & log)
{
  SizedEntries out;
  auto entries = utils::entries(log);
  size_t i = 0;
  for(const auto & e : entries)
  {
    auto s = utils::entrySize(log, e.first, e.second);
    // clang-format off
    switch(e.second)
    {
      case mc_rtc::log::LogType::Bool:
      case mc_rtc::log::LogType::Int8_t:
      case mc_rtc::log::LogType::Int16_t:
      case mc_rtc::log::LogType::Int32_t:
      case mc_rtc::log::LogType::Int64_t:
      case mc_rtc::log::LogType::Uint8_t:
      case mc_rtc::log::LogType::Uint16_t:
      case mc_rtc::log::LogType::Uint32_t:
      case mc_rtc::log::LogType::Uint64_t:
      case mc_rtc::log::LogType::Float:
      case mc_rtc::log::LogType::Double:
      case mc_rtc::log::LogType::String:
        ofs << e.first;
        break;
      case mc_rtc::log::LogType::Quaterniond:
        ofs << e.first << "_w;"
            << e.first << "_x;"
            << e.first << "_y;"
            << e.first << "_z";
        break;
      case mc_rtc::log::LogType::Vector2d:
        ofs << e.first << "_x;"
            << e.first << "_y";
        break;
      case mc_rtc::log::LogType::Vector3d:
        ofs << e.first << "_x;"
            << e.first << "_y;"
            << e.first << "_z";
        break;
      case mc_rtc::log::LogType::Vector6d:
        ofs << e.first << "_0;"
            << e.first << "_1;"
            << e.first << "_2;"
            << e.first << "_3;"
            << e.first << "_4;"
            << e.first << "_5";
        break;
      case mc_rtc::log::LogType::PTransformd:
        ofs << e.first << "_qw;"
            << e.first << "_qx;"
            << e.first << "_qy;"
            << e.first << "_qz;"
            << e.first << "_tx;"
            << e.first << "_ty;"
            << e.first << "_tz";
        break;
      case mc_rtc::log::LogType::ForceVecd:
        ofs << e.first << "_cx;"
            << e.first << "_cy;"
            << e.first << "_cz;"
            << e.first << "_fx;"
            << e.first << "_fy;"
            << e.first << "_fz";
        break;
      case mc_rtc::log::LogType::MotionVecd:
        ofs << e.first << "_wx;"
            << e.first << "_wy;"
            << e.first << "_wz;"
            << e.first << "_vx;"
            << e.first << "_vy;"
            << e.first << "_vz";
        break;
      case mc_rtc::log::LogType::VectorXd:
      case mc_rtc::log::LogType::VectorDouble:
        for(size_t j = 0; j < s; ++j)
        {
          ofs << e.first << "_" << j;
          if(j != s - 1)
          {
            ofs << ';';
          }
        }
        break;
      case mc_rtc::log::LogType::None:
        mc_rtc::log::error("{} cannot be written into log format", e.first);
        continue;
    }
    // clang-format on
    if(++i != entries.size())
    {
      ofs << ';';
    }
    else
    {
      ofs << '\n';
    }
    out[e.first] = {e.second, s};
  }
  return out;
}

// clang-format off
template<typename T>
void write_data(std::ofstream & ofs, const T & data, size_t /*fsize*/)
{
  ofs << data;
}

template<>
void write_data<Eigen::Quaterniond>(std::ofstream & ofs, const Eigen::Quaterniond & data, size_t /*fsize*/)
{
  ofs << data.w() << ";"
      << data.x() << ";"
      << data.y() << ";"
      << data.z();
}

template<>
void write_data<Eigen::Vector2d>(std::ofstream & ofs, const Eigen::Vector2d & data, size_t /*fsize*/)
{
  ofs << data.x() << ";"
      << data.y();
}

template<>
void write_data<Eigen::Vector3d>(std::ofstream & ofs, const Eigen::Vector3d & data, size_t /*fsize*/)
{
  ofs << data.x() << ";"
      << data.y() << ";"
      << data.z();
}

template<>
void write_data<Eigen::Vector6d>(std::ofstream & ofs, const Eigen::Vector6d & data, size_t /*fsize*/)
{
  ofs << data(0) << ";"
      << data(1) << ";"
      << data(2) << ";"
      << data(3) << ";"
      << data(4) << ";"
      << data(5);
}
// clang-format on

template<>
void write_data<Eigen::VectorXd>(std::ofstream & ofs, const Eigen::VectorXd & data, size_t fsize)
{
  for(long i = 0; i < data.size(); ++i)
  {
    ofs << data(i);
    if(i + 1 != static_cast<long>(fsize))
    {
      ofs << ';';
    }
  }
  for(long i = data.size(); i < static_cast<long>(fsize); ++i)
  {
    if(i + 1 != static_cast<long>(fsize))
    {
      ofs << ';';
    }
  }
}

template<>
void write_data<std::vector<double>>(std::ofstream & ofs, const std::vector<double> & data, size_t fsize)
{
  for(size_t i = 0; i < data.size(); ++i)
  {
    ofs << data[i];
    if(i + 1 != fsize)
    {
      ofs << ';';
    }
  }
  for(size_t i = data.size(); i < fsize; ++i)
  {
    if(i + 1 != fsize)
    {
      ofs << ';';
    }
  }
}

template<>
void write_data<sva::PTransformd>(std::ofstream & ofs, const sva::PTransformd & data, size_t s)
{
  write_data(ofs, Eigen::Quaterniond(data.rotation()), s);
  ofs << ';';
  write_data(ofs, data.translation(), s);
}

template<>
void write_data<sva::ForceVecd>(std::ofstream & ofs, const sva::ForceVecd & data, size_t s)
{
  write_data(ofs, data.vector(), s);
}

template<>
void write_data<sva::MotionVecd>(std::ofstream & ofs, const sva::MotionVecd & data, size_t s)
{
  write_data(ofs, data.vector(), s);
}

template<typename T>
void write_data(std::ofstream & ofs,
                const mc_rtc::log::FlatLog & log,
                const std::string & entry,
                size_t idx,
                size_t fsize)
{
  const T * data = log.getRaw<T>(entry, idx);
  if(data)
  {
    write_data<T>(ofs, *data, fsize);
  }
  else
  {
    for(size_t i = 0; i < fsize - 1; ++i)
    {
      ofs << ';';
    }
  }
}

void write_data(std::ofstream & ofs, const mc_rtc::log::FlatLog & log, const SizedEntries & entries, size_t idx)
{
  size_t i = 0;
  for(const auto & e : entries)
  {
    switch(e.second.type)
    {
      case mc_rtc::log::LogType::Bool:
        write_data<bool>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Int8_t:
        write_data<int8_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Int16_t:
        write_data<int16_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Int32_t:
        write_data<int32_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Int64_t:
        write_data<int64_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Uint8_t:
        write_data<uint8_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Uint16_t:
        write_data<uint16_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Uint32_t:
        write_data<uint32_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Uint64_t:
        write_data<uint64_t>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Float:
        write_data<float>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Double:
        write_data<double>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::String:
        write_data<std::string>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Quaterniond:
        write_data<Eigen::Quaterniond>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Vector2d:
        write_data<Eigen::Vector2d>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Vector3d:
        write_data<Eigen::Vector3d>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::Vector6d:
        write_data<Eigen::Vector6d>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::VectorXd:
        write_data<Eigen::VectorXd>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::PTransformd:
        write_data<sva::PTransformd>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::ForceVecd:
        write_data<sva::ForceVecd>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::MotionVecd:
        write_data<sva::MotionVecd>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::VectorDouble:
        write_data<std::vector<double>>(ofs, log, e.first, idx, e.second.size);
        break;
      case mc_rtc::log::LogType::None:
        continue;
    }
    if(++i != entries.size())
    {
      ofs << ';';
    }
    else
    {
      ofs << '\n';
    }
  }
}

void mc_bin_to_log(const std::string & in, const std::string & out)
{
  mc_rtc::log::FlatLog log(in);
  std::ofstream ofs(out);
  if(!ofs.is_open())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to open {} for conversion from bin to log", out);
  }
  auto entries = write_header(ofs, log);
  for(size_t i = 0; i < log.size(); ++i)
  {
    write_data(ofs, log, entries, i);
  }
}
