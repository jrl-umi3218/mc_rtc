/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_bin_utils.h"
#include <fstream>
#include <map>
#include <type_traits>

namespace utils
{

size_t nEntries(const mc_rtc::log::FlatLog & log, const std::map<std::string, mc_rtc::log::LogType> & entries)
{
  size_t s = 0;
  for(const auto & e : entries)
  {
    s += entrySize(log, e.first, e.second);
  }
  return s;
}

void write(uint64_t s, std::ostream & os)
{
  os.write((char *)&s, sizeof(uint64_t));
}

void write(const std::string & entry, bool numeric, std::ostream & os)
{
  os.put(numeric ? 1 : 0);
  write(entry.size(), os);
  os.write(entry.data(), static_cast<int>(entry.size() * sizeof(char)));
}

void write(const std::string & entry, const double * data, size_t size, std::ostream & os)
{
  write(entry, true, os);
  write(size, os);
  os.write((const char *)data, static_cast<int>(size * sizeof(double)));
}

void write(const std::string & entry, const std::vector<double> & data, std::ostream & os)
{
  write(entry, data.data(), data.size(), os);
}

void write(const std::string & entry, const std::vector<std::string> & data, std::ostream & os)
{
  write(entry, false, os);
  write(data.size(), os);
  for(const auto & s : data)
  {
    write(s.size(), os);
    os.write(s.data(), static_cast<int>(s.size() * sizeof(char)));
  }
}

template<typename T>
void write(const std::string & entry, const std::vector<T> & data, std::ostream & os)
{
  std::vector<double> out(data.size());
  for(size_t i = 0; i < data.size(); ++i)
  {
    out[i] = static_cast<double>(data[i]);
  }
  write(entry, out, os);
}

static const double nan = std::numeric_limits<double>::quiet_NaN();

template<typename T>
void write(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  static_assert(std::is_arithmetic<T>::value, "This default implementation only works for numeric types");
  static const T nan = std::numeric_limits<T>::quiet_NaN();
  write(entry, log.get<T>(entry, nan), os);
}

template<>
void write<std::string>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  write(entry, log.get<std::string>(entry, ""), os);
}

template<>
void write<Eigen::Quaterniond>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<Eigen::Quaterniond>(entry);
  std::vector<double> w(data.size(), nan);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  std::vector<double> z(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const Eigen::Quaterniond * v = data[i];
    if(v)
    {
      w[i] = v->w();
      x[i] = v->x();
      y[i] = v->y();
      z[i] = v->z();
    }
  }
  write(entry + "_w", w, os);
  write(entry + "_x", x, os);
  write(entry + "_y", y, os);
  write(entry + "_z", z, os);
}

template<>
void write<Eigen::Vector3d>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<Eigen::Vector3d>(entry);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  std::vector<double> z(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const Eigen::Vector3d * v = data[i];
    if(v)
    {
      x[i] = v->x();
      y[i] = v->y();
      z[i] = v->z();
    }
  }
  write(entry + "_x", x, os);
  write(entry + "_y", y, os);
  write(entry + "_z", z, os);
}

template<>
void write<Eigen::Vector6d>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<Eigen::Vector6d>(entry);
  std::vector<double> v0(data.size(), nan);
  std::vector<double> v1(data.size(), nan);
  std::vector<double> v2(data.size(), nan);
  std::vector<double> v3(data.size(), nan);
  std::vector<double> v4(data.size(), nan);
  std::vector<double> v5(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const Eigen::Vector6d * ptr = data[i];
    if(ptr)
    {
      const auto & v = *ptr;
      v0[i] = v(0);
      v1[i] = v(1);
      v2[i] = v(2);
      v3[i] = v(3);
      v4[i] = v(4);
      v5[i] = v(5);
    }
  }
  write(entry + "_0", v0, os);
  write(entry + "_1", v1, os);
  write(entry + "_2", v2, os);
  write(entry + "_3", v3, os);
  write(entry + "_4", v4, os);
  write(entry + "_5", v5, os);
}

template<>
void write<Eigen::Vector2d>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<Eigen::Vector2d>(entry);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const Eigen::Vector2d * v = data[i];
    if(v)
    {
      x[i] = v->x();
      y[i] = v->y();
    }
  }
  write(entry + "_x", x, os);
  write(entry + "_y", y, os);
}

template<>
void write<sva::PTransformd>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<sva::PTransformd>(entry);
  std::vector<double> qw(data.size(), nan);
  std::vector<double> qx(data.size(), nan);
  std::vector<double> qy(data.size(), nan);
  std::vector<double> qz(data.size(), nan);
  std::vector<double> tx(data.size(), nan);
  std::vector<double> ty(data.size(), nan);
  std::vector<double> tz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const sva::PTransformd * pt = data[i];
    if(pt)
    {
      Eigen::Quaterniond q(pt->rotation());
      qw[i] = q.w();
      qx[i] = q.x();
      qy[i] = q.y();
      qz[i] = q.z();
      tx[i] = pt->translation().x();
      ty[i] = pt->translation().y();
      tz[i] = pt->translation().z();
    }
  }
  write(entry + "_qw", qw, os);
  write(entry + "_qx", qx, os);
  write(entry + "_qy", qy, os);
  write(entry + "_qz", qz, os);
  write(entry + "_tx", tx, os);
  write(entry + "_ty", ty, os);
  write(entry + "_tz", tz, os);
}

template<>
void write<sva::ForceVecd>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<sva::ForceVecd>(entry);
  std::vector<double> cx(data.size(), nan);
  std::vector<double> cy(data.size(), nan);
  std::vector<double> cz(data.size(), nan);
  std::vector<double> fx(data.size(), nan);
  std::vector<double> fy(data.size(), nan);
  std::vector<double> fz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const sva::ForceVecd * fv = data[i];
    if(fv)
    {
      cx[i] = fv->couple().x();
      cy[i] = fv->couple().y();
      cz[i] = fv->couple().z();
      fx[i] = fv->force().x();
      fy[i] = fv->force().y();
      fz[i] = fv->force().z();
    }
  }
  write(entry + "_cx", cx, os);
  write(entry + "_cy", cy, os);
  write(entry + "_cz", cz, os);
  write(entry + "_fx", fx, os);
  write(entry + "_fy", fy, os);
  write(entry + "_fz", fz, os);
}

template<>
void write<sva::MotionVecd>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<sva::MotionVecd>(entry);
  std::vector<double> wx(data.size(), nan);
  std::vector<double> wy(data.size(), nan);
  std::vector<double> wz(data.size(), nan);
  std::vector<double> vx(data.size(), nan);
  std::vector<double> vy(data.size(), nan);
  std::vector<double> vz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const sva::MotionVecd * mv = data[i];
    if(mv)
    {
      wx[i] = mv->angular().x();
      wy[i] = mv->angular().y();
      wz[i] = mv->angular().z();
      vx[i] = mv->linear().x();
      vy[i] = mv->linear().y();
      vz[i] = mv->linear().z();
    }
  }
  write(entry + "_wx", wx, os);
  write(entry + "_wy", wy, os);
  write(entry + "_wz", wz, os);
  write(entry + "_vx", vx, os);
  write(entry + "_vy", vy, os);
  write(entry + "_vz", vz, os);
}

template<>
void write<std::vector<double>>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<std::vector<double>>(entry);
  std::vector<double> vec{};
  size_t maxS = 0;
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    if(v)
    {
      maxS = std::max<size_t>(maxS, v->size());
    }
  }
  vec.resize(maxS * data.size());
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    size_t vSize = 0;
    if(v)
    {
      vSize = v->size();
      const double * vData = v->data();
      for(size_t j = 0; j < vSize; ++j)
      {
        vec[j * data.size() + i] = vData[j];
      }
    }
    for(size_t j = vSize; j < maxS; ++j)
    {
      vec[j * data.size() + i] = nan;
    }
  }
  for(size_t i = 0; i < maxS; ++i)
  {
    write(entry + "_" + std::to_string(i), &vec[i * data.size()], data.size(), os);
  }
}

template<>
void write<Eigen::VectorXd>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<Eigen::VectorXd>(entry);
  std::vector<double> vec{};
  size_t maxS = 0;
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    if(v)
    {
      maxS = std::max<size_t>(maxS, static_cast<size_t>(v->size()));
    }
  }
  vec.resize(maxS * data.size());
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    size_t vSize = 0;
    if(v)
    {
      vSize = static_cast<size_t>(v->size());
      const double * vData = v->data();
      for(size_t j = 0; j < vSize; ++j)
      {
        vec[j * data.size() + i] = vData[j];
      }
    }
    for(size_t j = vSize; j < maxS; ++j)
    {
      vec[j * data.size() + i] = nan;
    }
  }
  for(size_t i = 0; i < maxS; ++i)
  {
    write(entry + "_" + std::to_string(i), &vec[i * data.size()], data.size(), os);
  }
}

} // namespace utils

void mc_bin_to_flat(const std::string & in,
                    const std::string & out,
                    const std::vector<std::string> & entriesFilter = {})
{
  mc_rtc::log::FlatLog log(in);
  auto entries = utils::entries(log, entriesFilter);
  std::ofstream ofs(out, std::ofstream::binary);
  utils::write(utils::nEntries(log, entries), ofs);
  for(const auto & e : entries)
  {
    const auto & entry = e.first;
    const auto & type = e.second;
    switch(type)
    {
      case mc_rtc::log::LogType::Bool:
        utils::write<bool>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Int8_t:
        utils::write<int8_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Int16_t:
        utils::write<int16_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Int32_t:
        utils::write<int32_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Int64_t:
        utils::write<int64_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Uint8_t:
        utils::write<uint8_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Uint16_t:
        utils::write<uint16_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Uint32_t:
        utils::write<uint32_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Uint64_t:
        utils::write<uint64_t>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Float:
        utils::write<float>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Double:
        utils::write<double>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::String:
        utils::write<std::string>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Quaterniond:
        utils::write<Eigen::Quaterniond>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Vector3d:
        utils::write<Eigen::Vector3d>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Vector2d:
        utils::write<Eigen::Vector2d>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::Vector6d:
        utils::write<Eigen::Vector6d>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::VectorXd:
        utils::write<Eigen::VectorXd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::PTransformd:
        utils::write<sva::PTransformd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::ForceVecd:
        utils::write<sva::ForceVecd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::MotionVecd:
        utils::write<sva::MotionVecd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogType::VectorDouble:
        utils::write<std::vector<double>>(log, entry, ofs);
        break;
      default:
        mc_rtc::log::error("Cannot convert {} into the flat format", entry);
        break;
    }
  }
}
