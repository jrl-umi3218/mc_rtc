/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/FlatLog.h>

#include <fstream>
#include <map>
#include <type_traits>

namespace utils
{

bool isValid(const mc_rtc::log::LogData & type)
{
  switch(type)
  {
    case mc_rtc::log::LogData_Bool:
    case mc_rtc::log::LogData_Double:
    case mc_rtc::log::LogData_UnsignedInt:
    case mc_rtc::log::LogData_UInt64:
    case mc_rtc::log::LogData_String:
    case mc_rtc::log::LogData_Quaterniond:
    case mc_rtc::log::LogData_Vector3d:
    case mc_rtc::log::LogData_Vector2d:
    case mc_rtc::log::LogData_PTransformd:
    case mc_rtc::log::LogData_ForceVecd:
    case mc_rtc::log::LogData_MotionVecd:
    case mc_rtc::log::LogData_DoubleVector:
      return true;
    default:
      return false;
  }
}

std::map<std::string, mc_rtc::log::LogData> entries(const mc_rtc::log::FlatLog & log)
{
  std::map<std::string, mc_rtc::log::LogData> ret;
  for(const auto & e : log.entries())
  {
    auto t = log.type(e);
    if(isValid(t))
    {
      ret[e] = t;
    }
    else
    {
      LOG_WARNING(e << " cannot be converted into a flat log")
    }
  }
  return ret;
}

size_t vectorEntrySize(const mc_rtc::log::FlatLog & log, const std::string & entry)
{
  size_t s = 0;
  auto data = log.getRaw<mc_rtc::log::DoubleVector>(entry);
  for(const auto & v : data)
  {
    if(v)
    {
      s = std::max<size_t>(s, v->v()->size());
    }
  }
  return s;
}

size_t nEntries(const mc_rtc::log::FlatLog & log, const std::map<std::string, mc_rtc::log::LogData> & entries)
{
  size_t s = 0;
  for(const auto & e : entries)
  {
    const auto & entry = e.first;
    const auto & type = e.second;
    switch(type)
    {
      case mc_rtc::log::LogData_Bool:
      case mc_rtc::log::LogData_Double:
      case mc_rtc::log::LogData_UnsignedInt:
      case mc_rtc::log::LogData_UInt64:
      case mc_rtc::log::LogData_String:
        s += 1;
        break;
      case mc_rtc::log::LogData_Quaterniond:
        s += 4;
        break;
      case mc_rtc::log::LogData_Vector3d:
        s += 3;
        break;
      case mc_rtc::log::LogData_Vector2d:
        s += 2;
        break;
      case mc_rtc::log::LogData_PTransformd:
        s += 7;
        break;
      case mc_rtc::log::LogData_ForceVecd:
      case mc_rtc::log::LogData_MotionVecd:
        s += 6;
        break;
      case mc_rtc::log::LogData_DoubleVector:
        s += vectorEntrySize(log, entry);
        break;
      default:
        break;
    }
  }
  return s;
}

void write(size_t s, std::ostream & os)
{
  os.write((char *)&s, sizeof(size_t));
}

void write(const std::string & entry, bool numeric, std::ostream & os)
{
  os.put(numeric ? 1 : 0);
  write(entry.size(), os);
  os.write(entry.data(), entry.size() * sizeof(char));
}

void write(const std::string & entry, const double * data, size_t size, std::ostream & os)
{
  write(entry, true, os);
  write(size, os);
  os.write((const char *)data, size * sizeof(double));
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
    os.write(s.data(), s.size() * sizeof(char));
  }
}

static double nan = std::numeric_limits<double>::quiet_NaN();

template<typename T>
void write(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  static_assert(std::is_arithmetic<T>::value, "This default implementation only works for numeric types");
  write(entry, log.get<double>(entry, nan), os);
}

template<>
void write<std::string>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  write(entry, log.get<std::string>(entry, ""), os);
}

template<>
void write<Eigen::Quaterniond>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<mc_rtc::log::Quaterniond>(entry);
  std::vector<double> w(data.size(), nan);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  std::vector<double> z(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::Quaterniond * v = data[i];
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
  auto data = log.getRaw<mc_rtc::log::Vector3d>(entry);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  std::vector<double> z(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::Vector3d * v = data[i];
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
void write<Eigen::Vector2d>(const mc_rtc::log::FlatLog & log, const std::string & entry, std::ostream & os)
{
  auto data = log.getRaw<mc_rtc::log::Vector2d>(entry);
  std::vector<double> x(data.size(), nan);
  std::vector<double> y(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::Vector2d * v = data[i];
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
  auto data = log.getRaw<mc_rtc::log::PTransformd>(entry);
  std::vector<double> qw(data.size(), nan);
  std::vector<double> qx(data.size(), nan);
  std::vector<double> qy(data.size(), nan);
  std::vector<double> qz(data.size(), nan);
  std::vector<double> tx(data.size(), nan);
  std::vector<double> ty(data.size(), nan);
  std::vector<double> tz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::PTransformd * pt = data[i];
    if(pt)
    {
      qw[i] = pt->ori()->w();
      qx[i] = pt->ori()->x();
      qy[i] = pt->ori()->y();
      qz[i] = pt->ori()->z();
      tx[i] = pt->pos()->x();
      ty[i] = pt->pos()->y();
      tz[i] = pt->pos()->z();
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
  auto data = log.getRaw<mc_rtc::log::ForceVecd>(entry);
  std::vector<double> cx(data.size(), nan);
  std::vector<double> cy(data.size(), nan);
  std::vector<double> cz(data.size(), nan);
  std::vector<double> fx(data.size(), nan);
  std::vector<double> fy(data.size(), nan);
  std::vector<double> fz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::ForceVecd * fv = data[i];
    if(fv)
    {
      cx[i] = fv->couple()->x();
      cy[i] = fv->couple()->y();
      cz[i] = fv->couple()->z();
      fx[i] = fv->force()->x();
      fy[i] = fv->force()->y();
      fz[i] = fv->force()->z();
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
  auto data = log.getRaw<mc_rtc::log::MotionVecd>(entry);
  std::vector<double> wx(data.size(), nan);
  std::vector<double> wy(data.size(), nan);
  std::vector<double> wz(data.size(), nan);
  std::vector<double> vx(data.size(), nan);
  std::vector<double> vy(data.size(), nan);
  std::vector<double> vz(data.size(), nan);
  for(size_t i = 0; i < data.size(); ++i)
  {
    const mc_rtc::log::MotionVecd * mv = data[i];
    if(mv)
    {
      wx[i] = mv->angular()->x();
      wy[i] = mv->angular()->y();
      wz[i] = mv->angular()->z();
      vx[i] = mv->linear()->x();
      vy[i] = mv->linear()->y();
      vz[i] = mv->linear()->z();
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
  auto data = log.getRaw<mc_rtc::log::DoubleVector>(entry);
  std::vector<double> vec{};
  size_t maxS = 0;
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    if(v)
    {
      maxS = std::max<size_t>(maxS, v->v()->size());
    }
  }
  vec.resize(maxS * data.size());
  for(size_t i = 0; i < data.size(); ++i)
  {
    auto * v = data[i];
    size_t vSize = 0;
    if(v)
    {
      vSize = v->v()->size();
      const double * vData = v->v()->data();
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

void mc_bin_to_flat(const std::string & in, const std::string & out)
{
  mc_rtc::log::FlatLog log(in);
  auto entries = utils::entries(log);
  std::ofstream ofs(out, std::ofstream::binary);
  utils::write(utils::nEntries(log, entries), ofs);
  for(const auto & e : entries)
  {
    const auto & entry = e.first;
    const auto & type = e.second;
    switch(type)
    {
      case mc_rtc::log::LogData_Bool:
      case mc_rtc::log::LogData_Double:
      case mc_rtc::log::LogData_UnsignedInt:
      case mc_rtc::log::LogData_UInt64:
        utils::write<double>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_String:
        utils::write<std::string>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_Quaterniond:
        utils::write<Eigen::Quaterniond>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_Vector3d:
        utils::write<Eigen::Vector3d>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_Vector2d:
        utils::write<Eigen::Vector2d>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_PTransformd:
        utils::write<sva::PTransformd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_ForceVecd:
        utils::write<sva::ForceVecd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_MotionVecd:
        utils::write<sva::MotionVecd>(log, entry, ofs);
        break;
      case mc_rtc::log::LogData_DoubleVector:
        utils::write<std::vector<double>>(log, entry, ofs);
        break;
      default:
        break;
    }
  }
}
