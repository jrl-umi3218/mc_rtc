#pragma once

#include "dtoa_milo.h"
#include "fb_utils.h"

namespace
{
char buffer[256];
}

namespace mc_rtc
{

namespace log
{

/** This struct simplifies the conversion from LogData entries to CSV data.
 *
 * The template parameter is a mc_rtc::log::LogData enum value.
 *
 * The default implementation does not generate output to the file
 *
 */
template<mc_rtc::log::LogData T>
struct CSVWriterHelper
{
  static size_t key_size(const void *)
  {
    return 0;
  }
  static void write_header(const std::string &, size_t, std::string &) {}
  static void write_data(const void *, std::string &) {}
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Bool>
{
  static size_t key_size(const void *)
  {
    return 1;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key;
  }
  static void write_data(const void * data, std::string & os)
  {
    auto b = static_cast<const mc_rtc::log::Bool *>(data);
    os += b->b() ? "1" : "0";
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Double>
{
  static size_t key_size(const void *)
  {
    return 1;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key;
  }
  static void write_data(const void * data, std::string & os)
  {
    auto d = static_cast<const mc_rtc::log::Double *>(data);
    os += dtoa_milo(d->d(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_DoubleVector>
{
  static size_t key_size(const void * data)
  {
    auto v = static_cast<const mc_rtc::log::DoubleVector *>(data);
    return v->v()->size();
  }
  static void write_header(const std::string & key, size_t size, std::string & os)
  {
    for(size_t i = 0; i < size; ++i)
    {
      if(i != 0)
      {
        os += ";";
      }
      os += key + "_" + std::to_string(i);
    }
  }
  static void write_data(const void * data, std::string & os)
  {
    auto s_v = static_cast<const mc_rtc::log::DoubleVector *>(data);
    const auto & v = *s_v->v();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    for(size_t i = 0; i < v.size(); ++i)
    {
      if(i != 0)
      {
        os += ";";
      }
      os += dtoa_milo(v[static_cast<flatbuffers::uoffset_t>(i)], buffer);
    }
#pragma GCC diagnostic pop
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_UnsignedInt>
{
  static size_t key_size(const void *)
  {
    return 1;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key;
  }
  static void write_data(const void * data, std::string & os)
  {
    auto u = static_cast<const mc_rtc::log::UnsignedInt *>(data);
    os += std::to_string(u->i());
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_UInt64>
{
  static size_t key_size(const void *)
  {
    return 1;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key;
  }
  static void write_data(const void * data, std::string & os)
  {
    auto u = static_cast<const mc_rtc::log::UInt64 *>(data);
    os += std::to_string(u->i());
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_String>
{
  static size_t key_size(const void *)
  {
    return 1;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key;
  }
  static void write_data(const void * data, std::string & os)
  {
    auto s_s = static_cast<const mc_rtc::log::String *>(data);
    std::string s = s_s->s()->str();
    std::string safe_s;
    safe_s.reserve(s.size());
    size_t pos = 0;
    size_t prev_pos = 0;
    while(pos != std::string::npos)
    {
      pos = s.find('"', prev_pos);
      safe_s.append(s, prev_pos, pos - prev_pos);
      if(pos != std::string::npos)
      {
        safe_s.append(2, '"');
      }
      prev_pos = pos + 1;
    }
    os += '"' + safe_s + '"';
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Vector2d>
{
  static size_t key_size(const void *)
  {
    return 2;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_x;" + key + "_y";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto v3d = static_cast<const mc_rtc::log::Vector2d *>(data);
    os += dtoa_milo(v3d->x(), buffer);
    os += ";";
    os += dtoa_milo(v3d->y(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Vector3d>
{
  static size_t key_size(const void *)
  {
    return 3;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_x;" + key + "_y;" + key + "_z";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto v3d = static_cast<const mc_rtc::log::Vector3d *>(data);
    os += dtoa_milo(v3d->x(), buffer);
    os += ";";
    os += dtoa_milo(v3d->y(), buffer);
    os += ";";
    os += dtoa_milo(v3d->z(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Quaterniond>
{
  static size_t key_size(const void *)
  {
    return 4;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_w;" + key + "_x;" + key + "_y;" + key + "_z";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto qd = static_cast<const mc_rtc::log::Quaterniond *>(data);
    os += dtoa_milo(qd->w(), buffer);
    os += ";";
    os += dtoa_milo(qd->x(), buffer);
    os += ";";
    os += dtoa_milo(qd->y(), buffer);
    os += ";";
    os += dtoa_milo(qd->z(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_PTransformd>
{
  static size_t key_size(const void *)
  {
    return 7;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_qw;" + key + "_qx;" + key + "_qy;" + key + "_qz;" + key + "_tx;" + key + "_ty;" + key + "_tz";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto pt = static_cast<const mc_rtc::log::PTransformd *>(data);
    auto q = pt->ori();
    auto t = pt->pos();
    os += dtoa_milo(q->w(), buffer);
    os += ";";
    os += dtoa_milo(q->x(), buffer);
    os += ";";
    os += dtoa_milo(q->y(), buffer);
    os += ";";
    os += dtoa_milo(q->z(), buffer);
    os += ";";
    os += dtoa_milo(t->x(), buffer);
    os += ";";
    os += dtoa_milo(t->y(), buffer);
    os += ";";
    os += dtoa_milo(t->z(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_ForceVecd>
{
  static size_t key_size(const void *)
  {
    return 6;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_fx;" + key + "_fy;" + key + "_fz;" + key + "_cx;" + key + "_cy;" + key + "_cz";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto fv = static_cast<const mc_rtc::log::ForceVecd *>(data);
    auto f = fv->force();
    auto c = fv->couple();
    os += dtoa_milo(f->x(), buffer);
    os += ";";
    os += dtoa_milo(f->y(), buffer);
    os += ";";
    os += dtoa_milo(f->z(), buffer);
    os += ";";
    os += dtoa_milo(c->x(), buffer);
    os += ";";
    os += dtoa_milo(c->y(), buffer);
    os += ";";
    os += dtoa_milo(c->z(), buffer);
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_MotionVecd>
{
  static size_t key_size(const void *)
  {
    return 6;
  }
  static void write_header(const std::string & key, size_t, std::string & os)
  {
    os += key + "_wx;" + key + "_wy;" + key + "_wz;" + key + "_vx;" + key + "_vy;" + key + "_vz";
  }
  static void write_data(const void * data, std::string & os)
  {
    auto mv = static_cast<const mc_rtc::log::MotionVecd *>(data);
    const auto & a = mv->angular();
    const auto & l = mv->linear();
    os += dtoa_milo(a->x(), buffer);
    os += ";";
    os += dtoa_milo(a->y(), buffer);
    os += ";";
    os += dtoa_milo(a->z(), buffer);
    os += ";";
    os += dtoa_milo(l->x(), buffer);
    os += ";";
    os += dtoa_milo(l->y(), buffer);
    os += ";";
    os += dtoa_milo(l->z(), buffer);
  }
};

} // namespace log

} // namespace mc_rtc
