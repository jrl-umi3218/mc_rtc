#pragma once

#include "MCLog_generated.h"

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rtc
{

namespace log
{

/** This structure allows to convert native C++ types to their flatbuffer
 * counterpart */
template<typename T>
struct LogDataHelper
{
  /** Holds the value type corresponding to the C++ type */
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_NONE;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
  /** Convert the C++ object to an anonymous flatbuffer object */
  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder &,
                                             const T &)
  {
    static_assert(sizeof(T) == 0, "This type is not handled by the logger interface");
  }
#pragma GCC diagnostic pop
};

/** This macro allows to define implementations for LogDataHelper in
 * simple cases */
#define IMPL_LDH(TYPE, LD_TYPE, S_FN, ARG_NAME, ...)\
template<>\
struct LogDataHelper<TYPE>\
{\
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LD_TYPE;\
  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,\
                                             const TYPE & ARG_NAME)\
  {\
    return mc_rtc::log::S_FN(builder, __VA_ARGS__).Union();\
  }\
}

IMPL_LDH(bool, LogData_Bool, CreateBool, b, b);
IMPL_LDH(double, LogData_Double, CreateDouble, d, d);
IMPL_LDH(std::vector<double>, LogData_DoubleVector, CreateDoubleVectorDirect, v, &v);
IMPL_LDH(unsigned int, LogData_UnsignedInt, CreateUnsignedInt, i, i);
IMPL_LDH(std::string, LogData_String, CreateStringDirect, s, s.c_str());
IMPL_LDH(Eigen::Vector2d, LogData_Vector2d, CreateVector2d, v, v.x(), v.y());
IMPL_LDH(Eigen::Vector3d, LogData_Vector3d, CreateVector3d, v, v.x(), v.y(), v.z());
IMPL_LDH(Eigen::Quaterniond, LogData_Quaterniond, CreateQuaterniond, q, q.w(), q.x(), q.y(), q.z());

#undef IMPL_DATA_HELPER

template<>
struct LogDataHelper<sva::PTransformd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_PTransformd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,
                                             const sva::PTransformd & pt)
  {
    auto q = Eigen::Quaterniond(pt.rotation());
    auto fb_q = mc_rtc::log::CreateQuaterniond(builder, q.w(), q.x(), q.y(), q.z());
    const auto & t = pt.translation();
    auto fb_t = mc_rtc::log::CreateVector3d(builder, t.x(), t.y(), t.z());
    return mc_rtc::log::CreatePTransformd(builder, fb_q, fb_t).Union();
  }
};

template<>
struct LogDataHelper<sva::ForceVecd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_ForceVecd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,
                                             const sva::ForceVecd & fv)
  {
    const auto & couple = fv.couple();
    auto fb_couple = mc_rtc::log::CreateVector3d(builder, couple.x(), couple.y(), couple.z());
    const auto & force = fv.force();
    auto fb_force = mc_rtc::log::CreateVector3d(builder, force.x(), force.y(), force.z());
    return mc_rtc::log::CreateForceVecd(builder, fb_couple, fb_force).Union();
  }
};

template<>
struct LogDataHelper<sva::MotionVecd>
{
  static constexpr mc_rtc::log::LogData value_type = mc_rtc::log::LogData_MotionVecd;

  static flatbuffers::Offset<void> serialize(flatbuffers::FlatBufferBuilder & builder,
                                             const sva::MotionVecd & mv)
  {
    const auto & angular = mv.angular();
    auto fb_angular = mc_rtc::log::CreateVector3d(builder, angular.x(), angular.y(), angular.z());
    const auto & linear = mv.linear();
    auto fb_linear = mc_rtc::log::CreateVector3d(builder, linear.x(), linear.y(), linear.z());
    return mc_rtc::log::CreateMotionVecd(builder, fb_angular, fb_linear).Union();
  }
};

/** Type-traits for serializable types
 *
 * value is true if T is serializable
 *
 */
template<typename T>
struct is_serializable
{
  static constexpr bool value = LogDataHelper<T>::value_type != mc_rtc::log::LogData_NONE;
};

/** Type-traits for callables that returns a serializable type
 *
 *  value is true if T() return type (after decaying) is serializable
 *
 */
template<typename T>
struct callback_is_serializable
{
  using ret_type = typename std::result_of<T()>::type;
  using base_type = typename std::decay<ret_type>::type;
  static constexpr bool value = is_serializable<base_type>::value;
};

/** Type-traits to recognize a vector */
template<typename T>
struct is_vector
{
  static constexpr bool value = false;
};

template<typename T>
struct is_vector<std::vector<T>>
{
  static constexpr bool value = true;
};

/** Type-traits to recognize a callable object that returns a const reference
 * to a vector */
template<typename T>
struct callback_returns_crv
{
  using ret_type = typename std::result_of<T()>::type;
  using base_type = typename std::decay<ret_type>::type;
  static constexpr bool value = ! is_serializable<base_type>::value &&
                                is_vector<base_type>::value &&
                                std::is_reference<ret_type>::value &&
                                std::is_const<typename std::remove_reference<ret_type>::type>::value;
};

/** Type-traits for callables that returns a const reference to a vector of a
 * serializable types
 *
 * value is true if T() returns a const reference to a vector of a serializable
 * type
 *
 */
template<typename T, typename enable = void>
struct callback_is_crv_of_serializable
{
  static constexpr bool value = false;
};

template<typename T>
struct callback_is_crv_of_serializable<T, typename std::enable_if<callback_returns_crv<T>::value>::type>
{
  static constexpr bool value = is_serializable<typename callback_returns_crv<T>::base_type::value_type>::value;
};

template<typename T, typename = typename std::enable_if<is_serializable<T>::value>::type>
void AddLogData(flatbuffers::FlatBufferBuilder & builder,
                std::vector<uint8_t> & value_types,
                std::vector<flatbuffers::Offset<void>> & values,
                const T & value)
{
  value_types.push_back(LogDataHelper<T>::value_type);
  values.push_back(LogDataHelper<T>::serialize(builder, value));
}

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
  static size_t key_size(const void *) { return 0; }
  static void write_header(const std::string &, size_t, std::ostream &) {}
  static void write_data(const void *, std::ostream &) {}
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Bool>
{
  static size_t key_size(const void *) { return 1; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key;
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto b = static_cast<const mc_rtc::log::Bool*>(data);
    os << b->b();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Double>
{
  static size_t key_size(const void *) { return 1; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key;
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto d = static_cast<const mc_rtc::log::Double*>(data);
    os << d->d();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_DoubleVector>
{
  static size_t key_size(const void * data)
  {
    auto v = static_cast<const mc_rtc::log::DoubleVector*>(data);
    return v->v()->size();
  }
  static void write_header(const std::string & key, size_t size,
                    std::ostream & os)
  {
    for(size_t i = 0; i < size; ++i)
    {
      if(i != 0) { os << ";"; }
      os << key << "_" << i;
    }
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto s_v = static_cast<const mc_rtc::log::DoubleVector*>(data);
    const auto & v = *s_v->v();
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    for(size_t i = 0; i < v.size(); ++i)
    {
      if(i != 0) { os << ";"; }
      os << v[i];
    }
#pragma GCC diagnostic pop
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_UnsignedInt>
{
  static size_t key_size(const void *) { return 1; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key;
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto u = static_cast<const mc_rtc::log::UnsignedInt*>(data);
    os << u->i();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_String>
{
  static size_t key_size(const void *) { return 1; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key;
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto s_s = static_cast<const mc_rtc::log::String*>(data);
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
    os << '"' << safe_s << '"';
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Vector2d>
{
  static size_t key_size(const void *) { return 2; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_x"
       << ";" << key << "_y";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto v3d = static_cast<const mc_rtc::log::Vector2d*>(data);
    os << v3d->x() << ";" << v3d->y();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Vector3d>
{
  static size_t key_size(const void *) { return 3; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_x"
       << ";" << key << "_y"
       << ";" << key << "_z";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto v3d = static_cast<const mc_rtc::log::Vector3d*>(data);
    os << v3d->x() << ";" << v3d->y() << ";" << v3d->z();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_Quaterniond>
{
  static size_t key_size(const void *) { return 4; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_w"
       << ";" << key << "_x"
       << ";" << key << "_y"
       << ";" << key << "_z";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto qd = static_cast<const mc_rtc::log::Quaterniond*>(data);
    os << qd->w() << ";" << qd->x() << ";" << qd->y() << ";" << qd->z();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_PTransformd>
{
  static size_t key_size(const void *) { return 7; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_qw"
       << ";" << key << "_qx"
       << ";" << key << "_qy"
       << ";" << key << "_qz"
       << ";" << key << "_tx"
       << ";" << key << "_ty"
       << ";" << key << "_tz";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto pt = static_cast<const mc_rtc::log::PTransformd*>(data);
    auto q = pt->ori();
    auto t = pt->pos();
    os << q->w()
       << ";" << q->x()
       << ";" << q->y()
       << ";" << q->z()
       << ";" << t->x()
       << ";" << t->y()
       << ";" << t->z();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_ForceVecd>
{
  static size_t key_size(const void *) { return 6; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_fx"
       << ";" << key << "_fy"
       << ";" << key << "_fz"
       << ";" << key << "_cx"
       << ";" << key << "_cy"
       << ";" << key << "_cz";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto fv = static_cast<const mc_rtc::log::ForceVecd*>(data);
    auto f = fv->force();
    auto c = fv->couple();
    os << f->x()
       << ";" << f->y()
       << ";" << f->z()
       << ";" << c->x()
       << ";" << c->y()
       << ";" << c->z();
  }
};

template<>
struct CSVWriterHelper<mc_rtc::log::LogData_MotionVecd>
{
  static size_t key_size(const void *) { return 6; }
  static void write_header(const std::string & key, size_t,
                    std::ostream & os)
  {
    os << key << "_wx"
       << ";" << key << "_wy"
       << ";" << key << "_wz"
       << ";" << key << "_vx"
       << ";" << key << "_vy"
       << ";" << key << "_vz";
  }
  static void write_data(const void * data, std::ostream & os)
  {
    auto mv = static_cast<const mc_rtc::log::MotionVecd*>(data);
    const auto & a = mv->angular();
    const auto & l = mv->linear();
    os << a->x()
       << ";" << a->y()
       << ";" << a->z()
       << ";" << l->x()
       << ";" << l->y()
       << ";" << l->z();
  }
};

}

}
