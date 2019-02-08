#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <fstream>

namespace mc_rtc
{

namespace log
{

FlatLog::FlatLog(const std::string & fpath)
{
  load(fpath);
}

void FlatLog::load(const std::string & fpath)
{
  data_.clear();
  append(fpath);
}

namespace
{

template<typename T>
void void_deleter(void const * ptr)
{
  delete static_cast<T const *>(ptr);
}

#define trivial_convert(ENUM, CPPT, MEMBER)                                             \
  FlatLog::record::unique_void_ptr convert(const mc_rtc::log::ENUM * v)                 \
  {                                                                                     \
    return FlatLog::record::unique_void_ptr(new CPPT(v->MEMBER()), void_deleter<CPPT>); \
  }
trivial_convert(Bool, bool, b);
trivial_convert(Double, double, d);
trivial_convert(UnsignedInt, unsigned int, i);
trivial_convert(UInt64, uint64_t, i);
trivial_convert(String, std::string, s()->str);
#undef trivial_convert

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::Vector2d * v)
{
  return FlatLog::record::unique_void_ptr(new Eigen::Vector2d(v->x(), v->y()), void_deleter<Eigen::Vector2d>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::Vector3d * v)
{
  return FlatLog::record::unique_void_ptr(new Eigen::Vector3d(v->x(), v->y(), v->z()), void_deleter<Eigen::Vector3d>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::Quaterniond * v)
{
  return FlatLog::record::unique_void_ptr(new Eigen::Quaterniond(v->w(), v->x(), v->y(), v->z()),
                                          void_deleter<Eigen::Quaterniond>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::PTransformd * pt)
{
  return FlatLog::record::unique_void_ptr(
      new sva::PTransformd(Eigen::Quaterniond{pt->ori()->w(), pt->ori()->x(), pt->ori()->y(), pt->ori()->z()},
                           {pt->pos()->x(), pt->pos()->y(), pt->pos()->z()}),
      void_deleter<sva::PTransformd>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::ForceVecd * fv)
{
  return FlatLog::record::unique_void_ptr(new sva::ForceVecd({fv->couple()->x(), fv->couple()->y(), fv->couple()->z()},
                                                             {fv->force()->x(), fv->force()->y(), fv->force()->z()}),
                                          void_deleter<sva::ForceVecd>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::MotionVecd * mv)
{
  return FlatLog::record::unique_void_ptr(
      new sva::ForceVecd({mv->angular()->x(), mv->angular()->y(), mv->angular()->z()},
                         {mv->linear()->x(), mv->linear()->y(), mv->linear()->z()}),
      void_deleter<sva::MotionVecd>);
}

FlatLog::record::unique_void_ptr convert(const mc_rtc::log::DoubleVector * v)
{
  auto * ret = new std::vector<double>(v->v()->size());
  for(size_t i = 0; i < ret->size(); ++i)
  {
    (*ret)[i] = v->v()->operator[](static_cast<flatbuffers::uoffset_t>(i));
  }
  return FlatLog::record::unique_void_ptr(ret, void_deleter<std::vector<double>>);
}

FlatLog::record::unique_void_ptr convert(LogData t, const void * v)
{
  switch(t)
  {
#define IMPL_CASE(ENUM)                                        \
  case mc_rtc::log::LogData_##ENUM:                            \
    return convert(static_cast<const mc_rtc::log::ENUM *>(v)); \
    break;
    IMPL_CASE(Bool)
    IMPL_CASE(Double)
    IMPL_CASE(DoubleVector)
    IMPL_CASE(UnsignedInt)
    IMPL_CASE(UInt64)
    IMPL_CASE(String)
    IMPL_CASE(Vector2d)
    IMPL_CASE(Vector3d)
    IMPL_CASE(Quaterniond)
    IMPL_CASE(PTransformd)
    IMPL_CASE(ForceVecd)
    IMPL_CASE(MotionVecd)
#undef IMPL_CASE
    default:
      break;
  };
  return FlatLog::record::unique_void_ptr(nullptr, void_deleter<double>);
}

} // namespace

FlatLog::record::record() : type(LogData_NONE), data(nullptr, void_deleter<double>) {}

void FlatLog::append(const std::string & f)
{
  auto fpath = bfs::path(f);
  if(!bfs::exists(f) || !bfs::is_regular(f))
  {
    LOG_ERROR("Could not open log " << f)
    return;
  }
  std::ifstream ifs(f, std::ifstream::binary);
  if(!ifs.is_open())
  {
    LOG_ERROR("Failed to open " << f)
    return;
  }
  auto allKeys = entries();
  std::vector<std::string> currentKeys = {};
  std::set<std::string> missingKeys = {};
  size_t size = 0;
  if(allKeys.size())
  {
    size = data_[*allKeys.begin()].size();
  }
  std::vector<char> buffer;
  while(ifs)
  {
    int entrySize = 0;
    ifs.read((char *)&entrySize, sizeof(int));
    if(!ifs)
    {
      size -= 1;
      break;
    }
    if(buffer.size() < static_cast<size_t>(entrySize))
    {
      buffer.resize(entrySize);
    }
    ifs.read(buffer.data(), entrySize);
    auto * log = mc_rtc::log::GetLog(buffer.data());
    if(log->keys() && log->keys()->size())
    {
      for(const auto & k : missingKeys)
      {
        data_[k].resize(size);
      }
      currentKeys.clear();
      const auto & keys = *log->keys();
      for(const auto & k_ffb : keys)
      {
        const auto & k = k_ffb->str();
        currentKeys.push_back(k);
        allKeys.insert(k);
        data_[k].resize(size);
      }
      missingKeys.clear();
      std::set<std::string> sortedKeys(currentKeys.begin(), currentKeys.end());
      std::set_difference(allKeys.begin(), allKeys.end(), sortedKeys.begin(), sortedKeys.end(),
                          std::inserter(missingKeys, missingKeys.begin()));
    }
    const auto & values = *log->values();
    const auto & values_type = *log->values_type();
    for(flatbuffers::uoffset_t i = 0; i < values_type.size(); ++i)
    {
      const auto & k = currentKeys[i];
      auto vt = mc_rtc::log::LogData(values_type[i]);
      const void * v = values[i];
      data_[k].push_back({vt, convert(vt, v)});
    }
    size += 1;
  }
  for(const auto & k : missingKeys)
  {
    data_[k].resize(size);
  }
}

std::set<std::string> FlatLog::entries() const
{
  std::set<std::string> ret;
  for(const auto & e : data_)
  {
    ret.insert(e.first);
  }
  return ret;
}

bool FlatLog::has(const std::string & entry) const
{
  return data_.count(entry) != 0;
}

std::set<LogData> FlatLog::types(const std::string & entry) const
{
  if(!has(entry))
  {
    LOG_ERROR("No entry named " << entry << " in the loaded log")
    return {};
  }
  std::set<LogData> ret;
  for(const auto & r : data_.at(entry))
  {
    if(r.type != mc_rtc::log::LogData_NONE)
    {
      ret.insert(r.type);
    }
  }
  return ret;
}

} // namespace log

} // namespace mc_rtc
