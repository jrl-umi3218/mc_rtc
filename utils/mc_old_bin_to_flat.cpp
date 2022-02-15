#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wsign-conversion")
#include "mc_bin_flatbuffers/MCLog_generated.h"
MC_RTC_diagnostic_pop
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

template<bool is_numeric>
struct LogData
{
  using data_t = std::vector<std::string>;
  static void write(const data_t & data, std::ofstream & os)
  {
    uint64_t s_sz = 0;
    for(const auto & s : data)
    {
      s_sz = s.size();
      os.write((char *)&s_sz, sizeof(uint64_t));
      os.write(s.data(), static_cast<long>(s.size() * sizeof(char)));
    }
  }

  static void read(std::ifstream & is, data_t & data)
  {
    uint64_t s_sz = 0;
    for(uint64_t i = 0; i < data.size(); ++i)
    {
      is.read((char *)&s_sz, sizeof(uint64_t));
      data[i].resize(s_sz);
      is.read(const_cast<char *>(data[i].data()), static_cast<long>(data[i].size() * sizeof(char)));
    }
  }
};

template<>
struct LogData<true>
{
  using data_t = std::vector<double>;
  static void write(const data_t & data, std::ofstream & os)
  {
    os.write((const char *)data.data(), static_cast<long>(data.size() * sizeof(double)));
  }

  static void read(std::ifstream & is, data_t & data)
  {
    is.read((char *)data.data(), static_cast<long>(data.size() * sizeof(double)));
  }
};

struct LogLine
{
  virtual ~LogLine() {}
  virtual void write(std::ofstream & os) const = 0;
  virtual void read(std::ifstream & is) = 0;
  virtual void push() = 0;
  virtual void push(double) {}
  virtual void push(const std::string &) {}
  virtual bool is_numeric()
  {
    return false;
  }
};

struct NumericLogLine : public LogLine
{
  using data_t = typename LogData<true>::data_t;
  std::string key_;
  data_t data_;

  bool is_numeric() override
  {
    return true;
  }

  NumericLogLine(const std::string & k, uint64_t s) : key_(k), data_(s, std::numeric_limits<double>::quiet_NaN()) {}

  void write(std::ofstream & os) const override
  {
    uint64_t key_s = key_.size();
    os.put(1);
    os.write((char *)&key_s, sizeof(uint64_t));
    os.write(key_.data(), static_cast<long>(key_.size() * sizeof(char)));
    key_s = data_.size();
    os.write((char *)&key_s, sizeof(uint64_t));
    LogData<true>::write(data_, os);
  }

  void read(std::ifstream & is) override
  {
    uint64_t key_s = 0;
    is.read((char *)&key_s, sizeof(uint64_t));
    key_.resize(key_s);
    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wcast-qual")
    is.read((char *)key_.data(), static_cast<long>(key_s * sizeof(char)));
    MC_RTC_diagnostic_pop
    is.read((char *)&key_s, sizeof(uint64_t));
    data_.resize(key_s);
    LogData<true>::read(is, data_);
  }

  void push() override
  {
    data_.push_back(std::numeric_limits<double>::quiet_NaN());
  }

  void push(double d) override
  {
    if(std::fabs(d) < std::numeric_limits<double>::min())
    {
      data_.push_back(0);
    }
    else
    {
      data_.push_back(d);
    }
  }
};

struct StringLogLine : public LogLine
{
  using data_t = typename LogData<false>::data_t;
  std::string key_;
  data_t data_;

  StringLogLine(const std::string & k, uint64_t s) : key_(k), data_(s, "") {}

  void write(std::ofstream & os) const override
  {
    uint64_t key_s = key_.size();
    os.put(0);
    os.write((char *)&key_s, sizeof(uint64_t));
    os.write(key_.data(), static_cast<long>(key_.size() * sizeof(char)));
    key_s = data_.size();
    os.write((char *)&key_s, sizeof(uint64_t));
    LogData<false>::write(data_, os);
  }

  void read(std::ifstream & is) override
  {
    uint64_t key_s = 0;
    is.read((char *)&key_s, sizeof(uint64_t));
    key_.resize(key_s);
    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wcast-qual")
    is.read((char *)key_.data(), static_cast<long>(key_s * sizeof(char)));
    MC_RTC_diagnostic_pop
    is.read((char *)&key_s, sizeof(uint64_t));
    data_.resize(key_s);
    LogData<false>::read(is, data_);
  }

  void push() override
  {
    data_.push_back("");
  }

  void push(const std::string & s) override
  {
    data_.push_back(s);
  }
};

std::unordered_map<std::string, std::shared_ptr<LogLine>> readLog(const std::string & file)
{
  std::unordered_map<std::string, std::shared_ptr<LogLine>> ret;
  std::ifstream ifs(file, std::ifstream::binary);
  if(!ifs.is_open())
  {
    mc_rtc::log::error_and_throw("{} could not be opened!", file);
  }
  std::vector<std::string> current_keys;
  std::vector<std::string> empty_keys;
  std::vector<size_t> data_size;
  size_t entries = 0;
  auto addKey = [&](const std::string & key, bool is_numeric) {
    current_keys.push_back(key);
    if(ret.count(key) == 0)
    {
      if(is_numeric)
      {
        ret[key] = std::make_shared<NumericLogLine>(key, entries);
      }
      else
      {
        ret[key] = std::make_shared<StringLogLine>(key, entries);
      }
    }
  };
  auto addVectorKey = [&](const std::string & key, size_t size) {
    for(size_t i = 0; i < size; ++i)
    {
      addKey(key + "_" + std::to_string(i), true);
    }
    return size;
  };
  auto pushVector = [&](size_t s, size_t j, const mc_rtc::log::DoubleVector * v) {
    for(size_t i = 0; i < s; ++i)
    {
      ret[current_keys[j + i]]->push(v->v()->operator[](static_cast<flatbuffers::uoffset_t>(i)));
    }
  };
  while(ifs)
  {
    int size = 0;
    ifs.read((char *)&size, sizeof(int));
    if(!ifs)
    {
      break;
    }
    char * data = new char[size];
    ifs.read(data, size);
    auto * log = mc_rtc::log::GetLog(data);
    const auto & values_type = *log->values_type();
    const auto & values = *log->values();
    if(log->keys() && log->keys()->size())
    {
      current_keys.resize(0);
      empty_keys.resize(0);
      data_size.resize(0);
      const auto & keys = *log->keys();
      for(flatbuffers::uoffset_t i = 0; i < static_cast<flatbuffers::uoffset_t>(keys.size()); ++i)
      {
        auto k = keys[i];
        auto vt = mc_rtc::log::LogData(values_type[i]);
        size_t s = 0;
        switch(vt)
        {
          case mc_rtc::log::LogData_Bool:
          case mc_rtc::log::LogData_Double:
          case mc_rtc::log::LogData_UnsignedInt:
          case mc_rtc::log::LogData_UInt64:
          case mc_rtc::log::LogData_String:
            addKey(k->str(), vt != mc_rtc::log::LogData_String);
            s = 1;
            break;
          case mc_rtc::log::LogData_Quaterniond:
            addKey(k->str() + "_w", true);
            s += 1;
            /* fall through */
          case mc_rtc::log::LogData_Vector3d:
            addKey(k->str() + "_z", true);
            s += 1;
            /* fall through */
          case mc_rtc::log::LogData_Vector2d:
            addKey(k->str() + "_x", true);
            addKey(k->str() + "_y", true);
            s += 2;
            break;
          case mc_rtc::log::LogData_PTransformd:
            addKey(k->str() + "_qw", true);
            addKey(k->str() + "_qx", true);
            addKey(k->str() + "_qy", true);
            addKey(k->str() + "_qz", true);
            addKey(k->str() + "_tx", true);
            addKey(k->str() + "_ty", true);
            addKey(k->str() + "_tz", true);
            s = 7;
            break;
          case mc_rtc::log::LogData_ForceVecd:
            addKey(k->str() + "_cx", true);
            addKey(k->str() + "_cy", true);
            addKey(k->str() + "_cz", true);
            addKey(k->str() + "_fx", true);
            addKey(k->str() + "_fy", true);
            addKey(k->str() + "_fz", true);
            s = 6;
            break;
          case mc_rtc::log::LogData_MotionVecd:
            addKey(k->str() + "_wx", true);
            addKey(k->str() + "_wy", true);
            addKey(k->str() + "_wz", true);
            addKey(k->str() + "_vx", true);
            addKey(k->str() + "_vy", true);
            addKey(k->str() + "_vz", true);
            s = 6;
            break;
          case mc_rtc::log::LogData_DoubleVector:
            s = addVectorKey(k->str(), static_cast<const mc_rtc::log::DoubleVector *>(values[i])->v()->size());
            break;
          default:
            break;
        };
        data_size.push_back(s);
      }
      for(auto & e : ret)
      {
        if(std::find(current_keys.begin(), current_keys.end(), e.first) == current_keys.end())
        {
          empty_keys.push_back(e.first);
        }
      }
    }
    size_t j = 0;
    for(flatbuffers::uoffset_t i = 0; i < static_cast<flatbuffers::uoffset_t>(data_size.size()); ++i)
    {
      auto vt = values_type[i];
      const void * v = values[i];
      switch(vt)
      {
        case mc_rtc::log::LogData_Bool:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::Bool *>(v)->b());
          break;
        case mc_rtc::log::LogData_Double:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::Double *>(v)->d());
          break;
        case mc_rtc::log::LogData_UnsignedInt:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::UnsignedInt *>(v)->i());
          break;
        case mc_rtc::log::LogData_UInt64:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::UnsignedInt *>(v)->i());
          break;
        case mc_rtc::log::LogData_String:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::String *>(v)->s()->str());
          break;
        case mc_rtc::log::LogData_Vector2d:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::Vector2d *>(v)->x());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::Vector2d *>(v)->y());
          break;
        case mc_rtc::log::LogData_Vector3d:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::Vector3d *>(v)->z());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::Vector3d *>(v)->x());
          ret[current_keys[j + 2]]->push(static_cast<const mc_rtc::log::Vector3d *>(v)->y());
          break;
        case mc_rtc::log::LogData_Quaterniond:
          ret[current_keys[j]]->push(static_cast<const mc_rtc::log::Quaterniond *>(v)->w());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::Quaterniond *>(v)->z());
          ret[current_keys[j + 2]]->push(static_cast<const mc_rtc::log::Quaterniond *>(v)->x());
          ret[current_keys[j + 3]]->push(static_cast<const mc_rtc::log::Quaterniond *>(v)->y());
          break;
        case mc_rtc::log::LogData_PTransformd:
          ret[current_keys[j + 0]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->ori()->w());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->ori()->x());
          ret[current_keys[j + 2]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->ori()->y());
          ret[current_keys[j + 3]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->ori()->z());
          ret[current_keys[j + 4]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->pos()->x());
          ret[current_keys[j + 5]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->pos()->y());
          ret[current_keys[j + 6]]->push(static_cast<const mc_rtc::log::PTransformd *>(v)->pos()->z());
          break;
        case mc_rtc::log::LogData_ForceVecd:
          ret[current_keys[j + 0]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->couple()->x());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->couple()->y());
          ret[current_keys[j + 2]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->couple()->z());
          ret[current_keys[j + 3]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->force()->x());
          ret[current_keys[j + 4]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->force()->y());
          ret[current_keys[j + 5]]->push(static_cast<const mc_rtc::log::ForceVecd *>(v)->force()->z());
          break;
        case mc_rtc::log::LogData_MotionVecd:
          ret[current_keys[j + 0]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->angular()->x());
          ret[current_keys[j + 1]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->angular()->y());
          ret[current_keys[j + 2]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->angular()->z());
          ret[current_keys[j + 3]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->linear()->x());
          ret[current_keys[j + 4]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->linear()->y());
          ret[current_keys[j + 5]]->push(static_cast<const mc_rtc::log::MotionVecd *>(v)->linear()->z());
          break;
        case mc_rtc::log::LogData_DoubleVector:
          pushVector(data_size[i], j, static_cast<const mc_rtc::log::DoubleVector *>(v));
          break;
        default:
          break;
      };
      j += data_size[i];
    }
    for(auto & e : empty_keys)
    {
      ret[e]->push();
    }
    entries++;
    delete[] data;
  }
  return ret;
}

void writeFlatLog(const std::unordered_map<std::string, std::shared_ptr<LogLine>> & data, const std::string & file)
{
  std::ofstream ofs(file, std::ofstream::binary);
  uint64_t s = data.size();
  ofs.write((char *)&s, sizeof(uint64_t));
  for(const auto & d : data)
  {
    d.second->write(ofs);
  }
}

void usage(const char * bin)
{
  mc_rtc::log::error("Usage: {} [bin] ([flat])", bin);
}

int main(int argc, char * argv[])
{
  if(argc != 3 && argc != 2)
  {
    usage(argv[0]);
    return 1;
  }
  std::string in = argv[1];
  std::string out = "";
  if(argc == 3)
  {
    out = argv[2];
  }
  else
  {
    out = bfs::path(argv[1]).filename().replace_extension(".flat").string();
    if(out == in)
    {
      mc_rtc::log::error("Please specify a different output name");
      return 1;
    }
    mc_rtc::log::info("Output converted log to {}", out);
  }
  writeFlatLog(readLog(in), out);
  return 0;
}
