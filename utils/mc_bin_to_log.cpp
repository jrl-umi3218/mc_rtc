#include <mc_rtc/log/serialization/fb_utils.h>

#include <mc_rtc/logging.h>

#include <fstream>


typedef std::tuple<std::string, mc_rtc::log::LogData, size_t> typed_key;

struct LogEntry
{
  const mc_rtc::log::Log * entry;
  char * raw;
  std::vector<typed_key> keys()
  {
    std::vector<typed_key> ret;
    if(entry->keys())
    {
      const auto & keys = *entry->keys();
      const auto & values_type = *entry->values_type();
      const auto & values = *entry->values();
      for(size_t i = 0; i < keys.size(); ++i)
      {
        auto k = keys[i];
        auto vt = mc_rtc::log::LogData(values_type[i]);
        size_t size = 0;
#define CASE_ENUM(VALUE)\
  case VALUE:\
    size = mc_rtc::log::CSVWriterHelper<VALUE>::key_size(values[i]);\
    break;
        switch(vt)
        {
          CASE_ENUM(mc_rtc::log::LogData_Bool)
          CASE_ENUM(mc_rtc::log::LogData_Double)
          CASE_ENUM(mc_rtc::log::LogData_DoubleVector)
          CASE_ENUM(mc_rtc::log::LogData_UnsignedInt)
          CASE_ENUM(mc_rtc::log::LogData_String)
          CASE_ENUM(mc_rtc::log::LogData_Vector2d)
          CASE_ENUM(mc_rtc::log::LogData_Vector3d)
          CASE_ENUM(mc_rtc::log::LogData_Quaterniond)
          CASE_ENUM(mc_rtc::log::LogData_PTransformd)
          CASE_ENUM(mc_rtc::log::LogData_ForceVecd)
          CASE_ENUM(mc_rtc::log::LogData_MotionVecd)
          default:
            break;
        };
#undef CASE_ENUM
        ret.push_back(std::make_tuple(k->str(), vt, size));
      }
    }
    return ret;
  }
};

struct Log
{
  Log(const std::string & name)
  {
    std::ifstream ifs(name, std::ifstream::binary);
    while(ifs)
    {
      int size = 0;
      ifs.read((char*)(&size), sizeof(int));
      if(ifs)
      {
        char * data = new char[size];
        ifs.read(data, size);
        entries.push_back({mc_rtc::log::GetLog(data), data});
        auto new_keys = entries.back().keys();
        if(keys.size() == 0)
        {
          keys = new_keys;
        }
        else
        {
          for(const auto & nk : new_keys)
          {
            const std::string & k = std::get<0>(nk);
            auto it = std::find_if(keys.begin(), keys.end(),
                                   [&k](std::vector<typed_key>::value_type & kit){
                                   return std::get<0>(kit) == k;
                                   });
            if(it != keys.end())
            {
              size_t n_size = std::get<2>(nk);
              size_t size = std::get<2>(*it);
              if(n_size != size)
              {
                LOG_ERROR("Key " << k << " was first seen with size " << size << " but was found again with size " << n_size)
                LOG_ERROR("The resulting CSV will be malformed")
              }
            }
            else
            {
              keys.push_back(nk);
            }
          }
        }
      }
    }
  }

  void write_header(std::ostream & os)
  {
    bool start = true;
    for(const auto & k : keys)
    {
      const auto & key = std::get<0>(k);
      auto vt = std::get<1>(k);
      size_t size = std::get<2>(k);
      if(size == 0)
      {
        continue;
      }
      if(start) { start = false; }
      else { os << ";"; }
#define CASE_ENUM(VALUE)\
  case VALUE:\
    mc_rtc::log::CSVWriterHelper<VALUE>::write_header(key, size, os);\
    break;
        switch(vt)
        {
          CASE_ENUM(mc_rtc::log::LogData_Bool)
          CASE_ENUM(mc_rtc::log::LogData_Double)
          CASE_ENUM(mc_rtc::log::LogData_DoubleVector)
          CASE_ENUM(mc_rtc::log::LogData_UnsignedInt)
          CASE_ENUM(mc_rtc::log::LogData_String)
          CASE_ENUM(mc_rtc::log::LogData_Vector2d)
          CASE_ENUM(mc_rtc::log::LogData_Vector3d)
          CASE_ENUM(mc_rtc::log::LogData_Quaterniond)
          CASE_ENUM(mc_rtc::log::LogData_PTransformd)
          CASE_ENUM(mc_rtc::log::LogData_ForceVecd)
          CASE_ENUM(mc_rtc::log::LogData_MotionVecd)
          default:
            break;
        };
#undef CASE_ENUM
    }
    os << std::endl;
  }

  void write_entry(LogEntry & entry, std::ostream & os, std::vector<typed_key> & current_keys)
  {
    auto entry_keys = entry.keys();
    if(entry_keys.size())
    {
      current_keys = entry_keys;
    }
    bool start = true;
    const auto & values = *entry.entry->values();
    auto get_key_index = [&current_keys](const std::string & k)
    {
      for(size_t i = 0; i < current_keys.size(); ++i)
      {
        const std::string & kn = std::get<0>(current_keys[i]);
        if(kn == k) { return i; }
      }
      return current_keys.size();
    };
    for(const auto & k : keys)
    {
      const std::string & key = std::get<0>(k);
      auto vt = std::get<1>(k);
      size_t size = std::get<2>(k);
      if(size == 0)
      {
        continue;
      }
      if(start) { start = false; }
      else { os << ";"; }
      size_t idx = get_key_index(key);
      if(idx != current_keys.size())
      {
        const void * value = values[idx];
#define CASE_ENUM(VALUE)\
  case VALUE:\
    mc_rtc::log::CSVWriterHelper<VALUE>::write_data(value, os);\
    break;
        switch(vt)
        {
          CASE_ENUM(mc_rtc::log::LogData_Bool)
          CASE_ENUM(mc_rtc::log::LogData_Double)
          CASE_ENUM(mc_rtc::log::LogData_DoubleVector)
          CASE_ENUM(mc_rtc::log::LogData_UnsignedInt)
          CASE_ENUM(mc_rtc::log::LogData_String)
          CASE_ENUM(mc_rtc::log::LogData_Vector2d)
          CASE_ENUM(mc_rtc::log::LogData_Vector3d)
          CASE_ENUM(mc_rtc::log::LogData_Quaterniond)
          CASE_ENUM(mc_rtc::log::LogData_PTransformd)
          CASE_ENUM(mc_rtc::log::LogData_ForceVecd)
          CASE_ENUM(mc_rtc::log::LogData_MotionVecd)
          default:
            break;
        };
#undef CASE_ENUM
      }
      else
      {
        for(size_t i = 1; i < size; ++i)
        {
          os << ";";
        }
      }
    }
    os << std::endl;
  }

  void convert(const std::string & out)
  {
    std::ofstream ofs(out);
    write_header(ofs);
    std::vector<typed_key> current_keys = {};
    for(auto & e : entries)
    {
      write_entry(e, ofs, current_keys);
    }
  }
  ~Log()
  {
    for(auto & e : entries)
    {
      delete[] e.raw;
    }
  }
  std::vector<LogEntry> entries;
  std::vector<typed_key> keys;
};

void usage(const char * bin)
{
  LOG_ERROR("Usage: " << bin << " [bin] [log]")
}

int main(int argc, char * argv[])
{
  if(argc != 3)
  {
    usage(argv[0]);
    return 1;
  }
  Log log(argv[1]);
  log.convert(argv[2]);
  return 0;
}
