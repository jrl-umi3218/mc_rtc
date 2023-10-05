#pragma once

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>

#include <optional>

#include "msgpack.h"

namespace mc_rtc::log::internal
{

template<typename T>
void void_deleter(void const * ptr)
{
  delete static_cast<T const *>(ptr);
}

inline std::optional<std::string_view> stringFromNode(mpack_node_t node)
{
  if(mpack_node_type(node) != mpack_type_str) { return std::nullopt; }
  return std::string_view(mpack_node_str(node), mpack_node_strlen(node));
}

inline std::optional<std::vector<std::string>> stringVectorFromNode(mpack_node_t node)
{
  if(mpack_node_type(node) != mpack_type_array)
  {
    mc_rtc::log::error("[stringVectorFromNode] mpack_node does not contain a vector of string");
    return std::nullopt;
  }
  std::vector<std::string> out;
  size_t s = mpack_node_array_length(node);
  out.reserve(s);
  for(size_t i = 0; i < s; ++i)
  {
    auto str = stringFromNode(mpack_node_array_at(node, i));
    if(!str)
    {
      mc_rtc::log::error("[stringVectorFromNode] mpack_node is not a string");
      return std::nullopt;
    }
    out.push_back(std::string(str.value()));
  }
  return out;
}

inline LogType logTypeFromNode(mpack_node_t node)
{
  static_assert(std::is_same<int32_t, std::underlying_type<LogType>::type>::value,
                "LogType should be an int32_t like thing");
  if(mpack_node_type(node) != mpack_type_int && mpack_node_type(node) != mpack_type_uint)
  {
    log::error("Stored LogType is not an integer");
    return LogType::None;
  }
  return LogType(mpack_node_i32(node));
}

inline LogType logTypeFromNode(mpack_node_t node, size_t idx)
{
  return logTypeFromNode(mpack_node_array_at(node, idx));
}

template<typename T>
struct DataFromNode
{
  static bool convert(mpack_node_t, T &)
  {
    static_assert(sizeof(T) == 0, "No conversion implemented for this type");
    return false;
  }
};

#define IMPL_DATAFROM_NODE(CPPT, CONDITION, MPACKGET)  \
  template<>                                           \
  struct DataFromNode<CPPT>                            \
  {                                                    \
    static bool convert(mpack_node_t node, CPPT & out) \
    {                                                  \
      const auto t = mpack_node_type(node);            \
      if(CONDITION) { return false; }                  \
      out = MPACKGET(node);                            \
      return true;                                     \
    }                                                  \
  };
IMPL_DATAFROM_NODE(bool, t != mpack_type_bool, mpack_node_bool)
IMPL_DATAFROM_NODE(int8_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_i8)
IMPL_DATAFROM_NODE(int16_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_i16)
IMPL_DATAFROM_NODE(int32_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_i32)
IMPL_DATAFROM_NODE(int64_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_i64)
IMPL_DATAFROM_NODE(uint8_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_u8)
IMPL_DATAFROM_NODE(uint16_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_u16)
IMPL_DATAFROM_NODE(uint32_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_u32)
IMPL_DATAFROM_NODE(uint64_t, t != mpack_type_int && t != mpack_type_uint, mpack_node_u64)
IMPL_DATAFROM_NODE(float, t != mpack_type_float, mpack_node_float)
#undef IMPL_DATAFROM_NODE

template<>
struct DataFromNode<double>
{
  static bool convert(mpack_node_t node, double & out)
  {
    if(mpack_node_type(node) != mpack_type_float && mpack_node_type(node) != mpack_type_double) { return false; }
    out = mpack_node_double(node);
    return true;
  }
};

template<>
struct DataFromNode<std::string>
{
  static bool convert(mpack_node_t node, std::string & out)
  {
    if(mpack_node_type(node) != mpack_type_str) { return false; }
    out = std::string(mpack_node_str(node), mpack_node_strlen(node));
    return true;
  }
};

inline double node_at(mpack_node_t node, size_t idx)
{
  assert(mpack_node_type(node) == mpack_type_array && mpack_node_array_length(node) > idx);
  return mpack_node_double(mpack_node_array_at(node, idx));
}

template<>
struct DataFromNode<Eigen::Vector2d>
{
  static bool convert(mpack_node_t node, Eigen::Vector2d & v)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 2) { return false; }
    v.x() = node_at(node, 0);
    v.y() = node_at(node, 1);
    return true;
  }
};

template<>
struct DataFromNode<Eigen::Vector3d>
{
  static bool convert(mpack_node_t node, Eigen::Vector3d & v)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 3) { return false; }
    v.x() = node_at(node, 0);
    v.y() = node_at(node, 1);
    v.z() = node_at(node, 2);
    return true;
  }
};

template<>
struct DataFromNode<Eigen::Vector6d>
{
  static bool convert(mpack_node_t node, Eigen::Vector6d & v)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 6) { return false; }
    for(Eigen::DenseIndex i = 0; i < v.size(); ++i) { v(i) = node_at(node, static_cast<size_t>(i)); }
    return true;
  }
};

template<>
struct DataFromNode<Eigen::VectorXd>
{
  static bool convert(mpack_node_t node, Eigen::VectorXd & v)
  {
    if(mpack_node_type(node) != mpack_type_array) { return false; }
    v.resize(static_cast<int>(mpack_node_array_length(node)));
    for(Eigen::DenseIndex i = 0; i < v.size(); ++i) { v(i) = node_at(node, static_cast<size_t>(i)); }
    return true;
  }
};

template<>
struct DataFromNode<Eigen::Quaterniond>
{
  static bool convert(mpack_node_t node, Eigen::Quaterniond & q)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 4) { return false; }
    q.w() = node_at(node, 0);
    q.x() = node_at(node, 1);
    q.y() = node_at(node, 2);
    q.z() = node_at(node, 3);
    return true;
  }
};

template<>
struct DataFromNode<sva::PTransformd>
{
  static bool convert(mpack_node_t node, sva::PTransformd & out)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 12) { return false; }
    out.rotation() << node_at(node, 0), node_at(node, 1), node_at(node, 2), node_at(node, 3), node_at(node, 4),
        node_at(node, 5), node_at(node, 6), node_at(node, 7), node_at(node, 8);
    out.translation() << node_at(node, 9), node_at(node, 10), node_at(node, 11);
    return true;
  }
};

template<>
struct DataFromNode<sva::ForceVecd>
{
  static bool convert(mpack_node_t node, sva::ForceVecd & out)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 6) { return false; }
    out.couple() << node_at(node, 0), node_at(node, 1), node_at(node, 2);
    out.force() << node_at(node, 3), node_at(node, 4), node_at(node, 5);
    return true;
  }
};

template<>
struct DataFromNode<sva::MotionVecd>
{
  static bool convert(mpack_node_t node, sva::MotionVecd & out)
  {
    if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) != 6) { return false; }
    out.angular() << node_at(node, 0), node_at(node, 1), node_at(node, 2);
    out.linear() << node_at(node, 3), node_at(node, 4), node_at(node, 5);
    return true;
  }
};

template<typename A>
struct DataFromNode<std::vector<double, A>>
{
  static bool convert(mpack_node_t node, std::vector<double, A> & out)
  {
    if(mpack_node_type(node) != mpack_type_array) { return false; }
    out.resize(mpack_node_array_length(node));
    for(size_t i = 0; i < out.size(); ++i) { out[i] = node_at(node, i); }
    return true;
  }
};

template<typename T>
struct PointerFromNode
{
  static FlatLog::record::unique_void_ptr convert(mpack_node_t node)
  {
    FlatLog::record::unique_void_ptr ret{new T{}, void_deleter<T>};
    if(DataFromNode<T>::convert(node, *static_cast<T *>(ret.get()))) { return ret; }
    return {nullptr, void_deleter<int>};
  }
};

inline FlatLog::record::unique_void_ptr dataFromNode(const LogType & type, mpack_node_t node)
{
  switch(type)
  {
    case LogType::Bool:
      return PointerFromNode<bool>::convert(node);
    case LogType::Int8_t:
      return PointerFromNode<int8_t>::convert(node);
    case LogType::Int16_t:
      return PointerFromNode<int16_t>::convert(node);
    case LogType::Int32_t:
      return PointerFromNode<int32_t>::convert(node);
    case LogType::Int64_t:
      return PointerFromNode<int64_t>::convert(node);
    case LogType::Uint8_t:
      return PointerFromNode<uint8_t>::convert(node);
    case LogType::Uint16_t:
      return PointerFromNode<uint16_t>::convert(node);
    case LogType::Uint32_t:
      return PointerFromNode<uint32_t>::convert(node);
    case LogType::Uint64_t:
      return PointerFromNode<uint64_t>::convert(node);
    case LogType::Float:
      return PointerFromNode<float>::convert(node);
    case LogType::Double:
      return PointerFromNode<double>::convert(node);
    case LogType::String:
      return PointerFromNode<std::string>::convert(node);
    case LogType::Vector2d:
      return PointerFromNode<Eigen::Vector2d>::convert(node);
    case LogType::Vector3d:
      return PointerFromNode<Eigen::Vector3d>::convert(node);
    case LogType::Vector6d:
      return PointerFromNode<Eigen::Vector6d>::convert(node);
    case LogType::VectorXd:
      return PointerFromNode<Eigen::VectorXd>::convert(node);
    case LogType::Quaterniond:
      return PointerFromNode<Eigen::Quaterniond>::convert(node);
    case LogType::PTransformd:
      return PointerFromNode<sva::PTransformd>::convert(node);
    case LogType::ForceVecd:
      return PointerFromNode<sva::ForceVecd>::convert(node);
    case LogType::MotionVecd:
      return PointerFromNode<sva::MotionVecd>::convert(node);
    case LogType::VectorDouble:
      return PointerFromNode<std::vector<double>>::convert(node);
    case LogType::None:
    default:
      return {nullptr, void_deleter<int>};
  }
}

// For version 0, type and data are stored in the node every iteration
inline FlatLog::record recordFromNode(mpack_node_t node, bool extract_data, size_t idx)
{
  if(mpack_node_type(node) != mpack_type_array || mpack_node_array_length(node) == 1)
  {
    log::error("Failed to read record from MessagePack node");
    return {};
  }
  auto type = logTypeFromNode(node, idx);
  if(extract_data)
  {
    auto data = mpack_node_array_at(node, idx + 1);
    return {type, dataFromNode(type, data)};
  }
  else { return {type, {nullptr, void_deleter<int>}}; }
}

// For version 1 and up, only data is stored in the node, type is from events
inline FlatLog::record recordFromNode(LogType type, mpack_node_t node, bool extract_data, size_t idx)
{
  if(extract_data)
  {
    auto data = mpack_node_array_at(node, idx);
    return {type, dataFromNode(type, data)};
  }
  else { return {type, {nullptr, void_deleter<int>}}; }
}

struct TypedKey
{
  LogType type;
  std::string key;
};

struct LogEntry : mpack_tree_t
{
  LogEntry(int8_t version,
           const std::vector<char> & data,
           size_t size,
           std::optional<Logger::Meta> & metaOut,
           std::vector<TypedKey> & keysOut,
           std::vector<Logger::GUIEvent> & eventsOut,
           bool & keysChanged,
           bool extract_data = true)
  : version_(version)
  {
    mpack_tree_init_data(this, data.data(), size);
    mpack_tree_parse(this);
    if(mpack_tree_error(this) != mpack_ok)
    {
      log::error("Failed to parse MessagePack data store into the log");
      valid_ = false;
      return;
    }
    root_ = mpack_tree_root(this);
    if(mpack_node_type(root_) != mpack_type_array || mpack_node_array_length(root_) != 2)
    {
      log::error("MessagePack stored data does not appear to be an array of size 2");
      if(mpack_node_type(root_) != mpack_type_array) { log::warning("Not an array"); }
      else { log::warning("Array of size: {}", mpack_node_array_length(root_)); }
      valid_ = false;
      return;
    }
    if(version_ == 0)
    {
      auto keys = mpack_node_array_at(root_, 0);
      std::vector<std::string> keys_;
      if(mpack_node_type(keys) == mpack_type_nil) {}
      else if(mpack_node_type(keys) == mpack_type_array)
      {
        size_t s = mpack_node_array_length(keys);
        keysOut.clear();
        keys_.reserve(s);
        for(size_t i = 0; i < s; ++i)
        {
          auto k = stringFromNode(mpack_node_array_at(keys, i));
          if(!k)
          {
            log::error("A key was not a string in log entries");
            valid_ = false;
            return;
          }
          keys_.push_back(std::string(*k));
        }
        keysChanged = true;
      }
      else
      {
        log::error("MessagePack stored data has keys that are neither an array nor nil");
        valid_ = false;
        return;
      }
      auto records = mpack_node_array_at(root_, 1);
      if(mpack_node_type(records) != mpack_type_array)
      {
        log::error("MessagePack stored records are not in an array");
        valid_ = false;
        return;
      }
      size_t s = mpack_node_array_length(records);
      for(size_t i = 0; i < s / 2; ++i)
      {
        records_.push_back(recordFromNode(records, extract_data, 2 * i));
        if(keys_.size()) { keysOut.push_back({records_.back().type, keys_[i]}); }
      }
    }
    else if(version_ == 1)
    {
      auto events = mpack_node_array_at(root_, 0);
      if(mpack_node_type(events) == mpack_type_nil)
      {
        // No event this time
      }
      else if(mpack_node_type(events) == mpack_type_array)
      {
        keysChanged = true;
        size_t s = mpack_node_array_length(events);
        for(size_t i = 0; i < s; ++i)
        {
          auto event = mpack_node_array_at(events, i);
          if(mpack_node_type(event) != mpack_type_array)
          {
            log::error("An event was not an array in the log");
            valid_ = false;
            return;
          }
          auto event_size = mpack_node_array_length(event);
          if(event_size < 1)
          {
            log::error("Not enough data in event");
            valid_ = false;
            return;
          }
          auto event_t_node = mpack_node_array_at(event, 0);
          if(mpack_node_type(event_t_node) != mpack_type_int && mpack_node_type(event_t_node) != mpack_type_uint)
          {
            log::error("Event type is not an integer");
            valid_ = false;
            return;
          }
          uint8_t event_t = mpack_node_u8(event_t_node);
          if(event_t == 0)
          {
            // Add key event
            if(event_size != 3)
            {
              log::error("Add key event should have three entries");
              valid_ = false;
              return;
            }
            auto type = logTypeFromNode(mpack_node_array_at(event, 1));
            if(type == LogType::None)
            {
              valid_ = false;
              return;
            }
            auto key = stringFromNode(mpack_node_array_at(event, 2));
            if(!key)
            {
              log::error("Add key event's key entry is not a string");
              valid_ = false;
              return;
            }
            keysOut.push_back({type, std::string(*key)});
          }
          else if(event_t == 1)
          {
            // Remove key event
            if(event_size != 2)
            {
              log::error("Remove key event should have two entries");
              valid_ = false;
              return;
            }
            auto key = stringFromNode(mpack_node_array_at(event, 1));
            if(!key)
            {
              log::error("Remove key event's key entry is not a string");
              valid_ = false;
              return;
            }
            for(auto it = keysOut.begin(); it != keysOut.end(); ++it)
            {
              if(it->key == *key)
              {
                keysOut.erase(it);
                break;
              }
            }
          }
          else if(event_t == 2)
          {
            // GUI event event
            if(event_size != 4)
            {
              log::error("GUI event should have four entries");
              valid_ = false;
              return;
            }
            auto category = stringVectorFromNode(mpack_node_array_at(event, 1));
            auto name = stringFromNode(mpack_node_array_at(event, 2));
            mc_rtc::Configuration data = ::mc_rtc::internal::fromMessagePack(mpack_node_array_at(event, 3));
            if(!category || !name)
            {
              log::error("GUI event is illformed");
              valid_ = false;
              return;
            }
            eventsOut.push_back({category.value(), std::string(name.value()), data});
          }
          else if(event_t == 3)
          {
            // StartEvent event
            if(event_size < 5)
            {
              log::error("Start event should have at least five entries");
              valid_ = false;
              return;
            }
            mc_rtc::Configuration data = ::mc_rtc::internal::fromMessagePack(event);
            Logger::Meta meta;
            meta.timestep = data[1];
            meta.main_robot = data[2].operator std::string();
            meta.main_robot_module = data[3];
            meta.init = data[4];
            if(data.size() > 5) { meta.init_q = data[5]; }
            if(data.size() > 6) { meta.calibs = data[6]; }
            metaOut = meta;
          }
          else
          {
            log::error("Unknown event type ({})", event_t);
            valid_ = false;
            return;
          }
        }
      }
      // At this point keysOut is up-to-date
      // Data is stored in the corresponding order
      auto records = mpack_node_array_at(root_, 1);
      if(mpack_node_type(records) != mpack_type_array)
      {
        log::error("MessagePack stored records are not in an array");
        valid_ = false;
        return;
      }
      size_t s = mpack_node_array_length(records);
      for(size_t i = 0; i < s; ++i) { records_.push_back(recordFromNode(keysOut[i].type, records, extract_data, i)); }
    }
    else
    {
      log::error("interal::LogEntry cannot handle version: {}", version_);
      valid_ = false;
      return;
    }
  }

  ~LogEntry() { mpack_tree_destroy(this); }

  LogEntry(const LogEntry &) = delete;
  LogEntry & operator=(const LogEntry &) = delete;
  LogEntry(LogEntry &&) = default;
  LogEntry & operator=(LogEntry &&) = default;

  bool valid() const { return valid_; }

  std::vector<FlatLog::record> & records() { return records_; }

  /** Should only be used to retrieve time values from the log */
  double getTime(size_t idx)
  {
    assert(valid_);
    auto values = mpack_node_array_at(root_, 1);
    if(version_ == 0) { return mpack_node_double(mpack_node_array_at(values, 2 * idx + 1)); }
    else { return mpack_node_double(mpack_node_array_at(values, idx)); }
  }

  /** Rebuild this log entry with new keys */
  void copy(mc_rtc::MessagePackBuilder & builder, const std::vector<std::string> & keys)
  {
    builder.start_array(2);
    if(keys.size() != records_.size())
    {
      mc_rtc::log::error_and_throw("Expected to copy {} but has {} records", keys.size(), records_.size());
    }
    builder.start_array(keys.size());
    for(size_t i = 0; i < keys.size(); ++i)
    {
      const auto & k = keys[i];
      const auto & r = records_[i];
      builder.start_array(3);
      builder.write(static_cast<uint8_t>(0));
      builder.write(static_cast<typename std::underlying_type<log::LogType>::type>(r.type));
      builder.write(k);
      builder.finish_array();
    }
    builder.finish_array();
    copy(builder, mpack_node_array_at(root_, 1));
    builder.finish_array();
  }

private:
  int8_t version_ = 0;
  bool valid_ = true;
  mpack_node_t root_;
  std::vector<FlatLog::record> records_;

  void copy_data(mc_rtc::MessagePackBuilder & builder, mpack_node_t data)
  {
    switch(mpack_node_type(data))
    {
      case mpack_type_bool:
        builder.write(mpack_node_bool(data));
        break;
      case mpack_type_int:
        builder.write(mpack_node_i64(data));
        break;
      case mpack_type_uint:
        builder.write(mpack_node_u64(data));
        break;
      case mpack_type_float:
        builder.write(mpack_node_float(data));
        break;
      case mpack_type_double:
        builder.write(mpack_node_double(data));
        break;
      case mpack_type_str:
        builder.write(mpack_node_str(data), mpack_node_strlen(data));
        break;
      case mpack_type_array:
        builder.start_array(mpack_node_array_length(data));
        for(size_t i = 0; i < mpack_node_array_length(data); ++i) { copy_data(builder, mpack_node_array_at(data, i)); }
        builder.finish_array();
        break;
      case mpack_type_map:
      case mpack_type_nil:
      case mpack_type_missing:
      default:
        log::error("This data should not appear in a log");
        break;
    };
  }

  void copy(mc_rtc::MessagePackBuilder & builder, mpack_node_t value)
  {
    size_t s = mpack_node_array_length(value);
    builder.start_array(s);
    for(size_t i = 0; i < s; ++i) { copy_data(builder, mpack_node_array_at(value, i)); }
    builder.finish_array();
  }
};

} // namespace mc_rtc::log::internal
