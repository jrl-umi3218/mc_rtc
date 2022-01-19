/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <boost/algorithm/string/case_conv.hpp>
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include "internals/json.h"
#include <fstream>
#include <stdexcept>

namespace
{
inline std::string to_lower(const std::string & in)
{
  return boost::algorithm::to_lower_copy(in);
}
} // namespace

namespace mc_rtc
{

bool Configuration::Json::isArray() const
{
  assert(value_);
  return static_cast<const internal::RapidJSONValue *>(value_)->IsArray();
}

bool Configuration::Json::isObject() const
{
  assert(value_);
  return static_cast<const internal::RapidJSONValue *>(value_)->IsObject();
}

std::vector<std::string> Configuration::Json::keys() const
{
  assert(isObject());
  std::vector<std::string> ret;
  auto v = static_cast<const internal::RapidJSONValue *>(value_);
  for(auto it = v->MemberBegin(); it != v->MemberEnd(); ++it)
  {
    ret.push_back(it->name.GetString());
  }
  return ret;
}

namespace
{

bool findPath(const internal::RapidJSONValue * root, const internal::RapidJSONValue * value, std::string & out)
{
  if(root == value)
  {
    return true;
  }
  if(root->IsArray())
  {
    for(size_t idx = 0; idx != root->Size(); ++idx)
    {
      if(findPath(&(*root)[idx], value, out))
      {
        out = fmt::format("[{}]{}", idx, out);
        return true;
      }
    }
  }
  else if(root->IsObject())
  {
    for(auto it = root->MemberBegin(); it != root->MemberEnd(); ++it)
    {
      if(findPath(&it->value, value, out))
      {
        out = fmt::format("(\"{}\"){}", it->name.GetString(), out);
        return true;
      }
    }
  }
  return false;
}

} // namespace

void Configuration::Json::path(std::string & out) const
{
  assert(doc_.get());
  assert(value_);
  auto root = static_cast<const internal::RapidJSONValue *>(doc_.get());
  auto value = static_cast<const internal::RapidJSONValue *>(value_);
  findPath(root, value, out);
}

bool Configuration::empty() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(v.isArray())
  {
    return value->Empty();
  }
  else if(v.isObject())
  {
    return value->ObjectEmpty();
  }
  return value->IsNull();
}

size_t Configuration::Json::size() const
{
  assert(value_);
  auto value = static_cast<const internal::RapidJSONValue *>(value_);
  return value->Size();
}

Configuration::Json Configuration::Json::operator[](size_t idx) const
{
  assert(value_);
  auto value = static_cast<internal::RapidJSONValue *>(value_);
  return {static_cast<void *>(&(*value)[idx]), doc_};
}

Configuration::Json Configuration::Json::operator[](const std::string & key) const
{
  assert(value_);
  auto value = static_cast<internal::RapidJSONValue *>(value_);
  return {static_cast<void *>(&(*value)[key]), doc_};
}

bool Configuration::Json::isNumeric() const
{
  assert(value_);
  auto value = static_cast<internal::RapidJSONValue *>(value_);
  return value->IsNumber();
}

double Configuration::Json::asDouble() const
{
  assert(isNumeric());
  auto value = static_cast<internal::RapidJSONValue *>(value_);
  return value->GetDouble();
}

Configuration::Exception::Exception(const std::string & msg, const Json & v) : msg_(msg), v_(v) {}

Configuration::Exception::~Exception() noexcept
{
  if(msg().size())
  {
    log::error(msg());
  }
}

const char * Configuration::Exception::what() const noexcept
{
  return msg().c_str();
}

void Configuration::Exception::silence() const noexcept
{
  msg_.resize(0);
  v_.value_ = nullptr;
}

const std::string & Configuration::Exception::msg() const noexcept
{
  if(v_.value_)
  {
    std::string path;
    v_.path(path);
    msg_ = fmt::format("{} (error path: {})", msg_, path.size() ? path : "()");
    v_.value_ = nullptr;
  }
  return msg_;
}

Configuration::Configuration() : v{nullptr, std::shared_ptr<void>(new internal::RapidJSONDocument())}
{
  auto doc = std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);
  auto value = rapidjson::GenericPointer<internal::RapidJSONValue>("/").Get(*doc);
  if(!value)
  {
    value = doc.get();
  }
  v.value_ = value;
  doc->SetObject();
}

Configuration::Configuration(const Json & v) : v(v) {}

Configuration::Configuration(const char * path) : Configuration(std::string(path)) {}

bool Configuration::isMember(const std::string & key) const
{
  return has(key);
}

bool Configuration::has(const std::string & key) const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  return v.isObject() && value->HasMember(key);
}

Configuration Configuration::operator()(const std::string & key) const
{
  if(has(key))
  {
    return Configuration(v[key]);
  }
  throw Exception("No entry named " + key + " in the configuration", v);
}

size_t Configuration::size() const
{
  if(v.isArray())
  {
    return v.size();
  }
  return 0;
}

Configuration Configuration::operator[](size_t i) const
{
  if(i >= size())
  {
    throw Exception("Out-of-bound access for a Configuration element", v);
  }
  return Configuration(v[i]);
}

Configuration::operator bool() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsBool())
  {
    return value->GetBool();
  }
  else if(value->IsUint())
  {
    return value->GetUint();
  }
  else if(value->IsInt())
  {
    return value->GetInt();
  }
  throw Exception("Stored Json value is not a bool", v);
}

Configuration::operator int() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsInt())
  {
    return value->GetInt();
  }
  throw Exception("Stored Json value is not an int", v);
}

Configuration::operator unsigned int() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsUint() || (value->IsInt() && value->GetInt() >= 0))
  {
    return value->GetUint();
  }
  throw Exception("Stored Json value is not an unsigned int", v);
}

Configuration::operator int64_t() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsInt64())
  {
    return value->GetInt64();
  }
  throw Exception("Stored Json value is not an int64_t", v);
}

Configuration::operator uint64_t() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsUint64() || (value->IsInt64() && value->GetInt64() >= 0))
  {
    return value->GetUint64();
  }
  throw Exception("Stored Json value is not an uint64_t", v);
}

Configuration::operator double() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsNumber())
  {
    return value->GetDouble();
  }
  throw Exception("Stored Json value is not a double", v);
}

Configuration::operator std::string() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsString())
  {
    return std::string(value->GetString(), value->GetStringLength());
  }
  throw Exception("Stored Json value is not a string", v);
}

Configuration::operator Eigen::Vector2d() const
{
  if(v.isArray() && v.size() == 2 && v[0].isNumeric())
  {
    Eigen::Vector2d ret;
    ret << v[0].asDouble(), v[1].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector2d", v);
}

Configuration::operator Eigen::Vector3d() const
{
  if(v.isArray() && v.size() == 3 && v[0].isNumeric())
  {
    Eigen::Vector3d ret;
    ret << v[0].asDouble(), v[1].asDouble(), v[2].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector3d", v);
}

Configuration::operator Eigen::Vector6d() const
{
  if(v.isArray() && v.size() == 6 && v[0].isNumeric())
  {
    Eigen::Vector6d ret;
    ret << v[0].asDouble(), v[1].asDouble(), v[2].asDouble(), v[3].asDouble(), v[4].asDouble(), v[5].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector6d", v);
}

Configuration::operator Eigen::VectorXd() const
{
  if(v.isArray() && (v.size() == 0 || v[0].isNumeric()))
  {
    Eigen::VectorXd ret(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      ret(static_cast<int>(i)) = v[i].asDouble();
    }
    return ret;
  }
  throw Exception("Stored Json value is not a VectorXd", v);
}

Configuration::operator Eigen::Quaterniond() const
{
  if(v.isArray() && v[0].isNumeric())
  {
    if(v.size() == 4)
    {
      return Eigen::Quaterniond(v[0].asDouble(), v[1].asDouble(), v[2].asDouble(), v[3].asDouble()).normalized();
    }
    else if(v.size() == 9)
    {
      Eigen::Matrix3d m;
      for(size_t i = 0; i < 3; ++i)
      {
        for(size_t j = 0; j < 3; ++j)
        {
          m(static_cast<int>(i), static_cast<int>(j)) = v[3 * i + j].asDouble();
        }
      }
      return Eigen::Quaterniond(m).normalized();
    }
    else if(v.size() == 3)
    {
      return Eigen::Quaterniond(mc_rbdyn::rpyToMat(v[0].asDouble(), v[1].asDouble(), v[2].asDouble())).normalized();
    }
  }
  throw Exception("Stored Json value is not a Quaterniond", v);
}

Configuration::operator Eigen::Matrix3d() const
{
  if(v.isArray() && v[0].isNumeric())
  {
    // Matrix representaton
    if(v.size() == 9)
    {
      Eigen::Matrix3d m;
      for(size_t i = 0; i < 3; ++i)
      {
        for(size_t j = 0; j < 3; ++j)
        {
          m(static_cast<int>(i), static_cast<int>(j)) = v[3 * i + j].asDouble();
        }
      }
      return m;
    }
    // RPY representation
    if(v.size() == 3)
    {
      return mc_rbdyn::rpyToMat(v[0].asDouble(), v[1].asDouble(), v[2].asDouble());
    }
    // Quaternion representation
    if(v.size() == 4)
    {
      return Eigen::Quaterniond(v[0].asDouble(), v[1].asDouble(), v[2].asDouble(), v[3].asDouble())
          .normalized()
          .toRotationMatrix();
    }
  }
  throw Exception("Stored Json value is not a Matrix3d", v);
}

Configuration::operator Eigen::Matrix6d() const
{
  if(v.isArray() && v.size() == 36 && v[0].isNumeric())
  {
    Eigen::Matrix6d m;
    for(size_t i = 0; i < 6; ++i)
    {
      for(size_t j = 0; j < 6; ++j)
      {
        m(static_cast<int>(i), static_cast<int>(j)) = v[6 * i + j].asDouble();
      }
    }
    return m;
  }
  throw Exception("Stored Json value is not a Matrix6d", v);
}

Configuration::operator Eigen::MatrixXd() const
{
  if(v.isArray() && v[0].isArray() && v[0][0].isNumeric())
  {
    Eigen::MatrixXd ret(v.size(), v[0].size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      auto row = v[i];
      assert(row.size() == ret.cols());
      for(size_t j = 0; j < row.size(); ++j)
      {
        ret(static_cast<int>(i), static_cast<int>(j)) = row[j].asDouble();
      }
    }
    return ret;
  }
  throw Exception("Stored Json value is not a MatrixXd", v);
}

Configuration::operator sva::PTransformd() const
{
  if(has("rotation"))
  {
    Eigen::Matrix3d r = (*this)("rotation");
    if(has("translation"))
    {
      Eigen::Vector3d t = (*this)("translation");
      return {r, t};
    }
    return {r};
  }
  else if(has("translation"))
  {
    Eigen::Vector3d t = (*this)("translation");
    return {t};
  }
  else if(size() == 7)
  {
    auto & config = *this;
    return {Eigen::Quaterniond{config[0], config[1], config[2], config[3]}.normalized(),
            {config[4], config[5], config[6]}};
  }
  else if(size() == 12)
  {
    auto & config = *this;
    Eigen::Matrix3d rot;
    rot << config[0], config[1], config[2], config[3], config[4], config[5], config[6], config[7], config[8];
    return {rot, {config[9], config[10], config[11]}};
  }
  throw Exception("Stored Json value is not a PTransformd", v);
}

Configuration::operator sva::ForceVecd() const
{
  if(has("couple") && has("force"))
  {
    return {(*this)("couple"), (*this)("force")};
  }
  else if(size() == 6)
  {
    auto & config = *this;
    return {{config[0], config[1], config[2]}, {config[3], config[4], config[5]}};
  }
  throw Exception("Stored Json value is not a ForceVecd", v);
}

Configuration::operator sva::MotionVecd() const
{
  if(has("angular") && has("linear"))
  {
    return {(*this)("angular"), (*this)("linear")};
  }
  else if(size() == 6)
  {
    auto & config = *this;
    return {{config[0], config[1], config[2]}, {config[3], config[4], config[5]}};
  }
  throw Exception("Stored Json value is not a MotionVecd", v);
}

Configuration::operator sva::ImpedanceVecd() const
{
  if(has("angular") && has("linear"))
  {
    Eigen::Vector3d angular = (*this)("angular");
    return {angular, (*this)("linear")};
  }
  else if(size() == 6)
  {
    auto & config = *this;
    return {{config[0], config[1], config[2]}, {config[3], config[4], config[5]}};
  }
  throw Exception("Stored Json value is not an ImpedanceVecd", v);
}

Configuration::Configuration(const std::string & path) : Configuration()
{
  auto doc = std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);
  doc->SetNull();
  load(path);
}

Configuration Configuration::fromData(const std::string & data)
{
  return Configuration::fromData(data.c_str());
}

Configuration Configuration::fromData(const char * data)
{
  mc_rtc::Configuration config;
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(config.v.doc_);
  mc_rtc::internal::loadData(data, target);
  return config;
}

Configuration Configuration::fromYAMLData(const std::string & data)
{
  return Configuration::fromYAMLData(data.c_str());
}

Configuration Configuration::fromYAMLData(const char * data)
{
  mc_rtc::Configuration config;
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(config.v.doc_);
  target.SetObject();
  mc_rtc::internal::loadYAMLData(data, config);
  return config;
}

Configuration Configuration::rootArray()
{
  mc_rtc::Configuration config;
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(config.v.doc_);
  target.SetArray();
  return config;
}

Configuration Configuration::fromMessagePack(const char * data, size_t size)
{
  mc_rtc::Configuration config;
  mc_rtc::internal::fromMessagePack(config, data, size);
  return config;
}

void Configuration::load(const std::string & path)
{
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);

  if(target.IsNull())
  {
    std::string extension = to_lower(bfs::path(path).extension().string());
    if(extension == ".yml" || extension == ".yaml")
    {
      target.SetObject();
      if(!mc_rtc::internal::loadYAMLDocument(path, *this))
      {
        log::warning("Configuration dump until the attempted conversion:\n{}", this->dump(true));
      }
    }
    else
    {
      mc_rtc::internal::loadDocument(path, target);
    }
  }
  else
  {
    load(mc_rtc::Configuration{path});
  }
}

void Configuration::load(const mc_rtc::Configuration & config)
{
  auto & doc = *std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);
  auto & target = *static_cast<internal::RapidJSONValue *>(v.value_);

  if(target.IsNull())
  {
    auto & docFrom = *static_cast<internal::RapidJSONDocument *>(config.v.value_);
    target.CopyFrom(docFrom, doc.GetAllocator());
  }
  else
  {
    const auto & v = *static_cast<internal::RapidJSONValue *>(config.v.value_);
    if(v.IsObject())
    {
      for(auto & m : v.GetObject())
      {
        if(target.HasMember(m.name))
        {
          if(m.value.IsObject() && target.FindMember(m.name)->value.IsObject())
          {
            (*this)(m.name.GetString()).load(config(m.name.GetString()));
            continue;
          }
          else
          {
            target.RemoveMember(m.name);
          }
        }
        internal::RapidJSONValue n(m.name, doc.GetAllocator());
        internal::RapidJSONValue v(m.value, doc.GetAllocator());
        target.AddMember(n, v, doc.GetAllocator());
      }
    }
    else if(v.IsArray())
    {
      if(!target.IsArray())
      {
        target.SetArray();
      }
      for(auto & value : v.GetArray())
      {
        target.PushBack(internal::RapidJSONValue{value, doc.GetAllocator()}.Move(), doc.GetAllocator());
      }
    }
    else
    {
      auto & docFrom = *static_cast<internal::RapidJSONDocument *>(config.v.value_);
      target.SetNull();
      target.CopyFrom(docFrom, doc.GetAllocator());
    }
  }
}

void Configuration::loadData(const std::string & data)
{
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);
  if(target.IsNull())
  {
    mc_rtc::internal::loadData(data.c_str(), target);
  }
  else
  {
    load(Configuration::fromData(data));
  }
}

void Configuration::loadYAMLData(const std::string & data)
{
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);
  if(target.IsNull())
  {
    target.SetObject();
    mc_rtc::internal::loadYAMLData(data.c_str(), *this);
  }
  else
  {
    load(Configuration::fromYAMLData(data));
  }
}

void Configuration::save(const std::string & path, bool pretty) const
{
  std::string extension = to_lower(bfs::path(path).extension().string());
  if(extension == ".yml" || extension == ".yaml")
  {
    mc_rtc::internal::saveYAML(path, *this);
  }
  else
  {
    auto & value = *static_cast<internal::RapidJSONValue *>(v.value_);
    mc_rtc::internal::saveDocument(path, value, pretty);
  }
}

std::string Configuration::dump(bool pretty, bool yaml) const
{
  if(yaml)
  {
    return mc_rtc::internal::dumpYAML(*this);
  }
  else
  {
    auto & value = *static_cast<internal::RapidJSONValue *>(v.value_);
    return mc_rtc::internal::dumpDocument(value, pretty);
  }
}

size_t Configuration::toMessagePack(std::vector<char> & data) const
{
  MessagePackBuilder builder(data);
  toMessagePack(builder);
  return builder.finish();
}

void Configuration::toMessagePack(MessagePackBuilder & builder) const
{
  auto & value = *static_cast<internal::RapidJSONValue *>(v.value_);
  mc_rtc::internal::toMessagePack(value, builder);
}

template<>
void Configuration::operator()(const std::string & key, std::string & v) const
{
  try
  {
    v = (std::string)(*this)(key);
  }
  catch(Exception & exc)
  {
    exc.silence();
  }
}

bool Configuration::operator==(const char * rhs) const
{
  return static_cast<std::string>(*this) == rhs;
}

namespace
{
template<typename T>
void add_impl(const mc_rtc::Configuration & source, const std::string & key, T value, void * json_p, void * doc_p)
{
  auto & json = *static_cast<internal::RapidJSONValue *>(json_p);
  if(!json.IsObject())
  {
    throw Configuration::Exception("You cannot add entry into a non-object", source);
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(doc_p);
  auto & allocator = doc.GetAllocator();
  internal::RapidJSONValue value_ = mc_rtc::internal::toJSON(value, allocator);
  auto member = json.FindMember(key.c_str());
  if(member == json.MemberEnd())
  {
    internal::RapidJSONValue key_(key.c_str(), allocator);
    json.AddMember(key_, value_, allocator);
  }
  else
  {
    member->value = value_;
  }
}

template<typename T>
void push_impl(const Configuration & source, T value, void * json_p, void * doc_p)
{
  auto & json = *static_cast<internal::RapidJSONValue *>(json_p);
  auto & doc = *static_cast<internal::RapidJSONDocument *>(doc_p);
  auto & allocator = doc.GetAllocator();
  internal::RapidJSONValue value_ = mc_rtc::internal::toJSON(value, allocator);
  if(!json.IsArray())
  {
    throw Configuration::Exception("Trying to push data in a non-array value", source);
  }
  json.PushBack(value_, allocator);
}
} // namespace

void Configuration::add_null(const std::string & key)
{
  if(!v.isObject())
  {
    throw Exception("You cannot add entry into a non-object", v);
  }
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  internal::RapidJSONValue value_;
  value_.SetNull();
  auto member = json.FindMember(key.c_str());
  if(member == json.MemberEnd())
  {
    internal::RapidJSONValue key_(key.c_str(), allocator);
    json.AddMember(key_, value_, allocator);
  }
  else
  {
    member->value = value_;
  }
}

void Configuration::push_null()
{
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  internal::RapidJSONValue value_;
  value_.SetNull();
  if(!json.IsArray())
  {
    throw Configuration::Exception("Trying to push data in a non-array value", v);
  }
  json.PushBack(value_, allocator);
}

// clang-format off
void Configuration::add(const std::string & key, bool value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, int value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, unsigned int value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, int64_t value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, uint64_t value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, double value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const char * value) { add(key, std::string(value)); }
void Configuration::add(const std::string & key, const std::string & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector2d & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector3d & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector6d & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::VectorXd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Quaterniond & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Matrix3d & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Matrix6d & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::MatrixXd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::PTransformd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::ForceVecd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::MotionVecd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::ImpedanceVecd & value) { add_impl(*this, key, value, v.value_, v.doc_.get()); }
// clang-format on

void Configuration::add(const std::string & key, const Configuration & value)
{
  if(!v.isObject())
  {
    throw Exception("You cannot add entry into a non-object", v);
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue key_(key.c_str(), allocator);
  internal::RapidJSONValue value_(*static_cast<internal::RapidJSONValue *>(value.v.value_), allocator);
  if(has(key))
  {
    json.RemoveMember(key.c_str());
  }
  json.AddMember(key_, value_, allocator);
}

Configuration Configuration::add(const std::string & key)
{
  if(!v.isObject())
  {
    throw Exception("You cannot add entry into a non-object", v);
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue key_(key.c_str(), allocator);
  internal::RapidJSONValue value(rapidjson::kObjectType);
  if(has(key))
  {
    json.RemoveMember(key.c_str());
  }
  json.AddMember(key_, value, allocator);
  return (*this)(key);
}

Configuration Configuration::array(const std::string & key, size_t size)
{
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue key_(key.c_str(), allocator);
  internal::RapidJSONValue value(rapidjson::kArrayType);
  if(size)
  {
    value.Reserve(size, allocator);
  }
  if(has(key))
  {
    json.RemoveMember(key.c_str());
  }
  json.AddMember(key_, value, allocator);
  return (*this)(key);
}

Configuration Configuration::array(size_t reserve)
{
  if(!v.isArray())
  {
    throw Exception("Cannot store an anonymous array outside of an array", v);
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue value(rapidjson::kArrayType);
  value.Reserve(reserve, allocator);
  json.PushBack(value, allocator);
  return (*this)[size() - 1];
}

Configuration Configuration::object()
{
  if(!v.isArray())
  {
    throw Exception("Cannot store an anonymous object outside of an array", v);
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue value(rapidjson::kObjectType);
  json.PushBack(value, allocator);
  return (*this)[size() - 1];
}

// clang-format off
void Configuration::push(bool value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(int value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(unsigned int value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(int64_t value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(uint64_t value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(double value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const char * value) { push(std::string(value)); }
void Configuration::push(const std::string & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector2d & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector3d & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector6d & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::VectorXd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Quaterniond & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Matrix3d & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Matrix6d & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::MatrixXd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::PTransformd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::ForceVecd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::MotionVecd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::ImpedanceVecd & value) { push_impl(*this, value, v.value_, v.doc_.get()); }
// clang-format on

void Configuration::push(const mc_rtc::Configuration & value)
{
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  if(!json.IsArray())
  {
    throw Exception("Trying to push data in a non-array value", v);
  }
  internal::RapidJSONValue value_(*static_cast<internal::RapidJSONValue *>(value.v.value_), allocator);
  json.PushBack(value_, allocator);
}

bool Configuration::remove(const std::string & key)
{
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  if(json.HasMember(key.c_str()))
  {
    json.RemoveMember(key.c_str());
    return true;
  }
  return false;
}

std::vector<std::string> Configuration::keys() const
{
  if(v.isObject())
  {
    return v.keys();
  }
  return {};
}

ConfigurationArrayIterator Configuration::begin() const
{
  return ConfigurationArrayIterator(*this);
}

ConfigurationArrayIterator Configuration::end() const
{
  auto ret = begin();
  ret.i = size();
  return ret;
}

ConfigurationArrayIterator::ConfigurationArrayIterator(const Configuration & conf) : i(0), conf(conf) {}

bool ConfigurationArrayIterator::operator!=(const ConfigurationArrayIterator & rhs) const
{
  return i != rhs.i;
}

ConfigurationArrayIterator & ConfigurationArrayIterator::operator++()
{
  ++i;
  return *this;
}

Configuration ConfigurationArrayIterator::operator*()
{
  return conf[i];
}

const Configuration ConfigurationArrayIterator::operator*() const
{
  return conf[i];
}

ConfigurationFile::ConfigurationFile(const std::string & path) : Configuration(path), path_(path) {}

void ConfigurationFile::reload()
{
  static_cast<mc_rtc::Configuration &>(*this) = Configuration(path_);
}

void ConfigurationFile::save() const
{
  save(path_);
}

} // namespace mc_rtc

std::ostream & operator<<(std::ostream & os, const mc_rtc::Configuration & c)
{
  os << static_cast<std::string>(c);
  return os;
}
