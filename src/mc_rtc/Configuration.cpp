#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include "internals/json.h"
#include <fstream>
#include <stdexcept>

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
  return true;
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

Configuration::Exception::Exception(const std::string & msg) : msg(msg) {}

Configuration::Exception::~Exception() noexcept
{
  if(msg.size())
  {
    LOG_ERROR(msg)
  }
}

const char * Configuration::Exception::what() const noexcept
{
  return msg.c_str();
}

void Configuration::Exception::silence() noexcept
{
  msg.resize(0);
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
  throw Exception("No entry named " + key + " in the configuration");
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
    throw Exception("Out-of-bound access for a Configuration element");
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
  throw Exception("Stored Json value is not a bool");
}

Configuration::operator int() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsInt())
  {
    return value->GetInt();
  }
  throw Exception("Stored Json value is not an int");
}

Configuration::operator unsigned int() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsUint() || (value->IsInt() && value->GetInt() >= 0))
  {
    return value->GetUint();
  }
  throw Exception("Stored Json value is not an unsigned int");
}

Configuration::operator double() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsNumber())
  {
    return value->GetDouble();
  }
  throw Exception("Stored Json value is not a double");
}

Configuration::operator std::string() const
{
  assert(v.value_);
  auto value = static_cast<const internal::RapidJSONValue *>(v.value_);
  if(value->IsString())
  {
    return std::string(value->GetString(), value->GetStringLength());
  }
  throw Exception("Stored Json value is not a string");
}

Configuration::operator Eigen::Vector2d() const
{
  if(v.isArray() && v.size() == 2 && v[0].isNumeric())
  {
    Eigen::Vector2d ret;
    ret << v[0].asDouble(), v[1].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector2d");
}

Configuration::operator Eigen::Vector3d() const
{
  if(v.isArray() && v.size() == 3 && v[0].isNumeric())
  {
    Eigen::Vector3d ret;
    ret << v[0].asDouble(), v[1].asDouble(), v[2].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector3d");
}

Configuration::operator Eigen::Vector6d() const
{
  if(v.isArray() && v.size() == 6 && v[0].isNumeric())
  {
    Eigen::Vector6d ret;
    ret << v[0].asDouble(), v[1].asDouble(), v[2].asDouble(), v[3].asDouble(), v[4].asDouble(), v[5].asDouble();
    return ret;
  }
  throw Exception("Stored Json value is not a Vector6d");
}

Configuration::operator Eigen::VectorXd() const
{
  if(v.isArray() && (v.size() == 0 || v[0].isNumeric()))
  {
    Eigen::VectorXd ret(v.size());
    for(size_t i = 0; i < v.size(); ++i)
    {
      ret(i) = v[static_cast<int>(i)].asDouble();
    }
    return ret;
  }
  throw Exception("Stored Json value is not a VectorXd");
}

Configuration::operator Eigen::Quaterniond() const
{
  if(v.isArray() && v.size() == 4 && v[0].isNumeric())
  {
    return Eigen::Quaterniond(v[0].asDouble(), v[1].asDouble(), v[2].asDouble(), v[3].asDouble()).normalized();
  }
  throw Exception("Stored Json value is not a Quaterniond");
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
          m(i, j) = v[3 * i + j].asDouble();
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
  throw Exception("Stored Json value is not a Matrix3d");
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
        m(i, j) = v[6 * i + j].asDouble();
      }
    }
    return m;
  }
  throw Exception("Stored Json value is not a Matrix6d");
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
        ret(i, j) = row[j].asDouble();
      }
    }
    return ret;
  }
  throw Exception("Stored Json value is not a MatrixXd");
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
  throw Exception("Stored Json value is not a PTransformd");
}

Configuration::operator sva::ForceVecd() const
{
  if(has("couple") && has("force"))
  {
    return {(*this)("couple"), (*this)("force")};
  }
  throw Exception("Stored Json value is not a ForceVecd");
}

Configuration::operator sva::MotionVecd() const
{
  if(has("angular") && has("linear"))
  {
    return {(*this)("angular"), (*this)("linear")};
  }
  throw Exception("Stored Json value is not a MotionVecd");
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

void Configuration::load(const std::string & path)
{
  auto & target = *std::static_pointer_cast<internal::RapidJSONDocument>(v.doc_);

  if(target.IsNull())
  {
    mc_rtc::internal::loadDocument(path, target);
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
    auto & docFrom = *std::static_pointer_cast<internal::RapidJSONDocument>(config.v.doc_);
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
      auto & docFrom = *std::static_pointer_cast<internal::RapidJSONDocument>(config.v.doc_);
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

void Configuration::save(const std::string & path, bool pretty) const
{
  auto & value = *static_cast<internal::RapidJSONValue *>(v.value_);
  mc_rtc::internal::saveDocument(path, value, pretty);
}

std::string Configuration::dump(bool pretty) const
{
  auto & value = *static_cast<internal::RapidJSONValue *>(v.value_);
  return mc_rtc::internal::dumpDocument(value, pretty);
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
void add_impl(const std::string & key, T value, void * json_p, void * doc_p)
{
  auto & json = *static_cast<internal::RapidJSONValue *>(json_p);
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
void push_impl(T value, void * json_p, void * doc_p)
{
  auto & json = *static_cast<internal::RapidJSONValue *>(json_p);
  auto & doc = *static_cast<internal::RapidJSONDocument *>(doc_p);
  auto & allocator = doc.GetAllocator();
  internal::RapidJSONValue value_ = mc_rtc::internal::toJSON(value, allocator);
  if(!json.IsArray())
  {
    throw Configuration::Exception("Trying to push data in a non-array value");
  }
  json.PushBack(value_, allocator);
}
} // namespace

// clang-format off
void Configuration::add(const std::string & key, bool value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, int value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, unsigned int value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, double value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const char * value) { add(key, std::string(value)); }
void Configuration::add(const std::string & key, const std::string & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector2d & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector3d & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Vector6d & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::VectorXd & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Quaterniond & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Matrix3d & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::Matrix6d & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const Eigen::MatrixXd & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::PTransformd & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::ForceVecd & value) { add_impl(key, value, v.value_, v.doc_.get()); }
void Configuration::add(const std::string & key, const sva::MotionVecd & value) { add_impl(key, value, v.value_, v.doc_.get()); }
// clang-format on

void Configuration::add(const std::string & key, const Configuration & value)
{
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
    throw Exception("Cannot store an anonymous array outside of an array");
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
    throw Exception("Cannot store an anonymous object outside of an array");
  }
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  internal::RapidJSONValue value(rapidjson::kObjectType);
  json.PushBack(value, allocator);
  return (*this)[size() - 1];
}

// clang-format off
void Configuration::push(bool value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(int value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(unsigned int value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(double value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const char * value) { push(std::string(value)); }
void Configuration::push(const std::string & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector2d & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector3d & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Vector6d & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::VectorXd & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Quaterniond & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Matrix3d & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::Matrix6d & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const Eigen::MatrixXd & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::PTransformd & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::ForceVecd & value) { push_impl(value, v.value_, v.doc_.get()); }
void Configuration::push(const sva::MotionVecd & value) { push_impl(value, v.value_, v.doc_.get()); }
// clang-format on

void Configuration::push(const mc_rtc::Configuration & value)
{
  auto & doc = *static_cast<internal::RapidJSONDocument *>(v.doc_.get());
  auto & allocator = doc.GetAllocator();
  auto & json = *static_cast<internal::RapidJSONValue *>(v.value_);
  if(!json.IsArray())
  {
    throw Exception("Trying to push data in a non-array value");
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

} // namespace mc_rtc

std::ostream & operator<<(std::ostream & os, const mc_rtc::Configuration & c)
{
  os << static_cast<std::string>(c);
  return os;
}
