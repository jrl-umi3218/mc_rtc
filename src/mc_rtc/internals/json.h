/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! The purpose of this header file is to provide helper functions related to
 * JSON manipulation in mc_rtc */

#define RAPIDJSON_HAS_STDSTRING 1
#define RAPIDJSON_PARSE_DEFAULT_FLAGS \
  rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag | rapidjson::kParseNanAndInfFlag
#define RAPIDJSON_WRITE_DEFAULT_FLAGS rapidjson::kWriteNanAndInfFlag
#define RAPIDJSON_NO_SIZETYPEDEFINE
namespace rapidjson
{
typedef size_t SizeType;
}

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include "mpack.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/pointer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>

namespace mc_rtc
{

namespace internal
{

using RapidJSONDocument = rapidjson::GenericDocument<rapidjson::UTF8<>, rapidjson::CrtAllocator>;
using RapidJSONValue = rapidjson::GenericValue<rapidjson::UTF8<>, rapidjson::CrtAllocator>;

/*! Load JSON data into the provided rapidjson::Document
 *
 * \param data JSON data to load
 *
 * \param document Document instance to load the document
 *
 * \param path If passed, will be used to provide a useful error message
 *
 * \returns True if the document was succesfully loaded, returns false and
 * display an error message otherwise
 */
inline bool loadData(const char * data, RapidJSONDocument & document, const std::string & path = "")
{
  rapidjson::ParseResult res = document.Parse(data);
  if(!res)
  {
    rapidjson::GetParseErrorFunc GetParseError = rapidjson::GetParseError_En;
    std::stringstream ss;
    ss << GetParseError(res.Code()) << std::endl;
    ss << "Position: " << res.Offset();
    if(path.size())
    {
      LOG_ERROR("Failed to read configuration file: " << path)
    }
    else
    {
      LOG_ERROR("Failed to read raw json data: " << data)
    }
    LOG_WARNING(ss.str())
    return false;
  }
  return true;
}

/*! Load a JSON document from the provided disk location into the provided
 * rapidjson::Document
 *
 * \param path Location of the document
 *
 * \param document Document instance to load the document
 *
 * \returns True if the document was succesfully loaded, returns false and
 * display an error message otherwise
 *
 */
inline bool loadDocument(const std::string & path, RapidJSONDocument & document)
{
  std::ifstream ifs(path);
  if(!ifs.is_open())
  {
    LOG_ERROR("Failed to open controller configuration file: " << path)
    return false;
  }
  std::stringstream json;
  json << ifs.rdbuf();
  return loadData(json.str().c_str(), document, path);
}

/*! Dump a JSON document into a stream
 *
 * \param document Document to be dumped
 *
 * \param pretty Pretty output
 *
 */
inline std::string dumpDocumentInternal(RapidJSONValue & document, bool pretty)
{
  rapidjson::StringBuffer buffer;
  if(pretty)
  {
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
  }
  else
  {
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    document.Accept(writer);
  }
  return {buffer.GetString(), buffer.GetSize()};
}

/*! Dump a JSON document and get the string back
 *
 * \param document Document to be dumped
 *
 * \param pretty Pretty output
 *
 */
inline std::string dumpDocument(RapidJSONValue & document, bool pretty)
{
  return dumpDocumentInternal(document, pretty);
}

inline static void toMessagePack(const RapidJSONValue & value, MessagePackBuilder & builder)
{
  switch(value.GetType())
  {
    case rapidjson::kNullType:
      builder.write();
      break;
    case rapidjson::kFalseType:
    case rapidjson::kTrueType:
      builder.write(value.GetBool());
      break;
    case rapidjson::kObjectType:
      builder.start_map(value.MemberCount());
      for(auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
      {
        builder.write(it->name.GetString(), it->name.GetStringLength());
        toMessagePack(it->value, builder);
      }
      builder.finish_map();
      break;
    case rapidjson::kArrayType:
      builder.start_array(value.Size());
      for(auto it = value.Begin(); it != value.End(); ++it)
      {
        toMessagePack(*it, builder);
      }
      builder.finish_array();
      break;
    case rapidjson::kStringType:
      builder.write(value.GetString(), value.GetStringLength());
      break;
    case rapidjson::kNumberType:
      if(value.IsInt())
      {
        builder.write(value.GetInt());
      }
      else if(value.IsUint())
      {
        builder.write(value.GetUint());
      }
      else if(value.IsInt64())
      {
        builder.write(value.GetInt64());
      }
      else if(value.IsUint64())
      {
        builder.write(value.GetUint64());
      }
      else // Assume double
      {
        builder.write(value.GetDouble());
      }
      break;
  }
}

namespace
{

void fromMessagePackArray(mc_rtc::Configuration config, mpack_node_t node);

void fromMessagePackMap(mc_rtc::Configuration config, mpack_node_t node);

std::string toString(mpack_node_t node)
{
  return {mpack_node_str(node), mpack_node_strlen(node)};
}

/** Add data into a map */
void fromMessagePack(mc_rtc::Configuration config, const std::string & key, mpack_node_t node)
{
  switch(mpack_node_type(node))
  {
    case mpack_type_missing:
      break;
    case mpack_type_nil:
      config.add_null(key);
      break;
    case mpack_type_bool:
      config.add(key, mpack_node_bool(node));
      break;
    case mpack_type_int:
      config.add(key, mpack_node_i64(node));
      break;
    case mpack_type_uint:
      config.add(key, mpack_node_u64(node));
      break;
    case mpack_type_float:
      config.add(key, mpack_node_float(node));
      break;
    case mpack_type_double:
      config.add(key, mpack_node_double(node));
      break;
    case mpack_type_str:
      config.add(key, toString(node));
      break;
    case mpack_type_array:
      fromMessagePackArray(config.array(key, mpack_node_array_length(node)), node);
      break;
    case mpack_type_map:
      fromMessagePackMap(config.add(key), node);
      break;
    default:
      LOG_ERROR_AND_THROW(std::runtime_error, "Unsupported type in MessagePack")
  }
}

/** Add data into an array */
void fromMessagePack(mc_rtc::Configuration config, mpack_node_t node)
{
  switch(mpack_node_type(node))
  {
    case mpack_type_missing:
      break;
    case mpack_type_nil:
      config.push_null();
      break;
    case mpack_type_bool:
      config.push(mpack_node_bool(node));
      break;
    case mpack_type_int:
      config.push(mpack_node_i64(node));
      break;
    case mpack_type_uint:
      config.push(mpack_node_u64(node));
      break;
    case mpack_type_float:
      config.push(mpack_node_float(node));
      break;
    case mpack_type_double:
      config.push(mpack_node_double(node));
      break;
    case mpack_type_str:
      config.push(toString(node));
      break;
    case mpack_type_array:
      fromMessagePackArray(config.array(mpack_node_array_length(node)), node);
      break;
    case mpack_type_map:
      fromMessagePackMap(config.object(), node);
      break;
    default:
      LOG_ERROR_AND_THROW(std::runtime_error, "Unsupported type in MessagePack")
  }
}

void fromMessagePackArray(mc_rtc::Configuration config, mpack_node_t node)
{
  for(size_t i = 0; i < mpack_node_array_length(node); ++i)
  {
    fromMessagePack(config, mpack_node_array_at(node, i));
  }
}

void fromMessagePackMap(mc_rtc::Configuration config, mpack_node_t node)
{
  for(size_t i = 0; i < mpack_node_map_count(node); ++i)
  {
    fromMessagePack(config, toString(mpack_node_map_key_at(node, i)), mpack_node_map_value_at(node, i));
  }
}

} // namespace

inline void fromMessagePack(mc_rtc::Configuration & config, const char * data, size_t size)
{
  mpack_tree_t tree;
  mpack_tree_init_data(&tree, data, size);
  mpack_tree_parse(&tree);
  if(mpack_tree_error(&tree) != mpack_ok)
  {
    LOG_ERROR("Failed to parse MessagePack data")
    return;
  }
  auto root = mpack_tree_root(&tree);
  if(mpack_node_type(root) == mpack_type_map)
  {
    fromMessagePackMap(config, root);
  }
  else if(mpack_node_type(root) == mpack_type_array)
  {
    config = mc_rtc::Configuration::rootArray();
    fromMessagePackArray(config, root);
  }
  else
  {
    LOG_ERROR("Cannot convert from MessagePack if the root type is not a map or an array")
  }
  mpack_tree_destroy(&tree);
}

/*! Save a JSON document to the provided disk location
 *
 * \param path Output path
 *
 * \param document Document to be saved
 *
 */
inline void saveDocument(const std::string & path, RapidJSONValue & document, bool pretty = false)
{
  std::ofstream ofs(path);
  ofs << dumpDocument(document, pretty);
}

/*! Create a JSON value from a C++ value
 *
 * \param value Value to be serialized
 *
 * \param allocator Allocator to use
 *
 */
template<typename T>
inline RapidJSONValue toJSON(const T & value, RapidJSONDocument::AllocatorType &)
{
  return RapidJSONValue(value);
}

template<>
inline RapidJSONValue toJSON(const std::string & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret;
  ret.SetString(value.c_str(), value.size(), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Vector2d & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(2, allocator);
  ret.PushBack(value(0), allocator);
  ret.PushBack(value(1), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Vector3d & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(3, allocator);
  ret.PushBack(value(0), allocator);
  ret.PushBack(value(1), allocator);
  ret.PushBack(value(2), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Vector6d & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(6, allocator);
  ret.PushBack(value(0), allocator);
  ret.PushBack(value(1), allocator);
  ret.PushBack(value(2), allocator);
  ret.PushBack(value(3), allocator);
  ret.PushBack(value(4), allocator);
  ret.PushBack(value(5), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::VectorXd & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(value.size(), allocator);
  for(Eigen::VectorXd::Index i = 0; i < value.size(); ++i)
  {
    ret.PushBack(value(i), allocator);
  }
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Quaterniond & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(4, allocator);
  ret.PushBack(value.w(), allocator);
  ret.PushBack(value.x(), allocator);
  ret.PushBack(value.y(), allocator);
  ret.PushBack(value.z(), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Matrix3d & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(9, allocator);
  for(size_t i = 0; i < 3; ++i)
  {
    for(size_t j = 0; j < 3; ++j)
    {
      ret.PushBack(value(i, j), allocator);
    }
  }
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::Matrix6d & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(36, allocator);
  for(size_t i = 0; i < 6; ++i)
  {
    for(size_t j = 0; j < 6; ++j)
    {
      ret.PushBack(value(i, j), allocator);
    }
  }
  return ret;
}

template<>
inline RapidJSONValue toJSON(const Eigen::MatrixXd & value, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kArrayType);
  ret.Reserve(value.rows(), allocator);
  for(int i = 0; i < value.rows(); ++i)
  {
    RapidJSONValue row(rapidjson::kArrayType);
    row.Reserve(value.cols(), allocator);
    for(int j = 0; j < value.cols(); ++j)
    {
      row.PushBack(value(i, j), allocator);
    }
    ret.PushBack(row, allocator);
  }
  return ret;
}

template<>
inline RapidJSONValue toJSON(const sva::PTransformd & pt, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kObjectType);
  ret.AddMember("translation", toJSON(pt.translation(), allocator), allocator);
  ret.AddMember("rotation", toJSON(pt.rotation(), allocator), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const sva::ForceVecd & fv, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kObjectType);
  ret.AddMember("couple", toJSON(fv.couple(), allocator), allocator);
  ret.AddMember("force", toJSON(fv.force(), allocator), allocator);
  return ret;
}

template<>
inline RapidJSONValue toJSON(const sva::MotionVecd & mv, RapidJSONDocument::AllocatorType & allocator)
{
  RapidJSONValue ret(rapidjson::kObjectType);
  ret.AddMember("angular", toJSON(mv.angular(), allocator), allocator);
  ret.AddMember("linear", toJSON(mv.linear(), allocator), allocator);
  return ret;
}

} // namespace internal

} // namespace mc_rtc
