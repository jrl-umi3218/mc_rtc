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

/** Inspired by mpack.c @ version 1.0 */
static void mpack_std_vector_writer_flush(mpack_writer_t * writer, const char * data, size_t count)
{
  auto & buffer = *static_cast<std::vector<char> *>(writer->context);
  // This is an intrusive flush function which modifies the writer's buffer
  // in response to a flush instead of emptying it in order to add more
  // capacity for data. This removes the need to copy data from a fixed buffer
  // into a growable one, improving performance.
  //
  // There are three ways flush can be called:
  //   - flushing the buffer during writing (used is zero, count is all data, data is buffer)
  //   - flushing extra data during writing (used is all flushed data, count is extra data, data is not buffer)
  //   - flushing during teardown (used and count are both all flushed data, data is buffer)
  //
  // In the first two cases, we grow the buffer by at least double, enough
  // to ensure that new data will fit. We ignore the teardown flush.

  if(data == writer->buffer)
  {

    // teardown, do nothing
    if(mpack_writer_buffer_used(writer) == count) return;

    // otherwise leave the data in the buffer and just grow
    writer->current = writer->buffer + count;
    count = 0;
  }

  size_t used = mpack_writer_buffer_used(writer);
  size_t size = mpack_writer_buffer_size(writer);

  mpack_log("flush size %i used %i data %p buffer %p\n", (int)count, (int)used, data, writer->buffer);

  mpack_assert(data == writer->buffer || used + count > size,
               "extra flush for %i but there is %i space left in the buffer! (%i/%i)", (int)count,
               (int)mpack_writer_buffer_left(writer), (int)used, (int)size);

  // grow to fit the data
  // TODO: this really needs to correctly test for overflow
  size_t new_size = size * 2;
  while(new_size < used + count) new_size *= 2;

  mpack_log("flush growing buffer size from %i to %i\n", (int)size, (int)new_size);

  // grow the buffer
  buffer.resize(new_size);
  char * new_buffer = buffer.data();
  if(new_buffer == NULL)
  {
    mpack_writer_flag_error(writer, mpack_error_memory);
    return;
  }
  writer->current = new_buffer + used;
  writer->buffer = new_buffer;
  writer->end = writer->buffer + new_size;

  // append the extra data
  if(count > 0)
  {
    mpack_memcpy(writer->current, data, count);
    writer->current += count;
  }

  mpack_log("new buffer %p, used %i\n", new_buffer, (int)mpack_writer_buffer_used(writer));
}

inline static void toMessagePackImpl(mpack_writer_t * writer, const RapidJSONValue & value)
{
  switch(value.GetType())
  {
    case rapidjson::kNullType:
      mpack_write_nil(writer);
      break;
    case rapidjson::kFalseType:
    case rapidjson::kTrueType:
      mpack_write_bool(writer, value.GetBool());
      break;
    case rapidjson::kObjectType:
      mpack_start_map(writer, value.MemberCount());
      for(auto it = value.MemberBegin(); it != value.MemberEnd(); ++it)
      {
        mpack_write_cstr(writer, it->name.GetString());
        toMessagePackImpl(writer, it->value);
      }
      mpack_finish_map(writer);
      break;
    case rapidjson::kArrayType:
      mpack_start_array(writer, value.Size());
      for(auto it = value.Begin(); it != value.End(); ++it)
      {
        toMessagePackImpl(writer, *it);
      }
      mpack_finish_array(writer);
      break;
    case rapidjson::kStringType:
      mpack_write_cstr(writer, value.GetString());
      break;
    case rapidjson::kNumberType:
      if(value.IsInt())
      {
        mpack_write_i32(writer, value.GetInt());
      }
      else if(value.IsUint())
      {
        mpack_write_u32(writer, value.GetUint());
      }
      else if(value.IsInt64())
      {
        mpack_write_i64(writer, value.GetInt64());
      }
      else if(value.IsUint64())
      {
        mpack_write_u64(writer, value.GetUint64());
      }
      else // Assume double
      {
        mpack_write_double(writer, value.GetDouble());
      }
      break;
  }
}

inline size_t toMessagePack(const RapidJSONValue & document, std::vector<char> & data)
{
  mpack_writer_t writer;
  if(data.size() == 0)
  {
    data.resize(MPACK_BUFFER_SIZE);
  }
  mpack_writer_init(&writer, data.data(), data.size());
  mpack_writer_set_context(&writer, &data);
  mpack_writer_set_flush(&writer, mpack_std_vector_writer_flush);

  toMessagePackImpl(&writer, document);

  if(mpack_writer_destroy(&writer) != mpack_ok)
  {
    LOG_ERROR("Failed to convert to MessagePack")
  }
  return mpack_writer_buffer_used(&writer);
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
    case mpack_type_nil:
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
    case mpack_type_nil:
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
  if(mpack_node_type(root) != mpack_type_map)
  {
    LOG_ERROR("Cannot convert from MessagePack if the root type is not a map")
    return;
  }
  fromMessagePackMap(config, root);
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
