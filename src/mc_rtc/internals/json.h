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

#include <mc_rtc/logging.h>

#include <SpaceVecAlg/SpaceVecAlg>

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
