#pragma once

/*! The purpose of this header file is to provide helper functions related to
 * JSON manipulation in mc_rtc */

#define RAPIDJSON_HAS_STDSTRING 1
#define RAPIDJSON_PARSE_DEFAULT_FLAGS rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag
#define RAPIDJSON_NO_SIZETYPEDEFINE
namespace rapidjson
{
    typedef size_t SizeType;
}

#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/pointer.h"
#include "rapidjson/prettywriter.h"
#include "rapidjson/writer.h"
#include "rapidjson/error/en.h"

#include <mc_rtc/logging.h>

#include <Eigen/Geometry>
#include <SpaceVecAlg/EigenTypedef.h>

#include <fstream>
#include <sstream>

namespace mc_rtc
{

namespace internal
{

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
inline bool loadDocument(const std::string & path, rapidjson::Document & document)
{
  std::ifstream ifs(path);
  if(!ifs.is_open())
  {
    LOG_ERROR("Failed to open controller configuration file: " << path)
    return false;
  }
  std::stringstream json;
  json << ifs.rdbuf();
  rapidjson::ParseResult res = document.Parse(json.str().c_str());
  if(!res)
  {
    rapidjson::GetParseErrorFunc GetParseError = rapidjson::GetParseError_En;
    std::stringstream ss;
    ss << GetParseError(res.Code()) << std::endl;
    ss << "Position: " << res.Offset();
    LOG_ERROR("Failed to read configuration file: " << path)
    LOG_WARNING(ss.str())
    return false;
  }
  return true;
}

/*! Save a JSON document to the provided disk location
 *
 * \param path Output path
 *
 * \param document Document to be saved
 *
 */
inline void saveDocument(const std::string & path, rapidjson::Value & document, bool pretty = false)
{
  std::ofstream ofs(path);
  rapidjson::OStreamWrapper osw(ofs);
  if(pretty)
  {
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(osw);
    document.Accept(writer);
  }
  else
  {
    rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
    document.Accept(writer);
  }
}

/*! Create a JSON value from a C++ value
 *
 * \param value Value to be serialized
 *
 * \param allocator Allocator to use
 *
 */
template<typename T>
inline rapidjson::Value toJSON(T value, rapidjson::Document::AllocatorType &)
{
  return rapidjson::Value(value);
}

template<>
inline rapidjson::Value toJSON(std::string value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret;
  ret.SetString(value.c_str(), value.size(), allocator);
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Vector2d value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(2, allocator);
  ret.PushBack(value(0), allocator);
  ret.PushBack(value(1), allocator);
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Vector3d value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(3, allocator);
  ret.PushBack(value(0), allocator);
  ret.PushBack(value(1), allocator);
  ret.PushBack(value(2), allocator);
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Vector6d value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
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
inline rapidjson::Value toJSON(Eigen::VectorXd value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(value.size(), allocator);
  for(Eigen::VectorXd::Index i = 0; i < value.size(); ++i)
  {
    ret.PushBack(value(i), allocator);
  }
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Quaterniond value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(4, allocator);
  ret.PushBack(value.w(), allocator);
  ret.PushBack(value.x(), allocator);
  ret.PushBack(value.y(), allocator);
  ret.PushBack(value.z(), allocator);
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Matrix3d value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(9, allocator);
  for(size_t i = 0; i < 3; ++i)
  {
    for(size_t j = 0; j < 3; ++j)
    {
      ret.PushBack(value(i,j), allocator);
    }
  }
  return ret;
}

template<>
inline rapidjson::Value toJSON(Eigen::Matrix6d value, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kArrayType);
  ret.Reserve(36, allocator);
  for(size_t i = 0; i < 6; ++i)
  {
    for(size_t j = 0; j < 6; ++j)
    {
      ret.PushBack(value(i,j), allocator);
    }
  }
  return ret;
}

} // namespace internal

} // namespace mc_rtc
