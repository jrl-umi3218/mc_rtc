#pragma once

/*! The purpose of this header file is to provide helper functions related to
 * JSON manipulation in mc_rtc */

#define RAPIDJSON_HAS_STDSTRING 1
#define RAPIDJSON_PARSE_DEFAULT_FLAGS rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag

#include "rapidjson/document.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/pointer.h"
#include "rapidjson/writer.h"
#include "rapidjson/error/en.h"

#include <mc_rtc/logging.h>

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
inline void saveDocument(const std::string & path, rapidjson::Document & document)
{
  std::ofstream ofs(path);
  rapidjson::OStreamWrapper osw(ofs);
  rapidjson::Writer<rapidjson::OStreamWrapper> writer(osw);
  document.Accept(writer);
}


}

}
