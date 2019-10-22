#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <yaml-cpp/yaml.h>

/** Set of utilities function to work with YAML format in mc_rtc::Configuration */

namespace mc_rtc
{

namespace internal
{

namespace
{

/** Attempt to convert a YAML node the provided type
 *
 * \param node YAML node containing a scalar
 *
 * \param out Output value
 *
 * \tparam T Conversion type
 *
 * \returns True if the conversion was successfull
 */
template<typename T>
bool try_convert(const YAML::Node & node, T & out)
{
  try
  {
    out = node.as<T>();
    return true;
  }
  catch(const YAML::TypedBadConversion<T> &)
  {
  }
  return false;
}

/** Attempt to convert a YAML node to the provided type and push it in a provided Configuration
 *
 * \param node YAML node containing a scalar
 *
 * \param out Output configuration
 *
 * \tparam T Conversion type
 *
 * \returns True if the conversion was successfull
 */
template<typename T>
bool try_push(const YAML::Node & node, Configuration & out)
{
  T value;
  if(try_convert<T>(node, value))
  {
    out.push(value);
    return true;
  }
  return false;
}

/** Attempt to convert a YAML node to the provided type and add it in a provided Configuration
 *
 * \param node YAML node containing a scalar
 *
 * \param out Output configuration
 *
 * \param key Key to add
 *
 * \tparam T Conversion type
 *
 * \returns True if the conversion was successfull
 */
template<typename T>
bool try_add(const YAML::Node & node, Configuration & out, const std::string & key)
{
  T value;
  if(try_convert<T>(node, value))
  {
    out.add(key, value);
    return true;
  }
  return false;
}

inline bool fromYAMLSequence(const YAML::Node & node, Configuration out);
inline bool fromYAMLMap(const YAML::Node & node, Configuration out);

inline bool fromYAMLSequence(const YAML::Node & node, Configuration out)
{
  for(size_t i = 0; i < node.size(); ++i)
  {
    const YAML::Node & ni = node[i];
    if(ni.IsScalar())
    {
      if(!try_push<bool>(ni, out) && !try_push<int64_t>(ni, out) && !try_push<uint64_t>(ni, out)
         && !try_push<double>(ni, out) && !try_push<std::string>(ni, out))
      {
        LOG_ERROR("Could not convert YAML node in a provided Sequence")
        LOG_WARNING("Scalar value: " << ni.Scalar())
        return false;
      }
    }
    else if(ni.IsSequence())
    {
      if(!fromYAMLSequence(ni, out.array(ni.size())))
      {
        return false;
      }
    }
    else if(ni.IsMap())
    {
      if(!fromYAMLMap(ni, out.object()))
      {
        return false;
      }
    }
    else
    {
      LOG_ERROR("Undefined or Null node in loaded YAML data")
      return false;
    }
  }
  return true;
}

inline bool fromYAMLMap(const YAML::Node & node, Configuration out)
{
  for(const auto & it : node)
  {
    const auto & key = it.first.as<std::string>();
    const YAML::Node & n = it.second;
    if(n.IsScalar())
    {
      if(!try_add<bool>(n, out, key) && !try_add<int64_t>(n, out, key) && !try_add<uint64_t>(n, out, key)
         && !try_add<double>(n, out, key) && !try_add<std::string>(n, out, key))
      {
        LOG_ERROR("Could not convert YAML node in a provided Map")
        LOG_WARNING("Key: " << key << ", Scalar value: " << n.Scalar())
        return false;
      }
    }
    else if(n.IsSequence())
    {
      if(!fromYAMLSequence(n, out.array(key, n.size())))
      {
        return false;
      }
    }
    else if(n.IsMap())
    {
      if(!fromYAMLMap(n, out.add(key)))
      {
        return false;
      }
    }
    else
    {
      LOG_ERROR("Undefined or Null node in loaded YAML data")
      return false;
    }
  }
  return true;
}

/*! Build a Configuration object from a YAML node
 *
 * \param node YAML node that will be used to construct the document
 *
 * \param out Output configuration object
 *
 * \returns True if the conversion succeeded, false otherwise
 */
inline bool YAMLToJSON(const YAML::Node & node, Configuration & out)
{
  if(node.IsSequence())
  {
    out = mc_rtc::Configuration::rootArray();
    return fromYAMLSequence(node, out);
  }
  else if(node.IsMap())
  {
    return fromYAMLMap(node, out);
  }
  else
  {
    LOG_ERROR("Cannot convert from YAML if the root type is not a map or a sequence")
    return false;
  }
}

} // namespace

/*! Load YAML data from the provided buffer into the provided Configuration object
 *
 * \param data Data buffer
 *
 * \param out Output configuration object
 *
 * \returns True if the document was successfully loaded, returns false and display an error message otherwise
 */
inline bool loadYAMLData(const char * data, Configuration & out)
{
  try
  {
    auto node = YAML::Load(data);
    return YAMLToJSON(node, out);
  }
  catch(const YAML::ParserException & exc)
  {
    LOG_ERROR("Encountered an error while parsing YAML data")
    LOG_WARNING(exc.msg)
  }
  return false;
}

/*! Load a YAML document from the provided disk location into the provided
 * Configuration object
 *
 * \param path Location of the document
 *
 * \param out Output configuration object
 *
 * \returns True if the document was successfully loaded, returns false and display an error message otherwise
 */
inline bool loadYAMLDocument(const std::string & path, Configuration & out)
{
  try
  {
    auto node = YAML::LoadFile(path);
    return YAMLToJSON(node, out);
  }
  catch(const YAML::BadFile & exc)
  {
    LOG_ERROR("Failed to open controller configuration file: " << path)
    LOG_WARNING(exc.msg)
  }
  catch(const YAML::ParserException & exc)
  {
    LOG_ERROR("Encountered an error while parsing YAML file: " << path)
    LOG_WARNING(exc.msg)
  }
  return false;
}

} // namespace internal

} // namespace mc_rtc
