#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <fstream>
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
  return YAML::convert<T>::decode(node, out);
}

template<>
inline bool try_convert<bool>(const YAML::Node & node, bool & out)
{
  if(YAML::convert<bool>::decode(node, out))
  {
    // yaml-cpp treats y, yes, n and no as valid bool (case insensitive) we check if this was the case here
    char scalar = node.Scalar()[0];
    return scalar != 'y' && scalar != 'Y' && scalar != 'n' && scalar != 'N';
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

/**
 * @brief Handles scalar YAML nodes when converting a YAML sequence to a Configuration.
 *
 * This function attempts to push the scalar value to the output Configuration.
 * If the scalar is explicitly tagged as a string (with key: !!str 12345), it is always treated as a string. Note that
 * yaml-cpp treats quoted numbers as numbers (that is "12345" is an int, not a string). Also note that numbers starting
 * with a leading zero are treated as octal numbers (so "0123" is 83 as int, not "0123" as string). Otherwise, it tries
 * to push as bool, int64_t, uint64_t, double, and finally as string.
 *
 * @param node The YAML scalar node to handle.
 * @param out The output Configuration to push the value into.
 * @return true if the value was successfully pushed, false otherwise.
 */
inline bool handleYAMLScalar(const YAML::Node & node, Configuration out)
{
  bool ok = false;
  if(node.Tag() == "tag:yaml.org,2002:str") { ok = try_push<std::string>(node, out); }
  else
  {
    ok = try_push<bool>(node, out) || try_push<int64_t>(node, out) || try_push<uint64_t>(node, out)
         || try_push<double>(node, out) || try_push<std::string>(node, out);
  }
  if(!ok)
  {
    log::error("Could not convert YAML node in a provided Sequence");
    log::warning("Scalar value: {}", node.Scalar());
    return false;
  }
  return true;
}

/**
 * @brief Handles scalar YAML nodes when converting a YAML map to a Configuration.
 *
 * \see handleYAMLScalar(const YAML::Node & node, Configuration out)
 *
 * @param node The YAML scalar node to handle.
 * @param out The output Configuration to add the value into.
 * @param key The key under which to add the value in the Configuration.
 * @return true if the value was successfully added, false otherwise.
 *
 */
inline bool handleYAMLScalar(const YAML::Node & node, Configuration out, const std::string & key)
{
  bool ok = false;
  if(node.Tag() == "tag:yaml.org,2002:str") { ok = try_add<std::string>(node, out, key); }
  else
  {
    ok = try_add<bool>(node, out, key) || try_add<int64_t>(node, out, key) || try_add<uint64_t>(node, out, key)
         || try_add<double>(node, out, key) || try_add<std::string>(node, out, key);
  }
  if(!ok)
  {
    log::error("Could not convert YAML node in a provided Map");
    log::warning("Key: {}, Scalar value: {}", key, node.Scalar());
    return false;
  }
  return true;
}

inline bool fromYAMLSequence(const YAML::Node & node, Configuration out)
{
  for(size_t i = 0; i < node.size(); ++i)
  {
    const YAML::Node & ni = node[i];
    if(ni.IsScalar())
    {
      if(!handleYAMLScalar(ni, out)) { return false; }
    }
    else if(ni.IsSequence())
    {
      if(!fromYAMLSequence(ni, out.array(ni.size()))) { return false; }
    }
    else if(ni.IsMap())
    {
      if(!fromYAMLMap(ni, out.object())) { return false; }
    }
    else
    {
      log::error("Undefined or Null node in loaded YAML data");
      return false;
    }
  }
  return true;
}

inline bool fromYAMLMap(const YAML::Node & node, Configuration out)
{
  // yaml-cpp does not handle YAML merge so we do it "manually"
  // we handle the merge node first (if any) so that the values it provides can be overwritten
  auto merge_it = std::find_if(node.begin(), node.end(),
                               [](const YAML::detail::iterator_value & it) { return it.first.Scalar() == "<<"; });
  if(merge_it != node.end())
  {
    if(!fromYAMLMap(merge_it->second, out)) { return false; }
  }
  for(const auto & it : node)
  {
    const auto & key = it.first.Scalar();
    if(key == "<<") { continue; }
    const YAML::Node & n = it.second;
    if(n.IsScalar())
    {
      if(!handleYAMLScalar(n, out, key)) { return false; }
    }
    else if(n.IsSequence())
    {
      if(!fromYAMLSequence(n, out.array(key, n.size()))) { return false; }
    }
    else if(n.IsMap())
    {
      if(!fromYAMLMap(n, key == "<<" ? out : out.add(key))) { return false; }
    }
    else
    {
      log::error("Undefined or Null node in loaded YAML data");
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
  else if(node.IsMap()) { return fromYAMLMap(node, out); }
  else if(node.IsNull()) { return true; }
  else
  {
    log::error("Cannot convert from YAML if the root type is not a map or a sequence");
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
    log::error("Encountered an error while parsing YAML data");
    log::warning("Error on line {}, column {}: {}", exc.mark.line + 1, exc.mark.column + 1, exc.msg);
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
    log::error("Failed to open controller configuration file: {}", path);
    log::warning(exc.msg);
  }
  catch(const YAML::ParserException & exc)
  {
    log::error("Encountered an error while parsing YAML file: {}", path);
    log::warning("Error on line {}, column {}: {}", exc.mark.line + 1, exc.mark.column + 1, exc.msg);
  }
  return false;
}

namespace
{

inline void dumpYAML(const mc_rtc::Configuration & in, YAML::Emitter & out)
{
  if(in.size())
  {
    size_t sz = in.size();
    auto isScalar = [](const mc_rtc::Configuration & c) { return c.size() == 0 && c.keys().size() == 0; };
    bool allScalar = true;
    for(size_t i = 0; i < sz; ++i)
    {
      if(!isScalar(in[i]))
      {
        allScalar = false;
        break;
      }
    }
    if(allScalar) { out << YAML::Flow; }
    out << YAML::BeginSeq;
    for(size_t i = 0; i < sz; ++i) { dumpYAML(in[i], out); }
    out << YAML::EndSeq;
  }
  else
  {
    const auto & keys = in.keys();
    if(keys.size())
    {
      out << YAML::BeginMap;
      for(const auto & k : keys)
      {
        out << YAML::Key << k;
        out << YAML::Value;
        dumpYAML(in(k), out);
      }
      out << YAML::EndMap;
    }
    else
    {
      auto dump = in.dump();
      if(dump == "[]") { out << YAML::Flow << YAML::BeginSeq << YAML::EndSeq; }
      else if(dump == "{}") { out << YAML::Flow << YAML::BeginMap << YAML::EndMap; }
      else if(dump[0] == '"' && dump[dump.size() - 1] == '"') { out << dump.substr(1, dump.size() - 2); }
      else
      {
        out << dump;
      }
    }
  }
}

} // namespace

inline std::string dumpYAML(const mc_rtc::Configuration & in)
{
  YAML::Emitter out;
  dumpYAML(in, out);
  if(!out.good()) { log::error("YAML dump error:\n{}", out.GetLastError()); }
  return out.c_str();
}

inline void saveYAML(const std::string & path, const mc_rtc::Configuration & in)
{
  std::ofstream ofs(path);
  if(!ofs)
  {
    log::error("Failed to open {} for writing", path);
    return;
  }
  ofs << dumpYAML(in) << "\n";
}

} // namespace internal

} // namespace mc_rtc
