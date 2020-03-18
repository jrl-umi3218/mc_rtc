/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

namespace mc_rtc
{

/** Attempts to convert a read a vector or single element as a vector, use
 * default value if it fails.
 *
 *
 * Example:
 * \code{.yaml}
 * configVector: ["String1", "String2"]
 * configString: "String"
 * \endcode
 *
 * The following are valid
 * \code{.cpp}
 * auto conf = mc_rtc::Configuration{};
 * conf.add("configVector", {"String1", "String2"})
 * conf.add("configString", "String1");
 * auto v1 = fromVectorOrElement<std::string>(conf("configVector"));
 * auto v2 = fromVectorOrElement<std::string>(conf("configString"));
 * \endcode
 *
 * @tparam T Type of the element to convert toward
 * @param config Configuration object from which to convert. This should contain
 * either a vector of elements convertible to T ([T, T, ...]) or a single T
 * element.
 *
 * @param key Name of the element to read
 *
 * @return Vector containing all elements in the configuration vector or the
 * single item. If it fails, return defaultVec.
 */
template<typename T>
std::vector<T> fromVectorOrElement(const mc_rtc::Configuration & config,
                                   const std::string & key,
                                   const std::vector<T> & defaultVec)
{
  if(!config.has(key))
  {
    return defaultVec;
  }
  const auto & c = config(key);
  try
  {
    // Try to convert configuration as a vector
    std::vector<T> vec = c;
    return vec;
  }
  catch(mc_rtc::Configuration::Exception & notAVec)
  { // If that fails, try to convert as an element
    notAVec.silence();
    try
    {
      T elem = c;
      return {elem};
    }
    catch(mc_rtc::Configuration::Exception & notAnElem)
    { // If that fails as well, return the default
      notAnElem.silence();
      return defaultVec;
    }
  }
}

/** Attempts to convert a read a vector or single element as a vector
 *
 * Variant of fromVectorOrElement(const mc_rtc::Configuration & config, const std::string & key, const std::vector<T> &
 * defaultVec)
 * @throws mc_rtc::Configuration::Exception If the key is not found or the configuration is neither convertible
 * as std::vector<T> or T.
 */
template<typename T>
std::vector<T> fromVectorOrElement(const mc_rtc::Configuration & config, const std::string & key)
{
  std::vector<T> vec;
  const auto & c = config(key);
  try
  {
    vec = c;
  }
  catch(mc_rtc::Configuration::Exception & notAVec)
  {
    notAVec.silence();
    try
    {
      T elem = c;
      vec.push_back(elem);
    }
    catch(mc_rtc::Configuration::Exception & notAnElem)
    {
      notAnElem.silence();
      LOG_ERROR_AND_THROW(mc_rtc::Configuration::Exception,
                          "Configuration " << key << " is not valid. It should be a vector or single element.");
    }
  }
  return vec;
}

} // namespace mc_rtc
