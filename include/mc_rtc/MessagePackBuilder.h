#pragma once

#include <mc_rtc/utils_api.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <array>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace mc_rtc
{

struct Configuration;

struct MessagePackBuilderImpl;

/** Helper class to build a MessagePack message
 *
 * After a message has been built, the builder object must be discarded to
 * create a new message.
 */
struct MC_RTC_UTILS_DLLAPI MessagePackBuilder
{
  /** Constructor
   *
   * \param buffer Buffer used to store the data, it may grow depending on the needs
   *
   */
  MessagePackBuilder(std::vector<char> & buffer);

  /** Destructor */
  ~MessagePackBuilder();

  /** @name Add data to the MessagePack (basic)
   *
   * These overload set allows to write basic data to the MessagePack
   *
   * @{
   */

  /** Write null */
  void write();
  /** Write a bool */
  void write(bool b);
  /** Write a int8_t */
  void write(int8_t i);
  /** Write a int16_t */
  void write(int16_t i);
  /** Write a int32_t */
  void write(int32_t i);
  /** Write a int64_t */
  void write(int64_t i);
  /** Write a uint8_t */
  void write(uint8_t i);
  /** Write a uint16_t */
  void write(uint16_t i);
  /** Write a uint32_t */
  void write(uint32_t i);
  /** Write a uint64_t */
  void write(uint64_t i);
  /** Write a float */
  void write(float f);
  /** Write a double */
  void write(double d);
  /** Write an std::string */
  void write(const std::string & s);
  /** Write a C-style string */
  void write(const char * s);
  /** Write a C-style string with available size */
  void write(const char * s, size_t len);

  /** @} */
  /* End Add data to the MessagePack section */

  /** @name Add data to the MessagePack (extended)
   *
   * These functions serialize common types used throughout mc_rtc. This does
   * not use MessagePack extension mechanism. Instead, this serializes to
   * simple primitives, if type retrieval is desired, one should implement a
   * specific mechanism.
   *
   * @{
   */

  /** Write an Eigen::Vector2d
   *
   * Serializes as an array of size 2
   */
  void write(const Eigen::Vector2d & v);

  /** Write an Eigen::Vector3d
   *
   * Serializes as an array of size 3
   */
  void write(const Eigen::Vector3d & v);

  /** Write an Eigen::Vector6d
   *
   * Serializes as an array of size 6
   */
  void write(const Eigen::Vector6d & v);

  /** Write an Eigen::VectorXd
   *
   * Serializes as an array of size X
   */
  void write(const Eigen::VectorXd & v);

  /** Write an Eigen::Quaterniond
   *
   * Serializes as an array of size X
   */
  void write(const Eigen::Quaterniond & q);

  /** Write an Eigen::Matrix3d
   *
   * Serializes as an array of size 9
   */
  void write(const Eigen::Matrix3d & m);

  /** Write an sva::PTransformd
   *
   * Serializes as an array of size 12 (Matrix3d + Vector3d)
   */
  void write(const sva::PTransformd & pt);

  /** Write an sva::ForceVecd
   *
   * Serialized as an array of size 6 (Torque + Force)
   */
  void write(const sva::ForceVecd & fv);

  /** Write an sva::MotionVecd
   *
   * Serialized as an array of size 6 (Angular + Linear)
   */
  void write(const sva::MotionVecd & mv);

  /** Write an sva::ImpedanceVecd
   *
   * Serialized as an array of size 6 (Angular + Linear)
   */
  void write(const sva::ImpedanceVecd & mv);

  /** Write an mc_rtc::Configuration
   *
   * Serialied as the JSON data it holds
   */
  void write(const mc_rtc::Configuration & config);

  /** @} */
  /* End Add data to the MessagePack section (advanced) */

  /** @name Add data to the MessagePack (containers)
   *
   * These functions support the serialization of standard containers of
   * serializable objects.
   *
   * @{
   */

  /** Write an std::vector<T, A> */
  template<typename T, typename A>
  void write(const std::vector<T, A> & v)
  {
    start_array(v.size());
    for(const auto & value : v)
    {
      write(value);
    }
    finish_array();
  }

  /** Write an std::array<T, N> */
  template<typename T, std::size_t N>
  void write(const std::array<T, N> & a)
  {
    start_array(N);
    for(const auto & value : a)
    {
      write(value);
    }
    finish_array();
  }

  /** Write an std::pair<T1, T2> */
  template<typename T1, typename T2>
  void write(const std::pair<T1, T2> & p)
  {
    start_array(2);
    write(p.first);
    write(p.second);
    finish_array();
  }

  /** Write an std::map<KeyT, T, C, A> */
  template<typename KeyT, typename T, typename C, typename A>
  void write(const std::map<KeyT, T, C, A> & m)
  {
    start_map(m.count());
    for(const auto & p : m)
    {
      write(p.first);
      write(p.second);
    }
    finish_map();
  }

  /** Write an std::set<T, C, A> */
  template<typename T, typename C, typename A>
  void write(const std::set<T, C, A> & s)
  {
    start_array(s.size());
    for(const auto & value : s)
    {
      write(value);
    }
    finish_array();
  }

  /** Write an std::tuple<Args...> */
  template<typename... Args>
  void write(const std::tuple<Args...> & t)
  {
    start_array(sizeof...(Args));
    write_impl<0>(t);
    finish_array();
  }

  /** @} */
  /* End Add data to the MessagePack section (containers) */

  /** Start serializing an array
   *
   * \param size Size of the array
   */
  void start_array(size_t size);

  /** Finished serializing an array */
  void finish_array();

  /** Start serializing a map
   *
   * \param size Size of the map, the map must then contains 2*size elements
   */
  void start_map(size_t size);

  /** Finished serializing a map */
  void finish_map();

  /** Write an existing object into the object being constructed
   *
   * \param data Data written into the object
   *
   * \param size Size of the data
   *
   */
  void write_object(const char * data, size_t s);

  /** Finish building the message
   *
   * Afterwards, data cannot be appended to the builder
   *
   * \returns Effective size of MessagePack data, note that buffer.size() is likely different
   *
   */
  size_t finish();

private:
  /** Hide MessagePack implementation choice in pimpl pattern */
  std::unique_ptr<MessagePackBuilderImpl> impl_;

  template<size_t i,
           typename... Args,
           typename std::enable_if<i<sizeof...(Args), int>::type = 0> void write_impl(const std::tuple<Args...> & t)
  {
    write(std::get<i>(t));
    write_impl<i + 1>(t);
  }

  template<size_t i, typename... Args, typename std::enable_if<i >= sizeof...(Args), int>::type = 0>
  void write_impl(const std::tuple<Args...> &)
  {
  }
};

} // namespace mc_rtc
