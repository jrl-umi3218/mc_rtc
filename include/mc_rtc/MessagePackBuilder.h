#pragma once

#include <mc_rtc/utils_api.h>

#include <mc_rbdyn/Gains.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <array>
#include <exception>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

namespace mc_rtc
{

struct Configuration;
struct MessagePackBuilder;
struct MessagePackBuilderImpl;

namespace internal
{

template<typename T>
constexpr bool is_integral_v = std::is_integral_v<T> || std::numeric_limits<std::decay_t<T>>::is_integer;

/** Compare two integral types */
template<typename RefT, typename T>
constexpr bool is_like()
{
  static_assert(std::is_integral_v<RefT>);
  if constexpr(is_integral_v<T>)
  {
    // clang-format off
    return std::numeric_limits<T>::is_signed == std::is_signed_v<RefT>
           && std::numeric_limits<T>::max() == std::numeric_limits<RefT>::max();
    // clang-format on
  }
  return false;
}

template<typename T>
constexpr bool is_like_int8_t = is_like<int8_t, T>();
template<typename T>
constexpr bool is_like_int16_t = is_like<int16_t, T>();
template<typename T>
constexpr bool is_like_int32_t = is_like<int32_t, T>();
template<typename T>
constexpr bool is_like_int64_t = is_like<int64_t, T>();
template<typename T>
constexpr bool is_like_uint8_t = is_like<uint8_t, T>();
template<typename T>
constexpr bool is_like_uint16_t = is_like<uint16_t, T>();
template<typename T>
constexpr bool is_like_uint32_t = is_like<uint32_t, T>();
template<typename T>
constexpr bool is_like_uint64_t = is_like<uint64_t, T>();

template<typename T, typename = void>
struct has_write_builder : std::false_type
{
};

template<typename T>
struct has_write_builder<T, std::void_t<decltype(std::declval<const T &>().write(std::declval<MessagePackBuilder &>()))>>
: std::true_type
{
};

template<typename T>
static inline constexpr bool has_write_builder_v = has_write_builder<T>::value;

} // namespace internal

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
   * Serialized as an array of size 2
   */
  void write(const Eigen::Vector2d & v);

  /** Write an Eigen::Vector3d
   *
   * Serialized as an array of size 3
   */
  void write(const Eigen::Vector3d & v);

  /** Write an Eigen::Vector4d
   *
   * Serialized as an array of size 4
   */
  void write(const Eigen::Vector4d & v);

  /** Write an Eigen::Vector6d
   *
   * Serialized as an array of size 6
   */
  void write(const Eigen::Vector6d & v);

  /** Write an Eigen::VectorXd
   *
   * Serialized as an array of size X
   */
  void write(const Eigen::VectorXd & v);

  template<int N, int _Options, int _MaxRows, int _MaxCols>
  void write(const Eigen::Matrix<double, N, 1, _Options, _MaxRows, _MaxCols> & v)
  {
    static_assert(N != -1 && N != 2 && N != 3 && N != 6, "Should have gone to specialized function");
    start_array(static_cast<size_t>(N));
    for(Eigen::Index i = 0; i < N; ++i) { write(v(i)); }
    finish_array();
  }

  /** Write an Eigen::Quaterniond
   *
   * Serialized as an array of size 4
   */
  void write(const Eigen::Quaterniond & q);

  /** Write an Eigen::Matrix3d
   *
   * Serialized as an array of size 9
   */
  void write(const Eigen::Matrix3d & m);

  /** Write an sva::PTransformd
   *
   * Serialized as an array of size 12 (Matrix3d + Vector3d)
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
   * Serialized as the JSON data it holds
   */
  void write(const mc_rtc::Configuration & config);

  /** Write numbers by finding their closest standard size match */
  template<typename T, typename std::enable_if_t<internal::is_integral_v<T>, int> = 0>
  void write(const T & number)
  {
    if constexpr(internal::is_like_int8_t<T>) { write(static_cast<int8_t>(number)); }
    else if constexpr(internal::is_like_int16_t<T>) { write(static_cast<int16_t>(number)); }
    else if constexpr(internal::is_like_int32_t<T>) { write(static_cast<int32_t>(number)); }
    else if constexpr(internal::is_like_int64_t<T>) { write(static_cast<int64_t>(number)); }
    else if constexpr(internal::is_like_uint8_t<T>) { write(static_cast<uint8_t>(number)); }
    else if constexpr(internal::is_like_uint16_t<T>) { write(static_cast<uint16_t>(number)); }
    else if constexpr(internal::is_like_uint32_t<T>) { write(static_cast<uint32_t>(number)); }
    else if constexpr(internal::is_like_uint64_t<T>) { write(static_cast<uint64_t>(number)); }
    else { static_assert(!std::is_same_v<T, T>, "T is integral but has an unsupported size"); }
  }

  /** Write \tparam T to MessagePack if T implements T::write(MessagePackBuilder &) const */
  template<typename T, typename = std::enable_if_t<internal::has_write_builder_v<T>>>
  void write(const T & value)
  {
    value.write(*this);
  }

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
    for(const auto & value : v) { write(value); }
    finish_array();
  }

  /** Write an std::array<T, N> */
  template<typename T, std::size_t N>
  void write(const std::array<T, N> & a)
  {
    start_array(N);
    for(const auto & value : a) { write(value); }
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
    start_map(m.size());
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
    for(const auto & value : s) { write(value); }
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

  /** Write an std::variant<Args...> */
  template<typename... Args>
  void write(const std::variant<Args...> & value)
  {
    start_array(2);
    write(value.index());
    std::visit([this](const auto & v) { write(v); }, value);
    finish_array();
  }

  /** Write an Eigen::Ref */
  template<typename Type, int Options, typename StrideType>
  void write(const Eigen::Ref<Type, Options, StrideType> & v)
  {
#if !EIGEN_VERSION_AT_LEAST(3, 2, 90)
    using Index = Eigen::DenseIndex;
#else
    using Index = Eigen::Index;
#endif
    start_array(v.size());
    for(Index i = 0; i < v.size(); ++i) { write(v(i)); }
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
