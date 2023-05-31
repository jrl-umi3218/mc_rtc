#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/StateBuilder.h>

namespace mc_rtc::schema
{

namespace details
{

/** Helper to pass otherwise non deducible template parameters to a constructor */
template<auto ptr>
struct MemberPointerWrapper
{
};

/** Helper when a Schema value has a finite number of choices */
template<typename T, bool HasChoices>
struct Choices
{
  Choices() = default;
  Choices(const std::vector<std::string> & choices) : choices(choices) {}
  std::vector<T> choices;
};

template<typename T>
struct Choices<T, false>
{
  Choices() = default;
};

/** Helper to get a default value for a given type */
template<typename T, typename Enable = void>
struct Default
{
  static_assert(!std::is_same_v<T, T>, "Must be specialized");
};

template<typename T>
struct Default<T, std::enable_if_t<std::is_arithmetic_v<T>>>
{
  inline static constexpr T value = 0;
};

template<typename Scalar, int N, int _Options, int _MaxRows, int _MaxCols>
struct Default<Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols>, std::enable_if_t<(N > 0)>>
{
  inline static const Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols> value =
      Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols>::Zero();
};

template<typename Scalar, int N, int _Options, int _MaxRows, int _MaxCols>
struct Default<Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols>, std::enable_if_t<(N > 1)>>
{
  inline static const Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols> value =
      Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols>::Identity();
};

template<>
struct Default<sva::PTransformd>
{
  inline static const sva::PTransformd value = sva::PTransformd::Identity();
};

template<>
struct Default<sva::MotionVecd>
{
  inline static const sva::MotionVecd value = sva::MotionVecd::Zero();
};

template<>
struct Default<sva::ForceVecd>
{
  inline static const sva::ForceVecd value = sva::ForceVecd::Zero();
};

template<>
struct Default<sva::ImpedanceVecd>
{
  inline static const sva::ImpedanceVecd value = sva::ImpedanceVecd::Zero();
};

template<>
struct Default<sva::AdmittanceVecd>
{
  inline static const sva::AdmittanceVecd value = sva::AdmittanceVecd::Zero();
};

template<>
struct Default<std::string>
{
  inline static const std::string value = "";
};

} // namespace details

/** Operations implemented by an object that can be represented with a Schema */
struct MC_RTC_UTILS_DLLAPI Operations
{
  /** Save to a configuration object */
  std::function<void(const void * self, Configuration & out)> save = [](const void *, Configuration &) {};

  /** Load from a configuration object */
  std::function<void(void * self, const Configuration & in)> load = [](void *, const Configuration &) {};

  using StandardForm = gui::details::FormImpl<std::function<void(const Configuration &)>>;

  /** Build a Form to load the object */
  std::function<void(const void * self, StandardForm & form)> buildForm = [](const void *, StandardForm &) {};

  /** Load from a Form */
  std::function<void(void * self, const Configuration & in)> loadForm = [](void *, const Configuration &) {};

  /** FIXME Add a saveSchema function */

  /** FIXME Add logging operation?*/
};

/** A simple value in a schema
 *
 * This comprises of:
 * - a value of type \tparam T
 * - an indentifier/name given at construction
 * - a description given at construction
 */
template<typename T>
struct alignas(T) Value
{
  /** Create the value
   *
   * \tparam Schema Schema this value belongs to
   *
   * \tparam ptr Member pointer in the Schema object
   *
   * \tparam IsRequired If true the value is required in the form
   *
   * \tparam HasChoices If true, we expect choices to be provided for the value instead of an open form
   *
   * \param ops Operations that we should setup
   *
   * \param name Name of the value
   *
   * \param description Description for the value
   *
   * \param default_ Default value
   *
   * \param required Pass the IsRequired value
   *
   * \param combo Possible choices when \tparam HasChoices is true
   */
  template<typename Schema, Value<T> Schema::*ptr, bool IsRequired = true, bool HasChoices = false>
  Value(Operations & ops,
        details::MemberPointerWrapper<ptr>,
        const std::string & name,
        const std::string & description,
        const T & default_ = details::Default<T>::value,
        const std::bool_constant<IsRequired> & = {},
        const details::Choices<T, HasChoices> & choices = {})
  : value_(default_)
  {
    ops.save = [save = ops.save, name](const void * self, mc_rtc::Configuration & out)
    {
      save(self, out);
      out.add(name, (static_cast<const Schema *>(self)->*ptr).operator const T &());
    };
    ops.load = [load = ops.load, name](void * self, const mc_rtc::Configuration & in)
    {
      load(self, in);
      (static_cast<Schema *>(self)->*ptr).operator T &() = in(name).operator T();
    };
    ops.loadForm = [loadForm = ops.loadForm, description](void * self, const mc_rtc::Configuration & in)
    {
      loadForm(self, in);
      T & value = static_cast<Schema *>(self)->*ptr;
      if(IsRequired || in.has(description)) { value = in(description).operator T(); }
    };
    ops.buildForm =
        [buildForm = ops.buildForm, description, choices](const void * self, Operations::StandardForm & form)
    {
      buildForm(self, form);
      const T & value = static_cast<Schema *>(self)->*ptr;
      auto get_value = [&value]() -> const T & { return value; };
      if constexpr(std::is_same_v<T, bool>)
      {
        form.addElement(mc_rtc::gui::FormCheckbox(description, IsRequired, get_value));
      }
      else if constexpr(std::is_integral_v<T>)
      {
        form.addElement(mc_rtc::gui::FormIntegerInput(description, IsRequired, get_value));
      }
      else if constexpr(std::is_floating_point_v<T>)
      {
        form.addElement(mc_rtc::gui::FormNumberInput(description, IsRequired, get_value));
      }
    };
  }

  inline operator T &() noexcept { return value_; }

  inline operator const T &() const noexcept { return value_; }

  T value_;
};

} // namespace mc_rtc::schema
