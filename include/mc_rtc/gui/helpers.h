#pragma once

#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/type_name.h>

namespace mc_rtc
{
namespace gui
{

#define MC_RTC_INPUT_LAMBDAS_REF(VARIABLE)                                                        \
  [&VARIABLE]() -> typename std::remove_reference<decltype(VARIABLE)>::type { return VARIABLE; }, \
      [&VARIABLE](const typename std::remove_reference<decltype(VARIABLE)>::type & newValue) { VARIABLE = newValue; }

/**
 * @brief Creates a Checkbox element
 *
 * @param name Name of the element
 * @param variable Reference to the boolean managed by this element. It is the
 * users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return A Checkbox element controlling the state of "variable"
 */
inline auto make_checkbox(std::string name, bool & variable)
{
  return CheckboxImpl(
      name, [&variable]() { return variable; }, [&variable]() { variable = !variable; });
}

template<typename T>
inline auto make_label(std::string name, const T & ref)
{
  return LabelImpl(name, [&ref]() { return ref; });
}

template<typename T>
inline auto make_label(std::string name, T && label)
{
  return Label(name, [label]() { return label; });
}

template<typename T>
inline auto make_array_label(std::string name, const T & ref)
{
  return ArrayLabelImpl(name, [&ref]() { return ref; });
}

template<typename T>
inline auto make_array_label(std::string name, T && ref)
{
  return ArrayLabelImpl(name, [ref]() { return ref; });
}

template<typename T>
inline auto make_array_label(std::string name, const std::vector<std::string> & labels, T & ref)
{
  return ArrayLabelImpl(name, labels, [ref]() { return ref; });
}

template<typename T>
inline auto make_array_label(std::string name, const std::vector<std::string> & labels, const T & ref)
{
  return ArrayLabelImpl(name, labels, [&ref]() { return ref; });
}

/**
 * @brief Creates a NumberInput element
 *
 * @param name Name of the element
 * @param value Reference to the input value managed by this element. It is the
 * users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.

 *
 * @return A NumberInput element for "value"
 */
template<typename T>
inline auto make_number_input(std::string name, T & value)
{
  mc_rtc::log::info("TYPE IS: {}", mc_rtc::type_name<T>());
  return NumberInputImpl(
      name, [&value]() -> T & { return value; }, [&value](const T & newValue) { value = newValue; });
}

inline auto make_number_input(std::string name, int & value)
{
  return IntegerInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(value));
}

template<typename T>
inline auto make_array_input(std::string name, T & value)
{
  return ArrayInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(value));
}

template<typename T>
inline auto make_array_input(std::string name, const std::vector<std::string> & labels, T & value)
{
  return ArrayInputImpl(name, labels, MC_RTC_INPUT_LAMBDAS_REF(value));
}

inline auto make_combo_input(const std::string & name,
                             const std::vector<std::string> & comboValues,
                             std::string & value)
{
  return ComboInputImpl(name, comboValues, MC_RTC_INPUT_LAMBDAS_REF(value));
}

template<typename T>
inline auto make_number_slider(std::string name, T & value, double min, double max)
{
  return NumberSliderImpl(name, MC_RTC_INPUT_LAMBDAS_REF(value), min, max);
}

template<typename T>
inline auto make_string_input(std::string name, T & value)
{
  return StringInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(value));
}

inline auto make_rpy_input(std::string name, Eigen::Matrix3d & orientation)
{
  return ArrayInputImpl(
      name, {"r [deg]", "p [deg]", "y [deg]"},
      [&orientation]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI; },
      [&orientation](const Eigen::Vector3d & rpy) {
        orientation = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180);
      });
}

inline auto make_motionvecd_input(std::string name, sva::MotionVecd & vel)
{
  return ArrayInputImpl(name, {"wx [rad/s]", "wy [rad/s]", "wz [rad/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
                        MC_RTC_INPUT_LAMBDAS_REF(vel));
}

inline auto make_admittancevecd_input(std::string name, sva::AdmittanceVecd & vel)
{
  return ArrayInputImpl(
      name, {"wx", "wy", "wz", "vx", "vy", "vz"}, [&vel]() -> Eigen::Vector6d { return vel.vector(); },
      [&vel](const Eigen::Vector6d & newVel) { vel = newVel; });
}

inline auto make_rpy_label(std::string name, const Eigen::Matrix3d & orientation)
{
  return ArrayLabel(name, {"r [deg]", "p [deg]", "y [deg]"}, [&orientation]() -> Eigen::Vector3d {
    return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI;
  });
}

inline auto make_rpy_label(std::string name, const Eigen::Vector3d & rpy)
{
  return ArrayLabel(name, {"r [deg]", "p [deg]", "y [deg]"},
                    [&rpy]() -> Eigen::Vector3d { return rpy * 180. / mc_rtc::constants::PI; });
}

inline auto make_input_element(const std::string & name, sva::MotionVecd & ref)
{
  return make_motionvecd_input(name, ref);
}

inline auto make_input_element(const std::string & name, sva::AdmittanceVecd & ref)
{
  return make_admittancevecd_input(name, ref);
}

// Arrays
inline auto make_input_element(const std::string & name, Eigen::Vector2d & ref)
{
  return make_array_input(name, ref);
}

inline auto make_input_element(const std::string & name, Eigen::Vector3d & ref)
{
  return make_array_input(name, ref);
}

inline auto make_input_element(const std::string & name, Eigen::Vector6d & ref)
{
  return make_array_input(name, ref);
}

inline auto make_input_element(const std::string & name, Eigen::VectorXd & ref)
{
  return make_array_input(name, ref);
}

inline auto make_input_element(const std::string & name, const std::vector<std::string> & labels, Eigen::Vector2d & ref)
{
  return make_array_input(name, labels, ref);
}

inline auto make_input_element(const std::string & name, const std::vector<std::string> & labels, Eigen::Vector3d & ref)
{
  return make_array_input(name, labels, ref);
}

inline auto make_input_element(const std::string & name, const std::vector<std::string> & labels, Eigen::Vector6d & ref)
{
  return make_array_input(name, labels, ref);
}

inline auto make_input_element(const std::string & name, const std::vector<std::string> & labels, Eigen::VectorXd & ref)
{
  return make_array_input(name, labels, ref);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & comboValues,
                               std::string & comboSelection)
{
  return make_combo_input(name, comboValues, comboSelection);
}

template<typename T, typename std::enable_if<mc_rtc::gui::details::is_array_or_vector<T>::value, int>::type = 0>
inline auto make_input_element(const std::string & name, T & ref)
{
  return make_array_input(name, ref);
}

inline auto make_input_element(const std::string & name, std::string & ref)
{
  return make_string_input(name, ref);
}

inline auto make_input_element(const std::string & name, bool & ref)
{
  return make_checkbox(name, ref);
}

template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
inline auto make_input_element(const std::string & name, T & ref)
{
  return make_number_input(name, ref);
}

} // namespace gui

} // namespace mc_rtc
