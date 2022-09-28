/**
 * @file
 * @brief Provides helper functions to simplify building of gui elements
 *
 * Provides helper functions simplifying the creation of common GUI elements.
 * For example
 * - mc_rtc::gui::make_input_element(std::string, T & var) : creates an input
 *   element that displays/modifies the value of "var". The appropriate element
 *   is automatically chosen based on the type of T
 *
 * For ease of use and readability in the GUI, angular quantities are expressed in degrees by default. If you prefer
 * displays in radians, *_rad variants are provided where appropriate.
 */

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
 * @param variable Reference to the boolean managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return A Checkbox element controlling the state of "variable"
 */
inline auto make_checkbox(std::string name, bool & variable)
{
  return CheckboxImpl(
      name, [&variable]() { return variable; }, [&variable]() { variable = !variable; });
}

/**
 * @brief Creates a Label element
 *
 * @tparam T Any type supported by mc_rtc::gui::Label
 * @param name Name of the element
 * @param variable Reference to the data displayed by this element
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return A Label element diplaying the value of "variable"
 */
template<typename T>
inline auto make_label(std::string name, const T & variable)
{
  return LabelImpl(name, [&variable]() { return variable; });
}

/**
 * @brief Creates a fixed Label element
 *
 * @tparam T Any type supported by mc_rtc::gui::Label
 * @param name Name of the element
 * @param label Fixed label displayed by this element (cannot be changed
 * afterwards).
 *
 * @return A Label element diplaying the value of "label"
 */
template<typename T>
inline auto make_label(std::string name, T && label)
{
  return Label(name, [label]() { return label; });
}

/**
 * @brief Creates an ArrayLabel element
 *
 * @tparam T Any type supported by mc_rtc::gui::ArrayLabel
 * @param name Name of the element
 * @param variable Reference to the data displayed by this element
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the element is active in the GUI.
 *
 * @return An ArrayLabel element diplaying the value of "variable"
 */
template<typename T>
inline auto make_array_label(std::string name, const T & variable)
{
  return ArrayLabelImpl(name, [&variable]() { return variable; });
}

/**
 * @brief Creates an ArrayLabel element
 *
 * @tparam T Any type supported by mc_rtc::gui::ArrayLabel
 * @param name Name of the element
 * @param label Fixed label displayed by this element (cannot be changed afterwards).
 *
 * @return An ArrayLabel element diplaying the value of "label"
 */
template<typename T>
inline auto make_array_label(std::string name, T && label)
{
  return ArrayLabelImpl(name, [label]() { return label; });
}

/**
 * @brief Creates an ArrayLabel element
 *
 * @tparam T Any type supported by mc_rtc::gui::ArrayLabel
 * @param name Name of the element
 * @param labels Label text for each element in the array. This must have the
 * same size as "variable".
 * @param variable Reference to the data displayed by this element
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayLabel element diplaying the value of "variable"
 */
template<typename T>
inline auto make_array_label(std::string name, const std::vector<std::string> & labels, const T & variable)
{
  return ArrayLabelImpl(name, labels, [&variable]() { return variable; });
}

/**
 * @brief Creates an ArrayLabel element
 *
 * @tparam T Any type supported by mc_rtc::gui::ArrayLabel
 * @param name Name of the element
 * @param labels Label text for each element in the array. This must have the
 * same size as "variable".
 * @param label Fixed label (cannot be changed afterwards)
 *
 * @return An ArrayLabel element diplaying the value of "label"
 */
template<typename T>
inline auto make_array_label(std::string name, const std::vector<std::string> & labels, T && label)
{
  return ArrayLabelImpl(name, labels, [label]() { return label; });
}

/**
 * @brief Creates a NumberInput element
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.

 * @return A NumberInput element for "variable"
 */
template<typename T>
inline auto make_number_input(std::string name, T & variable)
{
  return NumberInputImpl(
      name, [&variable]() -> T & { return variable; }, [&variable](const T & newValue) { variable = newValue; });
}

/**
 * @brief Specialization of make_number_input for IntegerInput
 */
inline auto make_number_input(std::string name, int & variable)
{
  return IntegerInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(variable));
}

/**
 * @brief Creates an ArrayInput element
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.

 * @return An ArrayInput element for "variable"
 */
template<typename T>
inline auto make_array_input(std::string name, T & variable)
{
  return ArrayInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(variable));
}

/**
 * @brief Creates an ArrayInput element
 *
 * @param name Name of the element
 * @param labels Label text for each element in the array. This must have the same size as "variable".
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.

 * @return An ArrayInput element for "variable"
 */
template<typename T>
inline auto make_array_input(std::string name, const std::vector<std::string> & labels, T & variable)
{
  return ArrayInputImpl(name, labels, MC_RTC_INPUT_LAMBDAS_REF(variable));
}

/**
 * @brief Creates a ComboInput element
 *
 * @param name Name of the element
 * @param comboValues Values that the ComboBox can take
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.

 * @return A ComboInput element for "variable"
 */
inline auto make_combo_input(const std::string & name,
                             const std::vector<std::string> & comboValues,
                             std::string & variable)
{
  return ComboInputImpl(name, comboValues, MC_RTC_INPUT_LAMBDAS_REF(variable));
}

/**
 * @brief Creates a NumberSlider element
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @param min Min value of the slider
 * @param max Max value of the slider

 * @return A NumberSlider element for "variable"
 */
template<typename T>
inline auto make_number_slider(std::string name, T & variable, double min, double max)
{
  return NumberSliderImpl(name, MC_RTC_INPUT_LAMBDAS_REF(variable), min, max);
}

/**
 * @brief Creates a StringInput element
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return A StringInput element for "variable"
 */
template<typename T>
inline auto make_string_input(std::string name, T & variable)
{
  return StringInputImpl(name, MC_RTC_INPUT_LAMBDAS_REF(variable));
}

/**
 * @brief Creates an ArrayInput element displaying orientation as RPY in degrees
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayInput element for "variable"
 */
inline auto make_rpy_input_deg(std::string name, Eigen::Matrix3d & orientation)
{
  return ArrayInputImpl(
      name, {"r [deg]", "p [deg]", "y [deg]"},
      [&orientation]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI; },
      [&orientation](const Eigen::Vector3d & rpy) {
        orientation = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180);
      });
}

/**
 * @brief Creates an ArrayInput element displaying orientation as RPY in radians
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayInput element for "variable"
 */
inline auto make_rpy_input_rad(std::string name, Eigen::Matrix3d & orientation)
{
  return ArrayInputImpl(
      name, {"r [rad]", "p [rad]", "y [rad]"},
      [&orientation]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(orientation); },
      [&orientation](const Eigen::Vector3d & rpy) { orientation = mc_rbdyn::rpyToMat(rpy); });
}

/**
 * @see make_rpy_input_deg
 */
inline auto make_rpy_input(std::string name, Eigen::Matrix3d & orientation)
{
  return make_rpy_input_deg(name, orientation);
}

/**
 * @brief Creates an ArrayLabel element displaying orientation as RPY in radians
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayLabel element for "variable"
 */
inline auto make_rpy_label_rad(std::string name, const Eigen::Matrix3d & orientation)
{
  return ArrayLabel(name, {"r [rad]", "p [rad]", "y [rad]"},
                    [&orientation]() -> Eigen::Vector3d { return mc_rbdyn::rpyFromMat(orientation); });
}

/**
 * @brief Creates an ArrayLabel element displaying orientation as RPY in degrees
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayLabel element for "variable"
 * @see make_rpy_label
 */
inline auto make_rpy_label_deg(std::string name, const Eigen::Matrix3d & orientation)
{
  return ArrayLabel(name, {"r [deg]", "p [deg]", "y [deg]"}, [&orientation]() -> Eigen::Vector3d {
    return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI;
  });
}

/**
 * @see make_rpy_label_deg
 */
inline auto make_rpy_label(std::string name, const Eigen::Matrix3d & orientation)
{
  return make_rpy_label_deg(name, orientation);
}

/**
 * @brief Creates an ArrayLabel element displaying orientation as RPY in radians
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayLabel element for "variable"
 */
inline auto make_rpy_label_rad(std::string name, const Eigen::Vector3d & rpy)
{
  return ArrayLabel(name, {"r [rad]", "p [rad]", "y [rad]"}, [&rpy]() -> Eigen::Vector3d { return rpy; });
}

/**
 * @brief Creates an ArrayLabel element displaying orientation as RPY in degrees
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayLabel element for "variable"
 * @see make_rpy_label
 */
inline auto make_rpy_label_deg(std::string name, const Eigen::Vector3d & rpy)
{
  return ArrayLabel(name, {"r [deg]", "p [deg]", "y [deg]"},
                    [&rpy]() -> Eigen::Vector3d { return rpy * 180. / mc_rtc::constants::PI; });
}

/**
 * @see make_rpy_label_deg
 */
inline auto make_rpy_label(std::string name, const Eigen::Vector3d & rpy)
{
  return make_rpy_label_deg(name, rpy);
}

/**
 * @brief Creates an ArrayInput element displaying MotionVecd (angular velocity
 * in radians/s, linear velocity in m/s)
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayInput element for "variable"
 */
inline auto make_motionvecd_input_rad(std::string name, sva::MotionVecd & vel)
{
  return ArrayInputImpl(name, {"wx [rad/s]", "wy [rad/s]", "wz [rad/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
                        MC_RTC_INPUT_LAMBDAS_REF(vel));
}

/**
 * @brief Creates an ArrayInput element displaying MotionVecd (angular velocity
 * in radians/s, linear velocity in m/s)
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayInput element for "variable"
 */
inline auto make_motionvecd_input_deg(std::string name, sva::MotionVecd & vel)
{
  return ArrayInputImpl(
      name, {"wx [deg/s]", "wy [deg/s]", "wz [deg/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
      [&vel]() {
        return sva::MotionVecd{vel.angular() * 180 / mc_rtc::constants::PI, vel.linear()};
      },
      [&vel](const sva::MotionVecd & newVel) {
        vel.angular() = newVel.angular() * mc_rtc::constants::PI / 180;
        vel.linear() = newVel.linear();
      });
}

/**
 * @see make_motionvecd_input_deg
 **/
inline auto make_motionvecd_input(std::string name, sva::MotionVecd & vel)
{
  return make_motionvecd_input_deg(name, vel);
}

/**
 * @brief Creates an ArrayInput element displaying an AdmittanceVecd with
 * admittance labels
 *
 * @param name Name of the element
 * @param variable Reference to the input variable managed by this element.
 * @warning It is the users responsibility to ensure that this reference remains valid as long as
 * the checkbox is active in the GUI.
 *
 * @return An ArrayInput element for "variable"
 */
inline auto make_admittancevecd_input(std::string name, sva::AdmittanceVecd & vel)
{
  return ArrayInputImpl(
      name, {"wx", "wy", "wz", "vx", "vy", "vz"}, [&vel]() -> Eigen::Vector6d { return vel.vector(); },
      [&vel](const Eigen::Vector6d & newVel) { vel = newVel; });
}

/**
 * @brief Creates an input element for the provided type
 * @warning The provided reference must remain valid as long as the element is in the GUI.
 */
///@{
inline auto make_input_element(const std::string & name, sva::MotionVecd & variable)
{
  return make_motionvecd_input(name, variable);
}

inline auto make_input_element(const std::string & name, sva::AdmittanceVecd & variable)
{
  return make_admittancevecd_input(name, variable);
}

// Arrays
inline auto make_input_element(const std::string & name, Eigen::Vector2d & variable)
{
  return make_array_input(name, variable);
}

inline auto make_input_element(const std::string & name, Eigen::Vector3d & variable)
{
  return make_array_input(name, variable);
}

inline auto make_input_element(const std::string & name, Eigen::Vector6d & variable)
{
  return make_array_input(name, variable);
}

inline auto make_input_element(const std::string & name, Eigen::VectorXd & variable)
{
  return make_array_input(name, variable);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & labels,
                               Eigen::Vector2d & variable)
{
  return make_array_input(name, labels, variable);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & labels,
                               Eigen::Vector3d & variable)
{
  return make_array_input(name, labels, variable);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & labels,
                               Eigen::Vector6d & variable)
{
  return make_array_input(name, labels, variable);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & labels,
                               Eigen::VectorXd & variable)
{
  return make_array_input(name, labels, variable);
}

inline auto make_input_element(const std::string & name,
                               const std::vector<std::string> & comboValues,
                               std::string & comboSelection)
{
  return make_combo_input(name, comboValues, comboSelection);
}

template<typename T, typename std::enable_if<mc_rtc::gui::details::is_array_or_vector<T>::value, int>::type = 0>
inline auto make_input_element(const std::string & name, T & variable)
{
  return make_array_input(name, variable);
}

inline auto make_input_element(const std::string & name, std::string & variable)
{
  return make_string_input(name, variable);
}

inline auto make_input_element(const std::string & name, bool & variable)
{
  return make_checkbox(name, variable);
}

template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
inline auto make_input_element(const std::string & name, T & variable)
{
  return make_number_input(name, variable);
}
///@}

} // namespace gui

} // namespace mc_rtc
