#pragma once

#include <mc_control/api.h>
#include <mc_tasks/MetaTask.h>

namespace mc_control
{

/** A generic MetaTask completion criteria
 *
 * Possible criteria:
 * - eval() norm is below a certain threshold
 * - speed() norm is below a certain threshold
 * - a given timeout (provided in seconds) has elapsed since the start
 *
 * The default criteria is always satisfied (i.e. no criteria)
 *
 * These criteria can be combined with AND and OR
 *
 * Example:
 *
 * \code{.json}
 * # 5 seconds timeout
 * {
 *   "timeout": 5.0
 * }
 * # eval().norm() < 1e-3 || timeout 20s
 * {
 *   "OR": [
 *    { "eval": 1e-3 },
 *    { "timeout": 20.0 }
 *   ]
 * }
 * # ( eval().norm() < 1e-3 && speed().norm() < 1e-6 ) || timeout 30s
 * {
 *   "OR": [
 *    {
 *      "AND": [
 *        { "eval": 1e-3 },
 *        { "speed": 1e-6 }
 *      ]
 *    },
 *    { "timeout": 30.0 }
 *   ]
 * }
 * \endcode
 *
 */
struct MC_CONTROL_DLLAPI CompletionCriteria
{
  /** Given a task returns true if that task fits the completion criteria */
  bool completed(const mc_tasks::MetaTask & task);

  /** Given a MetaTask and a Configuration, generate the criteria */
  void configure(const mc_tasks::MetaTask & task, double dt, const mc_rtc::Configuration & config);

  /** Returns the criteria that achieved the completion */
  const std::string & output() const;

private:
  /** Internally used to compose functions */
  std::function<bool(const mc_tasks::MetaTask &, std::string &)> build(const mc_tasks::MetaTask & task,
                                                                       double dt,
                                                                       const mc_rtc::Configuration & config);
  /** Programatically constructed based on the configuration values */
  std::function<bool(const mc_tasks::MetaTask &, std::string &)> fn_ = [](const mc_tasks::MetaTask &, std::string &) {
    return true;
  };
  /** Output string */
  std::string output_;
};

} // namespace mc_control
