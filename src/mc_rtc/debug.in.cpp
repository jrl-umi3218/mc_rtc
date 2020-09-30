#include <mc_rtc/debug.h>

/* !!! WARNING !!!
 * debug.cpp is a generated file
 * if you wish to change this file, edit:
 * @PROJECT_SOURCE_DIR@/src/mc_rtc/debug.in.cpp
 */

namespace mc_rtc
{

bool debug() noexcept
{
  // clang-format off
  return $<IF:$<CONFIG:debug>,true,false>;
  // clang-format on
}

} // namespace mc_rtc
