#include <mc_rtc/DataStore.h>

#ifdef __GNUG__

#  include <cstdlib>
#  include <cxxabi.h>
#  include <memory>

#endif

namespace mc_rtc
{

namespace internal
{

#ifdef __GNUG__

std::string demangle(const char * name)
{
  int status = 0;

  std::unique_ptr<char, void (*)(void *)> res{abi::__cxa_demangle(name, NULL, NULL, &status), std::free};

  return status == 0 ? res.get() : name;
}

#else

std::string demangle(const char * name)
{
  return name;
}

#endif

} // namespace internal

} // namespace mc_rtc
