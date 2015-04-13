#include "MCControlServiceSVC_impl.h"

/* Trick to disable compiler warning on this part of the code */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCControlServiceSK.cc"
#pragma GCC diagnostic pop

#include <iostream>

namespace OpenHRP
{

MCControlServiceSVC_impl::MCControlServiceSVC_impl()
{
}

MCControlServiceSVC_impl::~MCControlServiceSVC_impl()
{
}

void MCControlServiceSVC_impl::place_holder()
{
  std::cout << "Yup that's working" << std::endl;
}

}
