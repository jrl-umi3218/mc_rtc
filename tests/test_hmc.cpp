/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/LookupTable.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

template<typename Fun>
void testLookupTable(Fun f, double min, double max, size_t resolution, size_t nbTest, double precision = 1e-6)
{
  using namespace mc_planning;
  LookupTable<double> lookupTable(resolution, min, max, f);

  double oldrange = static_cast<double>(nbTest);
  double newrange = max - min;
  for(size_t i = 0; i < nbTest; ++i)
  {
    double v = ((static_cast<double>(i) * newrange) / oldrange) + min;
    BOOST_REQUIRE_CLOSE(lookupTable(v), f(v), precision);
  }
}

BOOST_AUTO_TEST_CASE(TestLookupTable)
{

  {
    // Test cosh loopup table in a realistic constext.
    //
    // Creates a lookup table from the value of omega to
    // both cosh(w * dt) and sinh(w * dt)
    //
    // The range is chosen to be similar to that encountered for a humanoid
    // robot (from 0.001m to 3m height)
    auto omega = [](double h) { return std::sqrt(mc_rtc::constants::GRAVITY / h); };
    double dt = 0.005;
    testLookupTable([dt](double x) { return cosh(x) * dt; }, omega(2.5), omega(0.01), 100000, 100000);
    testLookupTable([](double x) { return sinh(x); }, omega(2.5), omega(0.01), 100000, 100000);
  }

  {
    // Other tests for loopup table
    testLookupTable([](double x) { return cosh(x); }, -2., 3., 20000, 500);

    // Test lookuptable for sinh
    testLookupTable([](double x) { return sinh(x); }, 0., 3., 20000, 500);
    testLookupTable([](double x) { return sinh(x); }, -1.5, 3., 20000, 500);

    testLookupTable([](double x) { return sqrt(x); }, 0., 1000., 2000, 500);
  }
}
