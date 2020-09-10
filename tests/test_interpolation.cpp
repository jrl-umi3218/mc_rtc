
/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_trajectory/SequentialInterpolator.h>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

BOOST_AUTO_TEST_CASE(TestSequentialInterpolator)
{
  using namespace mc_trajectory;
  SequentialInterpolator<double, LinearInterpolation<double>> interpolation(0.005);
  LinearInterpolation<double> interpolator;
  using values_t = std::vector<std::pair<double, double>>;
  auto values = values_t{{0., 0.}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_THROW(interpolation.values(values_t{}), std::runtime_error);
  BOOST_REQUIRE_THROW(interpolation.values(values_t{1}), std::runtime_error);
  BOOST_REQUIRE_THROW(interpolation.values(values_t{2}), std::runtime_error);
  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1, 0}}), std::runtime_error);
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  // Check interpolation in first interval
  int i = 0;
  for(; i < 1 / 0.005; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * 0.005, 1e-10);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), interpolator(0, 1, i * 0.005), 1e-10);
  }
  // Check interpolation in second interval
  for(; i < 2 / 0.005; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * 0.005, 1e-10);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), interpolator(0, 1, i * 0.005), 1e-10);
  }
  // Check access out of bounds (returns last value)
  for(; i < 3 / 0.005; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * 0.005, 1e-10);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), 2., 1e-10);
  }
}
