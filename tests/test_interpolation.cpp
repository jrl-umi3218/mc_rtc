
/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_trajectory/SequentialInterpolator.h>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

BOOST_AUTO_TEST_CASE(TestSequentialInterpolator)
{
  constexpr double epsilon = 1e-10;
  using namespace mc_trajectory;
  LinearInterpolation<double> interpolator;
  using SequentialInterpolator = SequentialInterpolator<double, LinearInterpolation<double>>;

  using values_t = std::vector<std::pair<double, double>>;
  auto values = values_t{{0., 0.}, {1., 1.}, {2., 2.}};

  BOOST_REQUIRE_THROW(SequentialInterpolator{{}}, std::runtime_error);
  BOOST_REQUIRE_NO_THROW(SequentialInterpolator{values});
  SequentialInterpolator interpolation{values};
  double dt = 0.005;
  // Check interpolation in first interval
  int i = 0;
  for(; i < 1 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(dt), interpolator(0, 1, i * dt), epsilon);
  }
  // Check interpolation in second interval
  for(; i < 2 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(dt), interpolator(0, 1, i * dt), epsilon);
  }
  // Check access out of bounds (returns last value)
  for(; i < 3 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(dt), 2., epsilon);
  }

  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1, 0}}), std::runtime_error);
  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {2, 0}}), std::runtime_error);

  // Change values
  values = values_t{{0.5, 0.5}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  BOOST_REQUIRE_THROW(interpolation.values({{3.5, 0.5}, {1., 1.}, {2., 2.}}), std::runtime_error);
  BOOST_REQUIRE_CLOSE(interpolation.time(), 0.5, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(dt), 0.5, epsilon);

  BOOST_REQUIRE_NO_THROW(interpolation.values({{1.2, 2.3}}));
  BOOST_REQUIRE_CLOSE(interpolation.time(), 1.2, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(dt), 2.3, epsilon);
}
