
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
  constexpr double dt = 0.005;

  BOOST_REQUIRE_THROW(SequentialInterpolator(dt).compute(), std::runtime_error);
  BOOST_REQUIRE_THROW(SequentialInterpolator(dt, {}).compute(), std::runtime_error);
  BOOST_REQUIRE(!SequentialInterpolator(dt).hasValues());
  BOOST_REQUIRE_NO_THROW(SequentialInterpolator(dt, values));
  SequentialInterpolator interpolation{dt, values};
  // Check interpolation in first interval
  int i = 0;
  for(; i < 1 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), interpolator(0, 1, i * dt), epsilon);
  }
  // Check interpolation in second interval
  for(; i < 2 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), interpolator(0, 1, i * dt), epsilon);
  }
  // Check access out of bounds (returns last value)
  for(; i < 3 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.time(), i * dt, epsilon);
    BOOST_REQUIRE_CLOSE(interpolation.compute(), 2., epsilon);
  }

  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1.999, 0}}),
                      std::runtime_error); // values not correctly ordered
  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1.5, 0}}),
                      std::runtime_error); // values not correctly ordered
  BOOST_REQUIRE_THROW(interpolation.values({{0.5, 0.5}, {0.5, 0.5}}),
                      std::runtime_error); // twice the same initial time
  BOOST_REQUIRE_THROW(interpolation.values({{0.5, 0.5}, {0.5 + dt / 2., 0.5}}),
                      std::runtime_error); // values spread less than dt apart

  // Change values
  values = values_t{{0.5, 0.5}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  BOOST_REQUIRE_THROW(interpolation.values({{3.5, 0.5}, {1., 1.}, {2., 2.}}), std::runtime_error);
  BOOST_REQUIRE_CLOSE(interpolation.time(), 0.5, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(), 0.5, epsilon);

  BOOST_REQUIRE_NO_THROW(interpolation.values({{1.2, 2.3}}));
  BOOST_REQUIRE_CLOSE(interpolation.time(), 1.2, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(), 2.3, epsilon);
}
