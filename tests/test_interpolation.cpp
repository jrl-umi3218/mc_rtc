
/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

BOOST_AUTO_TEST_CASE(TestSequenceInterpolator)
{
  constexpr double epsilon = 1e-10;
  using namespace mc_trajectory;
  LinearInterpolation<double> interpolator;
  using SequenceInterpolator = SequenceInterpolator<double, LinearInterpolation<double>>;

  using values_t = std::vector<std::pair<double, double>>;
  auto values = values_t{{0., 0.}, {1., 1.}, {2., 2.}};
  constexpr double dt = 0.005;

  BOOST_REQUIRE_THROW(SequenceInterpolator{}.compute(0), std::runtime_error);
  BOOST_REQUIRE_THROW(SequenceInterpolator{{}}.compute(0), std::runtime_error);
  BOOST_REQUIRE(!SequenceInterpolator{}.hasValues());
  BOOST_REQUIRE_NO_THROW(SequenceInterpolator{values});
  SequenceInterpolator interpolation{values};
  // Check interpolation in first interval
  int i = 0;
  double time = 0;
  for(; i < 1 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.compute(time), interpolator(0, 1, time), epsilon);
    time += dt;
  }
  // Check interpolation in second interval
  for(; i < 2 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.compute(time), interpolator(1, 2, time - 1), epsilon);
    time += dt;
  }
  // Check access out of bounds (returns last value)
  for(; i < 3 / dt; i++)
  {
    BOOST_REQUIRE_CLOSE(interpolation.compute(time), 2., epsilon);
    time += dt;
  }

  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1.999, 0}}),
                      std::runtime_error); // values not correctly ordered
  BOOST_REQUIRE_THROW(interpolation.values(values_t{{2, 0}, {1.5, 0}}),
                      std::runtime_error); // values not correctly ordered

  // Change values
  values = values_t{{0.5, 0.5}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  BOOST_REQUIRE_THROW(interpolation.values({{3.5, 0.5}, {1., 1.}, {2., 2.}}), std::runtime_error);
  time = 0.5;
  BOOST_REQUIRE_CLOSE(interpolation.compute(time), 0.5, epsilon);

  BOOST_REQUIRE_NO_THROW(interpolation.values({{1.2, 2.3}}));
  time = 1.2;
  BOOST_REQUIRE_CLOSE(interpolation.compute(time), 2.3, epsilon);

  // test starting at second interval
  values = values_t{{0.0, 0.0}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  BOOST_REQUIRE_CLOSE(interpolation.compute(1.2), interpolator(1, 2, 0.2), epsilon);

  // test starting at any time
  values = values_t{{0.0, 0.0}, {1., 1.}, {2., 2.}};
  BOOST_REQUIRE_NO_THROW(interpolation.values(values));
  BOOST_REQUIRE_CLOSE(interpolation.compute(1.2), interpolator(1, 2, 0.2), epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(1.3), interpolator(1, 2, 0.3), epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(0.5), interpolator(0, 1, 0.5), epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(2.00001), 2.0, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(-2.00001), 0.0, epsilon);

  // single value
  interpolation.values({{1., 1.}});
  BOOST_REQUIRE_CLOSE(interpolation.compute(0), 1., epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(1), 1., epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(2), 1., epsilon);

  // negative value
  BOOST_REQUIRE_NO_THROW(interpolation.values({{-1., -1.}, {0., 0.}, {1., 1.}}));
  BOOST_REQUIRE_CLOSE(interpolation.compute(-0.2), -0.2, epsilon);

  // Realistic case: trajectory gains
  interpolation.values({{0, 20}, {3., 20}, {3.8, 50}, {4., 200}});
  BOOST_REQUIRE_CLOSE(interpolation.compute(0.), 20, epsilon);

  // Random access
  values = {{0, 0}, {3., 20}, {3.8, 50}, {4., 200}};
  interpolation.values(values);
  BOOST_REQUIRE_CLOSE(interpolation.compute(6), 200, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(-1), 0, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(3), 20, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(3.8), 50, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(2), 20 / 3. * 2, epsilon);
  BOOST_REQUIRE_CLOSE(interpolation.compute(3.9), interpolator(50, 200, 0.5), epsilon);
}
