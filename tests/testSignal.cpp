/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_signal/ExponentialMovingAverage.h>

#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

BOOST_AUTO_TEST_CASE(TestExponentialMovingAverage)
{
  using namespace mc_signal;
  double dt = 0.005;
  const double T = 15;
  const double C = 100;
  size_t iter = static_cast<size_t>(ceil(T / dt));

  {
    const Eigen::Vector3d constant{C, C, C};
    ExponentialMovingAverage<Eigen::Vector3d> average(dt, T);
    for(size_t i = 0; i < iter; ++i)
    {
      average.append(constant);
    }

    // Check that averaging a constant for the characteristic time T gives the
    // expected theoratical value 1-1/e
    BOOST_REQUIRE_CLOSE(average.eval().x(), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval().y(), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval().z(), C * (1 - std::exp(-1)), 10e-6);
  }

  {
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    const Vector6d constant = C * Vector6d::Ones();
    ExponentialMovingAverage<Vector6d> average(dt, T);
    for(size_t i = 0; i < iter; ++i)
    {
      average.append(constant);
    }

    // Check that averaging a constant for the characteristic time T gives the
    // expected theoratical value 1-1/e
    BOOST_REQUIRE_CLOSE(average.eval()(0), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval()(1), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval()(2), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval()(3), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval()(4), C * (1 - std::exp(-1)), 10e-6);
    BOOST_REQUIRE_CLOSE(average.eval()(5), C * (1 - std::exp(-1)), 10e-6);
  }
}
