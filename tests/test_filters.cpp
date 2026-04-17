/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_filter/ExponentialMovingAverage.h>
#include <mc_filter/utils/clamp.h>

#include <boost/test/unit_test.hpp>

template<typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA> & a,
    const Eigen::DenseBase<DerivedB> & b,
    const typename DerivedA::RealScalar & rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar & atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

BOOST_AUTO_TEST_CASE(TestExponentialMovingAverage)
{
  using namespace mc_filter;
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
    Eigen::Vector3d expected = C * (1 - std::exp(-1)) * Eigen::Vector3d::Ones();
    BOOST_CHECK(allclose(average.eval(), expected, 1e-10));
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
    Vector6d expected = C * (1 - std::exp(-1)) * Vector6d::Ones();
    BOOST_CHECK(allclose(average.eval(), expected, 1e-10));
  }
}

BOOST_AUTO_TEST_CASE(test_clamp)
{
  using namespace mc_filter;
  double min = -0.1;
  double max = 0.1;
  BOOST_REQUIRE_CLOSE(utils::clamp(0.5, min, max), max, 1e-10);
  BOOST_REQUIRE_CLOSE(utils::clamp(-0.5, min, max), min, 1e-10);
  BOOST_REQUIRE_CLOSE(utils::clampAndWarn(0.5, min, max, ""), max, 1e-10);
  BOOST_REQUIRE_CLOSE(utils::clampAndWarn(-0.5, min, max, ""), min, 1e-10);
  {
    double val = 0.5;
    utils::clampInPlace(val, min, max);
    BOOST_REQUIRE_CLOSE(val, max, 1e-10);
    val = -0.5;
    utils::clampInPlace(val, min, max);
    BOOST_REQUIRE_CLOSE(val, min, 1e-10);
  }
  {
    double val = 0.5;
    utils::clampInPlace(val, min, max);
    BOOST_REQUIRE_CLOSE(val, max, 1e-10);
    val = -0.5;
    utils::clampInPlace(val, min, max);
    BOOST_REQUIRE_CLOSE(val, min, 1e-10);
  }
  {
    double val = 0.5;
    utils::clampInPlaceAndWarn(val, min, max, "Value");
    BOOST_REQUIRE_CLOSE(val, max, 1e-10);
    val = -0.5;
    utils::clampInPlaceAndWarn(val, min, max, "Value");
    BOOST_REQUIRE_CLOSE(val, min, 1e-10);
  }
  {
    double val = 0.5;
    utils::clampInPlaceAndWarn(val, min, max, "Value");
    BOOST_REQUIRE_CLOSE(val, max, 1e-10);
    val = -0.5;
    utils::clampInPlaceAndWarn(val, min, max, "Value");
    BOOST_REQUIRE_CLOSE(val, min, 1e-10);
  }

  // Vector version
  {
    Eigen::Vector3d minV{-1, -2, -3};
    Eigen::Vector3d maxV{1, 2, 3};
    Eigen::Vector3d valMax = 10 * maxV;
    Eigen::Vector3d valMin = 10 * minV;
    BOOST_CHECK(allclose(utils::clamp(valMax, -1., 1.), 1. * Eigen::Vector3d::Ones()));
    BOOST_CHECK(allclose(utils::clamp(valMin, -1., 1.), -1. * Eigen::Vector3d::Ones()));
    utils::clampInPlace(valMax, -1., 1.);
    BOOST_CHECK(allclose(valMax, 1. * Eigen::Vector3d::Ones()));
    utils::clampInPlace(valMin, -1., 1.);
    BOOST_CHECK(allclose(valMin, -1. * Eigen::Vector3d::Ones()));
    valMax = 10 * maxV;
    valMin = 10 * minV;
    BOOST_CHECK(allclose(utils::clamp(valMax, minV, maxV), maxV));
    BOOST_CHECK(allclose(utils::clamp(valMin, minV, maxV), minV));
    utils::clampInPlace(valMax, minV, maxV);
    BOOST_CHECK(allclose(valMax, maxV));
    utils::clampInPlace(valMin, minV, maxV);
    BOOST_CHECK(allclose(valMin, minV));
  }
  // Vector version with warning log
  {
    Eigen::Vector3d minV{-1, -2, -3};
    Eigen::Vector3d maxV{1, 2, 3};
    Eigen::Vector3d valMax = 10 * Eigen::Vector3d::Ones();
    Eigen::Vector3d valMin = -valMax;
    BOOST_CHECK(allclose(utils::clampAndWarn(valMax, minV, maxV, "Vector"), maxV));
    BOOST_CHECK(allclose(utils::clampAndWarn(valMin, minV, maxV, "Vector"), minV));
    utils::clampInPlaceAndWarn(valMax, minV, maxV, "Vector");
    BOOST_CHECK(allclose(valMax, maxV));
    utils::clampInPlaceAndWarn(valMin, minV, maxV, "Vector");
    BOOST_CHECK(allclose(valMin, minV));
  }
}
