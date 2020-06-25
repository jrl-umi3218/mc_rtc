/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/LookupTable.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

#include <boost/test/unit_test.hpp>

#include <mc_planning/generator.h>
#include <mc_rtc/io_utils.h>

template<typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA> & a,
    const Eigen::DenseBase<DerivedB> & b,
    const typename DerivedA::RealScalar & rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar & atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

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
    testLookupTable([dt](double x) { return cosh(x * dt); }, omega(2.5), omega(0.01), 100000, 100000);
    testLookupTable([dt](double x) { return sinh(x * dt); }, omega(2.5), omega(0.01), 100000, 100000);
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

BOOST_AUTO_TEST_CASE(TestPreviewSteps)
{
  using PreviewSteps = mc_planning::PreviewSteps<Eigen::Vector2d>;
  PreviewSteps steps;
  steps.add({0, {0.0, 0.0}});
  steps.add({1.5, {0.0, 1.5}});
  steps.add({1.6, {0.0, 1.6}});
  steps.add({2.9, {0.0, 2.9}});
  steps.add({3.0, {0.0, 3.0}});
  std::cout << "Steps:\n" << mc_rtc::io::to_string(steps.steps(), "\n") << std::endl;

  auto checkPrevious = [&](double t, double expected) {
    std::cout << "Previous for t=" << t << ": " << steps.previous(t) << std::endl;
    BOOST_REQUIRE_CLOSE(steps.previous(t).t(), expected, 1e-10);
  };

  checkPrevious(-0.5, 0.0);
  checkPrevious(0, 0.0);
  checkPrevious(1.6, 1.6);
  checkPrevious(1.8, 1.6);
  checkPrevious(2.9, 2.9);
  checkPrevious(2.95, 2.9);
  checkPrevious(3.0, 3.0);
  checkPrevious(3.5, 3.0);

  auto checkNext = [&](double t, double expected) {
    std::cout << "Next for t=" << t << ": " << steps.next(t) << std::endl;
    BOOST_REQUIRE_CLOSE(steps.next(t).t(), expected, 1e-10);
  };

  checkNext(-0.5, 0);
  checkNext(0.0, 1.5);
  checkNext(0.5, 1.5);
  checkNext(1.6, 2.9);
  checkNext(1.8, 2.9);
  checkNext(2.9, 3.0);
  checkNext(2.95, 3.0);
  checkNext(3.0, 3.0);
  checkNext(3.5, 3.0);

  // Test replacing steps after a given value
  {
    auto steps2 = steps;
    PreviewSteps::PreviewStepsSet newSteps;
    newSteps.insert({1.6, {0.0, 11.6}});
    newSteps.insert({2.5, {0.0, 12.5}});
    newSteps.insert({3.0, {0.0, 13.0}});
    steps2.replaceAfter(newSteps);
    std::cout << "New Steps:\n" << mc_rtc::io::to_string(steps2.steps(), "\n") << std::endl;
    BOOST_REQUIRE(steps2.steps().size() == 5);
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 0)).step(), Eigen::Vector2d{0.0, 0.0}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 1)).step(), Eigen::Vector2d{0.0, 1.5}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 2)).step(), Eigen::Vector2d{0.0, 11.6}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 3)).step(), Eigen::Vector2d{0.0, 12.5}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 4)).step(), Eigen::Vector2d{0.0, 13.0}));
  }

  {
    auto steps2 = steps;
    PreviewSteps::PreviewStepsSet newSteps;
    newSteps.insert({3.5, {0.0, 13.5}});
    steps2.replaceAfter(newSteps);
    std::cout << "New Steps:\n" << mc_rtc::io::to_string(steps2.steps(), "\n") << std::endl;
    BOOST_REQUIRE(steps2.steps().size() == steps.steps().size() + 1);
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 0)).step(), Eigen::Vector2d{0.0, 0.0}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 1)).step(), Eigen::Vector2d{0.0, 1.5}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 2)).step(), Eigen::Vector2d{0.0, 1.6}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 3)).step(), Eigen::Vector2d{0.0, 2.9}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 4)).step(), Eigen::Vector2d{0.0, 3.0}));
    BOOST_REQUIRE(allclose((*std::next(steps2.steps().begin(), 5)).step(), Eigen::Vector2d{0.0, 13.5}));
  }
}

BOOST_AUTO_TEST_CASE(TestPreviewStepsSequential)
{
  using PreviewSteps = mc_planning::PreviewSteps<Eigen::Vector2d>;
  PreviewSteps steps;
  steps.add({0, {0.0, 0.0}});
  steps.add({1.5, {0.0, 1.5}});
  steps.add({1.6, {0.0, 1.6}});
  steps.add({2.9, {0.0, 2.9}});
  steps.add({3.0, {0.0, 3.0}});
  steps.initialize();
  PreviewSteps stepsSeq = steps;
  stepsSeq.initialize();
  std::cout << "Steps:\n" << mc_rtc::io::to_string(steps.steps(), "\n") << std::endl;

  double dt = 0.005;
  unsigned n_loop = (unsigned)std::lround(5. / dt);
  unsigned n_preview = (unsigned)std::lround(1.6 / dt);
  for(unsigned loop = 0; loop <= n_loop; loop++)
  {
    auto previewStart = loop * dt;
    // mc_rtc::log::success("Start of new preview window at time: {:.3f}", previewStart);
    stepsSeq.nextWindow(previewStart);

    for(unsigned previewWindow = loop; previewWindow < loop + 2 * n_preview + 1; ++previewWindow)
    {
      double t = previewWindow * dt;
      // mc_rtc::log::info("t: {}", t);
      stepsSeq.update(t);

      if(t >= steps.back().t())
      {
        BOOST_REQUIRE(stepsSeq.isLastStep());
      }
      else
      {
        BOOST_REQUIRE(!stepsSeq.isLastStep());
      }

      if(!stepsSeq.isLastStep())
      {
        // mc_rtc::log::info("t: {}", t);
        // mc_rtc::log::info("Steps:\n {}", mc_rtc::io::to_string(steps.steps(), "\n"));
        // mc_rtc::log::info("prev(t): {:.3f}, prev(sew): {:.3f}", steps.previous(t).t(), stepsSeq.previous().t());
        BOOST_REQUIRE_CLOSE(steps.previous(t).t(), stepsSeq.previous().t(), 1e-10);
        BOOST_REQUIRE_CLOSE(steps.next(t).t(), stepsSeq.next().t(), 1e-10);
      }
    }
  }
}
