/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <boost/test/unit_test.hpp>

#include <stdlib.h>

#include "utils.h"

static bool initialized = configureRobotLoader();

BOOST_AUTO_TEST_CASE(CONSTRUCTION_FAILURE)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  // In older versions of Boost, -- is not filtered out from argv
  int argi = 1;
  if(std::string(argv[1]) == "--")
  {
    argi = 2;
  }
  BOOST_CHECK_EQUAL(argc, argi + 1);
  std::string conf = argv[argi];
  BOOST_CHECK_THROW(mc_control::MCGlobalController controller(conf), std::exception);
}

BOOST_AUTO_TEST_CASE(RUN)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  // In older versions of Boost, -- is not filtered out from argv
  int argi = 1;
  if(std::string(argv[1]) == "--")
  {
    argi = 2;
  }
  BOOST_CHECK(argc >= argi + 2);
  std::string conf = argv[argi];
  unsigned int nrIter = static_cast<unsigned int>(std::atoi(argv[argi + 1]));
  std::string nextController = argc > argi + 2 ? argv[argi + 2] : "";
  BOOST_CHECK(nrIter > 0);
  mc_control::MCGlobalController controller(conf);
  // Simple init
  const auto & mb = controller.robot().mb();
  const auto & mbc = controller.robot().mbc();
  const auto & rjo = controller.ref_joint_order();
  std::vector<double> initq;
  for(const auto & jn : rjo)
  {
    for(const auto & qi : mbc.q[static_cast<unsigned int>(mb.jointIndexByName(jn))])
    {
      initq.push_back(qi);
    }
  }

  std::vector<double> qEnc(initq.size(), 0);
  std::vector<double> alphaEnc(initq.size(), 0);
  auto simulateSensors = [&, qEnc, alphaEnc]() mutable {
    auto & robot = controller.robot();
    for(unsigned i = 0; i < robot.refJointOrder().size(); i++)
    {
      auto jIdx = robot.jointIndexInMBC(i);
      if(jIdx != -1)
      {
        auto jointIndex = static_cast<unsigned>(jIdx);
        qEnc[i] = robot.mbc().q[jointIndex][0];
        alphaEnc[i] = robot.mbc().alpha[jointIndex][0];
      }
    }
    controller.setEncoderValues(qEnc);
    controller.setEncoderVelocities(alphaEnc);
    controller.setSensorPositions({{"FloatingBase", robot.posW().translation()}});
    controller.setSensorOrientations({{"FloatingBase", Eigen::Quaterniond{robot.posW().rotation()}}});
  };

  controller.setEncoderValues(qEnc);
  controller.init(initq, controller.robot().module().default_attitude());
  controller.running = true;
  for(size_t i = 0; i < nrIter; ++i)
  {
    simulateSensors();
    BOOST_REQUIRE(controller.run());
  }
  if(nextController != "")
  {
    controller.EnableController(nextController);
    for(size_t i = 0; i < nrIter; ++i)
    {
      simulateSensors();
      BOOST_REQUIRE(controller.run());
    }
  }
}
