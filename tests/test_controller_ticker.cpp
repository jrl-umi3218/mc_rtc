#include <boost/test/unit_test.hpp>

#include <mc_control/mc_global_controller.h>

BOOST_AUTO_TEST_CASE(CONSTRUCTION_FAILURE)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  BOOST_CHECK_EQUAL(argc, 2);
  std::string conf = argv[1];
  BOOST_CHECK_THROW(mc_control::MCGlobalController controller(conf), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(RUN)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  BOOST_CHECK_EQUAL(argc, 3);
  std::string conf = argv[1];
  unsigned int nrIter = std::atoi(argv[2]);
  BOOST_CHECK(nrIter > 0);
  mc_control::MCGlobalController controller(conf);
  // Simple init
  const auto & mb = controller.robot().mb();
  const auto & mbc = controller.robot().mbc();
  const auto & rjo = controller.ref_joint_order();
  std::vector<double> initq;
  for(const auto & jn : rjo)
  {
    for(const auto & qi : mbc.q[mb.jointIndexByName(jn)])
    {
      initq.push_back(qi);
    }
  }
  controller.init(initq);
  for(size_t i = 0; i < nrIter; ++i)
  {
    BOOST_CHECK(controller.run());
  }
}
