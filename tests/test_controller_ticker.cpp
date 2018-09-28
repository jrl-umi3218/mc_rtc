#include <mc_control/mc_global_controller.h>

#include <boost/test/unit_test.hpp>

#include <stdlib.h>

BOOST_AUTO_TEST_CASE(CONSTRUCTION_FAILURE)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  BOOST_CHECK_EQUAL(argc, 2);
  std::string conf = argv[1];
  BOOST_CHECK_THROW(mc_control::MCGlobalController controller(conf), std::exception);
}

BOOST_AUTO_TEST_CASE(RUN)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  BOOST_CHECK(argc >= 3);
  std::string conf = argv[1];
  unsigned int nrIter = std::atoi(argv[2]);
  std::string nextController = argc > 3 ? argv[3] : "";
  std::string pythonPath = argc > 4 ? argv[4] : "";
  if(pythonPath != "")
  {
    std::string PYTHONPATH = std::string(getenv("PYTHONPATH"));
    std::stringstream ss;
    ss << "PYTHONPATH=" << PYTHONPATH;
    if(PYTHONPATH.size() && PYTHONPATH[PYTHONPATH.size() - 1] != ':')
    {
      ss << ":";
    }
    ss << pythonPath;
    char * new_PYTHONPATH = new char[ss.str().size() + 1];
    strncpy(new_PYTHONPATH, ss.str().c_str(), ss.str().size());
    putenv(new_PYTHONPATH);
  }
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
  controller.setEncoderValues(initq);
  controller.init(initq);
  controller.running = true;
  for(size_t i = 0; i < nrIter; ++i)
  {
    BOOST_REQUIRE(controller.run());
  }
  if(nextController != "")
  {
    controller.EnableController(nextController);
    for(size_t i = 0; i < nrIter; ++i)
    {
      BOOST_REQUIRE(controller.run());
    }
  }
}
