/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/mc_global_controller.h>

#include <boost/test/unit_test.hpp>

#include <stdlib.h>

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
  unsigned int nrIter = std::atoi(argv[argi + 1]);
  std::string nextController = argc > argi + 2 ? argv[argi + 2] : "";
  std::string pythonPath = argc > argi + 3 ? argv[argi + 3] : "";
  if(pythonPath != "")
  {
    const char * PPATH = getenv("PYTHONPATH");
    std::string PYTHONPATH = ".";
    if(PPATH)
    {
      PYTHONPATH = std::string(PPATH);
    }
    std::stringstream ss;
    ss << "PYTHONPATH=" << PYTHONPATH;
#ifdef WIN32
    char PATH_SEP = ';';
#else
    char PATH_SEP = ':';
#endif
    if(PYTHONPATH.size() && PYTHONPATH[PYTHONPATH.size() - 1] != PATH_SEP)
    {
      ss << PATH_SEP;
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
