#include <boost/test/unit_test.hpp>

#include <mc_control/mc_global_controller.h>

BOOST_AUTO_TEST_CASE(TestController)
{
  auto argc = boost::unit_test::framework::master_test_suite().argc;
  auto argv = boost::unit_test::framework::master_test_suite().argv;
  BOOST_CHECK_EQUAL(argc, 2);
  std::string conf = argv[1];
  BOOST_CHECK_THROW(mc_control::MCGlobalController controller(conf), std::runtime_error);
}
