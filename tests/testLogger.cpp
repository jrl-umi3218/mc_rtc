/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/test/unit_test.hpp>

/** Check one iteration of the logger */
template<typename Callback>
void check(mc_rtc::Logger & logger, Callback && cb)
{
  logger.log();
  logger.flush();
  mc_rtc::log::FlatLog log(logger.path());
  cb(log);
}

BOOST_AUTO_TEST_CASE(TestLogger)
{
  using Policy = mc_rtc::Logger::Policy;
  mc_rtc::Logger logger(Policy::NON_THREADED, bfs::temp_directory_path().string(), "mc-rtc-test");
  logger.start("logger", 1.0);
  /** Iteration 1 only time */
  check(logger, [](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.entries() == std::set<std::string>{"t"});
    BOOST_REQUIRE(log.size() == 1);
    auto t0 = log.getRaw<double>("t", 0);
    BOOST_REQUIRE(t0 && *t0 == 0.0);
  });
}
