/*
 * Copyright 2015-2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#define EIGEN_RUNTIME_NO_MALLOC

#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/log/Logger.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/test/unit_test.hpp>

#include "utils.h"

bool operator==(const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  return lhs.vec() == rhs.vec();
}

/** Check one iteration of the logger */
template<bool malloc_allowed = false, typename Callback>
void check(mc_rtc::Logger & logger, Callback && cb)
{
  Eigen::internal::set_is_malloc_allowed(malloc_allowed);
  logger.log();
  Eigen::internal::set_is_malloc_allowed(true);
  logger.flush();
  mc_rtc::log::FlatLog log(logger.path());
  cb(log);
}

/** Check the value in a FlatLog at a given index */
template<typename T>
void check(const mc_rtc::log::FlatLog & log, const std::string & entry, size_t idx, const T & member)
{
  const T * raw = log.getRaw<T>(entry, idx);
  BOOST_REQUIRE(raw && *raw == member);
}

struct LogData
{
private:
  /** Some data members supported by the logger */
  bool b = random_bool();
  double d = rnd();
  std::string s = random_string();
  Eigen::Vector2d v2d = Eigen::Vector2d::Random();
  Eigen::Vector3d v3d = Eigen::Vector3d::Random();
  Eigen::Vector6d v6d = Eigen::Vector6d::Random();
  Eigen::VectorXd vxd = Eigen::VectorXd::Random(static_cast<Eigen::DenseIndex>(random_size()));
  Eigen::Quaterniond q = random_quat();
  sva::PTransformd pt = random_pt();
  sva::ForceVecd fv = random_fv();
  sva::MotionVecd mv = random_mv();
  std::vector<double> v = random_vector();

public:
#define DEFINE_GETTER(MEMBER)                               \
  auto get_##MEMBER() const->const decltype(this->MEMBER) & \
  {                                                         \
    return this->MEMBER;                                    \
  }
  DEFINE_GETTER(b)
  DEFINE_GETTER(d)
  DEFINE_GETTER(s)
  DEFINE_GETTER(v2d)
  DEFINE_GETTER(v3d)
  DEFINE_GETTER(v6d)
  DEFINE_GETTER(vxd)
  DEFINE_GETTER(q)
  DEFINE_GETTER(pt)
  DEFINE_GETTER(fv)
  DEFINE_GETTER(mv)
  DEFINE_GETTER(v)
#undef DEFINE_GETTER

  /** Change data */
  void refresh()
  {
    *this = LogData{};
  }

  void addToLogger(mc_rtc::Logger & logger, bool withSource)
  {
#define ADD_LOG_ENTRY(NAME, MEMBER)                                        \
  withSource ? logger.addLogEntry(NAME, this, [this]() { return MEMBER; }) \
             : logger.addLogEntry(NAME, [this]() { return MEMBER; });
    ADD_LOG_ENTRY("bool", b);
    ADD_LOG_ENTRY("double", d);
    ADD_LOG_ENTRY("std::string", s);
    ADD_LOG_ENTRY("Eigen::Vector2d", v2d);
    ADD_LOG_ENTRY("Eigen::Vector3d", v3d);
    ADD_LOG_ENTRY("Eigen::Vector6d", v6d);
    ADD_LOG_ENTRY("Eigen::VectorXd", vxd);
    ADD_LOG_ENTRY("Eigen::Quaterniond", q);
    ADD_LOG_ENTRY("sva::PTransformd", pt);
    ADD_LOG_ENTRY("sva::ForceVecd", fv);
    ADD_LOG_ENTRY("sva::MotionVecd", mv);
    ADD_LOG_ENTRY("std::vector<double>", v);
#undef ADD_LOG_ENTRY
  }

  void addToLoggerWithMemberPointer(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("bool", this, &LogData::b);
    logger.addLogEntry("double", this, &LogData::d);
    logger.addLogEntry("std::string", this, &LogData::s);
    logger.addLogEntry("Eigen::Vector2d", this, &LogData::v2d);
    logger.addLogEntry("Eigen::Vector3d", this, &LogData::v3d);
    logger.addLogEntry("Eigen::Vector6d", this, &LogData::v6d);
    logger.addLogEntry("Eigen::VectorXd", this, &LogData::vxd);
    logger.addLogEntry("Eigen::Quaterniond", this, &LogData::q);
    logger.addLogEntry("sva::PTransformd", this, &LogData::pt);
    logger.addLogEntry("sva::ForceVecd", this, &LogData::fv);
    logger.addLogEntry("sva::MotionVecd", this, &LogData::mv);
    logger.addLogEntry("std::vector<double>", this, &LogData::v);
  }

  void addToLoggerWithMemberPointerFast(mc_rtc::Logger & logger)
  {
    logger.addLogEntry<decltype(&LogData::b), &LogData::b>("bool", this);
    logger.addLogEntry<decltype(&LogData::d), &LogData::d>("double", this);
    logger.addLogEntry<decltype(&LogData::s), &LogData::s>("std::string", this);
    logger.addLogEntry<decltype(&LogData::v2d), &LogData::v2d>("Eigen::Vector2d", this);
    logger.addLogEntry<decltype(&LogData::v3d), &LogData::v3d>("Eigen::Vector3d", this);
    logger.addLogEntry<decltype(&LogData::v6d), &LogData::v6d>("Eigen::Vector6d", this);
    logger.addLogEntry<decltype(&LogData::vxd), &LogData::vxd>("Eigen::VectorXd", this);
    logger.addLogEntry<decltype(&LogData::q), &LogData::q>("Eigen::Quaterniond", this);
    logger.addLogEntry<decltype(&LogData::pt), &LogData::pt>("sva::PTransformd", this);
    logger.addLogEntry<decltype(&LogData::fv), &LogData::fv>("sva::ForceVecd", this);
    logger.addLogEntry<decltype(&LogData::mv), &LogData::mv>("sva::MotionVecd", this);
    logger.addLogEntry<decltype(&LogData::v), &LogData::v>("std::vector<double>", this);
  }

  void addToLoggerWithMemberPointerMacro(mc_rtc::Logger & logger)
  {
    MC_RTC_LOG_HELPER(logger, "bool", this, &LogData::b);
    MC_RTC_LOG_HELPER(logger, "double", this, &LogData::d);
    MC_RTC_LOG_HELPER(logger, "std::string", this, &LogData::s);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector2d", this, &LogData::v2d);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector3d", this, &LogData::v3d);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector6d", this, &LogData::v6d);
    MC_RTC_LOG_HELPER(logger, "Eigen::VectorXd", this, &LogData::vxd);
    MC_RTC_LOG_HELPER(logger, "Eigen::Quaterniond", this, &LogData::q);
    MC_RTC_LOG_HELPER(logger, "sva::PTransformd", this, &LogData::pt);
    MC_RTC_LOG_HELPER(logger, "sva::ForceVecd", this, &LogData::fv);
    MC_RTC_LOG_HELPER(logger, "sva::MotionVecd", this, &LogData::mv);
    MC_RTC_LOG_HELPER(logger, "std::vector<double>", this, &LogData::v);
  }

  void addToLoggerWithGetter(mc_rtc::Logger & logger)
  {
    logger.addLogEntry("bool", this, &LogData::get_b);
    logger.addLogEntry("double", this, &LogData::get_d);
    logger.addLogEntry("std::string", this, &LogData::get_s);
    logger.addLogEntry("Eigen::Vector2d", this, &LogData::get_v2d);
    logger.addLogEntry("Eigen::Vector3d", this, &LogData::get_v3d);
    logger.addLogEntry("Eigen::Vector6d", this, &LogData::get_v6d);
    logger.addLogEntry("Eigen::VectorXd", this, &LogData::get_vxd);
    logger.addLogEntry("Eigen::Quaterniond", this, &LogData::get_q);
    logger.addLogEntry("sva::PTransformd", this, &LogData::get_pt);
    logger.addLogEntry("sva::ForceVecd", this, &LogData::get_fv);
    logger.addLogEntry("sva::MotionVecd", this, &LogData::get_mv);
    logger.addLogEntry("std::vector<double>", this, &LogData::get_v);
  }

  void addToLoggerWithGetterFast(mc_rtc::Logger & logger)
  {
    logger.addLogEntry<decltype(&LogData::get_b), &LogData::get_b>("bool", this);
    logger.addLogEntry<decltype(&LogData::get_d), &LogData::get_d>("double", this);
    logger.addLogEntry<decltype(&LogData::get_s), &LogData::get_s>("std::string", this);
    logger.addLogEntry<decltype(&LogData::get_v2d), &LogData::get_v2d>("Eigen::Vector2d", this);
    logger.addLogEntry<decltype(&LogData::get_v3d), &LogData::get_v3d>("Eigen::Vector3d", this);
    logger.addLogEntry<decltype(&LogData::get_v6d), &LogData::get_v6d>("Eigen::Vector6d", this);
    logger.addLogEntry<decltype(&LogData::get_vxd), &LogData::get_vxd>("Eigen::VectorXd", this);
    logger.addLogEntry<decltype(&LogData::get_q), &LogData::get_q>("Eigen::Quaterniond", this);
    logger.addLogEntry<decltype(&LogData::get_pt), &LogData::get_pt>("sva::PTransformd", this);
    logger.addLogEntry<decltype(&LogData::get_fv), &LogData::get_fv>("sva::ForceVecd", this);
    logger.addLogEntry<decltype(&LogData::get_mv), &LogData::get_mv>("sva::MotionVecd", this);
    logger.addLogEntry<decltype(&LogData::get_v), &LogData::get_v>("std::vector<double>", this);
  }

  void addToLoggerWithGetterMacro(mc_rtc::Logger & logger)
  {
    MC_RTC_LOG_HELPER(logger, "bool", this, &LogData::get_b);
    MC_RTC_LOG_HELPER(logger, "double", this, &LogData::get_d);
    MC_RTC_LOG_HELPER(logger, "std::string", this, &LogData::get_s);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector2d", this, &LogData::get_v2d);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector3d", this, &LogData::get_v3d);
    MC_RTC_LOG_HELPER(logger, "Eigen::Vector6d", this, &LogData::get_v6d);
    MC_RTC_LOG_HELPER(logger, "Eigen::VectorXd", this, &LogData::get_vxd);
    MC_RTC_LOG_HELPER(logger, "Eigen::Quaterniond", this, &LogData::get_q);
    MC_RTC_LOG_HELPER(logger, "sva::PTransformd", this, &LogData::get_pt);
    MC_RTC_LOG_HELPER(logger, "sva::ForceVecd", this, &LogData::get_fv);
    MC_RTC_LOG_HELPER(logger, "sva::MotionVecd", this, &LogData::get_mv);
    MC_RTC_LOG_HELPER(logger, "std::vector<double>", this, &LogData::get_v);
  }

  void removeFromLogger(mc_rtc::Logger & logger)
  {
    logger.removeLogEntry("bool");
    logger.removeLogEntry("double");
    logger.removeLogEntry("std::string");
    logger.removeLogEntry("Eigen::Vector2d");
    logger.removeLogEntry("Eigen::Vector3d");
    logger.removeLogEntry("Eigen::Vector6d");
    logger.removeLogEntry("Eigen::VectorXd");
    logger.removeLogEntry("Eigen::Quaterniond");
    logger.removeLogEntry("sva::PTransformd");
    logger.removeLogEntry("sva::ForceVecd");
    logger.removeLogEntry("sva::MotionVecd");
    logger.removeLogEntry("std::vector<double>");
  }

  /** Check that the latest entry in the FlatLog has the same values as in the class */
  void check(const mc_rtc::log::FlatLog & log)
  {
    BOOST_REQUIRE(log.size() >= 1);
    size_t idx = log.size() - 1;
    ::check(log, "bool", idx, b);
    ::check(log, "double", idx, d);
    ::check(log, "std::string", idx, s);
    ::check(log, "Eigen::Vector2d", idx, v2d);
    ::check(log, "Eigen::Vector3d", idx, v3d);
    ::check(log, "Eigen::Vector6d", idx, v6d);
    ::check(log, "Eigen::VectorXd", idx, vxd);
    ::check(log, "Eigen::Quaterniond", idx, q);
    ::check(log, "sva::PTransformd", idx, pt);
    ::check(log, "sva::ForceVecd", idx, fv);
    ::check(log, "sva::MotionVecd", idx, mv);
    ::check(log, "std::vector<double>", idx, v);
  }

  /** Check that the entry at the given index has none of the class data */
  void check_empty(const mc_rtc::log::FlatLog & log, size_t idx)
  {
    BOOST_REQUIRE(log.getRaw<bool>("bool", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<double>("double", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<std::string>("std::string", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector2d>("Eigen::Vector2d", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector3d>("Eigen::Vector3d", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector6d>("Eigen::Vector6d", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::VectorXd>("Eigen::VectorXd", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Quaterniond>("Eigen::Quaterniond", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<sva::PTransformd>("sva::PTransformd", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<sva::ForceVecd>("sva::ForceVecd", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<sva::MotionVecd>("sva::MotionVecd", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<std::vector<double>>("std::vector<double>", idx) == nullptr);
  }

  /** Shortcut to check the latest iteration is empty */
  void check_empty(const mc_rtc::log::FlatLog & log)
  {
    BOOST_REQUIRE(log.size() >= 1);
    check_empty(log, log.size() - 1);
  }
};

BOOST_AUTO_TEST_CASE(TestLogger)
{
  using Policy = mc_rtc::Logger::Policy;
  mc_rtc::Logger logger(Policy::NON_THREADED, bfs::temp_directory_path().string(), "mc-rtc-test");
  logger.start("logger", 1.0);
  size_t iter = 1;
  /** Iteration 1 only time */
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.entries() == std::set<std::string>{"t"});
    BOOST_REQUIRE(log.size() == iter);
    auto t0 = log.getRaw<double>("t", 0);
    BOOST_REQUIRE(t0 && *t0 == 0.0);
    iter++;
  });
  /** Iteration 2, add callbacks without a source */
  LogData data;
  data.addToLogger(logger, false);
  /** Malloc is allowed in this test because we provided naive callbacks */
  check<true>(logger, [&](const mc_rtc::log::FlatLog & log) {
    std::set<std::string> entries = {"t",
                                     "bool",
                                     "double",
                                     "std::string",
                                     "Eigen::Vector2d",
                                     "Eigen::Vector3d",
                                     "Eigen::Vector6d",
                                     "Eigen::VectorXd",
                                     "Eigen::Quaterniond",
                                     "sva::PTransformd",
                                     "sva::ForceVecd",
                                     "sva::MotionVecd",
                                     "std::vector<double>"};
    BOOST_REQUIRE(log.entries() == entries);
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log, 0);
    data.check(log);
    iter++;
  });
  /** Iteration 3, remove everything "manually" */
  data.removeFromLogger(logger);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 4, add everything with a source */
  data.addToLogger(logger, true);
  /** Malloc is allowed in this test because we provided naive callbacks */
  check<true>(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check(log);
    iter++;
  });
  /** Iteration 5, remove everything via the source */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 6 to 100, refresh data and check everything is ok */
  data.addToLogger(logger, true);
  for(iter = 6; iter <= 100; ++iter)
  {
    /** Malloc is allowed in this test because we provided naive callbacks */
    check<true>(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 101, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 102 to 150, add from pointer to member, refresh data and check everything is ok */
  data.addToLoggerWithMemberPointer(logger);
  for(iter = 102; iter <= 150; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 151, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 152 to 200, add from pointer to method, refresh data and check everything is ok */
  data.addToLoggerWithGetter(logger);
  for(iter = 152; iter <= 200; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 201, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 202 to 250, add from fast pointer to member, refresh data and check everything is ok */
  data.addToLoggerWithMemberPointerFast(logger);
  for(iter = 202; iter <= 250; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 251, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 252 to 300, add from fast pointer to method, refresh data and check everything is ok */
  data.addToLoggerWithGetterFast(logger);
  for(iter = 252; iter <= 300; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 301, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 302 to 350, add from fast pointer to member, refresh data and check everything is ok */
  data.addToLoggerWithMemberPointerMacro(logger);
  for(iter = 302; iter <= 350; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 351, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
  /** Iteration 352 to 400, add from fast pointer to method, refresh data and check everything is ok */
  data.addToLoggerWithGetterMacro(logger);
  for(iter = 352; iter <= 400; ++iter)
  {
    check(logger, [&](const mc_rtc::log::FlatLog & log) {
      BOOST_REQUIRE(log.size() == iter);
      data.check(log);
      data.refresh();
    });
  }
  /** Iteration 401, remove the data once more */
  logger.removeLogEntries(&data);
  check(logger, [&](const mc_rtc::log::FlatLog & log) {
    BOOST_REQUIRE(log.size() == iter);
    data.check_empty(log);
    iter++;
  });
}
