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
  mutable Eigen::Vector2d v2d = Eigen::Vector2d::Random();
  mutable Eigen::Vector3d v3d = Eigen::Vector3d::Random();
  mutable Eigen::Vector6d v6d = Eigen::Vector6d::Random();
  mutable Eigen::VectorXd vxd = Eigen::VectorXd::Random(static_cast<Eigen::DenseIndex>(random_size()));
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

#define DEFINE_GET_AS_REF(MEMBER)                                               \
  auto get_##MEMBER##_as_ref() const->Eigen::Ref<decltype(this->MEMBER)>        \
  {                                                                             \
    return this->MEMBER;                                                        \
  }                                                                             \
  auto get_##MEMBER##_as_cref() const->Eigen::Ref<const decltype(this->MEMBER)> \
  {                                                                             \
    return this->MEMBER;                                                        \
  }
  DEFINE_GET_AS_REF(v2d)
  DEFINE_GET_AS_REF(v3d)
  DEFINE_GET_AS_REF(v6d)
  DEFINE_GET_AS_REF(vxd)
#undef DEFINE_GET_AS_REF

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
#define ADD_LOG_ENTRY_AS_REF(NAME, MEMBER)                                                       \
  {                                                                                              \
    using RefT = Eigen::Ref<decltype(MEMBER)>;                                                   \
    using CRefT = Eigen::Ref<const decltype(MEMBER)>;                                            \
    withSource ? logger.addLogEntry(NAME "_as_ref", this, [this]() -> RefT { return MEMBER; })   \
               : logger.addLogEntry(NAME "_as_ref", [this]() -> RefT { return MEMBER; });        \
    withSource ? logger.addLogEntry(NAME "_as_cref", this, [this]() -> CRefT { return MEMBER; }) \
               : logger.addLogEntry(NAME "_as_cref", [this]() -> RefT { return MEMBER; });       \
  }
    ADD_LOG_ENTRY_AS_REF("v2d", v2d);
    ADD_LOG_ENTRY_AS_REF("v3d", v3d);
    ADD_LOG_ENTRY_AS_REF("v6d", v6d);
    ADD_LOG_ENTRY_AS_REF("vxd", vxd);
#undef ADD_LOG_ENTRY_AS_REF
  }

  void addToLoggerWithMemberPointer(mc_rtc::Logger & logger)
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
#define ADD_LOG_ENTRY_AS_REF(NAME, MEMBER)                                           \
  {                                                                                  \
    using RefT = Eigen::Ref<decltype(MEMBER)>;                                       \
    using CRefT = Eigen::Ref<const decltype(MEMBER)>;                                \
    logger.addLogEntry(NAME "_as_ref", this, [this]() -> RefT { return MEMBER; });   \
    logger.addLogEntry(NAME "_as_cref", this, [this]() -> CRefT { return MEMBER; }); \
  }
    // Avoid special casing the check
    ADD_LOG_ENTRY_AS_REF("v2d", v2d);
    ADD_LOG_ENTRY_AS_REF("v3d", v3d);
    ADD_LOG_ENTRY_AS_REF("v6d", v6d);
    ADD_LOG_ENTRY_AS_REF("vxd", vxd);
#undef ADD_LOG_ENTRY_AS_REF
  }

  void addToLoggerWithMemberPointerMacro(mc_rtc::Logger & logger)
  {
    MC_RTC_LOG_HELPER("bool", b);
    MC_RTC_LOG_HELPER("double", d);
    MC_RTC_LOG_HELPER("std::string", s);
    MC_RTC_LOG_HELPER("Eigen::Vector2d", v2d);
    MC_RTC_LOG_HELPER("Eigen::Vector3d", v3d);
    MC_RTC_LOG_HELPER("Eigen::Vector6d", v6d);
    MC_RTC_LOG_HELPER("Eigen::VectorXd", vxd);
    MC_RTC_LOG_HELPER("Eigen::Quaterniond", q);
    MC_RTC_LOG_HELPER("sva::PTransformd", pt);
    MC_RTC_LOG_HELPER("sva::ForceVecd", fv);
    MC_RTC_LOG_HELPER("sva::MotionVecd", mv);
    MC_RTC_LOG_HELPER("std::vector<double>", v);
#define ADD_LOG_ENTRY_AS_REF(NAME, MEMBER)                                           \
  {                                                                                  \
    using RefT = Eigen::Ref<decltype(MEMBER)>;                                       \
    using CRefT = Eigen::Ref<const decltype(MEMBER)>;                                \
    logger.addLogEntry(NAME "_as_ref", this, [this]() -> RefT { return MEMBER; });   \
    logger.addLogEntry(NAME "_as_cref", this, [this]() -> CRefT { return MEMBER; }); \
  }
    // Avoid special casing the check
    ADD_LOG_ENTRY_AS_REF("v2d", v2d);
    ADD_LOG_ENTRY_AS_REF("v3d", v3d);
    ADD_LOG_ENTRY_AS_REF("v6d", v6d);
    ADD_LOG_ENTRY_AS_REF("vxd", vxd);
#undef ADD_LOG_ENTRY_AS_REF
  }

  void addToLoggerWithGetter(mc_rtc::Logger & logger)
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
    logger.addLogEntry<decltype(&LogData::get_v2d_as_ref), &LogData::get_v2d_as_ref>("v2d_as_ref", this);
    logger.addLogEntry<decltype(&LogData::get_v2d_as_cref), &LogData::get_v2d_as_cref>("v2d_as_cref", this);
    logger.addLogEntry<decltype(&LogData::get_v3d_as_ref), &LogData::get_v3d_as_ref>("v3d_as_ref", this);
    logger.addLogEntry<decltype(&LogData::get_v3d_as_cref), &LogData::get_v3d_as_cref>("v3d_as_cref", this);
    logger.addLogEntry<decltype(&LogData::get_v6d_as_ref), &LogData::get_v6d_as_ref>("v6d_as_ref", this);
    logger.addLogEntry<decltype(&LogData::get_v6d_as_cref), &LogData::get_v6d_as_cref>("v6d_as_cref", this);
    logger.addLogEntry<decltype(&LogData::get_vxd_as_ref), &LogData::get_vxd_as_ref>("vxd_as_ref", this);
    logger.addLogEntry<decltype(&LogData::get_vxd_as_cref), &LogData::get_vxd_as_cref>("vxd_as_cref", this);
  }

  void addToLoggerWithGetterMacro(mc_rtc::Logger & logger)
  {
    MC_RTC_LOG_HELPER("bool", get_b);
    MC_RTC_LOG_HELPER("double", get_d);
    MC_RTC_LOG_HELPER("std::string", get_s);
    MC_RTC_LOG_HELPER("Eigen::Vector2d", get_v2d);
    MC_RTC_LOG_HELPER("Eigen::Vector3d", get_v3d);
    MC_RTC_LOG_HELPER("Eigen::Vector6d", get_v6d);
    MC_RTC_LOG_HELPER("Eigen::VectorXd", get_vxd);
    MC_RTC_LOG_HELPER("Eigen::Quaterniond", get_q);
    MC_RTC_LOG_HELPER("sva::PTransformd", get_pt);
    MC_RTC_LOG_HELPER("sva::ForceVecd", get_fv);
    MC_RTC_LOG_HELPER("sva::MotionVecd", get_mv);
    MC_RTC_LOG_HELPER("std::vector<double>", get_v);
    MC_RTC_LOG_HELPER("v2d_as_ref", get_v2d_as_ref);
    MC_RTC_LOG_HELPER("v2d_as_cref", get_v2d_as_cref);
    MC_RTC_LOG_HELPER("v3d_as_ref", get_v3d_as_ref);
    MC_RTC_LOG_HELPER("v3d_as_cref", get_v3d_as_cref);
    MC_RTC_LOG_HELPER("v6d_as_ref", get_v6d_as_ref);
    MC_RTC_LOG_HELPER("v6d_as_cref", get_v6d_as_cref);
    MC_RTC_LOG_HELPER("vxd_as_ref", get_vxd_as_ref);
    MC_RTC_LOG_HELPER("vxd_as_cref", get_vxd_as_cref);
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
    logger.removeLogEntry("v2d_as_ref");
    logger.removeLogEntry("v2d_as_cref");
    logger.removeLogEntry("v3d_as_ref");
    logger.removeLogEntry("v3d_as_cref");
    logger.removeLogEntry("vxd_as_ref");
    logger.removeLogEntry("v6d_as_ref");
    logger.removeLogEntry("v6d_as_cref");
    logger.removeLogEntry("vxd_as_cref");
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
    ::check(log, "v2d_as_ref", idx, v2d);
    ::check(log, "v2d_as_cref", idx, v2d);
    ::check(log, "v3d_as_ref", idx, v3d);
    ::check(log, "v3d_as_cref", idx, v3d);
    ::check(log, "v6d_as_ref", idx, v6d);
    ::check(log, "v6d_as_cref", idx, v6d);
    ::check(log, "vxd_as_ref", idx, vxd);
    ::check(log, "vxd_as_cref", idx, vxd);
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
    BOOST_REQUIRE(log.getRaw<Eigen::Vector2d>("v2d_as_ref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector2d>("v2d_as_cref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector3d>("v3d_as_ref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector3d>("v3d_as_cref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector6d>("v6d_as_ref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::Vector6d>("v6d_as_cref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::VectorXd>("vxd_as_ref", idx) == nullptr);
    BOOST_REQUIRE(log.getRaw<Eigen::VectorXd>("vxd_as_cref", idx) == nullptr);
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
  std::string path;
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
                                       "std::vector<double>",
                                       "v2d_as_ref",
                                       "v2d_as_cref",
                                       "v3d_as_ref",
                                       "v3d_as_cref",
                                       "v6d_as_ref",
                                       "v6d_as_cref",
                                       "vxd_as_ref",
                                       "vxd_as_cref"};
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
    data.addToLoggerWithMemberPointerMacro(logger);
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
    data.addToLoggerWithGetterMacro(logger);
    for(iter = 252; iter <= 200; ++iter)
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
    path = logger.path();
  }
  auto latest = bfs::temp_directory_path() / "mc-rtc-test-logger-latest.bin";
  if(bfs::exists(latest))
  {
    bfs::remove(latest);
  }
  if(bfs::exists(path))
  {
    bfs::remove(path);
  }
}
