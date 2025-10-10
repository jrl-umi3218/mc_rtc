/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/Configuration.h>
#include <mc_rtc/pragma.h>

#include <boost/test/tools/old/interface.hpp>
#include <boost/test/unit_test.hpp>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <boost/mpl/list.hpp>

#include "utils.h"
#include <fstream>
#include <iostream>

bool silence_exception(const mc_rtc::Configuration::Exception & e)
{
  e.silence();
  return true;
}
#define MC_RTC_CHECK_THROW(statement) \
  BOOST_CHECK_EXCEPTION(statement, mc_rtc::Configuration::Exception, silence_exception)

namespace Eigen
{

bool operator==(const Quaterniond & lhs, const Quaterniond & rhs)
{
  return lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
}

} // namespace Eigen

static std::string YAML_DATA = R"(
int: 42
sint: -42
double: 42.5
doubleOrInt: 42.0
string: sometext
intV: [0, 1, 2, 3, 4, 5]
stringV: [a, b, c, foo, bar]
doubleA3: [1.1, 2.2, 3.3]
v2d: [1.0, 2.3]
v3d: [1.0, 2.3, -100]
v4d: [1.0, 2.3, 3.3, -100]
v6d: [1.0, -1.5, 2.0, -2.5, 3.0, -3.5]
g6d: [1.0, -1.5, 2.0, -2.5, 3.0, -3.5]
vXd: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
quat: [0.71, 0, 0.71, 0]
emptyArray: []
boolTrue: true
boolFalse: false
bool1: 1
bool0: 0
doubleDoublePair: [42.5, -42.5]
doubleStringPair: [42.5, sometext]
doubleDoublePairV: [[0.0, 1.1], [2.2, 3.3], [4.4, 5.5]]
tuple: [true, 42.42, sometext, [1.2, 3.4, 5.6]]
dict:
  boolTrue: true
  bool0: 0
  int: 42
  double: 42.5
  string: sometext
  intV: [0, 1, 2, 3, 4, 5]
  stringV: [a, b, c, foo, bar]
  v2d: [1.0, 2.3]
  v3d: [1.0, 2.3, -100]
  v4d: [1.0, 2.3, 3.3, -100]
  v6d: [1.0, -1.5, 2.0, -2.5, 3.0, -3.5]
  quat: [0.71, 0, 0.71, 0]
  doubleDoublePair: [42.5, -42.5]
  doubleStringPair: [42.5, sometext]
mapStr:
  str1: sometext1
  str2: sometext2
  str3: sometext3
  str4: sometext4
mapDouble:
  str1: 1.1
  str2: 2.2
  str3: 3.3
  str4: 4.4
mapDoubleV:
  str1: [1.0, 2.3, -100]
  str2: [1.0, -1.5, 2.0, -2.5, 3.0, -3.5]
  str3: [0.71, 0, 0.71, 0]
scalar_octal_number: 0123
scalar_octal_quoted_number: '0123'
scalar_octal_forced_string: !!str 0123
scalar_decimal_number: 123
scalar_decimal_quoted_number: '123'
scalar_decimal_forced_string: !!str 123
scalar_map:
  octal_number: 0123
  octal_quoted_number: '0123'
  octal_forced_string: !!str 0123
  decimal_number: 123
  decimal_quoted_number: '123'
  decimal_forced_string: !!str 123
)";

static std::string JSON_DATA = R"(
{
  "int": 42,
  "sint": -42,
  "double": 42.5,
  "doubleOrInt": 42.0,
  "string": "sometext",
  "intV": [0, 1, 2, 3, 4, 5],
  "stringV": ["a", "b", "c", "foo", "bar"],
  "doubleA3": [1.1, 2.2, 3.3],
  "v2d": [1.0, 2.3],
  "v3d": [1.0, 2.3, -100],
  "v4d": [1.0, 2.3, 3.3, -100],
  "v6d": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
  "vXd": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
  "quat": [0.71, 0, 0.71, 0],
  "emptyArray": [],
  "boolTrue": true,
  "boolFalse": false,
  "bool1": 1,
  "bool0": 0,
  "doubleDoublePair": [42.5, -42.5],
  "doubleStringPair": [42.5, "sometext"],
  "doubleDoublePairV": [[0.0, 1.1],[2.2, 3.3], [4.4, 5.5]],
  "tuple" : [true, 42.42, "sometext", [1.2, 3.4, 5.6]],
  "dict":
  {
    "boolTrue": true,
    "bool0": 0,
    "int" : 42,
    "double": 42.5,
    "string": "sometext",
    "intV": [0, 1, 2, 3, 4, 5],
    "stringV": ["a", "b", "c", "foo", "bar"],
    "v2d": [1.0, 2.3],
    "v3d": [1.0, 2.3, -100],
    "v4d": [1.0, 2.3, 3.3, -100],
    "v6d": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
    "quat": [0.71, 0, 0.71, 0],
    "doubleDoublePair": [42.5, -42.5],
    "doubleStringPair": [42.5, "sometext"]
  },
  "mapStr":
  {
    "str1": "sometext1",
    "str2": "sometext2",
    "str3": "sometext3",
    "str4": "sometext4"
  },
  "mapDouble":
  {
    "str1" : 1.1,
    "str2" : 2.2,
    "str3" : 3.3,
    "str4" : 4.4
  },
  "mapDoubleV":
  {
    "str1": [1.0, 2.3, -100],
    "str2": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
    "str3": [0.71, 0, 0.71, 0]
  },
  "scalar_octal_number": 83,
  "scalar_octal_quoted_number": 83,
  "scalar_octal_forced_string": "0123",
  "scalar_decimal_number": 123,
  "scalar_decimal_quoted_number": 123,
  "scalar_decimal_forced_string": "123",
  "scalar_map": {
    "octal_number": 83,
    "octal_quoted_number": 83,
    "octal_forced_string": "0123",
    "decimal_number": 123,
    "decimal_quoted_number": 123,
    "decimal_forced_string": "123"
  }
}
)";

std::string sampleConfig(bool fromDisk, bool json)
{
  const auto & data = json ? JSON_DATA : YAML_DATA;
  const std::string ext = json ? ".json" : ".yaml";
  return fromDisk ? makeConfigFile(data, ext) : data;
}

static std::string YAML_DATA2 = R"(
stringV: [a2, b2, c2]
int: 12
)";

static std::string JSON_DATA2 = R"(
{
  "stringV": ["a2", "b2", "c2"],
  "int": 12
}
)";

std::string sampleConfig2(bool fromDisk, bool json)
{
  const auto & data = json ? JSON_DATA2 : YAML_DATA2;
  const std::string ext = json ? ".json" : ".yaml";
  return fromDisk ? makeConfigFile(data, ext) : data;
}

void testConfigurationReading(mc_rtc::Configuration & config, bool fromDisk2, bool json2)
{
  /*! Check that accessing a non-existing entry throws */
  MC_RTC_CHECK_THROW(config("NONE"));

  BOOST_CHECK(config("NONE", std::optional<mc_rtc::Configuration>{}) == std::nullopt);

  BOOST_CHECK(config.find("NONE") == std::nullopt);

  /* int tests */
  {
    /*! Check that we can access a direct int entry */
    int a = config("int");
    BOOST_CHECK_EQUAL(a, 42);
    BOOST_CHECK(config("int").isInteger());
    BOOST_CHECK(config("int").isInt32());
    BOOST_CHECK(config("int").isInt64());
    BOOST_CHECK(config("int").isUInt32());
    BOOST_CHECK(config("int").isUInt64());

    /*! Check that we can get the value of an int entry out of there */
    int b = 0;
    config("int", b);
    BOOST_CHECK_EQUAL(b, 42);

    /*! Check that accessing an unexisting entry does not modify the
     * value */
    int c = 100;
    config("NONE", c);
    BOOST_CHECK_EQUAL(c, 100);

    /*! Check that accessing a different type of entry does not modify
     * the value */
    int d = 10;
    config("double", d);
    BOOST_CHECK_EQUAL(d, 10);
    BOOST_CHECK_EQUAL(config("double").isDouble(), true);

    /*! Access a int from a dict */
    int f = 0;
    f = config("dict")("int");
    BOOST_CHECK_EQUAL(f, 42);

    /*! Access a int from a dict */
    int g = 0;
    config("dict")("int", g);
    BOOST_CHECK_EQUAL(g, 42);

    /*! Similar checks with optional */

    std::optional<int> h = config("int");
    BOOST_CHECK(h.has_value() && h.value() == 42);

    std::optional<int> i;
    config("int", i);
    BOOST_CHECK(i.has_value() && i.value() == 42);

    auto j = config("double").operator std::optional<int>();
    BOOST_CHECK(!j.has_value());

    auto k = config("double", std::optional<int>{std::nullopt});
    BOOST_CHECK(k == std::nullopt);

    auto l = config.find<int>("int");
    BOOST_CHECK(l.has_value() && *l == 42);

    MC_RTC_CHECK_THROW([[maybe_unused]] auto m = config.find<int>("double"));
  }

  /* unsigned int tests */
  {
    /*! Check that we can access a direct unsigned int entry */
    unsigned int a = config("int");
    BOOST_CHECK_EQUAL(a, 42);
    BOOST_CHECK(config("int").isUInt32());
    BOOST_CHECK(config("int").isUInt64());

    /*! Check that we can get the value of an unsigned int entry out of there */
    unsigned int b = 0;
    config("int", b);
    BOOST_CHECK_EQUAL(b, 42);

    /*! Check that accessing an unexisting entry does not modify the
     * value */
    unsigned int c = 100;
    config("NONE", c);
    BOOST_CHECK_EQUAL(c, 100);

    /*! Check that accessing a different type of entry does not modify
     * the value */
    unsigned int d = 10;
    config("double", d);
    BOOST_CHECK_EQUAL(d, 10);

    /*! Check int to unsigned int does not happen */
    unsigned int e = 10;
    config("sint", e);
    BOOST_CHECK_EQUAL(e, 10);

    /*! Access a unsigned int from a dict */
    unsigned int f = 0;
    f = config("dict")("int");
    BOOST_CHECK_EQUAL(f, 42);

    /*! Access a unsigned int from a dict */
    unsigned int g = 0;
    config("dict")("int", g);
    BOOST_CHECK_EQUAL(g, 42);

    auto h = config.find<unsigned int>("int");
    BOOST_CHECK(h.has_value() && *h == 42);

    MC_RTC_CHECK_THROW([[maybe_unused]] auto i = config.find<unsigned int>("double"));
  }

  /* double tests */
  {
    /*! Check that we can access a direct double entry */
    double a = config("double");
    BOOST_CHECK_EQUAL(a, 42.5);
    BOOST_CHECK_EQUAL(config("double").isDouble(), true);

    /*! Check that we can get the value of an double entry out of there */
    double b = 0.;
    config("double", b);
    BOOST_CHECK_EQUAL(b, 42.5);

    /*! Check that accessing an unexisting entry does not modify the
     * value */
    double c = 100.;
    config("NONE", c);
    BOOST_CHECK_EQUAL(c, 100.);

    /*! Check that int entries are promoted to double */
    double d = 10.;
    config("int", d);
    BOOST_CHECK_EQUAL(d, static_cast<double>(42));

    /*! Access a double from a dict */
    double e = 0.;
    e = config("dict")("double");
    BOOST_CHECK_EQUAL(e, 42.5);

    /*! Access a double from a dict */
    double f = 0.;
    config("dict")("double", f);
    BOOST_CHECK_EQUAL(f, 42.5);
  }

  /* string tests */
  {
    /*! Check that we can access a string entry */
    std::string a = config("string");
    BOOST_CHECK_EQUAL(a, "sometext");

    std::string f(config("string"));
    BOOST_CHECK_EQUAL(f, "sometext");

    std::string b = "";
    config("string", b);
    BOOST_CHECK_EQUAL(b, "sometext");

    std::string c = "another";
    config("NONE", c);
    BOOST_CHECK_EQUAL(c, "another");

    config("int", c);
    BOOST_CHECK_EQUAL(c, "another");

    std::string e = "another";
    e = (std::string)config("string");
    BOOST_CHECK_EQUAL(e, "sometext");

    std::string d = "another";
    config("dict")("string", d);
    BOOST_CHECK_EQUAL(d, "sometext");
  }

  /* Eigen::Vector2d test */
  {
    Eigen::Vector2d ref;
    ref << 1.0, 2.3;
    Eigen::Vector2d zero = Eigen::Vector2d::Zero();

    Eigen::Vector2d a = config("v2d");
    BOOST_CHECK_EQUAL(a, ref);

    Eigen::Vector2d b = Eigen::Vector2d::Zero();
    config("v2d", b);
    BOOST_CHECK_EQUAL(b, ref);

    Eigen::Vector2d c = Eigen::Vector2d::Zero();
    config("v6d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(Eigen::Vector2d d = config("v6d"));
    MC_RTC_CHECK_THROW(auto d = config.find<Eigen::Vector2d>("v6d"));
    MC_RTC_diagnostic_pop

    Eigen::Vector2d e = Eigen::Vector2d::Zero();
    e = config("dict")("v2d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector2d f = Eigen::Vector2d::Zero();
    config("dict")("v2d", f);
    BOOST_CHECK_EQUAL(f, ref);
  }

  /* mc_rbdyn::Gains2d test */
  {
    mc_rbdyn::Gains2d ref;
    ref << 1.0, 2.3;
    mc_rbdyn::Gains2d zero = mc_rbdyn::Gains2d::Zero();

    mc_rbdyn::Gains2d a = config("v2d");
    BOOST_CHECK_EQUAL(a, ref);

    mc_rbdyn::Gains2d b = mc_rbdyn::Gains2d::Zero();
    config("v2d", b);
    BOOST_CHECK_EQUAL(b, ref);

    mc_rbdyn::Gains2d c = mc_rbdyn::Gains2d::Zero();
    config("v6d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(mc_rbdyn::Gains2d d = config("v6d"));
    MC_RTC_CHECK_THROW(auto d = config.find<mc_rbdyn::Gains2d>("v6d"));
    MC_RTC_diagnostic_pop

    mc_rbdyn::Gains2d e = mc_rbdyn::Gains2d::Zero();
    e = config("dict")("v2d");
    BOOST_CHECK_EQUAL(e, ref);

    mc_rbdyn::Gains2d f = mc_rbdyn::Gains2d::Zero();
    config("dict")("v2d", f);
    BOOST_CHECK_EQUAL(f, ref);

    ref.setConstant(42.5);

    mc_rbdyn::Gains2d g = config("double");
    BOOST_CHECK_EQUAL(g, ref);

    mc_rbdyn::Gains2d h = mc_rbdyn::Gains2d::Zero();
    config("double", h);
    BOOST_CHECK_EQUAL(h, ref);

    mc_rbdyn::Gains2d i = mc_rbdyn::Gains2d::Zero();
    i = config("dict")("double");
    BOOST_CHECK_EQUAL(i, ref);
  }

  /* Eigen::Vector3d test */
  {
    Eigen::Vector3d ref;
    ref << 1.0, 2.3, -100;
    Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    Eigen::Vector3d a = config("v3d");
    BOOST_CHECK_EQUAL(a, ref);

    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    config("v3d", b);
    BOOST_CHECK_EQUAL(b, ref);

    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    config("v6d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(Eigen::Vector3d d = config("v6d"));
    MC_RTC_CHECK_THROW(auto d = config.find<Eigen::Vector3d>("v6d"));
    MC_RTC_diagnostic_pop

    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    e = config("dict")("v3d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    config("dict")("v3d", f);
    BOOST_CHECK_EQUAL(f, ref);
  }

  /* mc_rbdyn::Gains3d test */
  {
    mc_rbdyn::Gains3d ref;
    ref << 1.0, 2.3, -100;
    mc_rbdyn::Gains3d zero = mc_rbdyn::Gains3d::Zero();

    mc_rbdyn::Gains3d a = config("v3d");
    BOOST_CHECK_EQUAL(a, ref);

    mc_rbdyn::Gains3d b = mc_rbdyn::Gains3d::Zero();
    config("v3d", b);
    BOOST_CHECK_EQUAL(b, ref);

    mc_rbdyn::Gains3d c = mc_rbdyn::Gains3d::Zero();
    config("v6d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(mc_rbdyn::Gains3d d = config("v6d"));
    MC_RTC_CHECK_THROW(auto d = config.find<mc_rbdyn::Gains3d>("v6d"));
    MC_RTC_diagnostic_pop

    mc_rbdyn::Gains3d e = mc_rbdyn::Gains3d::Zero();
    e = config("dict")("v3d");
    BOOST_CHECK_EQUAL(e, ref);

    mc_rbdyn::Gains3d f = mc_rbdyn::Gains3d::Zero();
    config("dict")("v3d", f);
    BOOST_CHECK_EQUAL(f, ref);

    ref.setConstant(42.5);

    mc_rbdyn::Gains3d g = config("double");
    BOOST_CHECK_EQUAL(g, ref);

    mc_rbdyn::Gains3d h = mc_rbdyn::Gains3d::Zero();
    config("double", h);
    BOOST_CHECK_EQUAL(h, ref);

    mc_rbdyn::Gains3d i = mc_rbdyn::Gains3d::Zero();
    i = config("dict")("double");
    BOOST_CHECK_EQUAL(i, ref);
  }

  /* Eigen::Vector4d test */
  {
    Eigen::Vector4d ref;
    ref << 1.0, 2.3, 3.3, -100;
    Eigen::Vector4d zero = Eigen::Vector4d::Zero();

    Eigen::Vector4d a = config("v4d");
    BOOST_CHECK_EQUAL(a, ref);

    Eigen::Vector4d b = Eigen::Vector4d::Zero();
    config("v4d", b);
    BOOST_CHECK_EQUAL(b, ref);

    Eigen::Vector4d c = Eigen::Vector4d::Zero();
    config("v6d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(Eigen::Vector4d d = config("v6d"));
    MC_RTC_CHECK_THROW(auto d = config.find<Eigen::Vector4d>("v6d"));
    MC_RTC_diagnostic_pop

    Eigen::Vector4d e = Eigen::Vector4d::Zero();
    e = config("dict")("v4d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector4d f = Eigen::Vector4d::Zero();
    config("dict")("v4d", f);
    BOOST_CHECK_EQUAL(f, ref);
  }

  /* Eigen::Vector6d test */
  {
    Eigen::Vector6d ref;
    ref << 1.0, -1.5, 2.0, -2.5, 3.0, -3.5;
    Eigen::Vector6d zero = Eigen::Vector6d::Zero();

    Eigen::Vector6d a = config("v6d");
    BOOST_CHECK_EQUAL(a, ref);

    Eigen::Vector6d b = Eigen::Vector6d::Zero();
    config("v6d", b);
    BOOST_CHECK_EQUAL(b, ref);

    Eigen::Vector6d c = Eigen::Vector6d::Zero();
    config("v3d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(Eigen::Vector6d d = config("v3d"));
    MC_RTC_CHECK_THROW(auto d = config.find<Eigen::Vector6d>("v3d"));
    MC_RTC_diagnostic_pop

    Eigen::Vector6d e = Eigen::Vector6d::Zero();
    e = config("dict")("v6d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector6d f = Eigen::Vector6d::Zero();
    config("dict")("v6d", f);
    BOOST_CHECK_EQUAL(f, ref);
  }

  /* mc_rbdyn::Gains6d test */
  {
    mc_rbdyn::Gains6d ref;
    ref << 1.0, -1.5, 2.0, -2.5, 3.0, -3.5;
    mc_rbdyn::Gains6d zero = mc_rbdyn::Gains6d::Zero();

    mc_rbdyn::Gains6d a = config("v6d");
    BOOST_CHECK_EQUAL(a, ref);

    mc_rbdyn::Gains6d b = mc_rbdyn::Gains6d::Zero();
    config("v6d", b);
    BOOST_CHECK_EQUAL(b, ref);

    mc_rbdyn::Gains6d c = mc_rbdyn::Gains6d::Zero();
    config("v3d", c);
    BOOST_CHECK_EQUAL(c, zero);

    MC_RTC_diagnostic_push
    MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
    MC_RTC_CHECK_THROW(mc_rbdyn::Gains6d d = config("v3d"));
    MC_RTC_CHECK_THROW(auto d = config.find<mc_rbdyn::Gains6d>("v3d"));
    MC_RTC_diagnostic_pop

    mc_rbdyn::Gains6d e = mc_rbdyn::Gains6d::Zero();
    e = config("dict")("v6d");
    BOOST_CHECK_EQUAL(e, ref);

    mc_rbdyn::Gains6d f = mc_rbdyn::Gains6d::Zero();
    config("dict")("v6d", f);
    BOOST_CHECK_EQUAL(f, ref);

    ref.setConstant(42.5);

    mc_rbdyn::Gains6d g = config("double");
    BOOST_CHECK_EQUAL(g, ref);

    mc_rbdyn::Gains6d h = mc_rbdyn::Gains6d::Zero();
    config("double", h);
    BOOST_CHECK_EQUAL(h, ref);

    mc_rbdyn::Gains6d i = mc_rbdyn::Gains6d::Zero();
    i = config("dict")("double");
    BOOST_CHECK_EQUAL(i, ref);
  }

  /* Eigen::VectorXd test */
  {
    Eigen::VectorXd ref3(3);
    ref3 << 1.0, 2.3, -100;

    Eigen::VectorXd ref6(6);
    ref6 << 1.0, -1.5, 2.0, -2.5, 3.0, -3.5;

    Eigen::VectorXd ref(11);
    ref << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;

    Eigen::VectorXd a = config("v6d");
    BOOST_CHECK_EQUAL(a, ref6);

    Eigen::VectorXd b = Eigen::VectorXd::Zero(0);
    config("v6d", b);
    BOOST_CHECK_EQUAL(b, ref6);

    Eigen::VectorXd c = Eigen::VectorXd::Zero(0);
    config("v3d", c);
    BOOST_CHECK_EQUAL(c, ref3);

    Eigen::VectorXd d = config("vXd");
    BOOST_CHECK_EQUAL(d, ref);

    MC_RTC_CHECK_THROW(Eigen::VectorXd e = config("int"));
    MC_RTC_CHECK_THROW(auto d = config.find<Eigen::VectorXd>("int"));

    Eigen::VectorXd f = Eigen::VectorXd::Zero(0);
    f = config("dict")("v6d");
    BOOST_CHECK_EQUAL(f, ref6);

    Eigen::VectorXd g = Eigen::VectorXd::Zero(0);
    config("dict")("v3d", g);
    BOOST_CHECK_EQUAL(g, ref3);

    Eigen::VectorXd h = Eigen::VectorXd::Zero(100);
    config("emptyArray", h);
    BOOST_CHECK_EQUAL(h, Eigen::VectorXd::Zero(0));
  }

  /* Eigen::Quaterniond test */
  {
    Eigen::Quaterniond ref(0.71, 0, 0.71, 0);
    ref.normalize();

    Eigen::Quaterniond a = config("quat");
    if(a == ref) {}
    BOOST_CHECK(a == ref);

    Eigen::Quaterniond b;
    config("quat", b);
    BOOST_CHECK(b == ref);

    Eigen::Quaterniond c = config("dict")("quat");
    if(c == ref) {}
    BOOST_CHECK(c == ref);

    Eigen::Quaterniond d;
    config("dict")("quat", d);
    BOOST_CHECK(d == ref);
  }

  /* Bool test */
  {
    bool a = config("boolTrue");
    BOOST_CHECK_EQUAL(a, true);

    bool b = false;
    config("boolTrue", b);
    BOOST_CHECK_EQUAL(b, true);

    bool c = config("boolFalse");
    BOOST_CHECK_EQUAL(c, false);

    bool d = true;
    config("boolFalse", d);
    BOOST_CHECK_EQUAL(d, false);

    bool e = false;
    config("bool1", e);
    BOOST_CHECK_EQUAL(e, true);

    bool f = true;
    config("bool0", f);
    BOOST_CHECK_EQUAL(f, false);

    bool g = config("dict")("boolTrue");
    BOOST_CHECK_EQUAL(g, true);

    bool h = true;
    config("dict")("bool0", h);
    BOOST_CHECK_EQUAL(h, false);
  }

  /* vector<int> test */
  {
    std::vector<int> ref = {0, 1, 2, 3, 4, 5};

    std::vector<int> a = config("intV");
    BOOST_CHECK(a == ref);

    std::vector<int> b(0);
    config("intV", b);
    BOOST_CHECK(b == ref);

    std::vector<int> c(0);
    config("dict")("intV", c);
    BOOST_CHECK(c == ref);
  }

  /* vector<double> test */
  {
    std::vector<double> ref = {0, 1, 2, 3, 4, 5};

    std::vector<double> a = config("intV");
    BOOST_CHECK(a == ref);

    std::vector<double> b(0);
    config("intV", b);
    BOOST_CHECK(b == ref);

    std::vector<double> c(0);
    config("dict")("intV", c);
    BOOST_CHECK(c == ref);
  }

  /* vector<string> test */
  {
    std::vector<std::string> ref = {"a", "b", "c", "foo", "bar"};

    std::vector<std::string> a = config("stringV");
    BOOST_CHECK(a == ref);

    std::vector<std::string> b(0);
    config("stringV", b);
    BOOST_CHECK(b == ref);

    std::vector<std::string> c(0);
    config("dict")("stringV", c);
    BOOST_CHECK(c == ref);
  }

  /* array<double, 3> test */
  {
    std::array<double, 3> ref = {{1.1, 2.2, 3.3}};

    std::array<double, 3> a = config("doubleA3");
    BOOST_CHECK(a == ref);

    std::array<double, 3> b;
    config("doubleA3", b);
    BOOST_CHECK(b == ref);
  }

  /* pair<double, double> test */
  {
    std::pair<double, double> ref = {42.5, -42.5};

    std::pair<double, double> a = config("doubleDoublePair");
    BOOST_CHECK(a == ref);

    std::pair<double, double> b = {0, 0};
    config("doubleDoublePair", b);
    BOOST_CHECK(b == ref);

    std::pair<double, double> c = {0, 0};
    config("dict")("doubleDoublePair", c);
    BOOST_CHECK(c == ref);

    MC_RTC_CHECK_THROW(c = (std::pair<double, double>)config("quat"));
    MC_RTC_CHECK_THROW(c = (std::pair<double, double>)config("doubleStringPair"));
  }

  /* pair<double, string> test */
  {
    std::pair<double, std::string> ref = {42.5, "sometext"};

    std::pair<double, std::string> a = config("doubleStringPair");
    BOOST_CHECK(a == ref);

    std::pair<double, std::string> b = {0, ""};
    config("doubleStringPair", b);
    BOOST_CHECK(b == ref);

    std::pair<double, std::string> c = {0, ""};
    config("dict")("doubleStringPair", c);
    BOOST_CHECK(b == ref);
  }

  /* vector<pair<double,double>> */
  {
    using test_t = std::vector<std::pair<double, double>>;
    test_t ref = {{0.0, 1.1}, {2.2, 3.3}, {4.4, 5.5}};

    test_t a = config("doubleDoublePairV");
    BOOST_CHECK(a == ref);

    test_t b = {};
    config("doubleDoublePairV", b);
    BOOST_CHECK(b == ref);
  }

  /* "tuple" */
  {
    auto tuple = config("tuple");
    BOOST_CHECK(tuple.size() == 4);
    Eigen::Vector3d vec = tuple[3];
    std::tuple<bool, double, std::string, Eigen::Vector3d> data = std::make_tuple(tuple[0], tuple[1], tuple[2], vec);
    BOOST_CHECK(std::get<0>(data));
    BOOST_CHECK(std::get<1>(data) == 42.42);
    BOOST_CHECK(std::get<2>(data) == "sometext");
    BOOST_CHECK(std::get<3>(data) == Eigen::Vector3d(1.2, 3.4, 5.6));
  }

  /* map<string, string> */
  {
    using test_t = std::map<std::string, std::string>;

    test_t ref;
    ref["str1"] = "sometext1";
    ref["str2"] = "sometext2";
    ref["str3"] = "sometext3";
    ref["str4"] = "sometext4";

    test_t a = config("mapStr");
    BOOST_CHECK(a == ref);

    test_t b = {};
    config("mapStr", b);
    BOOST_CHECK(b == ref);
  }

  /* map<string, double> */
  {
    using test_t = std::map<std::string, double>;

    test_t ref;
    ref["str1"] = 1.1;
    ref["str2"] = 2.2;
    ref["str3"] = 3.3;
    ref["str4"] = 4.4;

    test_t a = config("mapDouble");
    BOOST_CHECK(a == ref);

    test_t b = {};
    config("mapDouble", b);
    BOOST_CHECK(b == ref);
  }

  /* map<string, vector<double>> */
  {
    using test_t = std::map<std::string, std::vector<double>>;

    test_t ref;
    ref["str1"] = {1.0, 2.3, -100};
    ref["str2"] = {1.0, -1.5, 2.0, -2.5, 3.0, -3.5};
    ref["str3"] = {0.71, 0, 0.71, 0};

    test_t a = config("mapDoubleV");
    BOOST_CHECK(a == ref);

    test_t b = {};
    config("mapDoubleV", b);
    BOOST_CHECK(b == ref);
  }

  /* Check load */
  {
    if(fromDisk2)
    {
      std::string path = sampleConfig2(true, json2);
      config.load(path);
      bfs::remove(path);
    }
    else
    {
      if(json2) { config.loadData(sampleConfig2(false, json2)); }
      else
      {
        config.loadYAMLData(sampleConfig2(false, json2));
      }
    }
    int a = config("int");
    BOOST_CHECK_EQUAL(a, 12);

    /* Check that other values are still there */
    int b = config("sint");
    BOOST_CHECK_EQUAL(b, -42);
    BOOST_CHECK(config("sint").isInteger());
    BOOST_CHECK(config("sint").isInt32());
    BOOST_CHECK(config("sint").isInt64());
    BOOST_CHECK(!config("sint").isUInt32());
    BOOST_CHECK(!config("sint").isUInt64());

    /* Check with a more complex type */
    std::vector<std::string> ref2 = {"a2", "b2", "c2"};
    std::vector<std::string> c = config("stringV");
    BOOST_CHECK(c == ref2);
  }

  auto check_scalar = [](const mc_rtc::Configuration & cfg, const std::string & key, bool expect_string, int expect_int,
                         const std::string & expect_str)
  {
    if(expect_string)
    {
      BOOST_CHECK(cfg(key).isString());
      BOOST_CHECK_EQUAL((std::string)cfg(key), expect_str);
    }
    else
    {
      BOOST_CHECK(cfg(key).isInteger());
      BOOST_CHECK_EQUAL((int)cfg(key), expect_int);
    }
  };

  /* YAML octal scalar tests */
  check_scalar(config, "scalar_octal_number", false, 83, "");
  check_scalar(config, "scalar_octal_quoted_number", false, 83, "");
  check_scalar(config, "scalar_octal_forced_string", true, 0, "0123");

  /* YAML decimal scalar tests */
  check_scalar(config, "scalar_decimal_number", false, 123, "");
  check_scalar(config, "scalar_decimal_quoted_number", false, 123, "");
  check_scalar(config, "scalar_decimal_forced_string", true, 0, "123");

  /* YAML scalar map tests */
  auto scalar_map = config("scalar_map");
  check_scalar(scalar_map, "octal_number", false, 83, "");
  check_scalar(scalar_map, "octal_quoted_number", false, 83, "");
  check_scalar(scalar_map, "octal_forced_string", true, 0, "0123");
  check_scalar(scalar_map, "decimal_number", false, 123, "");
  check_scalar(scalar_map, "decimal_quoted_number", false, 123, "");
  check_scalar(scalar_map, "decimal_forced_string", true, 0, "123");
}

mc_rtc::Configuration makeConfig(bool fromDisk, bool json)
{
  if(fromDisk)
  {
    auto path = sampleConfig(fromDisk, json);
    auto out = mc_rtc::Configuration(path);
    bfs::remove(path);
    return out;
  }
  else
  {
    const auto & data = sampleConfig(fromDisk, json);
    if(json) { return mc_rtc::Configuration::fromData(data); }
    else
    {
      return mc_rtc::Configuration::fromYAMLData(data);
    }
  }
}

void TestConfigurationLoadingAndReading(bool fromDisk, bool json, bool fromDisk2, bool json2)
{
  auto config = makeConfig(fromDisk, json);
  testConfigurationReading(config, fromDisk2, json2);
}

BOOST_AUTO_TEST_CASE(TestConfigurationReading)
{
  TestConfigurationLoadingAndReading(false, false, false, false);
  TestConfigurationLoadingAndReading(false, false, false, true);
  TestConfigurationLoadingAndReading(false, false, true, false);
  TestConfigurationLoadingAndReading(false, false, true, true);
  TestConfigurationLoadingAndReading(false, true, false, false);
  TestConfigurationLoadingAndReading(false, true, false, true);
  TestConfigurationLoadingAndReading(false, true, true, false);
  TestConfigurationLoadingAndReading(false, true, true, true);
  TestConfigurationLoadingAndReading(true, false, false, false);
  TestConfigurationLoadingAndReading(true, false, false, true);
  TestConfigurationLoadingAndReading(true, false, true, false);
  TestConfigurationLoadingAndReading(true, false, true, true);
  TestConfigurationLoadingAndReading(true, true, false, false);
  TestConfigurationLoadingAndReading(true, true, false, true);
  TestConfigurationLoadingAndReading(true, true, true, false);
  TestConfigurationLoadingAndReading(true, true, true, true);
}

BOOST_AUTO_TEST_CASE(TestConfigurationWriting)
{
  std::string tmpF = getTmpFile();
  mc_rtc::Configuration config_ref;

  bool ref_bool = false;
  config_ref.add("bool", ref_bool);
  unsigned int ref_uint = 42;
  config_ref.add("uint", ref_uint);
  int ref_int = -42;
  config_ref.add("int", ref_int);
  double ref_double = 42.5;
  config_ref.add("double", ref_double);
  std::string ref_string = "sometext";
  config_ref.add("string", ref_string);
  Eigen::Vector3d ref_v3d = {1.2, 3.4, 5.6};
  config_ref.add("v3d", ref_v3d);
  Eigen::Vector6d ref_v6d;
  ref_v6d << 0.1, 1.2, 2.3, 3.4, 4.5, 5.6;
  config_ref.add("v6d", ref_v6d);
  Eigen::VectorXd ref_vxd(5);
  ref_vxd << 0.1, 3.2, 4.2, 4.5, 5.4;
  config_ref.add("vxd", ref_vxd);
  Eigen::Quaterniond ref_quat{0.71, 0., 0.71, 0.};
  ref_quat.normalize();
  config_ref.add("quat", ref_quat);
  std::vector<int> ref_int_v = {0, 1, 2, 3, 4, 5};
  config_ref.add("int_v", ref_int_v);
  std::vector<double> ref_double_v = {0.1, 1.0, 0.2, 2.0, 0.3};
  config_ref.add("double_v", ref_double_v);
  std::vector<std::vector<double>> ref_double_v_v = {ref_double_v, ref_double_v, {0}, {}, {5.0, 4.0, 3.5}};
  config_ref.add("double_v_v", ref_double_v_v);
  std::vector<Eigen::Vector3d> ref_v3d_v;
  for(size_t i = 0; i < 10; ++i) { ref_v3d_v.push_back(Eigen::Vector3d::Random()); }
  config_ref.add("v3d_v", ref_v3d_v);
  std::array<double, 3> ref_d_a3 = {{1.1, 2.2, 3.3}};
  config_ref.add("d_a3", ref_d_a3);
  std::vector<std::array<double, 3>> ref_a3_v;
  for(size_t i = 0; i < 5; ++i) { ref_a3_v.push_back(ref_d_a3); };
  config_ref.add("a3_v", ref_a3_v);
  std::array<std::array<double, 3>, 3> ref_a3_a;
  for(size_t i = 0; i < 3; ++i) { ref_a3_a[i] = ref_d_a3; }
  config_ref.add("a3_a", ref_a3_a);
  config_ref.add("dict");
  config_ref("dict").add("int", ref_int);
  config_ref.add("dict2").add("double_v", ref_double_v);

  config_ref.save(tmpF);

  mc_rtc::Configuration config_test(tmpF);
  BOOST_CHECK(config_test("bool") == ref_bool);
  BOOST_CHECK(config_test("uint") == ref_uint);
  BOOST_CHECK(config_test("int") == ref_int);
  BOOST_CHECK(config_test("double") == ref_double);
  BOOST_CHECK(config_test("string") == ref_string);
  BOOST_CHECK(config_test("v3d") == ref_v3d);
  BOOST_CHECK(config_test("v6d") == ref_v6d);
  BOOST_CHECK(config_test("vxd") == ref_vxd);
  BOOST_CHECK(config_test("quat") == ref_quat);
  BOOST_CHECK(config_test("int_v") == ref_int_v);
  BOOST_CHECK(config_test("double_v") == ref_double_v);
  BOOST_CHECK(config_test("double_v_v") == ref_double_v_v);
  std::vector<Eigen::Vector3d> test_v3d_v = config_test("v3d_v");
  BOOST_REQUIRE(test_v3d_v.size() == ref_v3d_v.size());
  for(size_t i = 0; i < test_v3d_v.size(); ++i) { BOOST_CHECK(test_v3d_v[i].isApprox(ref_v3d_v[i], 1e-9)); }
  BOOST_CHECK(config_test("d_a3") == ref_d_a3);
  BOOST_CHECK(config_test("a3_v") == ref_a3_v);
  BOOST_CHECK(config_test("a3_a") == ref_a3_a);
  BOOST_REQUIRE(config_test.has("dict"));
  BOOST_REQUIRE(config_test.find("dict").has_value());
  BOOST_CHECK(config_test("dict")("int") == ref_int);
  BOOST_REQUIRE(config_test.has("dict2"));
  BOOST_REQUIRE(config_test.find("dict2").has_value());
  BOOST_CHECK(config_test("dict2")("double_v") == ref_double_v);

  /* Save a part of the configuration */
  config_test("dict2")("double_v").save(tmpF);

  mc_rtc::Configuration config_partial(tmpF);
  BOOST_CHECK(config_partial == ref_double_v);

  bfs::remove(tmpF);
}

BOOST_AUTO_TEST_CASE(TestConfigurationCeption)
{
  int ref_int = -42;
  std::vector<double> ref_double_v = {0.1, 1.0, 0.2, 2.0, 0.3};
  /* Create a simple config */
  mc_rtc::Configuration config_1;
  config_1.add("int", ref_int);
  /* Create a second simple config */
  mc_rtc::Configuration config_2;
  config_2.add("double_v", ref_double_v);
  /* Add config_2 in config_1 */
  config_1.add("config_2", config_2);
  BOOST_REQUIRE(config_1.has("config_2") && config_1("config_2").has("double_v"));
  BOOST_CHECK(config_1("config_2")("double_v") == ref_double_v);
  /* Add a part of config_2 in config_1 */
  config_1.add("double_v", config_2("double_v"));
  BOOST_REQUIRE(config_1.has("double_v"));
  BOOST_CHECK(config_1("double_v") == ref_double_v);
  /* Create an array and put a few copies of config_2 inside */
  config_1.array("config_2_v", 5); // Reserve 5 but add 10 elements just 'cause
  for(size_t i = 0; i < 10; ++i) { config_1("config_2_v").push(config_2); }
  BOOST_REQUIRE(config_1.has("config_2_v"));
  BOOST_REQUIRE(config_1("config_2_v").size() == 10);
  for(size_t i = 0; i < 10; ++i) { BOOST_CHECK(config_1("config_2_v")[i]("double_v") == ref_double_v); }
  /* Add config_1 to config_1 */
  config_1.add("config_1", config_1);
  BOOST_REQUIRE(config_1.has("config_1"));
  BOOST_CHECK(config_1("config_1")("int") == ref_int);
  BOOST_REQUIRE(config_1("config_1").has("config_2_v"));
  BOOST_REQUIRE(config_1("config_1")("config_2_v").size() == 10);
  for(size_t i = 0; i < 10; ++i) { BOOST_CHECK(config_1("config_1")("config_2_v")[i]("double_v") == ref_double_v); }
}

struct Foo;
Foo & operator<<(Foo & f, const mc_rtc::Configuration & config);
struct Foo
{
  Foo() {}
  Foo(const std::string & name, double d) : name(name), d(d) {}
  std::string name;
  double d = 0.0;
  bool operator==(const Foo & rhs) const { return rhs.name == this->name && rhs.d == this->d; }
};

namespace mc_rtc
{
template<>
struct ConfigurationLoader<Foo>
{
  static Foo load(const mc_rtc::Configuration & config) { return {config("name"), config("d")}; }

  static mc_rtc::Configuration save(const Foo & f)
  {
    mc_rtc::Configuration config;
    config.add("name", f.name);
    config.add("d", f.d);
    return config;
  }
};
} // namespace mc_rtc

namespace test
{

struct Bar
{
  Bar() {}
  Bar(const std::string & name, double d) : name(name), d(d) {}
  std::string name;
  double d = 0.0;
  inline bool operator==(const Bar & rhs) const { return rhs.name == this->name && rhs.d == this->d; }

  static Bar fromConfiguration(const mc_rtc::Configuration & config) { return {config("name"), config("d")}; }

  mc_rtc::Configuration toConfiguration() const
  {
    mc_rtc::Configuration config;
    config.add("name", name);
    config.add("d", d);
    return config;
  }
};

} // namespace test

static_assert(!mc_rtc::internal::has_static_fromConfiguration_v<double>);
static_assert(!mc_rtc::internal::has_static_fromConfiguration_v<Foo>);
static_assert(mc_rtc::internal::has_static_fromConfiguration_v<test::Bar>);

static_assert(!mc_rtc::internal::has_toConfiguration_method_v<double>);
static_assert(!mc_rtc::internal::has_toConfiguration_method_v<Foo>);
static_assert(mc_rtc::internal::has_toConfiguration_method_v<test::Bar>);

using user_types = boost::mpl::list<Foo, test::Bar>;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestUserDefinedConversions, T, user_types)
{
  T f_ref{"foo", 1.0};
  mc_rtc::Configuration config;
  config.add("foo", f_ref);

  T f1 = [&config]()
  {
    if constexpr(mc_rtc::internal::has_configuration_load_object_v<T>)
    {
      T f1 = mc_rtc::ConfigurationLoader<T>::load(config("foo"));
      return f1;
    }
    else
    {
      T f1 = T::fromConfiguration(config("foo"));
      return f1;
    }
  }();
  BOOST_CHECK(f1 == f_ref);

  T f2;
  f2 = config("foo");
  BOOST_CHECK(f2 == f_ref);

  T f3 = config("foo");
  BOOST_CHECK(f3 == f_ref);

  std::vector<T> v_ref{f1, f2};
  config.add("foo_v", v_ref);

  std::vector<T> v1 = config("foo_v");
  BOOST_CHECK(v1 == v_ref);

  config.array("foo_v2");
  for(const auto & f : v_ref) { config("foo_v2").push(f); }
  std::vector<T> v2 = config("foo_v2");
  BOOST_CHECK(v2 == v_ref);
}

BOOST_AUTO_TEST_CASE(TestLoadConfigurationInConfiguration)
{
  auto make_c1 = []()
  {
    mc_rtc::Configuration c1;
    c1.add("a", std::vector<int>{0, 1, 2, 3});
    c1.add("i", 42);
    c1.add("d", 42.42);
    {
      auto o1 = c1.add("o");
      o1.add("i", 42);
      o1.add("d", 42.42);
      auto o2 = o1.add("o");
      o2.add("i", 42);
      o2.add("d", 42.42);
    }
    return c1;
  };
  auto c1 = make_c1();
  // c2 should:
  // - overwrite "a"
  // - overwrite "i"
  // - add "dd
  // - change "o"("d")
  // - add "o"("ii")
  // - replace "o"("o") with an array
  std::vector<double> ref_v = {0.1, 0.2, 0.3};
  mc_rtc::Configuration c2;
  c2.add("a", ref_v);
  c2.add("i", 0);
  c2.add("dd", 100.0);
  {
    auto o1 = c2.add("o");
    o1.add("ii", 42);
    o1.add("d", 0.42);
    o1.add("o", ref_v);
  }
  c1.load(c2);
  BOOST_REQUIRE(c1("a") == ref_v);
  BOOST_REQUIRE(c1("i") == 0);
  BOOST_REQUIRE(c1("dd") == 100.0);
  BOOST_REQUIRE(c1("o")("ii") == 42);
  BOOST_REQUIRE(c1("o")("d") == 0.42);
  BOOST_REQUIRE(c1("o")("o") == ref_v);
}

BOOST_AUTO_TEST_CASE(TestNestedLoading)
{
  mc_rtc::Configuration c1;
  c1.add("a", std::vector<int>{0, 1, 2, 3});
  c1.add("i", 42);
  {
    auto nested = c1.add("in");
    nested.add("value", 42.42);
  }
  mc_rtc::Configuration c2;
  {
    auto nested = c2.add("nested");
    nested.add("b", true);
  }
  c1("in").load(c2("nested"));
  BOOST_REQUIRE(c1("in").has("b"));
  BOOST_REQUIRE(c1("in")("b") == true);
  c1("in").load(c2("nested")("b"));
  BOOST_REQUIRE(c1("in").keys().size() == 0);
  BOOST_REQUIRE(c1("in").size() == 0);
  BOOST_REQUIRE(c1("in") == true);
  mc_rtc::Configuration c3;
  c3.load(c2("nested")("b"));
  BOOST_REQUIRE(c3 == true);
  MC_RTC_CHECK_THROW(c3.add("nested"));
  c1.load(c2("nested")("b"));
  BOOST_REQUIRE(c1 == true);
  MC_RTC_CHECK_THROW(c1.add("nested"));
}

BOOST_AUTO_TEST_CASE(TestConfigurartionRemove)
{
  auto config = mc_rtc::Configuration::fromData(sampleConfig(false, true));
  BOOST_CHECK(!config.remove("NONE"));
  BOOST_CHECK(config.has("v3d"));
  BOOST_CHECK(config.remove("v3d"));
  BOOST_CHECK(!config.has("v3d"));
  BOOST_CHECK(config("dict").has("bool0"));
  BOOST_CHECK(config("dict").remove("bool0"));
  BOOST_CHECK(!config("dict").has("bool0"));
  BOOST_CHECK(config.remove("dict"));
  BOOST_CHECK(!config.has("dict"));
}

static std::string YAML_DATA3 = R"(
v3d: [1.0, 2.0, 3.0]
v3dPair:
  - [1.0, 2.0, 3.0]
  - [1.0, 2.0, 3.0]
dict:
  v3d: [1.0, 2.0, 3.0]
  v3dPair:
    - [1.0, 2.0, 3.0]
    - [1.0, 2.0, 3.0]
array:
  - v3d: [1.0, 2.0, 3.0]
  - v3dPair:
      - [1.0, 2.0, 3.0]
      - [1.0, 2.0, 3.0]
deep:
  - {}
  - {}
  - object1:
      key1: {}
      key2:
        - {}
        - object2:
            key1: {}
            v3d: [1.0, 2.0, 3.0]
)";

#define CHECK_ERROR_MESSAGE(EXPECTED, ...)      \
  try                                           \
  {                                             \
    __VA_ARGS__;                                \
  }                                             \
  catch(mc_rtc::Configuration::Exception & exc) \
  {                                             \
    BOOST_REQUIRE(exc.msg() == EXPECTED);       \
    exc.silence();                              \
  }

BOOST_AUTO_TEST_CASE(TestConfigurationErrorMessage)
{
  auto config = mc_rtc::Configuration::fromYAMLData(YAML_DATA3);
  CHECK_ERROR_MESSAGE("No entry named NONE in the configuration (error path: ())", config("NONE"));
  MC_RTC_diagnostic_push
  MC_RTC_diagnostic_ignored(GCC, "-Wunused", ClangOnly, "-Wunknown-warning-option", GCC, "-Wunused-but-set-variable")
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"v3d\"))", Eigen::Vector6d v = config("v3d"));
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"v3dPair\")[0])",
                      Eigen::Vector6d v = config("v3dPair")[0]);
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"v3dPair\")[1])",
                      std::pair<Eigen::Vector3d, Eigen::Vector6d> v = config("v3dPair"));
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"dict\")(\"v3d\"))",
                      Eigen::Vector6d v = config("dict")("v3d"));
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"dict\")(\"v3dPair\")[0])",
                      Eigen::Vector6d v = config("dict")("v3dPair")[0]);
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"dict\")(\"v3dPair\")[1])",
                      std::pair<Eigen::Vector3d, Eigen::Vector6d> v = config("dict")("v3dPair"));
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"array\")[0](\"v3d\"))",
                      Eigen::Vector6d v = config("array")[0]("v3d"));
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"array\")[1](\"v3dPair\")[0])",
                      Eigen::Vector6d v = config("array")[1]("v3dPair")[0]);
  CHECK_ERROR_MESSAGE("Stored Json value is not a Vector6d (error path: (\"array\")[1](\"v3dPair\")[1])",
                      std::pair<Eigen::Vector3d, Eigen::Vector6d> v = config("array")[1]("v3dPair"));
  CHECK_ERROR_MESSAGE(
      "Stored Json value is not a Vector6d (error path: (\"deep\")[2](\"object1\")(\"key2\")[1](\"object2\")(\"v3d\"))",
      Eigen::Vector6d v = config("deep")[2]("object1")("key2")[1]("object2")("v3d"));
  MC_RTC_diagnostic_pop
}

static std::string YAML_TEST_TWEAKS = R"(
col-default: &col-default
  iDist: 0.05
  sDist: 0.01
  damping: 0
collisions:
- b1: body1
  b2: body2
  <<: *col-default
- b1: body1
  b2: body3
  <<: *col-default
special:
  b1: body1
  b2: body2
  <<: *col-default
  sDist: 0.04
Y_is_string: Y
N_is_string: N
yes_is_string: yes
no_is_string: no
Yes_is_string: Yes
No_is_string: No
)";

BOOST_AUTO_TEST_CASE(TestConfigurationYAMLTweaks)
{
  auto config = mc_rtc::Configuration::fromYAMLData(YAML_TEST_TWEAKS);
  BOOST_REQUIRE(config.has("collisions"));
  auto collisions = config("collisions");
  BOOST_REQUIRE(collisions.size() == 2);
  for(const auto & col : collisions)
  {
    BOOST_REQUIRE(col.has("b1"));
    BOOST_REQUIRE(col.has("b2"));
    BOOST_REQUIRE(col.has("iDist"));
    double iDist = col("iDist");
    BOOST_REQUIRE_EQUAL(iDist, 0.05);
    BOOST_REQUIRE(col.has("sDist"));
    double sDist = col("sDist");
    BOOST_REQUIRE_EQUAL(sDist, 0.01);
    BOOST_REQUIRE(col.has("damping"));
    double damping = col("damping");
    BOOST_REQUIRE_EQUAL(damping, 0.0);
  }
  BOOST_REQUIRE(config.has("special"));
  {
    auto special = config("special");
    BOOST_REQUIRE(special.has("iDist"));
    double iDist = special("iDist");
    BOOST_REQUIRE_EQUAL(iDist, 0.05);
    BOOST_REQUIRE(special.has("sDist"));
    double sDist = special("sDist");
    BOOST_REQUIRE_EQUAL(sDist, 0.04);
    BOOST_REQUIRE(special.has("damping"));
    double damping = special("damping");
    BOOST_REQUIRE_EQUAL(damping, 0.0);
  }
  for(const auto & k : {"Y", "N", "yes", "no", "Yes", "No"})
  {
    std::string key = fmt::format("{}_is_string", k);
    BOOST_REQUIRE(config.has(key));
    std::string value = config(key);
    BOOST_REQUIRE_EQUAL(value, k);
  }
}

BOOST_AUTO_TEST_CASE(TestFileConfiguration)
{
  auto file = sampleConfig2(true, false);
  {
    mc_rtc::ConfigurationFile config(file);
    BOOST_REQUIRE_EQUAL(config.path(), file);
    BOOST_REQUIRE(config.has("int"));
    int i = config("int");
    BOOST_REQUIRE_EQUAL(i, 12);
    config.remove("int");
    BOOST_REQUIRE(!config.has("int"));
    config.reload();
    BOOST_REQUIRE(config.has("int"));
    i = config("int");
    BOOST_REQUIRE_EQUAL(i, 12);
    config.add("int", 42);
    config.add("string", "Hello world");
    config.save();
  }
  {
    mc_rtc::Configuration config(file);
    BOOST_REQUIRE(config.has("int"));
    int i = config("int");
    BOOST_REQUIRE(config.has("string"));
    std::string s = config("string");
    BOOST_REQUIRE_EQUAL(s, "Hello world");
    BOOST_REQUIRE_EQUAL(i, 42);
  }
  bfs::remove(file);
}

/** We purposefully create a number-like class that would create an ambiguity without the numeric_limits specialization
 */
struct MyNumber
{
  uint64_t n = 42;
  MyNumber() = default;
  MyNumber(uint64_t n) : n(n) {}

  operator uint64_t() const { return n; }
  operator uint32_t() const { return static_cast<uint32_t>(n); }
};

namespace std
{

template<>
class numeric_limits<MyNumber> : public numeric_limits<uint64_t>
{
};

} // namespace std

BOOST_AUTO_TEST_CASE(TestIntegralTypes)
{
  MyNumber number;
  mc_rtc::Configuration config;
  config.add("number", number);
  {
    MyNumber number2 = config("number");
    BOOST_CHECK_EQUAL(number.n, number2.n);
  }
  {
    auto array = config.array("array");
    array.push(number);
    MyNumber number2 = config("array")[0];
    BOOST_CHECK_EQUAL(number.n, number2.n);
  }
  {
    std::vector<char> buffer(512);
    mc_rtc::MessagePackBuilder builder(buffer);
    builder.write(number);
    builder.finish();
  }
}

BOOST_AUTO_TEST_CASE(TestVariant)
{
  using variant_t = std::variant<double, std::string>;
  variant_t v = 42.42;
  mc_rtc::Configuration config;
  config.add("v", v);
  {
    std::pair<size_t, double> serialized = config("v");
    BOOST_REQUIRE_EQUAL(serialized.first, 0);
    BOOST_REQUIRE_EQUAL(serialized.second, 42.42);
  }
  {
    variant_t out = config("v");
    BOOST_REQUIRE(out.index() == 0);
    BOOST_REQUIRE(std::get<0>(out) == 42.42);
  }
  {
    auto array = config.array("array");
    array.push(v);
  }
  {
    std::vector<variant_t> variant_array = config("array");
    BOOST_REQUIRE(variant_array.size() == 1);
    BOOST_REQUIRE(variant_array[0].index() == 0);
    BOOST_REQUIRE(std::get<0>(variant_array[0]) == 42.42);
  }
  v = "hello";
  config.add("v", v);
  {
    std::pair<size_t, std::string> serialized = config("v");
    BOOST_REQUIRE_EQUAL(serialized.first, 1);
    BOOST_REQUIRE_EQUAL(serialized.second, "hello");
  }
  {
    variant_t out = config("v");
    BOOST_REQUIRE(out.index() == 1);
    BOOST_REQUIRE(std::get<1>(out) == "hello");
  }
  {
    config("array").push(v);
  }
  {
    std::vector<variant_t> variant_array = config("array");
    BOOST_REQUIRE(variant_array.size() == 2);
    BOOST_REQUIRE(variant_array[0].index() == 0);
    BOOST_REQUIRE(std::get<0>(variant_array[0]) == 42.42);
    BOOST_REQUIRE(variant_array[1].index() == 1);
    BOOST_REQUIRE(std::get<1>(variant_array[1]) == "hello");
  }
}
