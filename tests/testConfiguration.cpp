#include <boost/test/unit_test.hpp>

#include <mc_control/Configuration.h>

#include <fstream>
#include <iostream>

#ifdef WIN32
#include <Windows.h>
inline int mkstemp(char * out)
{
  char tmp_dir[MAX_PATH + 1];
  GetTempPath(MAX_PATH + 1, tmp_dir);
  int ret = GetTempFileName(tmp_dir, "mkstemp", 0, out);
  if (ret == 0) { return -1; }
  else { return 0; }
}
#endif

std::string getConfigFile(const std::string & data)
{
#ifndef WIN32
  char fIn[17] = "/tmp/tConfXXXXXX";
#else
  char fIn[MAX_PATH + 1];
  memset(fIn, 0, MAX_PATH + 1);
#endif
  int err = mkstemp(fIn);
  if(err < 0)
  {
    std::cerr << "Failed to create temporary file, abort test" << std::endl;
    throw std::runtime_error("Failed to create file");
  }
  std::ofstream ofs(fIn);
  ofs << data;
  return fIn;
}

bool operator==(const Eigen::Quaterniond & lhs,
                const Eigen::Quaterniond & rhs)
{
  return lhs.w() == rhs.w() &&
         lhs.x() == rhs.x() &&
         lhs.y() == rhs.y() &&
         lhs.z() == rhs.z();
}

std::string sampleConfig()
{
  std::string data = R"(
{
  "int": 42,
  "sint": -42,
  "double": 42.5,
  "doubleOrInt": 42.0,
  "string": "sometext",
  "intV": [0, 1, 2, 3, 4, 5],
  "stringV": ["a", "b", "c", "foo", "bar"],
  "v3d": [1.0, 2.3, -100],
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
  "dict":
  {
    "boolTrue": true,
    "bool0": 0,
    "int" : 42,
    "double": 42.5,
    "string": "sometext",
    "intV": [0, 1, 2, 3, 4, 5],
    "stringV": ["a", "b", "c", "foo", "bar"],
    "v3d": [1.0, 2.3, -100],
    "v6d": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
    "quat": [0.71, 0, 0.71, 0],
    "doubleDoublePair": [42.5, -42.5],
    "doubleStringPair": [42.5, "sometext"]
  }
}
)";
  return getConfigFile(data);
}

std::string sampleConfig2()
{
  std::string data = R"(
  {
    "stringV": ["a2", "b2", "c2"],
    "int": 12
  }
  )";
  return getConfigFile(data);
}

BOOST_AUTO_TEST_CASE(TestConfiguration)
{
  /*! Check construction from Json::Value */
  mc_control::Configuration config(sampleConfig());

  /*! Check that accessing a non-existing entry throws */
  BOOST_CHECK_THROW(config("NONE"), mc_control::Configuration::Exception);

  /* int tests */
  {
    /*! Check that we can access a direct int entry */
    int a = config("int");
    BOOST_CHECK_EQUAL(a, 42);

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

    /*! Access a int from a dict */
    int f = 0;
    f = config("dict")("int");
    BOOST_CHECK_EQUAL(f, 42);

    /*! Access a int from a dict */
    int g = 0;
    config("dict")("int", g);
    BOOST_CHECK_EQUAL(g, 42);
  }

  /* unsigned int tests */
  {
    /*! Check that we can access a direct unsigned int entry */
    unsigned int a = config("int");
    BOOST_CHECK_EQUAL(a, 42);

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
    unsigned int h = 10;
    config("sint", h);
    BOOST_CHECK_EQUAL(h, 10);

    /*! Access a unsigned int from a dict */
    unsigned int f = 0;
    f = config("dict")("int");
    BOOST_CHECK_EQUAL(f, 42);

    /*! Access a unsigned int from a dict */
    unsigned int g = 0;
    config("dict")("int", g);
    BOOST_CHECK_EQUAL(g, 42);
  }

  /* double tests */
  {
    /*! Check that we can access a direct double entry */
    double a = config("double");
    BOOST_CHECK_EQUAL(a, 42.5);

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

    BOOST_CHECK_THROW(Eigen::Vector3d d = config("v6d"), mc_control::Configuration::Exception);

    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    e = config("dict")("v3d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector3d f = Eigen::Vector3d::Zero();
    config("dict")("v3d", f);
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

    BOOST_CHECK_THROW(Eigen::Vector6d d = config("v3d"), mc_control::Configuration::Exception);

    Eigen::Vector6d e = Eigen::Vector6d::Zero();
    e = config("dict")("v6d");
    BOOST_CHECK_EQUAL(e, ref);

    Eigen::Vector6d f = Eigen::Vector6d::Zero();
    config("dict")("v6d", f);
    BOOST_CHECK_EQUAL(f, ref);
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

    BOOST_CHECK_THROW(Eigen::VectorXd e = config("int"), mc_control::Configuration::Exception);

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

  /* Eigen::Quaterniod test */
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
    BOOST_CHECK(b == ref);

    BOOST_CHECK_THROW(c = config("quat"), mc_control::Configuration::Exception);
    BOOST_CHECK_THROW(c = config("doubleStringPair"), mc_control::Configuration::Exception);
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
    test_t ref = { {0.0, 1.1}, {2.2, 3.3}, {4.4, 5.5} };

    test_t a = config("doubleDoublePairV");
    BOOST_CHECK(a == ref);

    test_t b = {};
    config("doubleDoublePairV", b);
    BOOST_CHECK(b == ref);
  }

  /* Check load */
  {
    config.load(sampleConfig2());
    int a = config("int");
    BOOST_CHECK_EQUAL(a, 12);

    /* Check that other values are still there */
    int b = config("sint");
    BOOST_CHECK_EQUAL(b, -42);

    /* Check with a more complex type */
    std::vector<std::string> ref2 = {"a2", "b2", "c2"};
    std::vector<std::string> c = config("stringV");
    BOOST_CHECK(c == ref2);
  }
}
