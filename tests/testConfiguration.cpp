#include <boost/test/unit_test.hpp>

#include <mc_rtc/Configuration.h>

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

std::string getConfigFile()
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
  return fIn;
}

std::string getConfigFile(const std::string & data)
{
  std::string fIn = getConfigFile();
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

BOOST_AUTO_TEST_CASE(TestConfigurationReading)
{
  /*! Check construction from Json::Value */
  mc_rtc::Configuration config(sampleConfig());

  /*! Check that accessing a non-existing entry throws */
  BOOST_CHECK_THROW(config("NONE"), mc_rtc::Configuration::Exception);

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

    BOOST_CHECK_THROW(Eigen::Vector3d d = config("v6d"), mc_rtc::Configuration::Exception);

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

    BOOST_CHECK_THROW(Eigen::Vector6d d = config("v3d"), mc_rtc::Configuration::Exception);

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

    BOOST_CHECK_THROW(Eigen::VectorXd e = config("int"), mc_rtc::Configuration::Exception);

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

    BOOST_CHECK_THROW(c = config("quat"), mc_rtc::Configuration::Exception);
    BOOST_CHECK_THROW(c = config("doubleStringPair"), mc_rtc::Configuration::Exception);
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

  /* "tuple" */
  {
    auto tuple = config("tuple");
    BOOST_CHECK(tuple.size() == 4);
    Eigen::Vector3d vec = tuple[3];
    std::tuple<bool, double, std::string, Eigen::Vector3d> data =
      std::make_tuple(tuple[0], tuple[1], tuple[2], vec);
    BOOST_CHECK(std::get<0>(data));
    BOOST_CHECK(std::get<1>(data) == 42.42);
    BOOST_CHECK(std::get<2>(data) == "sometext");
    BOOST_CHECK(std::get<3>(data) == Eigen::Vector3d(1.2, 3.4, 5.6));
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

BOOST_AUTO_TEST_CASE(TestConfigurationWriting)
{
  std::string tmpF = getConfigFile();
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
  Eigen::Quaterniond ref_quat {0.71, 0., 0.71, 0.};
  ref_quat.normalize();
  config_ref.add("quat", ref_quat);
  std::vector<int> ref_int_v = {0, 1, 2, 3, 4, 5};
  config_ref.add("int_v", ref_int_v);
  std::vector<double> ref_double_v = {0.1, 1.0, 0.2, 2.0, 0.3};
  config_ref.add("double_v", ref_double_v);
  std::vector<std::vector<double>> ref_double_v_v = { ref_double_v, ref_double_v, {0}, {}, {5.0, 4.0, 3.5} };
  config_ref.add("double_v_v", ref_double_v_v);
  std::vector<Eigen::Vector3d> ref_v3d_v;
  for(size_t i = 0; i < 10; ++i) { ref_v3d_v.push_back(Eigen::Vector3d::Random()); }
  config_ref.add("v3d_v", ref_v3d_v);
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
  for(size_t i = 0; i < test_v3d_v.size(); ++i)
  {
    BOOST_CHECK(test_v3d_v[i].isApprox(ref_v3d_v[i], 1e-9));
  }
  BOOST_REQUIRE(config_test.has("dict"));
  BOOST_CHECK(config_test("dict")("int") == ref_int);
  BOOST_REQUIRE(config_test.has("dict2"));
  BOOST_CHECK(config_test("dict2")("double_v") == ref_double_v);

  /* Save a part of the configuration */
  config_test("dict2")("double_v").save(tmpF);

  mc_rtc::Configuration config_partial(tmpF);
  BOOST_CHECK(config_partial == ref_double_v);
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
  for(size_t i = 0; i < 10; ++i)
  {
    config_1("config_2_v").push(config_2);
  }
  BOOST_REQUIRE(config_1.has("config_2_v"));
  BOOST_REQUIRE(config_1("config_2_v").size() == 10);
  for(size_t i = 0; i < 10; ++i)
  {
    BOOST_CHECK(config_1("config_2_v")[i]("double_v") == ref_double_v);
  }
  /* Add config_1 to config_1 */
  config_1.add("config_1", config_1);
  BOOST_REQUIRE(config_1.has("config_1"));
  BOOST_CHECK(config_1("config_1")("int") == ref_int);
  BOOST_REQUIRE(config_1("config_1").has("config_2_v"));
  BOOST_REQUIRE(config_1("config_1")("config_2_v").size() == 10);
  for(size_t i = 0; i < 10; ++i)
  {
    BOOST_CHECK(config_1("config_1")("config_2_v")[i]("double_v") == ref_double_v);
  }
}
