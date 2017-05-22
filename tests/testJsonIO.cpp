#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <mc_rtc/Configuration.h>

#include <mc_rbdyn/configuration_io.h>

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

std::string getTmpFile()
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

sva::PTransformd random_pt()
{
  Eigen::Vector3d t = Eigen::Vector3d::Random();
  Eigen::Quaterniond q {Eigen::Vector4d::Random()};
  q.normalize();
  return {q, t};
}

template<typename T>
T make_ref()
{
  T ret;
  return ret;
}

template<>
mc_rbdyn::Base make_ref()
{
  return {"base", random_pt(), random_pt(), rbd::Joint::Cylindrical};
}

bool operator==(const mc_rbdyn::Base & lhs, const mc_rbdyn::Base & rhs)
{
  return lhs.baseName == rhs.baseName &&
         lhs.X_0_s == rhs.X_0_s &&
         lhs.X_b0_s == rhs.X_b0_s &&
         lhs.baseType == rhs.baseType;
}

typedef boost::mpl::list<mc_rbdyn::Base> test_types;

BOOST_AUTO_TEST_CASE_TEMPLATE(TestJsonIO, T, test_types)
{
  BOOST_CHECK(mc_rtc::internal::has_configuration_load_object<T>::value);
  BOOST_CHECK(mc_rtc::internal::has_configuration_save_object<T>::value);

  T ref = make_ref<T>();
  mc_rtc::Configuration config;
  config.add("object", ref);

  T test = config("object");
  BOOST_CHECK(test == ref);

  std::vector<T> ref_v = {ref, ref, ref, ref, ref};
  config.add("object_v", ref_v);

  std::vector<T> test_v = config("object_v");
  BOOST_REQUIRE(test_v.size() == ref_v.size());
  for(size_t i = 0; i < test_v.size(); ++i)
  {
    BOOST_CHECK(test_v[i] == ref_v[i]);
  }

  std::array<T, 3> ref_a = {ref, ref, ref};
  config.add("object_a", ref_a);

  std::array<T, 3> test_a = config("object_a");
  BOOST_REQUIRE(test_a.size() == ref_a.size());
  for(size_t i = 0; i < test_a.size(); ++i)
  {
    BOOST_CHECK(test_a[i] == ref_a[i]);
  }

  config.save("/tmp/config.json");
}
