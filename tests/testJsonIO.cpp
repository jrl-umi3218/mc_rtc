#include <boost/test/unit_test.hpp>
#include <boost/mpl/list.hpp>

#include <mc_rtc/Configuration.h>

#include <mc_rbdyn/configuration_io.h>

#include <fstream>
#include <iostream>
#include <random>

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

double rnd()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_real_distribution<double> dis(-100.0, 100.0);
  return dis(gen);
}

size_t random_size()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<size_t> dis(10, 100);
  return dis(gen);
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

template<>
mc_rbdyn::BodySensor make_ref()
{
  return {"sensor", "parent", random_pt()};
}

bool operator==(const mc_rbdyn::BodySensor & lhs, const mc_rbdyn::BodySensor & rhs)
{
  return lhs.name() == rhs.name() &&
         lhs.parentBody() == rhs.parentBody() &&
         lhs.X_b_s() == rhs.X_b_s();
}

template<>
mc_rbdyn::Collision make_ref()
{
  return {"Body1", "Body2", 0.05, 0.01, 0.123};
}

template<>
std::shared_ptr<mc_rbdyn::PlanarSurface> make_ref()
{
  std::vector<std::pair<double, double>> pPoints = { {0.1, 2.3}, {4.5, 6.7}, {8.9, 0.1} };
  return std::make_shared<mc_rbdyn::PlanarSurface>("planarSurfaceName", "bodyName", random_pt(), "material", pPoints);
}

/* Note: mc_rbdyn already defines comparison between two surfaces but it checks very superficially */
bool operator==(const std::shared_ptr<mc_rbdyn::PlanarSurface> & lhs_p, const std::shared_ptr<mc_rbdyn::PlanarSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() &&
         lhs.bodyName() == rhs.bodyName() &&
         lhs.X_b_s() == rhs.X_b_s() &&
         lhs.materialName() == rhs.materialName() &&
         lhs.planarPoints() == rhs.planarPoints();
}

template<>
std::shared_ptr<mc_rbdyn::CylindricalSurface> make_ref()
{
  return std::make_shared<mc_rbdyn::CylindricalSurface>("planarSurfaceName", "bodyName", random_pt(), "material", 0.42, 1.42);
}

bool operator==(const std::shared_ptr<mc_rbdyn::CylindricalSurface> & lhs_p, const std::shared_ptr<mc_rbdyn::CylindricalSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() &&
         lhs.bodyName() == rhs.bodyName() &&
         lhs.X_b_s() == rhs.X_b_s() &&
         lhs.materialName() == rhs.materialName() &&
         lhs.radius() == rhs.radius() &&
         lhs.width() == rhs.width();
}

template<>
std::shared_ptr<mc_rbdyn::GripperSurface> make_ref()
{
  std::vector<sva::PTransformd> pFo = { random_pt(), random_pt(), random_pt() };
  return std::make_shared<mc_rbdyn::GripperSurface>("planarSurfaceName", "bodyName", random_pt(), "material", pFo, random_pt(), 1.42);
}

bool operator==(const std::shared_ptr<mc_rbdyn::GripperSurface> & lhs_p, const std::shared_ptr<mc_rbdyn::GripperSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() &&
         lhs.bodyName() == rhs.bodyName() &&
         lhs.X_b_s() == rhs.X_b_s() &&
         lhs.materialName() == rhs.materialName() &&
         lhs.pointsFromOrigin() == rhs.pointsFromOrigin() &&
         lhs.X_b_motor() == rhs.X_b_motor() &&
         lhs.motorMaxTorque() == rhs.motorMaxTorque();
}

template<>
std::shared_ptr<mc_rbdyn::Surface> make_ref()
{
  static int i = -1;
  i++;
  if(i % 3 == 0)
  {
    return make_ref<std::shared_ptr<mc_rbdyn::PlanarSurface>>();
  }
  else if(i % 3 == 1)
  {
    return make_ref<std::shared_ptr<mc_rbdyn::CylindricalSurface>>();
  }
  else
  {
    return make_ref<std::shared_ptr<mc_rbdyn::GripperSurface>>();
  }
}

bool operator==(const std::shared_ptr<mc_rbdyn::Surface> & lhs, const std::shared_ptr<mc_rbdyn::Surface> & rhs)
{
  if(lhs->type() == rhs->type())
  {
    auto type = lhs->type();
    if(type == "planar")
    {
      return std::static_pointer_cast<mc_rbdyn::PlanarSurface>(lhs) ==
             std::static_pointer_cast<mc_rbdyn::PlanarSurface>(rhs);
    }
    else if(type == "cylindrical")
    {
      return std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(lhs) ==
             std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(rhs);
    }
    else if(type == "gripper")
    {
      return std::static_pointer_cast<mc_rbdyn::GripperSurface>(lhs) ==
             std::static_pointer_cast<mc_rbdyn::GripperSurface>(rhs);
    }
  }
  return false;
}

template<>
mc_rbdyn::Flexibility make_ref()
{
  return {"jointName", rnd(), rnd(), rnd()};
}

bool operator==(const mc_rbdyn::Flexibility & lhs, const mc_rbdyn::Flexibility & rhs)
{
  return lhs.jointName == rhs.jointName &&
         lhs.K == rhs.K &&
         lhs.C == rhs.C &&
         lhs.O == rhs.O;
}

template<>
mc_rbdyn::ForceSensor make_ref()
{
  return {"forceSensor", "parentBody", random_pt()};
}

bool operator==(const mc_rbdyn::ForceSensor & lhs, const mc_rbdyn::ForceSensor & rhs)
{
  return lhs.name() == rhs.name() &&
         lhs.parentBody() == rhs.parentBody() &&
         lhs.X_p_f() == rhs.X_p_f();
}

template<>
mc_rbdyn::PolygonInterpolator make_ref()
{
  auto random_tuple = []()
  {
    return std::array<double, 2>{{rnd(), rnd()}};
  };
  auto random_tuple_pair = [&random_tuple]()
  {
    return std::make_pair(random_tuple(), random_tuple());
  };
  size_t size = random_size();
  std::vector<mc_rbdyn::PolygonInterpolator::tuple_pair_t> vec(size);
  for(size_t i = 0; i < size; ++i)
  {
    vec[i] = random_tuple_pair();
  }
  return {vec};
}

bool operator==(const mc_rbdyn::PolygonInterpolator & lhs, const mc_rbdyn::PolygonInterpolator & rhs)
{
  return lhs.tuple_pairs() == rhs.tuple_pairs();
}

template<>
mc_rbdyn::Springs make_ref()
{
  mc_rbdyn::Springs spr;
  spr.springsBodies = {"Body1", "Body2"};
  spr.afterSpringsBodies = {"ABody1", "ABody2"};
  spr.springsJoints = {{"Joint1"}, {"Joint2", "Joint3"}};
  return spr;
}

bool operator==(const mc_rbdyn::Springs & lhs, const mc_rbdyn::Springs & rhs)
{
  return lhs.springsBodies == rhs.springsBodies &&
         lhs.afterSpringsBodies == rhs.afterSpringsBodies &&
         lhs.springsJoints == rhs.springsJoints;
}

typedef boost::mpl::list<mc_rbdyn::Base,
                         mc_rbdyn::BodySensor,
                         mc_rbdyn::Collision,
                         std::shared_ptr<mc_rbdyn::PlanarSurface>,
                         std::shared_ptr<mc_rbdyn::CylindricalSurface>,
                         std::shared_ptr<mc_rbdyn::GripperSurface>,
                         std::shared_ptr<mc_rbdyn::Surface>,
                         mc_rbdyn::Flexibility,
                         mc_rbdyn::ForceSensor,
                         mc_rbdyn::PolygonInterpolator,
                         mc_rbdyn::Springs> test_types;

template<typename T,
         typename std::enable_if<std::is_default_constructible<T>::value, int>::type = 0>
void test_config_array_helper(mc_rtc::Configuration & config, int)
{
  std::array<T, 3> ref_a = {make_ref<T>(), make_ref<T>(), make_ref<T>()};
  config.add("object_a", ref_a);

  std::array<T, 3> test_a = config("object_a");
  BOOST_REQUIRE(test_a.size() == ref_a.size());
  for(size_t i = 0; i < test_a.size(); ++i)
  {
    BOOST_CHECK(test_a[i] == ref_a[i]);
  }
}

template<typename T>
void test_config_array_helper(mc_rtc::Configuration &, ...)
{
}

template<typename T>
void test_config_array(mc_rtc::Configuration & config)
{
  return test_config_array_helper<T>(config, 0);
}

BOOST_AUTO_TEST_CASE_TEMPLATE(TestJsonIO, T, test_types)
{
  static_assert(mc_rtc::internal::has_configuration_load_object<T>::value, "No Configuration load function for this type");
  static_assert(mc_rtc::internal::has_configuration_save_object<T>::value, "No Configuration save function for this type");
  BOOST_CHECK(mc_rtc::internal::has_configuration_load_object<T>::value);
  BOOST_CHECK(mc_rtc::internal::has_configuration_save_object<T>::value);

  T ref = make_ref<T>();
  mc_rtc::Configuration config;
  config.add("object", ref);

  T test = config("object");
  BOOST_CHECK(test == ref);

  std::vector<T> ref_v = {make_ref<T>(), make_ref<T>(), make_ref<T>(), make_ref<T>(), make_ref<T>()};
  config.add("object_v", ref_v);

  std::vector<T> test_v = config("object_v");
  BOOST_REQUIRE(test_v.size() == ref_v.size());
  for(size_t i = 0; i < test_v.size(); ++i)
  {
    BOOST_CHECK(test_v[i] == ref_v[i]);
  }

  test_config_array<T>(config);

  config.save("/tmp/config.json");
}
