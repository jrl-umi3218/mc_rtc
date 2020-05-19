/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_rtc/Configuration.h>

#include <boost/filesystem.hpp>
#include <boost/mpl/list.hpp>
#include <boost/test/unit_test.hpp>
namespace bfs = boost::filesystem;

#include "utils.h"
#include <fstream>
#include <iostream>
#include <random>

#include <Eigen/StdVector>

template<typename T, typename A>
bool compare_vectors(const std::vector<T, A> & lhs, const std::vector<T, A> & rhs)
{
  if(lhs.size() != rhs.size())
  {
    return false;
  }
  for(size_t i = 0; i < lhs.size(); ++i)
  {
    if(!(lhs[i] == rhs[i]))
    {
      return false;
    }
  }
  return true;
}

template<typename T>
bool compare_vector_maps(const std::map<std::string, std::vector<T>> & lhs,
                         const std::map<std::string, std::vector<T>> & rhs)
{
  for(const auto & el : lhs)
  {
    if(rhs.count(el.first))
    {
      if(!compare_vectors(el.second, rhs.at(el.first)))
      {
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  for(const auto & el : rhs)
  {
    if(!lhs.count(el.first))
    {
      return false;
    }
  }
  return true;
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
  return lhs.baseName == rhs.baseName && lhs.X_0_s == rhs.X_0_s && lhs.X_b0_s == rhs.X_b0_s
         && lhs.baseType == rhs.baseType;
}

template<>
mc_rbdyn::BodySensor make_ref()
{
  return {"sensor", "parent", random_pt()};
}

namespace mc_rbdyn
{

bool operator==(const mc_rbdyn::BodySensor & lhs, const mc_rbdyn::BodySensor & rhs)
{
  return lhs.name() == rhs.name() && lhs.parentBody() == rhs.parentBody() && lhs.X_b_s() == rhs.X_b_s();
}

} // namespace mc_rbdyn

template<>
mc_rbdyn::Collision make_ref()
{
  return {"Body1", "Body2", 0.05, 0.01, 0.123};
}

template<>
std::shared_ptr<mc_rbdyn::PlanarSurface> make_ref()
{
  std::vector<std::pair<double, double>> pPoints = {{0.1, 2.3}, {4.5, 6.7}, {8.9, 0.1}};
  return std::make_shared<mc_rbdyn::PlanarSurface>("planarSurfaceName", "bodyName", random_pt(), "material", pPoints);
}

/* Note: mc_rbdyn already defines comparison between two surfaces but it checks very superficially */
bool operator==(const std::shared_ptr<mc_rbdyn::PlanarSurface> & lhs_p,
                const std::shared_ptr<mc_rbdyn::PlanarSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() && lhs.bodyName() == rhs.bodyName() && lhs.X_b_s() == rhs.X_b_s()
         && lhs.materialName() == rhs.materialName() && lhs.planarPoints() == rhs.planarPoints();
}

template<>
std::shared_ptr<mc_rbdyn::CylindricalSurface> make_ref()
{
  return std::make_shared<mc_rbdyn::CylindricalSurface>("planarSurfaceName", "bodyName", random_pt(), "material", 0.42,
                                                        1.42);
}

bool operator==(const std::shared_ptr<mc_rbdyn::CylindricalSurface> & lhs_p,
                const std::shared_ptr<mc_rbdyn::CylindricalSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() && lhs.bodyName() == rhs.bodyName() && lhs.X_b_s() == rhs.X_b_s()
         && lhs.materialName() == rhs.materialName() && lhs.radius() == rhs.radius() && lhs.width() == rhs.width();
}

template<>
std::shared_ptr<mc_rbdyn::GripperSurface> make_ref()
{
  std::vector<sva::PTransformd> pFo = {random_pt(), random_pt(), random_pt()};
  return std::make_shared<mc_rbdyn::GripperSurface>("planarSurfaceName", "bodyName", random_pt(), "material", pFo,
                                                    random_pt(), 1.42);
}

bool operator==(const std::shared_ptr<mc_rbdyn::GripperSurface> & lhs_p,
                const std::shared_ptr<mc_rbdyn::GripperSurface> & rhs_p)
{
  assert(lhs_p);
  assert(rhs_p);
  const auto & lhs = *lhs_p;
  const auto & rhs = *rhs_p;
  return lhs.name() == rhs.name() && lhs.bodyName() == rhs.bodyName() && lhs.X_b_s() == rhs.X_b_s()
         && lhs.materialName() == rhs.materialName() && lhs.pointsFromOrigin() == rhs.pointsFromOrigin()
         && lhs.X_b_motor() == rhs.X_b_motor() && lhs.motorMaxTorque() == rhs.motorMaxTorque();
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
      return std::static_pointer_cast<mc_rbdyn::PlanarSurface>(lhs)
             == std::static_pointer_cast<mc_rbdyn::PlanarSurface>(rhs);
    }
    else if(type == "cylindrical")
    {
      return std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(lhs)
             == std::static_pointer_cast<mc_rbdyn::CylindricalSurface>(rhs);
    }
    else if(type == "gripper")
    {
      return std::static_pointer_cast<mc_rbdyn::GripperSurface>(lhs)
             == std::static_pointer_cast<mc_rbdyn::GripperSurface>(rhs);
    }
  }
  return false;
}

template<>
mc_rbdyn::Flexibility make_ref()
{
  return {"jointName", rnd(), rnd(), rnd()};
}

namespace mc_rbdyn
{
bool operator==(const mc_rbdyn::Flexibility & lhs, const mc_rbdyn::Flexibility & rhs)
{
  return lhs.jointName == rhs.jointName && lhs.K == rhs.K && lhs.C == rhs.C && lhs.O == rhs.O;
}
} // namespace mc_rbdyn

template<>
mc_rbdyn::ForceSensor make_ref()
{
  return {"forceSensor", "parentBody", random_pt()};
}

namespace mc_rbdyn
{
bool operator==(const mc_rbdyn::ForceSensor & lhs, const mc_rbdyn::ForceSensor & rhs)
{
  return lhs.name() == rhs.name() && lhs.parentBody() == rhs.parentBody() && lhs.X_p_f() == rhs.X_p_f();
}
} // namespace mc_rbdyn

template<>
mc_rbdyn::Plane make_ref()
{
  return {Eigen::Vector3d::Random(), rnd()};
}

namespace mc_rbdyn
{
bool operator==(const mc_rbdyn::Plane & lhs, const mc_rbdyn::Plane & rhs)
{
  return lhs.normal == rhs.normal && lhs.offset == rhs.offset;
}
} // namespace mc_rbdyn

template<>
mc_rbdyn::PolygonInterpolator make_ref()
{
  auto random_tuple = []() { return std::array<double, 2>{{rnd(), rnd()}}; };
  auto random_tuple_pair = [&random_tuple]() { return std::make_pair(random_tuple(), random_tuple()); };
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
  return lhs.springsBodies == rhs.springsBodies && lhs.afterSpringsBodies == rhs.afterSpringsBodies
         && lhs.springsJoints == rhs.springsJoints;
}

template<>
mc_rbdyn::RobotModule make_ref()
{
  configureRobotLoader();
  auto rm_ptr = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  mc_rbdyn::RobotModule rm = *rm_ptr;
  mc_rbdyn::RobotModule::Gripper::Safety s{0.5, 1.5, 0.5, 10};
  std::vector<mc_rbdyn::Mimic> mimics = {{"R_FAKETHUMB", "R_UTHUMB", 12.0, -42.0},
                                         {"R_FAKETHUMB2", "R_UTHUMB", -2.0, 0.0}};
  rm._grippers = {{"l_gripper", {"L_UTHUMB"}, true},
                  {"r_gripper", {"R_UTHUMB"}, false, s, mimics},
                  {"fake_gripper", {"R_UTHUMB"}, false, s}};
  rm.expand_stance();
  return rm;
}

namespace rbd
{

namespace parsers
{

bool operator==(const Geometry::Box & lhs, const Geometry::Box & rhs)
{
  return lhs.size == rhs.size;
}

bool operator==(const Geometry::Cylinder & lhs, const Geometry::Cylinder & rhs)
{
  return lhs.radius == rhs.radius && lhs.length == rhs.length;
}

bool operator==(const Geometry::Sphere & lhs, const Geometry::Sphere & rhs)
{
  return lhs.radius == rhs.radius;
}

bool operator==(const Geometry::Mesh & lhs, const Geometry::Mesh & rhs)
{
  return lhs.filename == rhs.filename && lhs.scale == rhs.scale;
}

bool operator==(const Geometry::Superellipsoid & lhs, const Geometry::Superellipsoid & rhs)
{
  return lhs.size == rhs.size && lhs.epsilon1 == rhs.epsilon1 && lhs.epsilon2 == rhs.epsilon2;
}

bool operator==(const Geometry & lhs, const Geometry & rhs)
{
  bool ret = lhs.type == rhs.type;
  if(ret)
  {
    switch(lhs.type)
    {
      case Geometry::Type::BOX:
        return boost::get<Geometry::Box>(lhs.data) == boost::get<Geometry::Box>(rhs.data);
      case Geometry::Type::CYLINDER:
        return boost::get<Geometry::Cylinder>(lhs.data) == boost::get<Geometry::Cylinder>(rhs.data);
      case Geometry::Type::SPHERE:
        return boost::get<Geometry::Sphere>(lhs.data) == boost::get<Geometry::Sphere>(rhs.data);
      case Geometry::Type::MESH:
        return boost::get<Geometry::Mesh>(lhs.data) == boost::get<Geometry::Mesh>(rhs.data);
      case Geometry::Type::SUPERELLIPSOID:
        return boost::get<Geometry::Superellipsoid>(lhs.data) == boost::get<Geometry::Superellipsoid>(rhs.data);
      default:
        break;
    }
  }
  return ret;
}

bool operator==(const Visual & lhs, const Visual & rhs)
{
  return lhs.name == rhs.name && lhs.origin == rhs.origin && lhs.geometry == rhs.geometry;
}

} // namespace parsers

} // namespace rbd

namespace mc_rbdyn
{

bool operator==(const Mimic & lhs, const Mimic & rhs)
{
  return lhs.name == rhs.name && lhs.joint == rhs.joint && lhs.multiplier == rhs.multiplier && lhs.offset == rhs.offset;
}

bool operator==(const RobotModule::Gripper::Safety & lhs, const RobotModule::Gripper::Safety & rhs)
{
  return lhs.percentVMax == rhs.percentVMax && lhs.actualCommandDiffTrigger == rhs.actualCommandDiffTrigger
         && lhs.releaseSafetyOffset == rhs.releaseSafetyOffset
         && lhs.overCommandLimitIterN == rhs.overCommandLimitIterN;
}

bool operator==(const RobotModule::Gripper & lhs, const RobotModule::Gripper & rhs)
{
  auto compareMimics = [&]() {
    auto lmimics = lhs.mimics();
    auto rmimics = rhs.mimics();
    if(lmimics == nullptr && rmimics == nullptr)
    {
      return true;
    }
    if(lmimics == nullptr || rmimics == nullptr)
    {
      return false;
    }
    return *lmimics == *rmimics;
  };
  auto compareSafety = [&]() {
    auto lsafety = lhs.safety();
    auto rsafety = rhs.safety();
    if(lsafety == nullptr && rsafety == nullptr)
    {
      return true;
    }
    if(lsafety == nullptr || rsafety == nullptr)
    {
      return false;
    }
    return *lsafety == *rsafety;
  };
  return lhs.name == rhs.name && lhs.joints == rhs.joints && lhs.reverse_limits == rhs.reverse_limits && compareSafety()
         && compareMimics();
}

} // namespace mc_rbdyn

bool compareHulls(const std::map<std::string, std::pair<std::string, std::string>> & lhs,
                  const std::map<std::string, std::pair<std::string, std::string>> & rhs)
{
  for(const auto & itm : lhs)
  {
    const auto & name = itm.first;
    const auto & body = itm.second.first;
    const auto & path = bfs::path(itm.second.second);
    if(!rhs.count(name))
    {
      return false;
    }
    const auto & ritm = rhs.at(name);
    const auto & rbody = ritm.first;
    const auto & rpath = bfs::path(ritm.second);
    if(rbody != body || path != rpath)
    {
      return false;
    }
  }
  return true;
}

bool operator==(const mc_rbdyn::RobotModule & lhs, const mc_rbdyn::RobotModule & rhs)
{
  return bfs::path(lhs.path) == bfs::path(rhs.path) && lhs.name == rhs.name
         && bfs::path(lhs.urdf_path) == bfs::path(rhs.urdf_path) && bfs::path(lhs.rsdf_dir) == bfs::path(rhs.rsdf_dir)
         && bfs::path(lhs.calib_dir) == bfs::path(rhs.calib_dir) && lhs._bounds == rhs._bounds
         && lhs._stance == rhs._stance && compareHulls(lhs._convexHull, rhs._convexHull)
         && compareHulls(lhs._stpbvHull, rhs._stpbvHull) && compare_vector_maps(lhs._visual, rhs._visual)
         && lhs._collisionTransforms == rhs._collisionTransforms && compare_vectors(lhs._flexibility, rhs._flexibility)
         && compare_vectors(lhs._forceSensors, rhs._forceSensors) && compare_vectors(lhs._bodySensors, rhs._bodySensors)
         && lhs._springs == rhs._springs && lhs._minimalSelfCollisions == rhs._minimalSelfCollisions
         && lhs._commonSelfCollisions == rhs._commonSelfCollisions && compare_vectors(lhs._grippers, rhs._grippers)
         && lhs._ref_joint_order == rhs._ref_joint_order && lhs._default_attitude == rhs._default_attitude
         && lhs._gripperSafety == rhs._gripperSafety;
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
                         mc_rbdyn::Plane,
                         mc_rbdyn::PolygonInterpolator,
                         mc_rbdyn::Springs,
                         mc_rbdyn::RobotModule>
    test_types;

template<typename T, typename std::enable_if<std::is_default_constructible<T>::value, int>::type = 0>
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

template<typename T>
struct AllocatorHelper
{
  using Allocator = std::allocator<T>;
};

template<>
struct AllocatorHelper<mc_rbdyn::BodySensor>
{
  using Allocator = Eigen::aligned_allocator<mc_rbdyn::BodySensor>;
};

template<>
struct AllocatorHelper<mc_rbdyn::RobotModule>
{
  using Allocator = Eigen::aligned_allocator<mc_rbdyn::RobotModule>;
};

BOOST_AUTO_TEST_CASE_TEMPLATE(TestJsonIO, T, test_types)
{
  static_assert(mc_rtc::internal::has_configuration_load_object<T>::value,
                "No Configuration load function for this type");
  static_assert(mc_rtc::internal::has_configuration_save_object<T>::value,
                "No Configuration save function for this type");
  BOOST_CHECK(mc_rtc::internal::has_configuration_load_object<T>::value);
  BOOST_CHECK(mc_rtc::internal::has_configuration_save_object<T>::value);

  T ref = make_ref<T>();
  mc_rtc::Configuration config;
  config.add("object", ref);

  T test = config("object");
  BOOST_CHECK(test == ref);

  using vector_t = std::vector<T, typename AllocatorHelper<T>::Allocator>;
  vector_t ref_v;
  for(size_t i = 0; i < 5; ++i)
  {
    ref_v.push_back(make_ref<T>());
  }
  config.add("object_v", ref_v);

  vector_t test_v = config("object_v");
  BOOST_REQUIRE(test_v.size() == ref_v.size());
  for(size_t i = 0; i < test_v.size(); ++i)
  {
    BOOST_CHECK(test_v[i] == ref_v[i]);
  }

  test_config_array<T>(config);
}

BOOST_AUTO_TEST_CASE(TestRobotModuleAltSave)
{
  auto rm = make_ref<mc_rbdyn::RobotModule>();
  mc_rtc::Configuration config;
  config.add("rm", rm, false);
  std::vector<mc_rbdyn::RobotModule, Eigen::aligned_allocator<mc_rbdyn::RobotModule>> rmV;
  for(size_t i = 0; i < 3; ++i)
  {
    rmV.push_back(rm);
  }
  config.add("rmV", rmV, false, std::vector<std::string>{}, true);
  std::array<mc_rbdyn::RobotModule, 2> rmA = {{rm, rm}};
  config.add("rmA", rmA, false, std::vector<std::string>{"RARM_LINK6", "LARM_LINK6"});
}
