/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/DataStore.h>
#include <boost/test/unit_test.hpp>
#include "utils.h"

using DataStore = mc_rtc::datastore::DataStore;

struct A
{
  A(const std::string & name) : name_(name) {}
  virtual std::string hello() const
  {
    return "A::Hello " + name_;
  }
  void printHello() const
  {
    std::cout << hello() << std::endl;
  }
  std::string type() const
  {
    return "A";
  }
  std::string name_;
};

struct B : public A
{
  using A::A;
  std::string hello() const override
  {
    return "B::Hello " + name_;
  }
  std::string type() const
  {
    return "B";
  }
};

BOOST_AUTO_TEST_CASE(TestDataStore)
{
  DataStore store;
  store.make<std::vector<double>>("data", 4, 42.0);
  auto & data = store.get<std::vector<double>>("data");
  BOOST_REQUIRE(data.size() == 4);
  BOOST_CHECK_CLOSE(data[0], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[1], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[2], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[3], 42, 1e-10);
  BOOST_CHECK_THROW(store.get<double>("data"), std::runtime_error);
  BOOST_CHECK_THROW(store.get<std::vector<int>>("data"), std::runtime_error);
  BOOST_CHECK_THROW(store.get<std::vector<double>>("non-existing key"), std::runtime_error);
  std::cout << "data.size() " << data.size() << "\n";
  for(const auto & d : data)
  {
    std::cout << "- " << d << "\n";
  }

  data.resize(100);
  std::cout << "data.size() " << data.size() << "\n";
  auto & data2 = store.get<std::vector<double>>("data");
  BOOST_REQUIRE(data2.size() == data.size());
  std::cout << "data2.size() " << data2.size() << "\n";

  // Creating an inherited object and checking virtual inheritance
  store.make<B, A>("b", "World");
  auto & a = store.get<A>("b");
  // Checking member function
  BOOST_REQUIRE(a.type() == "A");
  // Checking virtual function
  BOOST_REQUIRE(a.hello() == "B::Hello World");
  auto & b = store.get<B>("b");
  BOOST_REQUIRE(b.type() == "B");
  BOOST_REQUIRE(b.hello() == "B::Hello World");

  // Test make_initializer
  struct Test
  {
    int a;
    std::string name;
  };
  store.make_initializer<Test>("Test", 42, "Test");
  BOOST_REQUIRE(store.get<Test>("Test").a == 42);
  BOOST_REQUIRE(store.get<Test>("Test").name == "Test");

  // Try creating an object that already exists on the datastore
  BOOST_CHECK_THROW(store.make_initializer<Test>("Test", 42, "Test"), std::runtime_error);
  // Remove object
  store.remove("Test");
  BOOST_CHECK(!store.has("Test"));
  BOOST_CHECK(store.has("b"));
  BOOST_CHECK(store.has("data"));
  // Recreate it with the same name and directly assign some value to it
  // Recreate it with the same name and directly assign some value to it
  store.make_initializer<Test>("Test", 42, "Test").name = "Test2";
  BOOST_REQUIRE(store.get<Test>("Test").name == "Test2");

  // Check creating object of a different type with same name
  store.remove("Test");
  store.make_initializer<std::vector<double>>("Test", 1, 2);
  auto & v = store.get<std::vector<double>>("Test");
  BOOST_REQUIRE(v.size() == 2);
  BOOST_REQUIRE(v[0] == 1);
  BOOST_REQUIRE(v[1] == 2);

  // Test operators
  store.make<double>("TestAssign", 42);
  double value = 0;
  store.assign("TestAssign", value);
  BOOST_REQUIRE(value == 42);
  store.assign("TestAssignNonExisting", value);
  BOOST_REQUIRE(value == 42);
}

BOOST_AUTO_TEST_CASE(TestRobotDataStore)
{
  configureRobotLoader();
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
  auto env = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH),
                                                     std::string("ground"));

  DataStore store;
  auto & robots = store.make<mc_rbdyn::Robots>("robots");
  robots.load({rm, env});
  BOOST_REQUIRE(robots.size() == 2);

  // Get another reference to robots
  // XXX would be nice to support store.get("robots") and infer type automatically
  auto & robots2 = store.get<mc_rbdyn::Robots>("robots");
  BOOST_REQUIRE(robots2.size() == 2);

  robots.robot().posW(sva::PTransformd(Eigen::Vector3d{42, 42, 42}));
  BOOST_CHECK_CLOSE(robots.robot().posW().translation().x(), 42, 1e-10);
  BOOST_CHECK_CLOSE(robots.robot().posW().translation().y(), 42, 1e-10);
  BOOST_CHECK_CLOSE(robots.robot().posW().translation().z(), 42, 1e-10);
  BOOST_CHECK_CLOSE(robots2.robot().posW().translation().x(), 42, 1e-10);
  BOOST_CHECK_CLOSE(robots2.robot().posW().translation().y(), 42, 1e-10);
  BOOST_CHECK_CLOSE(robots2.robot().posW().translation().z(), 42, 1e-10);
}
