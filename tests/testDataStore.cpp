/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/datastore.h>
#include <boost/test/unit_test.hpp>

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
  std::string name_;
};

struct B : public A
{
  using A::A;
  std::string hello() const override
  {
    return "B::Hello " + name_;
  }
};

BOOST_AUTO_TEST_CASE(TestDataStoreInheritance)
{
  DataStore store;
  store.make<std::vector<double>>("data", 4, 42.0);
  auto & data = store.get<std::vector<double>>("data");
  BOOST_REQUIRE(data.size() == 4);
  BOOST_CHECK_CLOSE(data[0], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[1], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[2], 42, 1e-10);
  BOOST_CHECK_CLOSE(data[3], 42, 1e-10);
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
  BOOST_REQUIRE(a.hello() == "B::Hello World");
}
