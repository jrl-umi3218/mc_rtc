/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/io_utils.h>
#include <boost/test/unit_test.hpp>
#include <iostream>

BOOST_AUTO_TEST_CASE(TestIOUtils)
{
  using namespace mc_rtc::io;
  std::vector<std::string> vecs{"str1", "str2"};
  BOOST_CHECK(to_string(vecs) == "str1, str2");
  BOOST_CHECK(to_string(vecs, " + ") == "str1 + str2");
  BOOST_CHECK(to_string(vecs, "; ") == "str1; str2");

  std::vector<std::string> emptyVec{};
  BOOST_CHECK(to_string(emptyVec) == "");

  std::vector<double> vecOfDouble{2, 3, 4};
  BOOST_CHECK(to_string(vecOfDouble, ", ", 2) == "2.00, 3.00, 4.00");
  BOOST_CHECK(!to_string(std::vector<double>{42, 42, 42}).empty());

  struct TestStruct
  {
    using value_type = int;
    TestStruct(int v) : val{v} {}
    value_type val = 0;
  };
  std::vector<TestStruct> structVec;
  structVec.emplace_back(1);
  structVec.emplace_back(2);
  structVec.emplace_back(3);
  BOOST_CHECK(to_string(structVec, [](const TestStruct & c) -> const std::string { return std::to_string(c.val); })
              == "1, 2, 3");
  BOOST_CHECK(to_string(
                  structVec, [](const TestStruct & c) -> const std::string { return std::to_string(c.val); }, " + ")
              == "1 + 2 + 3");

  BOOST_CHECK(to_string(std::vector<std::string>{"a", "aa", "aaa"},
                        [](const std::string & s) { return std::to_string(s.size()); })
              == "1, 2, 3");

  BOOST_CHECK(to_string(
                  std::vector<std::string>{"a", "aa", "aaa"},
                  [](const std::string & s) { return std::to_string(s.size()); }, " + ")
              == "1 + 2 + 3");

  auto lambda = [](const std::string & s) { return std::to_string(s.size()); };

  BOOST_CHECK(to_string(std::vector<std::string>{"a", "aa", "aaa"}, lambda) == "1, 2, 3");

  BOOST_CHECK(to_string(std::vector<std::string>{"a", "aa", "aaa"}, lambda, " + ") == "1 + 2 + 3");
}
