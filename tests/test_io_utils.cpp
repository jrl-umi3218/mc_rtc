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
  BOOST_CHECK(
      to_string(structVec, [](const TestStruct & c) -> const std::string { return std::to_string(c.val); }, " + ")
      == "1 + 2 + 3");
}
