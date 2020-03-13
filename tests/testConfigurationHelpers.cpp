/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/ConfigurationHelpers.h>
#include <boost/test/unit_test.hpp>

template<typename ElemT,
         typename WrongElemT,
         typename VecT = std::vector<ElemT>,
         typename WrongVecT = std::vector<WrongElemT>>
void test_fromVectorOrElement(const ElemT & elem,
                              const WrongElemT & wrongElem,
                              const VecT & vec,
                              const WrongVecT & wrongVec)
{
  using namespace mc_rtc;

  Configuration vectorConf;
  vectorConf.add("vector", vec);
  vectorConf.add("vectorNonConvertible", wrongVec);
  vectorConf.add("elem", elem);
  vectorConf.add("elemNonConvertible", wrongElem);
  LOG_INFO(vectorConf.dump(true));

  VecT elemVec{elem};

  // Try to load from element
  BOOST_CHECK(fromVectorOrElement<ElemT>(vectorConf, "elem") == elemVec);
  BOOST_CHECK(fromVectorOrElement<ElemT>(vectorConf, "elemNonExisting", elemVec) == elemVec);
  BOOST_REQUIRE_THROW(fromVectorOrElement<ElemT>(vectorConf, "elemNonExisting"), Configuration::Exception);
  BOOST_REQUIRE_THROW(fromVectorOrElement<ElemT>(vectorConf, "elemNonConvertible"), Configuration::Exception);

  // Try to load from vector of elements
  BOOST_CHECK(fromVectorOrElement<ElemT>(vectorConf, "vector") == vec);
  BOOST_CHECK(fromVectorOrElement<ElemT>(vectorConf, "vectorNonExisting", vec) == vec);
  BOOST_REQUIRE_THROW(fromVectorOrElement<ElemT>(vectorConf, "vectorNonExisting"), Configuration::Exception);
  BOOST_REQUIRE_THROW(fromVectorOrElement<ElemT>(vectorConf, "vectorNonConvertible"), Configuration::Exception);
}

BOOST_AUTO_TEST_CASE(TestIOUtils)
{
  using namespace mc_rtc;
  { // Test the most common use case: vector of string
    using ElemT = std::string;
    using WrongElemT = double;
    test_fromVectorOrElement<ElemT, WrongElemT>("str", 42, {"str1", "str2"}, {0, 1, 2});
  }
  { // Test vector of double
    using ElemT = double;
    using WrongElemT = std::vector<std::string>;
    test_fromVectorOrElement<ElemT, WrongElemT>(2, {"2", "3", "4"}, {0, 1, 2}, {{"0", "1"}, {"2"}});
  }
  { // Test reading other type
    using ElemT = std::vector<double>;
    using WrongElemT = std::vector<std::string>;
    test_fromVectorOrElement<ElemT, WrongElemT>({0, 1, 2}, {"str"}, {{11, 12, 13}, {21, 22}}, {{"str1", "str2"}});
  }
}
