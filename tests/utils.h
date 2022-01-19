/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotLoader.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
namespace bfs = boost::filesystem;

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <tests_config.h>

template<typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA> & a,
    const Eigen::DenseBase<DerivedB> & b,
    const typename DerivedA::RealScalar & rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar & atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

bool configureRobotLoader()
{
  static bool done = false;
  if(!done)
  {
    done = true;
    mc_rtc::Loader::debug_suffix = "";
    mc_rbdyn::RobotLoader::clear();
    mc_rbdyn::RobotLoader::update_robot_module_path({std::string(ROBOTS_BUILD_DIR)});
  }
  return true;
}

/** This file contains various functions that are useful in unit tests */

/** Return a temporary file */
std::string getTmpFile(const std::string & ext = "")
{
  bfs::path out = bfs::temp_directory_path() / bfs::unique_path("mcTmpFile-%%%%-%%%%-%%%%-%%%%");
  return out.string() + ext;
}

/** Make a temporary configuration file from the content provided */
std::string makeConfigFile(const std::string & data, const std::string & ext = ".json")
{
  std::string fIn = getTmpFile(ext);
  std::ofstream ofs(fIn);
  ofs << data;
  return fIn;
}

Eigen::Quaterniond random_quat()
{
  Eigen::Quaterniond q{Eigen::Vector4d::Random()};
  q.normalize();
  return q;
}

sva::PTransformd random_pt()
{
  Eigen::Vector3d t = Eigen::Vector3d::Random();
  Eigen::Quaterniond q = random_quat();
  return {q, t};
}

sva::ForceVecd random_fv()
{
  return {Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
}

sva::MotionVecd random_mv()
{
  return {Eigen::Vector3d::Random(), Eigen::Vector3d::Random()};
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
  static std::uniform_int_distribution<size_t> dis(10, 1024);
  return dis(gen);
}

bool random_bool()
{
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<> dis(0, 1);
  return dis(gen);
}

std::vector<double> random_vector()
{
  std::vector<double> v(random_size());
  for(size_t i = 0; i < v.size(); ++i)
  {
    v[i] = rnd();
  }
  return v;
}

char random_char()
{
  static const std::array<char, 70> chars = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D',
                                             'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R',
                                             'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'a', 'b', 'c', 'd', 'e', 'f',
                                             'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't',
                                             'u', 'v', 'w', 'x', 'y', 'z', '.', ':', '<', '>', ';', '-', '_', ' '};
  static std::random_device rd;
  static std::mt19937_64 gen(rd());
  static std::uniform_int_distribution<size_t> dis(0, chars.size() - 1);
  return chars[dis(gen)];
}

std::string random_string()
{
  std::string s(random_size(), 'a');
  std::generate_n(s.begin(), s.size(), random_char);
  return s;
}
