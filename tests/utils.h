/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/RobotLoader.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

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

#ifdef WIN32
#  include <Windows.h>
/** WIN32 port of mkstemp */
inline int mkstemp(char * out)
{
  char tmp_dir[MAX_PATH + 1];
  GetTempPath(MAX_PATH + 1, tmp_dir);
  int ret = GetTempFileName(tmp_dir, "mkstemp", 0, out);
  if(ret == 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}
#endif

/** Return a temporary file */
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

/** Make a temporary configuration file from the content provided */
std::string makeConfigFile(const std::string & data, const std::string & ext = ".json")
{
  std::string fIn = getTmpFile() + ext;
  std::ofstream ofs(fIn);
  ofs << data;
  return fIn;
}

sva::PTransformd random_pt()
{
  Eigen::Vector3d t = Eigen::Vector3d::Random();
  Eigen::Quaterniond q{Eigen::Vector4d::Random()};
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
