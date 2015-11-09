#pragma once

#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/MultiBodyConfig.h>

#include <memory>
#include <string>
#include <vector>

namespace mc_rbdyn
{

struct Robot;

struct SurfaceImpl;

struct Surface
{
public:
  Surface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName);

  ~Surface();

  const std::string & name() const;

  const std::string & bodyName() const;

  const std::string & materialName() const;

  const std::vector<sva::PTransformd> & points() const;

  unsigned int bodyIndex(const mc_rbdyn::Robot & robot) const;

  int bodyId(const mc_rbdyn::Robot & robot) const;

  sva::PTransformd X_0_s(const mc_rbdyn::Robot & robot) const;

  sva::PTransformd X_0_s(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc) const;

  const sva::PTransformd & X_b_s() const;

  void X_b_s(const sva::PTransformd & X_b_s);

  virtual void computePoints() = 0;

  std::string toStr();

  virtual std::shared_ptr<Surface> copy() const = 0;

  virtual std::string type() const = 0;
protected:
  std::vector<sva::PTransformd> & points();
private:
  std::unique_ptr<SurfaceImpl> impl;
};

bool operator==(const Surface & lhs, const Surface & rhs);

bool operator!=(const Surface & lhs, const Surface & rhs);

typedef std::shared_ptr<Surface> SurfacePtr;

}
