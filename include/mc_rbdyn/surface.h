#ifndef _H_MCRBDYNSURFACE_H_
#define _H_MCRBDYNSURFACE_H_

#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/MultiBodyConfig.h>

#include <tinyxml2.h>

#include <memory>
#include <string>
#include <vector>

namespace mc_rbdyn
{

struct Robot;

struct Surface
{
public:
  Surface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName);

  sva::PTransformd X_0_s(const mc_rbdyn::Robot & robot) const;

  sva::PTransformd X_0_s(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc) const;

  const sva::PTransformd & X_b_s() const;

  void X_b_s(const sva::PTransformd & X_b_s);

  virtual void computePoints() = 0;

  std::string toStr();

  virtual std::shared_ptr<Surface> copy() const = 0;

  virtual std::string type() const = 0;
public:
  std::string name;
  std::string bodyName;
  sva::PTransformd _X_b_s;
  std::string materialName;
  std::vector<sva::PTransformd> points;
};

inline bool operator==(const Surface & lhs, const Surface & rhs)
{
  return lhs.name == rhs.name;
}
inline bool operator!=(const Surface & lhs, const Surface & rhs)
{
  return not (lhs == rhs);
}

struct PlanarSurface : public Surface
{
  PlanarSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const std::vector< std::pair<double, double> > & planarPoints);

  virtual void computePoints();

  void planarTransform(const double & T, const double & B, const double & N_rot);

  const std::vector< std::pair<double, double> >& planarPoints() const;

  void planarPoints(const std::vector< std::pair<double, double> > & planarPoints);

  virtual std::shared_ptr<Surface> copy() const;

  virtual std::string type() const override;
public:
  std::vector< std::pair<double, double> > _planarPoints;
};

struct CylindricalSurface : public Surface
{
  CylindricalSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const double & radius, const double & width);

  virtual void computePoints();

  const double& width() const;

  void width(const double & width);

  virtual std::shared_ptr<Surface> copy() const;

  virtual std::string type() const override;
public:
  double radius;
  double _width;
};

struct GripperSurface : public Surface
{
public:
  GripperSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const std::vector<sva::PTransformd> & pointsFromOrigin, const sva::PTransformd & X_b_motor, const double & motorMaxTorque);

  virtual void computePoints();

  void originTransform(const sva::PTransformd & X_s_sp);

  virtual std::shared_ptr<Surface> copy() const;

  virtual std::string type() const override;
public:
  std::vector<sva::PTransformd> pointsFromOrigin;
  sva::PTransformd X_b_motor;
  double motorMaxTorque;
};

Eigen::Matrix3d rpyToMat(const double & r, const double & p, const double & y);

Eigen::Matrix3d rpyToMat(const Eigen::Vector3d & rpy);

sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom);

void readRSDF(const std::string & rsdf_string, std::vector< std::shared_ptr<Surface> > & surfaces);

std::vector< std::shared_ptr<Surface> > readRSDFFromDir(const std::string & dirname);

}

#endif
