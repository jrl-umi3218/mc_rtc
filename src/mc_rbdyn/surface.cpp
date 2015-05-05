#include <mc_rbdyn/surface.h>

#include <mc_rbdyn/contact_transform.h>
#include <mc_rbdyn/robot.h>

// For some dom manipulations
#include <mc_rbdyn_urdf/urdf.h>

#include <boost/filesystem.hpp>
#include <fstream>

namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

Surface::Surface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName)
: name(name), bodyName(bodyName), _X_b_s(X_b_s), materialName(materialName), points(0)
{
}

sva::PTransformd Surface::X_0_s(const mc_rbdyn::Robot & robot) const
{
  return X_0_s(robot, *(robot.mbc));
}

sva::PTransformd Surface::X_0_s(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc) const
{
  unsigned int bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd X_0_b = mbc.bodyPosW[bodyIndex];
  return _X_b_s*X_0_b;
}

const sva::PTransformd & Surface::X_b_s() const
{
  return _X_b_s;
}

void Surface::X_b_s(const sva::PTransformd & X_b_s)
{
  _X_b_s = X_b_s;
  computePoints();
}

std::string Surface::toStr()
{
  std::stringstream ss;
  ss << bodyName << ":" << name;
  return ss.str();
}

PlanarSurface::PlanarSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const std::vector< std::pair<double, double> > & planarPoints)
: Surface(name, bodyName, X_b_s, materialName),
  _planarPoints(planarPoints)
{
  computePoints();
}

void PlanarSurface::computePoints()
{
  points.clear();
  for(std::pair<double, double> & p : _planarPoints)
  {
    points.push_back(sva::PTransformd(Eigen::Vector3d(p.first, p.second, 0))*_X_b_s);
  }
}

void PlanarSurface::planarTransform(const double & T, const double & B, const double & N_rot)
{
  sva::PTransformd X = mc_rbdyn::planar(T, B, N_rot);
  sva::PTransformd X_b_s_new = X*_X_b_s;
  std::vector< std::pair<double, double> > newPlanarPoints(0);
  for(std::pair<double, double> & p : _planarPoints)
  {
    sva::PTransformd newP = sva::PTransformd(Eigen::Vector3d(p.first, p.second,0))*X.inv();
    newPlanarPoints.push_back(std::pair<double, double>(newP.translation().x(), newP.translation().y()));
  }
  _planarPoints = newPlanarPoints;
  // Call compute points
  X_b_s(X_b_s_new);
}

const std::vector< std::pair<double, double> >& PlanarSurface::planarPoints() const
{
  return _planarPoints;
}

void PlanarSurface::planarPoints(const std::vector< std::pair<double, double> > & planarPoints)
{
  _planarPoints = planarPoints;
  computePoints();
}

std::shared_ptr<Surface> PlanarSurface::copy() const
{
  return std::shared_ptr<Surface>(new PlanarSurface(*this));
}

std::string PlanarSurface::type() const
{
  return "planar";
}

CylindricalSurface::CylindricalSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const double & radius, const double & width)
: Surface(name, bodyName, X_b_s, materialName),
  radius(radius), _width(width)
{
  computePoints();
}

void CylindricalSurface::computePoints()
{
  points.clear();
  points.push_back(sva::PTransformd(Eigen::Vector3d(-_width/2,0,0))*_X_b_s);
  points.push_back(sva::PTransformd(Eigen::Vector3d(_width/2,0,0))*_X_b_s);
}

const double& CylindricalSurface::width() const
{
  return _width;
}

void CylindricalSurface::width(const double & width)
{
  _width = width;
  computePoints();
}

std::shared_ptr<Surface> CylindricalSurface::copy() const
{
  return std::shared_ptr<Surface>(new CylindricalSurface(*this));
}

std::string CylindricalSurface::type() const
{
  return "cylindrical";
}

GripperSurface::GripperSurface(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s, const std::string & materialName, const std::vector<sva::PTransformd> & pointsFromOrigin, const sva::PTransformd & X_b_motor, const double & motorMaxTorque)
: Surface(name, bodyName, X_b_s, materialName),
  pointsFromOrigin(pointsFromOrigin), X_b_motor(X_b_motor), motorMaxTorque(motorMaxTorque)
{
  computePoints();
}

void GripperSurface::computePoints()
{
  points.clear();
  for(sva::PTransformd & p : pointsFromOrigin)
  {
    points.push_back(p*_X_b_s);
  }
}

void GripperSurface::originTransform(const sva::PTransformd & X_s_sp)
{
  for(sva::PTransformd & p : pointsFromOrigin)
  {
    p = p*X_s_sp.inv();
  }
  X_b_s(X_s_sp*_X_b_s);
}

std::shared_ptr<Surface> GripperSurface::copy() const
{
  return std::shared_ptr<Surface>(new GripperSurface(*this));
}

std::string GripperSurface::type() const
{
  return "gripper";
}

Eigen::Matrix3d rpyToMat(const double & r, const double & p, const double & y)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
  return sva::RotX(r)*sva::RotY(p)*sva::RotZ(y);
#pragma GCC diagnostic pop
}

Eigen::Matrix3d rpyToMat(const Eigen::Vector3d & rpy)
{
  return rpyToMat(rpy(0), rpy(1), rpy(2));
}

sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom)
{
  Eigen::Vector3d xyz = mc_rbdyn_urdf::attrToVector(dom, "xyz");
  Eigen::Vector3d rpy = mc_rbdyn_urdf::attrToVector(dom, "rpy");
  return sva::PTransformd(rpyToMat(rpy), xyz);
}

void readRSDF(const std::string & rsdf_string, std::vector< std::shared_ptr<Surface> > & surfaces)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(rsdf_string.c_str());

  tinyxml2::XMLElement * root = doc.FirstChildElement("robot");
  std::vector<tinyxml2::XMLElement*> psurfaces;
  {
    tinyxml2::XMLElement * psurface = root->FirstChildElement("planar_surface");
    while(psurface)
    {
      psurfaces.push_back(psurface);
      psurface = psurface->NextSiblingElement("planar_surface");
    }
  }
  for(tinyxml2::XMLElement * pdom : psurfaces)
  {
    std::string name = pdom->Attribute("name");
    std::string bodyName = pdom->Attribute("link");
    sva::PTransformd X_b_s = tfFromOriginDom(*(pdom->FirstChildElement("origin")));
    std::string materialName = pdom->FirstChildElement("material")->Attribute("name");
    std::vector< std::pair<double, double> > points;
    tinyxml2::XMLElement * pointdom = pdom->FirstChildElement("points")->FirstChildElement("point");
    while(pointdom)
    {
      std::vector<double> pdata = mc_rbdyn_urdf::attrToList(*pointdom, "xy");
      points.push_back(std::pair<double,double>(pdata[0], pdata[1]));
      pointdom = pointdom->NextSiblingElement("point");
    }
    surfaces.push_back(std::shared_ptr<Surface>(new PlanarSurface(name, bodyName, X_b_s, materialName, points)));
  }

  std::vector<tinyxml2::XMLElement*> csurfaces;
  {
    tinyxml2::XMLElement * csurface = root->FirstChildElement("cylindrical_surface");
    while(csurface)
    {
      csurfaces.push_back(csurface);
      csurface = csurface->NextSiblingElement("cylindrical_surface");
    }
  }
  for(tinyxml2::XMLElement * cdom : csurfaces)
  {
    std::string name = cdom->Attribute("name");
    std::string bodyName = cdom->Attribute("link");
    double width = cdom->DoubleAttribute("width");
    double radius = cdom->DoubleAttribute("radius");
    sva::PTransformd X_b_s = tfFromOriginDom(*(cdom->FirstChildElement("origin")));
    std::string materialName = cdom->FirstChildElement("material")->Attribute("name");
    surfaces.push_back(std::shared_ptr<Surface>(new CylindricalSurface(name, bodyName, X_b_s, materialName, radius, width)));
  }

  std::vector<tinyxml2::XMLElement*> gsurfaces;
  {
    tinyxml2::XMLElement * gsurface = root->FirstChildElement("gripper_surface");
    while(gsurface)
    {
      gsurfaces.push_back(gsurface);
      gsurface = gsurface->NextSiblingElement("gripper_surface");
    }
  }
  for(tinyxml2::XMLElement * gdom : gsurfaces)
  {
    std::string name = gdom->Attribute("name");
    std::string bodyName = gdom->Attribute("link");
    sva::PTransformd X_b_s = tfFromOriginDom(*(gdom->FirstChildElement("origin")));
    std::string materialName = gdom->FirstChildElement("material")->Attribute("name");
    tinyxml2::XMLElement * motorDom = gdom->FirstChildElement("motor");
    sva::PTransformd X_b_motor = tfFromOriginDom(*motorDom);
    double motorMaxTorque = motorDom->DoubleAttribute("max_torque");
    std::vector<sva::PTransformd> points;
    tinyxml2::XMLElement * pointdom = gdom->FirstChildElement("points")->FirstChildElement("origin");
    while(pointdom)
    {
      points.push_back(tfFromOriginDom(*pointdom));
      pointdom = pointdom->NextSiblingElement("origin");
    }
    surfaces.push_back(std::shared_ptr<Surface>(new GripperSurface(name, bodyName, X_b_s, materialName, points, X_b_motor, motorMaxTorque)));
  }
}

std::vector< std::shared_ptr<Surface> > readRSDFFromDir(const std::string & dirname)
{
  std::vector< std::shared_ptr<Surface> > res;

  bfs::path p(dirname);

  if(bfs::exists(p) and bfs::is_directory(p))
  {
    std::vector<bfs::path> files;
    std::copy(bfs::directory_iterator(p), bfs::directory_iterator(), std::back_inserter(files));
    for(const bfs::path & file : files)
    {
      if(file.extension() == ".rsdf")
      {
        std::ifstream ifs(file.string());
        std::stringstream ss;
        ss << ifs.rdbuf();
        readRSDF(ss.str(), res);
      }
    }
  }

  return res;
}

}
