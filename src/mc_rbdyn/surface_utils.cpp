#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rbdyn/surface_utils.h>

// For some dom manipulations
#include <mc_rbdyn_urdf/urdf.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <tinyxml2.h>

namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

inline sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom)
{
  Eigen::Vector3d xyz = mc_rbdyn_urdf::attrToVector(dom, "xyz");
  Eigen::Vector3d rpy = mc_rbdyn_urdf::attrToVector(dom, "rpy");
  return sva::PTransformd(rpyToMat(rpy), xyz);
}

inline void readRSDF(const std::string & rsdf_string, std::vector<std::shared_ptr<Surface>> & surfaces)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(rsdf_string.c_str());

  tinyxml2::XMLElement * root = doc.FirstChildElement("robot");
  std::vector<tinyxml2::XMLElement *> psurfaces;
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
    std::vector<std::pair<double, double>> points;
    tinyxml2::XMLElement * pointdom = pdom->FirstChildElement("points")->FirstChildElement("point");
    while(pointdom)
    {
      std::vector<double> pdata = mc_rbdyn_urdf::attrToList(*pointdom, "xy");
      points.push_back(std::pair<double, double>(pdata[0], pdata[1]));
      pointdom = pointdom->NextSiblingElement("point");
    }
    surfaces.push_back(std::shared_ptr<Surface>(new PlanarSurface(name, bodyName, X_b_s, materialName, points)));
  }

  std::vector<tinyxml2::XMLElement *> csurfaces;
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
    surfaces.push_back(
        std::shared_ptr<Surface>(new CylindricalSurface(name, bodyName, X_b_s, materialName, radius, width)));
  }

  std::vector<tinyxml2::XMLElement *> gsurfaces;
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
    surfaces.push_back(std::shared_ptr<Surface>(
        new GripperSurface(name, bodyName, X_b_s, materialName, points, X_b_motor, motorMaxTorque)));
  }
}

std::vector<std::shared_ptr<Surface>> readRSDFFromDir(const std::string & dirname)
{
  std::vector<std::shared_ptr<Surface>> res;

  bfs::path p(dirname);

  if(bfs::exists(p) && bfs::is_directory(p))
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

} // namespace mc_rbdyn
