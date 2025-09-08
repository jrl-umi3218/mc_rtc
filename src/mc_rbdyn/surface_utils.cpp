/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rbdyn/surface_utils.h>

// For some dom manipulations
#include <RBDyn/parsers/urdf.h>

#include <boost/filesystem.hpp>

#include <fmt/format.h>
#include <fstream>
#include <tinyxml2.h>

namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom)
{
  Eigen::Vector3d xyz = rbd::parsers::attrToVector(dom, "xyz");
  Eigen::Vector3d rpy = rbd::parsers::attrToVector(dom, "rpy");
  return sva::PTransformd(rpyToMat(rpy), xyz);
}

tinyxml2::XMLElement * tfToOriginDom(tinyxml2::XMLDocument & doc,
                                     const sva::PTransformd & X,
                                     const std::string_view & tagName)
{
  const Eigen::Vector3d & xyz = X.translation();
  const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X.rotation().transpose());

  auto * elem = doc.NewElement(std::string(tagName).c_str());
  elem->SetAttribute("rpy", fmt::format("{:.8f} {:.8f} {:.8f}", rpy.x(), rpy.y(), rpy.z()).c_str());
  elem->SetAttribute("xyz", fmt::format("{:.8f} {:.8f} {:.8f}", xyz.x(), xyz.y(), xyz.z()).c_str());
  return elem;
}

void surfacesToXML(tinyxml2::XMLDocument & doc,
                   const std::string & robotName,
                   const std::vector<std::shared_ptr<Surface>> & surfaces)
{
  auto * robotElem = doc.NewElement("robot");
  robotElem->SetAttribute("name", robotName.c_str());
  doc.InsertFirstChild(robotElem);

  for(const auto & s : surfaces) { robotElem->InsertEndChild(s->toXML(doc)); }
}

inline void readRSDF(const std::string & rsdf_string, std::vector<std::shared_ptr<Surface>> & surfaces)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(rsdf_string.c_str());

  tinyxml2::XMLElement * root = doc.FirstChildElement("robot");
  for(auto * elem = root->FirstChildElement(); elem; elem = elem->NextSiblingElement())
  {
    if(auto surf = Surface::fromXML(*elem)) { surfaces.emplace_back(std::move(surf)); }
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
