#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotModule_visual.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/inertia.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rtc/visual_utils.h> // for makeVisualSphere...
#include <RBDyn/parsers/urdf.h>
#include <filesystem>
namespace fs = std::filesystem;

namespace mc_rbdyn
{

/**
 * @brief Computes the inertia from a visual geometry description.
 *
 * This function determines the inertia based on the type of geometry (box, sphere, etc.) and its parameters.
 *
 * @param visual The visual geometry description.
 * @param mass The mass of the object.
 * @return sva::RBInertiad The computed inertia.
 * @throws Throws if the geometry type is unsupported.
 */
sva::RBInertiad computeInertiaFromVisual(const rbd::parsers::Visual & visual, double mass)
{
  switch(visual.geometry.type)
  {
    case rbd::parsers::Geometry::BOX:
    {
      auto geom =
          mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::BOX>(const_cast<rbd::parsers::Visual &>(visual));
      return computeBoxInertia(mass, geom.size);
    }
    case rbd::parsers::Geometry::SPHERE:
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SPHERE>(
          const_cast<rbd::parsers::Visual &>(visual));
      return computeSphereInertia(mass, geom.radius);
    }
    case rbd::parsers::Geometry::CYLINDER:
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::CYLINDER>(
          const_cast<rbd::parsers::Visual &>(visual));
      return computeCylinderInertia(mass, geom.radius, geom.length);
    }
    case rbd::parsers::Geometry::SUPERELLIPSOID:
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SUPERELLIPSOID>(
          const_cast<rbd::parsers::Visual &>(visual));
      return computeSuperEllipsoidInertia(mass, geom.size, geom.epsilon1, geom.epsilon2);
    }
    case rbd::parsers::Geometry::MESH:
      mc_rtc::log::error_and_throw("computeIntertiaFromVisual: Mesh geometry not supported for inertia computation");
      break;
    default:
      mc_rtc::log::error_and_throw("computeIntertiaFromVisual: Unsupported geometry type {}", visual.geometry.type);
  }
}

std::vector<std::shared_ptr<Surface>> genSurfacesFromVisual(const rbd::parsers::Visual & visual)
{
  struct SurfaceGen
  {
    std::string name;
    Eigen::Vector3d direction;
    Eigen::Vector3d rpy; // orientation in radians
    Eigen::Vector3d rpyFlipped; // orientation in radians
  };

  auto generators = std::vector<SurfaceGen>{{"Front", {1, 0, 0}, {0, M_PI_2, 0}, {0, M_PI + M_PI_2, 0}},
                                            {"Back", {-1, 0, 0}, {0, -M_PI_2, 0}, {0, M_PI - M_PI_2, 0}},
                                            {"Left", {0, 1, 0}, {-M_PI_2, 0, 0}, {M_PI - M_PI_2, 0, 0}},
                                            {"Right", {0, -1, 0}, {M_PI_2, 0, 0}, {M_PI + M_PI_2, 0, 0}},
                                            {"Top", {0, 0, 1}, {0, 0, 0}, {M_PI, 0, 0}},
                                            {"Bottom", {0, 0, -1}, {M_PI, 0, 0}, {2 * M_PI, 0, 0}}};

  auto surfaces = std::vector<std::shared_ptr<Surface>>{};
  surfaces.reserve(generators.size());

  for(const auto & generator : generators)
  {
    auto & name = generator.name;
    auto & dir = generator.direction;
    auto & rpyExterior = generator.rpy;
    auto & rpyInterior = generator.rpyFlipped;

    Eigen::Matrix3d rotation = mc_rbdyn::rpyToMat(rpyExterior).inverse();
    Eigen::Matrix3d rotationFlipped = mc_rbdyn::rpyToMat(rpyInterior).inverse();

    auto genBoxSurfaces = [&](const Eigen::Vector3d & size)
    {
      double hx = size.x() / 2.0;
      double hy = size.y() / 2.0;
      double hz = size.z() / 2.0;

      std::vector<std::pair<double, double>> points;
      if(std::abs(dir.x()) == 1) // Front/Back face (YZ plane)
      {
        points = {{-hz, -hy}, {hz, -hy}, {hz, hy}, {-hz, hy}};
      }
      else if(std::abs(dir.y()) == 1) // Left/Right face (XZ plane)
      {
        points = {{-hx, -hz}, {hx, -hz}, {hx, hz}, {-hx, hz}};
      }
      else if(std::abs(dir.z()) == 1) // Top/Bottom face (XY plane)
      {
        points = {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
      }

      surfaces.emplace_back(std::make_shared<mc_rbdyn::PlanarSurface>(
          name + "_exterior",
          visual.name, // bodyName
          sva::PTransformd(rotation, dir
                                         * (std::abs(dir.x())   ? hx
                                            : std::abs(dir.y()) ? hy
                                                                : hz)), // X_b_s

          "plastic", // materialName
          points));
      surfaces.emplace_back(surfaces.back()->copy());
      surfaces.back()->name(name + "_interior");
      surfaces.back()->X_b_s(sva::PTransformd{rotationFlipped, dir
                                                                   * (std::abs(dir.x())   ? hx
                                                                      : std::abs(dir.y()) ? hy
                                                                                          : hz)});
    };

    if(visual.geometry.type == rbd::parsers::Geometry::SPHERE)
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SPHERE>(
          const_cast<rbd::parsers::Visual &>(visual));
      auto d = geom.radius;
      surfaces.emplace_back(std::make_shared<mc_rbdyn::PlanarSurface>(
          name + "_exterior",
          visual.name, // bodyName
          sva::PTransformd(rotation, dir * d), // X_b_s

          "plastic", // materialName
          std::vector<std::pair<double, double>>{{-d, -d}, {d, -d}, {d, d}, {-d, d}}));
      surfaces.emplace_back(surfaces.back()->copy());
      surfaces.back()->name(name + "_interior");
      surfaces.back()->X_b_s(sva::PTransformd{rotationFlipped, dir * d});
    }
    else if(visual.geometry.type == rbd::parsers::Geometry::BOX)
    {
      auto geom =
          mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::BOX>(const_cast<rbd::parsers::Visual &>(visual));
      genBoxSurfaces(geom.size);
    }
    else if(visual.geometry.type == rbd::parsers::Geometry::SUPERELLIPSOID)
    {
      auto geom = mc_rtc::details::getVisualGeometry<rbd::parsers::Geometry::SUPERELLIPSOID>(
          const_cast<rbd::parsers::Visual &>(visual));
      // a superellispoid is a rounded box
      genBoxSurfaces(geom.size);
    }
  }

  return surfaces;
}

RobotModulePtr robotModuleFromVisual(const std::string & name,
                                     const rbd::parsers::Visual & visual,
                                     const sva::RBInertiad & inertia,
                                     bool isFixed)
{
  auto pr = rbd::parsers::ParserResult{};
  pr.visual[name] = {visual};
  pr.collision[name] = {visual};

  rbd::Body body(inertia, name);
  pr.mbg.addBody(body);

  pr.mb = pr.mbg.makeMultiBody(name, isFixed);
  pr.mbc = rbd::MultiBodyConfig(pr.mb);
  pr.mbc.zero(pr.mb);

  // Create the robot module
  auto rmPtr = std::make_shared<RobotModule>(name, pr);
  auto & rm = *rmPtr;

  // generate surfaces
  auto surfaces = genSurfacesFromVisual(visual);

  // Module is created, but we still need to export it for other tools (visualization, ...)
  auto saveModule = [&]()
  {
    auto path = mc_rtc::make_temporary_path(name);

    auto urdf_path = (fs::path(path) / "urdf" / (name + ".urdf")).string();
    auto urdf_dir = fs::path(urdf_path).parent_path();
    if(!fs::exists(urdf_dir)) { fs::create_directories(urdf_dir); }

    auto rsdf_dir = fs::path(path) / "rsdf";
    if(!fs::exists(rsdf_dir)) { fs::create_directories(rsdf_dir); }

    auto doc = tinyxml2::XMLDocument{};
    surfacesToXML(doc, name, surfaces);
    auto rsdf_path = (rsdf_dir / (name + ".rsdf")).string();
    doc.SaveFile(rsdf_path.c_str());

    std::ofstream ofs(urdf_path);
    ofs << rbd::parsers::to_urdf(pr);

    // Generate a module file so that the generated RobotModule can be re-used
    // XXX: currently devices are not saved so strictly speaking the generated yaml robot
    // won't be exactly identical to the one generated here
    {
      std::string module_yaml = fmt::format("{}/{}.yaml", path, name);
      rm.path = path;
      rm.urdf_path = urdf_path;
      rm.rsdf_dir = rsdf_dir.string();
      rm._parameters = {"json", module_yaml};
      rm._canonicalParameters = rm._parameters;

      auto yaml = mc_rtc::ConfigurationLoader<mc_rbdyn::RobotModule>::save(rm, false, {}, rm.mb.joint(0).dof() == 0);
      yaml.save(module_yaml);
      mc_rtc::log::info("Result module for {} in: {}", name, module_yaml);
    }
  };
  saveModule();

  return rmPtr;
}

RobotModulePtr robotModuleFromVisual(const std::string & name,
                                     const rbd::parsers::Visual & visual,
                                     double mass,
                                     bool isFixed)
{
  return robotModuleFromVisual(name, visual, computeInertiaFromVisual(visual, mass), isFixed);
}

RobotModulePtr robotModuleFromVisual(const std::string & name, const mc_rtc::Configuration & config)
{
  double mass = 0;
  std::optional<sva::RBInertiad> inertia = std::nullopt;
  if(auto inertiaC = config.find("inertia"))
  {
    if(auto mass_ = inertiaC->find("mass")) { mass = *mass_; }
    else
    {
      mc_rtc::log::error_and_throw("robotModuleFromVisualConfig for visual {}: inertia provided but no mass. You may "
                                   "ommit other inertia fields, in which case a default inertia will be computed based "
                                   "on the type of geometry and assuming an homogeneous mass distribution",
                                   name);
    }

    // explicit inertia is provided
    if(auto spatialMomentum = inertiaC->find("momentum"))
    {
      if(auto inertia_ = inertiaC->find("inertia")) { inertia = sva::RBInertiad(mass, *spatialMomentum, *inertia_); }
      else
      {
        mc_rtc::log::error_and_throw("robotModuleFromVisualConfig: momentum provided but no inertia matrix");
      }
    }
  }
  else
  {
    mc_rtc::log::error_and_throw("robotModuleFromVisualConfig for visual {}: you must provide an inertia with at least "
                                 "the mass, e.g:\ninertia:\n  mass: 2.0",
                                 name);
  }

  rbd::parsers::Visual visual = config;
  if(inertia) { return robotModuleFromVisual(name, visual, *inertia, config("fixed", false)); }
  else
  {
    return robotModuleFromVisual(name, visual, mass, config("fixed", false));
  }
}
} // namespace mc_rbdyn
