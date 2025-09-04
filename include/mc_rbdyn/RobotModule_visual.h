#pragma once
#include <mc_rbdyn/api.h>
#include <mc_rtc/path.h>

#include <mc_rbdyn/RobotModule.h>
#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

struct Surface;

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
MC_RBDYN_DLLAPI sva::RBInertiad computeInertiaFromVisual(const rbd::parsers::Visual & visual, double mass);

/**
 * @brief Converts a Surface object to its XML representation.
 *
 * @param surface The surface to convert.
 * @return std::string XML representation of the surface.
 * @throws Throws if the surface type is unsupported.
 */
MC_RBDYN_DLLAPI std::string surfaceToXML(const Surface & surface);

/**
 * @brief Converts a vector of surfaces to an XML representation for a robot.
 *
 * @param robotName Name of the robot.
 * @param surfaces Vector of shared pointers to Surface objects.
 * @return std::string XML representation of all surfaces.
 */
MC_RBDYN_DLLAPI std::string surfacesToXML(const std::string & robotName,
                                          const std::vector<std::shared_ptr<Surface>> & surfaces);

/**
 * @brief Generates planar surfaces from a visual geometry description.
 *
 * @param visual The visual geometry description.
 * @return std::vector<std::shared_ptr<Surface>> Vector of generated surfaces.
 */
MC_RBDYN_DLLAPI std::vector<std::shared_ptr<Surface>> genSurfacesFromVisual(const rbd::parsers::Visual & visual);

/**
 * @brief Creates a RobotModule from a visual geometry and inertia.
 *
 * @param name Name of the robot module.
 * @param visual The visual geometry description.
 * @param inertia The inertia of the robot.
 * @param isFixed Whether the robot is fixed (default: true).
 * @return RobotModulePtr Shared pointer to the created RobotModule.
 */
MC_RBDYN_DLLAPI RobotModulePtr robotModuleFromVisual(const std::string & name,
                                                     const rbd::parsers::Visual & visual,
                                                     const sva::RBInertiad & inertia,
                                                     bool isFixed = true);

/**
 * @brief Creates a RobotModule from a visual geometry and mass.
 * The inertia is computed based on the geometry type assuming the CoM to be at the geometric center of the shape and an
 * homogenous density.
 *
 * @param name Name of the generated robot module.
 * @param visual The visual geometry description.
 * @param mass The mass of the robot.
 * @param isFixed Whether the robot is fixed (default: true).
 * @return RobotModulePtr Shared pointer to the created RobotModule.
 */
MC_RBDYN_DLLAPI RobotModulePtr robotModuleFromVisual(const std::string & name,
                                                     const rbd::parsers::Visual & visual,
                                                     double mass,
                                                     bool isFixed = true);

/**
 * @brief Creates a RobotModule from a configuration.
 *
 * @param name Name of the robot module.
 * @param config Configuration object. This should be the configuration of a rbd::parsers::Visual, with the addition of
 * an inertia field. The inertia field is expected in the format of RBInertiad configuration, i.e.:
 * - mass: double # mandatory
 * - momentum: [x, y, z] # optional, defaults to [0,0,0]
 * - inertia: [[Ixx, Ixy, Ixz], [Iyx, Iyy, Iyz], [Izx, Izy, Izz]] # optional, defaults to the shape's inertia, mandatory
 * if momentum is provided
 *
 * \code{.yaml}
 * name: box
 * origin:
 *   translation: [0, 0, 0]
 *   rotation: [0, 0, 0]
 * material:
 *   color:
 *     r: 1
 *     g: 0
 *     b: 0
 *     a: 1
 * geometry:
 *   box:
 *     size: [1., 0.5, 2.]
 * inertia:
 *   mass: 10.0
 * \endcode
 *
 * @return RobotModulePtr Shared pointer to the created RobotModule.
 */
MC_RBDYN_DLLAPI RobotModulePtr robotModuleFromVisual(const std::string & name, const mc_rtc::Configuration & config);

} // namespace mc_rbdyn
