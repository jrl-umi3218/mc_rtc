/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/CompoundJointConstraintDescription.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Mimic.h>
#include <mc_rbdyn/Springs.h>
#include <mc_rbdyn/api.h>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>

#include <mc_rbdyn_urdf/urdf.h>

#include <array>
#include <map>
#include <vector>

/* This is an interface designed to provide additionnal information about a robot */

namespace mc_rbdyn
{

/* TODO Functions are declared const here but most implementations will likely not respect the constness */
struct MC_RBDYN_DLLAPI RobotModule
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Holds information regarding the bounds
   *
   * The vector should have 6 entries:
   * - lower/upper position bounds
   * - lower/upper velocity bounds
   * - lower/upper torque bounds
   *
   * Each entry is a map joint name <-> bound
   */
  using bounds_t = std::vector<std::map<std::string, std::vector<double>>>;

  /*! Holds necessary information to create a gripper */
  struct Gripper
  {
    /*! Gripper's name */
    std::string name;
    /*! Active joints in the gripper */
    std::vector<std::string> joints;
    /*! Whether the limits should be reversed, see mc_control::Gripper */
    bool reverse_limits;
  };

  /** Construct from a provided path and name
   *
   * As a result:
   * - name is defined as \p name
   * - path is defined as \p path
   * - urdf_path is path + /urdf/ + name + .urdf
   * - rsdf_dir is path + /rsdf/ + name
   * - calib_dir is path + /calib/ + name:q
   *
   * No further action is taken. This constructor is useful to inherit from
   *
   * \param path Path to the robot description
   *
   * \param name Name of the robot
   */
  RobotModule(const std::string & path, const std::string & name)
  : RobotModule(path, name, path + "/urdf/" + name + ".urdf")
  {
  }

  /** Construct from a provided path, name and urdf_path
   *
   * \see RobotModule(const std::string &, const std::string &)
   *
   * The different is that urdf_path is defined to \p urdf_path
   *
   * \param path Path to the robot description
   *
   * \param name Name of the robot
   *
   * \param urdf_path Path to the robot URDF
   */
  RobotModule(const std::string & path, const std::string & name, const std::string & urdf_path)
  : path(path), name(name), urdf_path(urdf_path), rsdf_dir(path + "/rsdf/" + name), calib_dir(path + "/calib/" + name)
  {
  }

  /** Construct from the result of an URDF parse */
  RobotModule(const std::string & name, const mc_rbdyn_urdf::URDFParserResult & res);

  /** Returns the robot's bounds
   *
   * The vector should hold 6 string -> vector<double> map
   *
   * Each map's keys are joint names and values are joint limits.
   *
   * They should be provided in the following order:
   * - joint limits (lower/upper)
   * - velocity limits (lower/upper)
   * - torque limits (lower/upper)
   */
  const std::vector<std::map<std::string, std::vector<double>>> & bounds() const
  {
    return _bounds;
  }

  /** Returns a default configuration for the robot
   *
   * Keys are joint names and values are joint configurations.
   *
   * It should be ok to include joints that are not in the robot (e.g. generate
   * the same default stance for variants of a robot. However, each actuated
   * joint in the robot should have a valid entry, see \ref expand_stance
   *
   * For the floating base see \ref default_attitude
   */
  const std::map<std::string, std::vector<double>> & stance() const
  {
    return _stance;
  }

  /** Returns a map describing the convex hulls for the robot
   *
   * A key defines a valid collision name, a value is composed of two strings:
   *
   * 1. the name of the body the convex is attached to
   *
   * 2. the path to the file containing the convex description
   *
   * The transformation between the convex and the body it's attached to are
   * provided in a separate map see \ref collisionTransforms()
   */
  const std::map<std::string, std::pair<std::string, std::string>> & convexHull() const
  {
    return _convexHull;
  }

  /** Returns a map describing the STPBV hulls for the robot
   *
   * A key defines a valid collision name, a value is composed of two strings:
   *
   * 1. the name of the body the convex is attached to
   *
   * 2. the path to the file containing the STPBV description
   *
   * The transformation between the STPBV and the body it's attached to are
   * provided in a separate map see \ref collisionTransforms()
   */
  const std::map<std::string, std::pair<std::string, std::string>> & stpbvHull() const
  {
    return _stpbvHull;
  }

  /** Returns a map describing the transformation between convex/STPBV hulls
   * and their parent bodies
   *
   * A key defines the collision name. The value is the transformation between
   * this collision object and its parent body
   */
  const std::map<std::string, sva::PTransformd> & collisionTransforms() const
  {
    return _collisionTransforms;
  }

  /** Return the flexibilities of the robot
   *
   * \see mc_rbdyn::Flexibility for details on the expected data
   */
  const std::vector<Flexibility> & flexibility() const
  {
    return _flexibility;
  }

  /** Return the force sensors of the robot
   *
   * \see mc_rbdyn::ForceSensor for details on the expected data
   */
  const std::vector<ForceSensor> & forceSensors() const
  {
    return _forceSensors;
  }

  /** Return the body sensors of the robot
   *
   * \see mc_rbdyn::BodySensor for details on the expected data
   */
  const BodySensorVector & bodySensors() const
  {
    return _bodySensors;
  }

  /** Return the springs of a robot
   *
   * \see mc_rbdyn::Spring for details on the expected data
   */
  const Springs & springs() const
  {
    return _springs;
  }

  /** Return a minimal self-collision set
   *
   * This set of collision describe self-collisions that you always want to
   * enable regardless of the application
   *
   * \see mc_rbdyn::Collision for details on the expected data
   */
  const std::vector<mc_rbdyn::Collision> & minimalSelfCollisions() const
  {
    return _minimalSelfCollisions;
  }

  /** Return a common self-collision set
   *
   * This set of collision describe self-collisions that you want to enable for
   * general applications. Generally this is a super-set of \ref
   * minimalSelfCollisions
   *
   * \see mc_rbdyn::Collision for details on the expected data
   */
  const std::vector<mc_rbdyn::Collision> & commonSelfCollisions() const
  {
    return _commonSelfCollisions;
  }

  /** Return the grippers in the robot
   *
   * \see mc_rbdyn::Gripper for details on the expected data
   */
  const std::vector<Gripper> & grippers() const
  {
    return _grippers;
  }

  /** Return a map of mimics joints associated to a gripper.
   *
   * If this is empty, the mimics are deduced from the URDF.
   */
  const std::map<std::string, std::vector<Mimic>> & gripperMimics() const
  {
    return _gripperMimics;
  }

  /** Return the reference (native controller) joint order of the robot
   *
   * If it is empty, \ref make_default_ref_joint_order() will be used to
   * generate one
   */
  const std::vector<std::string> & ref_joint_order() const
  {
    return _ref_joint_order;
  }

  /** Return the default attitude of the floating base
   *
   * This attitute is associated to the \ref stance() configuration
   */
  const std::array<double, 7> & default_attitude() const
  {
    return _default_attitude;
  }

  /** Return default configuration for the lipm stabilizer */
  const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & defaultLIPMStabilizerConfiguration() const
  {
    return _lipmStabilizerConfig;
  }

  /** Generate correct bounds from URDF bounds
   *
   * URDF outputs bounds in {lower, upper, velocity, torque} forms
   *
   * This function set _bounds to:
   * {lower, upper, -velocity, velocity, -torque, torque}
   */
  void boundsFromURDF(const mc_rbdyn_urdf::Limits & limits);

  /** Add missing elements to the current module stance
   *
   * If joints are present in the MultiBody but absent from the default stance,
   * this will add a default value for this joint to the stance (the joint's
   * zero configuration).
   *
   */
  void expand_stance();

  /** Make a valid ref_joint_order
   *
   * If \ref ref_joint_order() is empty, this will generate a list of actuated
   * joints in the order they appear in the kinematic tree
   *
   */
  void make_default_ref_joint_order();

  /** Returns a list of compound joint constraint description
   *
   * \see mc_rbdyn::CompoundJointConstraintDescription for details on the expected data
   */
  inline const CompoundJointConstraintDescriptionVector & compoundJoints() const
  {
    return _compoundJoints;
  }

  /** Returns the list of parameters passed to mc_rbdyn::RobotLoader::get_robot_module to obtain this module */
  inline const std::vector<std::string> & parameters() const
  {
    return _parameters;
  }

  /** Path to the robot's description package */
  std::string path;
  /** (default) Name of the robot */
  std::string name;
  /** Path to the robot's URDF file */
  std::string urdf_path;
  /** Path to the robot's RSDF folder */
  std::string rsdf_dir;
  /** Path to the robot's calib folder */
  std::string calib_dir;
  /** RBDyn representation of this robot */
  rbd::MultiBody mb;
  /** RBDyn configuration of this robot */
  rbd::MultiBodyConfig mbc;
  /** RBDyn graph representation of this robot */
  rbd::MultiBodyGraph mbg;
  /** \see bounds() */
  bounds_t _bounds;
  /** \see stance() */
  std::map<std::string, std::vector<double>> _stance;
  /** \see convexHull() */
  std::map<std::string, std::pair<std::string, std::string>> _convexHull;
  /** \see stpbvHull() */
  std::map<std::string, std::pair<std::string, std::string>> _stpbvHull;
  /** Holds visual representation of bodies in the robot */
  std::map<std::string, std::vector<mc_rbdyn_urdf::Visual>> _visual;
  /** \see collisionTransforms() */
  std::map<std::string, sva::PTransformd> _collisionTransforms;
  /** \see flexibility() */
  std::vector<Flexibility> _flexibility;
  /** \see forceSensors() */
  std::vector<ForceSensor> _forceSensors;
  /** \see bodySensors() */
  BodySensorVector _bodySensors;
  /** \see springs() */
  Springs _springs;
  /** \see minimalSelfCollisions() */
  std::vector<mc_rbdyn::Collision> _minimalSelfCollisions;
  /** \see commonSelfCollisions() */
  std::vector<mc_rbdyn::Collision> _commonSelfCollisions;
  /** \see grippers() */
  std::vector<Gripper> _grippers;
  /** \see gripperMimics() */
  std::map<std::string, std::vector<Mimic>> _gripperMimics;
  /** \see ref_joint_order() */
  std::vector<std::string> _ref_joint_order;
  /** \see default_attitude() */
  std::array<double, 7> _default_attitude = {{1., 0., 0., 0., 0., 0., 0.}};
  /** \see compoundJoints() */
  CompoundJointConstraintDescriptionVector _compoundJoints;
  /** \see parameters() */
  std::vector<std::string> _parameters;
  /** \see defaultLIPMStabilizerConfiguration() */
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration _lipmStabilizerConfig;
};

typedef std::shared_ptr<RobotModule> RobotModulePtr;

/*! \brief Converts limits provided by mc_rbdyn_urdf to bounds
 *
 * \param limits Limits as provided by mc_rbdyn_urdf
 *
 */
RobotModule::bounds_t MC_RBDYN_DLLAPI urdf_limits_to_bounds(const mc_rbdyn_urdf::Limits & limits);

using RobotModuleVector = std::vector<RobotModule, Eigen::aligned_allocator<RobotModule>>;

} // namespace mc_rbdyn

#ifdef WIN32
#  define ROBOT_MODULE_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define ROBOT_MODULE_API __attribute__((visibility("default")))
#  else
#    define ROBOT_MODULE_API
#  endif
#endif
