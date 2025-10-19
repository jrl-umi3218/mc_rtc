/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/CompoundJointConstraintDescription.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/JointSensor.h>
#include <mc_rbdyn/Mimic.h>
#include <mc_rbdyn/RobotConverterConfig.h>
#include <mc_rbdyn/Springs.h>
#include <mc_rbdyn/api.h>
#include <mc_rbdyn/lipm_stabilizer/StabilizerConfiguration.h>

#include <mc_rtc/constants.h>
#include <mc_rtc/deprecated.h>

#include <RBDyn/parsers/common.h>

#include <sch/S_Object/S_Object.h>

#include <array>
#include <map>
#include <vector>

/* This is an interface designed to provide additionnal information about a robot */

namespace mc_rbdyn
{

using S_ObjectPtr = std::shared_ptr<sch::S_Object>;

/** Holds a vector of unique pointers
 *
 * This enables copy operation by cloning the devices
 */
struct DevicePtrVector : public std::vector<DevicePtr>
{
  inline DevicePtrVector() = default;

  MC_RBDYN_DLLAPI DevicePtrVector(const DevicePtrVector & v);
  MC_RBDYN_DLLAPI DevicePtrVector & operator=(const DevicePtrVector & v);

  inline DevicePtrVector(DevicePtrVector && v) = default;
  inline DevicePtrVector & operator=(DevicePtrVector && v) = default;
};

/** Holds a map from body's names to visual representations
 *
 * Provide a conversion operator for backward compatibility
 */
struct VisualMap : public std::map<std::string, std::vector<rbd::parsers::Visual>>
{
  inline VisualMap() = default;

  inline VisualMap(const VisualMap & v) = default;
  inline VisualMap & operator=(const VisualMap & v) = default;

  inline VisualMap(VisualMap && v) = default;
  inline VisualMap & operator=(VisualMap && v) = default;

  using std::map<std::string, std::vector<rbd::parsers::Visual>>::operator=;
};

struct MC_RBDYN_DLLAPI RobotModule
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! Holds information regarding the bounds (specified in the urdf)
   *
   * The vector should have 6 entries:
   * - lower/upper position bounds
   * - lower/upper velocity bounds
   * - lower/upper torque bounds
   *
   * Each entry is a map joint name <-> bound
   */
  using bounds_t = std::vector<std::map<std::string, std::vector<double>>>;

  /*! Holds information regarding the additional acceleration bounds (specified in addition to urdf limits)
   *
   * The vector should have 2 entries:
   * - lower/upper acceleration bounds
   *
   * Each entry is a map joint name <-> bound
   */
  using accelerationBounds_t = std::vector<std::map<std::string, std::vector<double>>>;

  /*! Holds information regarding the additional jerk bounds (specified in addition to urdf limits)
   *
   * The vector should have 2 entries:
   * - lower/upper jerk bounds
   *
   * Each entry is a map joint name <-> bound
   */
  using jerkBounds_t = std::vector<std::map<std::string, std::vector<double>>>;

  /*! Holds information regarding the additional torque-derivative bounds (specified in addition to urdf limits)
   *
   * The vector should have 2 entries:
   * - lower/upper torque-derivative bounds
   *
   * Each entry is a map joint name <-> bound
   */
  using torqueDerivativeBounds_t = std::vector<std::map<std::string, std::vector<double>>>;

  /*! Holds necessary information to create a gripper */
  struct MC_RBDYN_DLLAPI Gripper
  {
    /*! Holds information regarding gripper control safety parameters */
    struct MC_RBDYN_DLLAPI Safety
    {
      /*! Percentage of max velocity of active joints in the gripper */
      static constexpr double DEFAULT_PERCENT_VMAX = 0.25;
      /*! Difference between the command and the reality that triggers the safety */
      static constexpr double DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER = mc_rtc::constants::toRad(8.);
      /** Release offset [gripper units] */
      static constexpr double DEFAULT_RELEASE_OFFSET = mc_rtc::constants::toRad(2);
      /*! Number of iterations before the security is triggered */
      static constexpr unsigned int DEFAULT_OVER_COMMAND_LIMIT_ITER_N = 5;

      // FIXME These constructors are only needed to facilitate initialization and not required in C++14
      inline Safety() : Safety(DEFAULT_PERCENT_VMAX) {}

      inline Safety(double percentVMax,
                    double actualCommandDiffTrigger = DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER,
                    double releaseSafetyOffset = DEFAULT_RELEASE_OFFSET,
                    unsigned int overCommandLimitIterN = DEFAULT_OVER_COMMAND_LIMIT_ITER_N)
      : percentVMax(percentVMax), actualCommandDiffTrigger(actualCommandDiffTrigger),
        releaseSafetyOffset(releaseSafetyOffset), overCommandLimitIterN(overCommandLimitIterN)
      {
      }

      /*! Load safety parameters from a configuration object */
      void load(const mc_rtc::Configuration & config);
      /*! Save safety parameters */
      mc_rtc::Configuration save() const;

      /*! Percentage of max velocity of active joints in the gripper */
      double percentVMax;
      /*! Difference between the command and the reality that triggers the safety */
      double actualCommandDiffTrigger;
      /** Offset by which the gripper is released when safety is triggered */
      double releaseSafetyOffset;
      /*! Number of iterations before the security is triggered */
      unsigned int overCommandLimitIterN;
    };

    /** Constructor with no mimics and no safety information */
    Gripper(const std::string & name, const std::vector<std::string> & joints, bool reverse_limits);

    /** Constructor with safety parameters but no mimics information */
    Gripper(const std::string & name,
            const std::vector<std::string> & joints,
            bool reverse_limits,
            const Safety & safety);

    /** Constructor with mimics and safety information */
    Gripper(const std::string & name,
            const std::vector<std::string> & joints,
            bool reverse_limits,
            const Safety & safety,
            const std::vector<Mimic> & mimics);

    /*! Gripper's name */
    std::string name;
    /*! Active joints in the gripper */
    std::vector<std::string> joints;
    /*! Whether the limits should be reversed, see mc_control::Gripper */
    bool reverse_limits;

    // FIXME In C++17 std::optional is a better semantic
    /*! Returns the safety parameters if provided, otherwise a nullptr */
    inline const Safety * safety() const { return hasSafety_ ? &safety_ : nullptr; }

    /*! Returns the mimics parameters if provided, otherwise a nullptr */
    inline const std::vector<Mimic> * mimics() const { return hasMimics_ ? &mimics_ : nullptr; }

  private:
    /*! True if safety parameters were provided by the user */
    bool hasSafety_ = false;
    /*! Gripper safety parameters */
    Safety safety_;
    /*! True if mimic parameters were provided by the user */
    bool hasMimics_ = false;
    /*! User-provided mimic information */
    std::vector<Mimic> mimics_;
    /*! Internal constructor used by the others */
    Gripper(const std::string & name,
            const std::vector<std::string> & joints,
            bool reverse_limits,
            const Safety * safety,
            const std::vector<Mimic> * mimics);
  };

  /*! A lightweight frame description
   *
   * This will be used when creating extra frames in the robot
   *
   * These frames shouldn't:
   * - share a name with bodies in the robot
   * - share a name with surfaces in the robot
   */
  struct MC_RBDYN_DLLAPI FrameDescription
  {
    /** Constructor */
    FrameDescription(const std::string & n, const std::string & p, const sva::PTransformd & pt, bool baked = false)
    : name(n), parent(p), X_p_f(pt), baked(baked)
    {
    }

    /** Name of the frame */
    std::string name;
    /** Frame's parent */
    std::string parent;
    /** Transformation from the parent frame to this one */
    sva::PTransformd X_p_f;
    /** If true the frame is baked */
    bool baked = false;
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
  : path(path), name(name), urdf_path(urdf_path), rsdf_dir(path + "/rsdf/" + name), calib_dir(path + "/calib/" + name),
    _real_urdf(urdf_path)
  {
  }

  /** Construct from a parser result */
  RobotModule(const std::string & name, const rbd::parsers::ParserResult & res);

  /** Gives fine control over the connection operation \see RobotModule::connect */
  struct ConnectionParameters
  {

/**
 * Helper to define a member variables and
 * methods that allow chaining calls to setters e.g
 * ConnecionParameters{}.name(...).urdf_path(...)
 */
#define CONNECTION_PROPERTY(NAME, TYPE, DEFAULT)              \
  TYPE NAME##_ = DEFAULT;                                     \
  inline ConnectionParameters & NAME(const TYPE & b) noexcept \
  {                                                           \
    NAME##_ = b;                                              \
    return *this;                                             \
  }                                                           \
  inline auto & NAME() noexcept { return NAME##_; }

    /** @name General parameters
     *
     * These parameters affect the general output
     *
     * @{
     */

    /** Where all the temporary assembly files are put, defaults to a randomly generated temporary folder */
    CONNECTION_PROPERTY(path, std::string, "")
    /** Name of the resulting robot, defaults to `this.name`_`prefix`_`other.name` */
    CONNECTION_PROPERTY(name, std::string, "")
    /** Path to the resulting robot's URDF file, defaults to `outputPath`/urdf/`name`.urdf */
    CONNECTION_PROPERTY(urdf_path, std::string, "")
    /** Path to the resulting robot's RSDF files, defaults to `outputPath`/rsdf/`name`/ */
    CONNECTION_PROPERTY(rsdf_dir, std::string, "")
    /** Path to the calibration directory, defaults to `outputPath`/calib/ */
    CONNECTION_PROPERTY(calib_dir, std::string, "")
    /** Whether the default gripper safety parameters are taken from this (default) or other */
    CONNECTION_PROPERTY(useGripperSafetyFromThis, bool, true)
    /** Whether the default LIPM stabilizer configuration is taken from this (default) or other */
    CONNECTION_PROPERTY(useLIPMStabilizerConfigFromThis, bool, true)

    /** @} */
    /* End General parameters section */

    /** @name Connection joint
     *
     * These parameters let you control how the two RobotModule are connected
     *
     * @{
     */

    /** Type of joint used for connection, defaults to rbd::Joint::Type::Fixed */
    CONNECTION_PROPERTY(jointType, rbd::Joint::Type, rbd::Joint::Type::Fixed)

    /** Joint axis, defaults to Eigen::Vector3d::UnitZ() */
    CONNECTION_PROPERTY(jointAxis, Eigen::Vector3d, Eigen::Vector3d::UnitZ())
    /** Is joint forward, defaults to true */
    CONNECTION_PROPERTY(jointForward, bool, true)
    /** Joint name, must be unique in the assembly, defaults to `this.name`_connect_`prefix`_`other.name` */
    CONNECTION_PROPERTY(jointName, std::string, "")
    /** Default (half-sitting) configuration for the connection joint */
    CONNECTION_PROPERTY(jointStance, std::vector<double>, {})
    /** Joint limits (position, velocity, torque) for the connection joint, \ref RobotModule::connect will throw if the
     * limits are not compatible with the connection joint */
    using JointLimits = std::array<std::vector<double>, 6>;
    CONNECTION_PROPERTY(jointLimits, JointLimits, {})
    /** Joint acceleration limits for the connection joint, can remain empty regardless of the joint type but throws if
     * the provided limits are not compatible with the connection joint otherwise */
    using JointAccelerationLimits = std::array<std::vector<double>, 2>;
    CONNECTION_PROPERTY(jointAccelerationLimits, JointAccelerationLimits, {})
    /** Joint jerk limits for the connection joint, can remain empty regardless of the joint type but throws if
     * the provided limits are not compatible with the connection joint otherwise */
    using JointJerkLimits = std::array<std::vector<double>, 2>;
    CONNECTION_PROPERTY(jointJerkLimits, JointJerkLimits, {})
    /** Joint torque derivative limits for the connection joint, can remain empty regardless of the joint type but
     * throws if the provided limits are not compatible with the connection joint otherwise */
    using JointTorqueDerivativeLimits = std::array<std::vector<double>, 2>;
    CONNECTION_PROPERTY(jointTorqueDerivativeLimits, JointTorqueDerivativeLimits, {})
    /** Transformation from this_body to the connection joint, defaults to identity */
    CONNECTION_PROPERTY(X_this_connection, sva::PTransformd, sva::PTransformd::Identity())
    /** Transformation from other_body to the connection joint, defaults to identity */
    CONNECTION_PROPERTY(X_other_connection, sva::PTransformd, sva::PTransformd::Identity())

    /** @} */
    /* End Connection joint section */

    /** @name Name mapping
     *
     * These parameters let you define a finer mapping between names in other and the names in the resulting
     * RobotModule. If a specific mapping is provided, then the provided name is used instead, otherwise the default is
     * used, i.e. the provided prefix is applied to the name
     *
     * @{
     */

    using mapping_t = std::unordered_map<std::string, std::string>;

    /** Remap body names */
    CONNECTION_PROPERTY(bodyMapping, mapping_t, {})

    /** Remap joint names */
    CONNECTION_PROPERTY(jointMapping, mapping_t, {})

    /** Remap convex names (applies to stpbv and collision transformations mapping) */
    CONNECTION_PROPERTY(convexMapping, mapping_t, {})

    /** Remap gripper names */
    CONNECTION_PROPERTY(gripperMapping, mapping_t, {})

    /** Remap surface names */
    CONNECTION_PROPERTY(surfaceMapping, mapping_t, {})

    /** Remap force sensor names */
    CONNECTION_PROPERTY(forceSensorMapping, mapping_t, {})

    /** Remap body sensor names */
    CONNECTION_PROPERTY(bodySensorMapping, mapping_t, {})

    /** Remap device names */
    CONNECTION_PROPERTY(deviceMapping, mapping_t, {})

#undef CONNECTION_PROPERTY
    /** @} */
    /* End Name mapping section */
  };

  /** Create a new RobotModule by connecting this instance with another RobotModule
   *
   * \param other The module that will be connected to this instance
   *
   * \param this_body Which body in this RobotModule is used to connect the two instances
   *
   * \param other_body Which body in the other RobotModule is used to connect the two instances
   *
   * \param prefix Prefix applied to every named entities in \p other before they are added to the resulting module
   *
   * \param params Parameters that control how \p other is transformed during the connection, \see ConnectionParameters
   * documentation for details
   *
   * \note If \p other is has a floating then this base is eliminated in the process
   *
   * \note Devices from both instances are cloned into the new instance as needed
   *
   * \note You are responsible for handling force sensor calibration files, by default this function creates an empty
   * calibration directory that you can populate with appropriate files
   *
   * \throws If a name collision occurs and in conditions specified by wrong \p params
   *
   */
  RobotModule connect(const mc_rbdyn::RobotModule & other,
                      const std::string & this_body,
                      const std::string & other_body,
                      const std::string & prefix,
                      const ConnectionParameters & params) const;

  /** Create a new RobotModule by disconnecting the provided RobotModule from this one
   *
   * \param other The module that will be disconnected from this instance
   *
   * \param this_body Which body in this RobotModule was used to connect the two instances
   *
   * \param other_body Which body in the other RobotModule was used to connect the two instances
   *
   * \param prefix Prefix applied to every named entities in \p other before they were added to this module
   *
   * \param params Parameters that controlled how \p other was connected to this module, \see ConnectionParameters for
   * details
   *
   * \note If you used connect(other, this_body, other_body, prefix, params) to create this module then using
   * disconnect(other, this_body, other_body, prefix, params) will reverse the operation except:
   * - if \p params.name is empty then `prefix`_`other.name` is removed from the name if present, otherwise \p
   * params.name is used as the output name
   * - \p params.useGripperSafetyFromThis cannot be reversed if it was false
   * - \p params.useLIPMStabilizerConfigFromThis cannot be reversed if it was false
   *
   * \note You are responsible for handling force sensor calibration files, by default this function creates an empty
   * calibration directory that you can populate with appropriate files
   */
  RobotModule disconnect(const mc_rbdyn::RobotModule & other,
                         const std::string & this_body,
                         const std::string & other_body,
                         const std::string & prefix,
                         const ConnectionParameters & params) const;

  /** Initialize the module from a parser result
   *
   * - Initialize mb, mbc and mbg
   * - Initial limits
   * - Initialize _collisionTransforms
   * - Initialize _visual
   * - Create a default joint order
   * - Create a default stance
   */
  void init(const rbd::parsers::ParserResult & res);

  /** Returns the robot's bounds obtained from parsing a urdf
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
  const std::vector<std::map<std::string, std::vector<double>>> & bounds() const { return _bounds; }

  /** Returns the robot's acceleration bounds
   *
   * The vector should hold 2 string -> vector<double> map
   *
   * Each map's keys are joint names and values are joint limits.
   *
   * They should be provided in the following order:
   * - acceleration limits (lower/upper)
   */
  const std::vector<std::map<std::string, std::vector<double>>> & accelerationBounds() const
  {
    return _accelerationBounds;
  }

  /** Returns the robot's jerk bounds
   *
   * The vector should hold 2 string -> vector<double> map
   *
   * Each map's keys are joint names and values are joint limits.
   *
   * They should be provided in the following order:
   * - jerk limits (lower/upper)
   */
  const std::vector<std::map<std::string, std::vector<double>>> & jerkBounds() const { return _jerkBounds; }

  /** Returns the robot's torque-derivative bounds
   *
   * The vector should hold 2 string -> vector<double> map
   *
   * Each map's keys are joint names and values are joint limits.
   *
   * They should be provided in the following order:
   * - torque-derivative limits (lower/upper)
   */
  const std::vector<std::map<std::string, std::vector<double>>> & torqueDerivativeBounds() const
  {
    return _torqueDerivativeBounds;
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
  const std::map<std::string, std::vector<double>> & stance() const { return _stance; }

  /** Returns a map describing the convex hulls for the robot
   *
   * A key defines a valid collision name, there should be no collision with the names in \ref collisionObjects()
   *
   * A value is composed of two strings:
   *
   * 1. the name of the body the convex is attached to
   *
   * 2. the path to the file containing the convex description
   *
   * The transformation between the convex and the body it's attached to are
   * provided in a separate map see \ref collisionTransforms()
   */
  const std::map<std::string, std::pair<std::string, std::string>> & convexHull() const { return _convexHull; }

  /** Returns a map describing collision objects for the robot
   *
   * A key defines a valid collision name, there should be no collision with the names \ref convexHull()
   *
   * A value is composed of two strings:
   *
   * 1. the name of the body the object is attached to
   *
   * 2. the collision object
   *
   * The transformation between the convex and the body it's attached to are
   * provided in a separate map see \ref collisionTransforms()
   */
  const std::map<std::string, std::pair<std::string, S_ObjectPtr>> & collisionObjects() const
  {
    return _collisionObjects;
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
  const std::map<std::string, std::pair<std::string, std::string>> & stpbvHull() const { return _stpbvHull; }

  /** Returns a map describing the transformation between convex/STPBV hulls
   * and their parent bodies
   *
   * A key defines the collision name. The value is the transformation between
   * this collision object and its parent body
   */
  const std::map<std::string, sva::PTransformd> & collisionTransforms() const { return _collisionTransforms; }

  /** Return the flexibilities of the robot
   *
   * \see mc_rbdyn::Flexibility for details on the expected data
   */
  const std::vector<Flexibility> & flexibility() const { return _flexibility; }

  /** Return the force sensors of the robot
   *
   * \see mc_rbdyn::ForceSensor for details on the expected data
   */
  const std::vector<ForceSensor> & forceSensors() const { return _forceSensors; }

  /** Return the body sensors of the robot
   *
   * \see mc_rbdyn::BodySensor for details on the expected data
   */
  const BodySensorVector & bodySensors() const { return _bodySensors; }

  /** Return the joint sensors of the robot
   *
   * \see mc_rbdyn::JointSensor for details on the expected data
   */
  const std::vector<JointSensor> & jointSensors() const { return _jointSensors; }

  /** Return the springs of a robot
   *
   * \see mc_rbdyn::Spring for details on the expected data
   */
  const Springs & springs() const { return _springs; }

  /** Return a minimal self-collision set
   *
   * This set of collision describe self-collisions that you always want to
   * enable regardless of the application
   *
   * \see mc_rbdyn::Collision for details on the expected data
   */
  const std::vector<mc_rbdyn::Collision> & minimalSelfCollisions() const { return _minimalSelfCollisions; }

  /** Return a common self-collision set
   *
   * This set of collision describe self-collisions that you want to enable for
   * general applications. Generally this is a super-set of \ref
   * minimalSelfCollisions
   *
   * \see mc_rbdyn::Collision for details on the expected data
   */
  const std::vector<mc_rbdyn::Collision> & commonSelfCollisions() const { return _commonSelfCollisions; }

  /** Return the grippers in the robot
   *
   * \see mc_rbdyn::Gripper for details on the expected data
   */
  const std::vector<Gripper> & grippers() const { return _grippers; }

  /** Returns default gripper safety parameters if one is not provided by a gripper.
   *
   * This can also be used to provide identical settings for every grippers in a robot
   *
   * \see mc_rbdyn::Gripper::Safety for details on the safety parameters
   */
  inline const Gripper::Safety & gripperSafety() const { return _gripperSafety; }

  /** Return the reference (native controller) joint order of the robot
   *
   * If it is empty, \ref make_default_ref_joint_order() will be used to
   * generate one
   */
  const std::vector<std::string> & ref_joint_order() const { return _ref_joint_order; }

  /** Return the default attitude of the floating base
   *
   * This attitute is associated to the \ref stance() configuration
   */
  const std::array<double, 7> & default_attitude() const { return _default_attitude; }

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
  void boundsFromURDF(const rbd::parsers::Limits & limits);

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
  inline const CompoundJointConstraintDescriptionVector & compoundJoints() const { return _compoundJoints; }

  /** Returns the list of parameters passed to mc_rbdyn::RobotLoader::get_robot_module to obtain this module */
  inline const std::vector<std::string> & parameters() const { return _parameters; }

  /** Returns the list of parameters to get a RobotModule that is a canonical representation of this module
   *
   * An empty vector means that the canonical and control module are the same
   **/
  inline const std::vector<std::string> & canonicalParameters() const { return _canonicalParameters; }

  /** Returns the configuration for the control to canonical conversion
   *
   * The default configuration:
   * - copies the common configuration values from the control model to the canonical model
   * - copies the initial encoders value into the canonical model for other joints
   * - enforce mimic relations
   * - copy the world pose of the control model to the canonical model
   */
  RobotConverterConfig controlToCanonicalConfig;

  /* Post-processing for control to canonical
   *
   * The default implementation does nothing
   *
   * This function is called automatically by mc_rtc after each iteration of MCGlobalController::run()
   *
   * It is called last, after the controller/observer/grippers have run and before the plugins/log/GUI
   */
  std::function<void(const mc_rbdyn::Robot & control, mc_rbdyn::Robot & canonical)> controlToCanonicalPostProcess =
      [](const mc_rbdyn::Robot &, mc_rbdyn::Robot &) {};

  /** Returns the path to a "real" URDF file
   *
   * This will be used to show a visually distinct robot for displaying the
   * control and observed models simulatenously.
   *
   * This defaults to urdf_path
   */
  std::string real_urdf() const { return _real_urdf; }

  /** Returns a list of non standard sensors supported by this module */
  inline const DevicePtrVector & devices() const { return _devices; }

  /** Returns a list of robot frames supported by this module */
  inline const std::vector<FrameDescription> & frames() const noexcept { return _frames; }

public:
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
  /** \see accelerationBounds() */
  accelerationBounds_t _accelerationBounds;
  /** \see jerkBounds() */
  jerkBounds_t _jerkBounds;
  /** \see torqueDerivativeBounds() */
  torqueDerivativeBounds_t _torqueDerivativeBounds;
  /** \see stance() */
  std::map<std::string, std::vector<double>> _stance;
  /** \see convexHull() */
  std::map<std::string, std::pair<std::string, std::string>> _convexHull;
  /** \see collisionObjects() */
  std::map<std::string, std::pair<std::string, S_ObjectPtr>> _collisionObjects;
  /** \see stpbvHull() */
  std::map<std::string, std::pair<std::string, std::string>> _stpbvHull;
  /** Holds visual representation of bodies in the robot */
  VisualMap _visual;
  /** Holds collision representation of bodies in the robot */
  VisualMap _collision;
  /** \see collisionTransforms() */
  std::map<std::string, sva::PTransformd> _collisionTransforms;
  /** \see flexibility() */
  std::vector<Flexibility> _flexibility;
  /** \see forceSensors() */
  std::vector<ForceSensor> _forceSensors;
  /** \see bodySensors() */
  BodySensorVector _bodySensors;
  /** \see jointSensors() */
  std::vector<JointSensor> _jointSensors;
  /** \see springs() */
  Springs _springs;
  /** \see minimalSelfCollisions() */
  std::vector<mc_rbdyn::Collision> _minimalSelfCollisions;
  /** \see commonSelfCollisions() */
  std::vector<mc_rbdyn::Collision> _commonSelfCollisions;
  /** \see grippers() */
  std::vector<Gripper> _grippers;
  /** \see gripperSafety() */
  Gripper::Safety _gripperSafety;
  /** \see ref_joint_order() */
  std::vector<std::string> _ref_joint_order;
  /** \see default_attitude() */
  std::array<double, 7> _default_attitude = {{1., 0., 0., 0., 0., 0., 0.}};
  /** \see compoundJoints() */
  CompoundJointConstraintDescriptionVector _compoundJoints;
  /** \see parameters() */
  std::vector<std::string> _parameters;
  /** \see canonicalParameters() */
  std::vector<std::string> _canonicalParameters;
  /** \see defaultLIPMStabilizerConfiguration() */
  mc_rbdyn::lipm_stabilizer::StabilizerConfiguration _lipmStabilizerConfig;
  /** \see real_urdf() */
  std::string _real_urdf;
  /** \see sensors() */
  DevicePtrVector _devices;
  /** \see frames() */
  std::vector<FrameDescription> _frames;
};

typedef std::shared_ptr<RobotModule> RobotModulePtr;

/*! \brief Converts limits provided by RBDyn parsers to bounds
 *
 * \param limits Limits as provided by RBDyn parsers
 *
 */
MC_RBDYN_DLLAPI RobotModule::bounds_t urdf_limits_to_bounds(const rbd::parsers::Limits & limits);

using RobotModuleVector = std::vector<RobotModule, Eigen::aligned_allocator<RobotModule>>;

/** Checks that two RobotModule are compatible for control
 *
 * The requirements are:
 * - Same reference joint order
 * - Same force sensors
 * - Same body sensors
 * - Same joint sensors
 * - Same grippers
 * - Same devices
 *
 * \returns True if the two modules are compatible, false otherwise
 */
MC_RBDYN_DLLAPI bool check_module_compatibility(const RobotModule & lhs, const RobotModule & rhs);

inline bool operator==(const RobotModule::Gripper::Safety & lhs, const RobotModule::Gripper::Safety & rhs)
{
  return lhs.percentVMax == rhs.percentVMax && lhs.actualCommandDiffTrigger == rhs.actualCommandDiffTrigger
         && lhs.releaseSafetyOffset == rhs.releaseSafetyOffset
         && lhs.overCommandLimitIterN == rhs.overCommandLimitIterN;
}

inline bool operator==(const RobotModule::Gripper & lhs, const RobotModule::Gripper & rhs)
{
  auto compareMimics = [&]()
  {
    auto lmimics = lhs.mimics();
    auto rmimics = rhs.mimics();
    if(lmimics == nullptr && rmimics == nullptr) { return true; }
    if(lmimics == nullptr || rmimics == nullptr) { return false; }
    return *lmimics == *rmimics;
  };
  auto compareSafety = [&]()
  {
    auto lsafety = lhs.safety();
    auto rsafety = rhs.safety();
    if(lsafety == nullptr && rsafety == nullptr) { return true; }
    if(lsafety == nullptr || rsafety == nullptr) { return false; }
    return *lsafety == *rsafety;
  };
  return lhs.name == rhs.name && lhs.joints == rhs.joints && lhs.reverse_limits == rhs.reverse_limits && compareSafety()
         && compareMimics();
}

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
