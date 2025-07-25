#include <mc_rbdyn/Collision.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/Mimic.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>

#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;
using RobotModule = mc_rbdyn::RobotModule;
using RobotLoader = mc_rbdyn::RobotLoader;
using Device = mc_rbdyn::Device;

namespace mc_rtc_python
{
void bind_RobotLoader(nanobind::module_ & m)
{
  auto c = nb::class_<mc_rbdyn::RobotLoader>(m, "RobotLoader");
  c.doc() = R"(
        Load RobotModule instances from shared libraries and robot aliases
    )";
  c.def_static(
       "get_robot_module",
       [](const std::string & robotName) { return mc_rbdyn::RobotLoader::get_robot_module(robotName); }, "robotName"_a,
       R"(
  :returns: a RobotModule constructed with the provided Args

  :param: name The module name
        )")
      .def_static(
          "get_robot_module",
          [](const std::vector<std::string> & args) { return mc_rbdyn::RobotLoader::get_robot_module(args); }, "args"_a,
          R"(
  :returns: a RobotModule constructed with the provided Args

  :param args: Arguments to pass to the robot module creation. This is arbitrarly limited to 3 parameters
        )");
  c.def_static("update_robot_module_path", RobotLoader::update_robot_module_path, "paths"_a,
               R"(
  Add additional directories to the robot module path

  :param paths: Directories to be added to the module path
                    )");
  c.def_static("has_robot", &RobotLoader::has_robot, "name"_a,
               R"(
  Check if a robot is available

  :param name: Robot name
  :returns: True if the robot is available
                    )");
  c.def_static("set_verbosity", &RobotLoader::set_verbosity, "verbose"_a);
  c.def_static("available_robots", &RobotLoader::available_robots,
               R"(
          :returns: a list of available robots
          )");
  c.def_static("load_aliases", &RobotLoader::load_aliases, "fname"_a,
               R"(
  Load aliases

  An aliases file should be a map of alias to param vector

  :param fname: A JSON or YAML containing aliases
          )");
}

void bind_Mimic(nb::module_ & m)
{
  using Mimic = mc_rbdyn::Mimic;
  auto mimic = nb::class_<mc_rbdyn::Mimic>(m, "Mimic");
  mimic.doc() = "Stores mimic joint information";
  mimic.def_rw("name", &Mimic::name, "Name of the mimic joint")
      .def_rw("joint", &Mimic::joint, "Which joint this joint mimics")
      .def_rw("multiplier", &Mimic::multiplier, "Mimic multiplier (usually -1/+1)")
      .def_rw("offset", &Mimic::offset, "Mimic offset");
}

void bind_Flexibility(nb::module_ & m)
{
  using Flexibility = mc_rbdyn::Flexibility;
  auto flex = nb::class_<Flexibility>(m, "Flexibility");
  flex.doc() = R"(
  This structure holds a flexible joint, if such a joint is part of a robot,
  then the following dynamic constraint will be applied for the flexible joint
  torques:

  \f{align}
  \underline{\mathbf{\tau}} = \overline{\mathbf{\tau}} = -K\mathbf{q} -C\dot{\mathbf{q}} - O
  \f}
)";
  flex.def_rw("jointName", &Flexibility::jointName, "Name of the joint")
      .def_rw("K", &Flexibility::K, "Stiffness")
      .def_rw("C", &Flexibility::C, "Damping")
      .def_rw("O", &Flexibility::O, "Bias");
}

void bind_Springs(nb::module_ & m)
{
  using Springs = mc_rbdyn::Springs;
  auto springs = nb::class_<Springs>(m, "Springs");
  springs.doc() = "Holds data regarding springs in a robot";
  springs.def_rw("springBodies", &Springs::springsBodies, "Bodies that have springs attached to them")
      .def_rw("afterSpringsBodies", &Springs::afterSpringsBodies,
              "Bodies that come after the bodies that have springs attached to them")
      .def_rw("springsJoints", &Springs::springsJoints, "Joints forming the spring");
}

void bind_Collision(nb::module_ & m)
{
  using Col = mc_rbdyn::Collision;
  auto col = nb::class_<mc_rbdyn::Collision>(m, "Collision");
  col.doc() = "Used to define a collision constraint between two bodies";
  col.def(nb::init())
      .def(nb::init<const std::string &, const std::string &, double, double, double,
                    const std::optional<std::vector<std::string>> &, const std::optional<std::vector<std::string>> &,
                    bool, bool>(),
           "body1"_a, "body2"_a, "iDist"_a, "sDist"_a, "damping"_a, "r1Joints"_a = nb::none(),
           "r2Joints"_a = nb::none(), "r1JointsInactive"_a = false, "r2JointsInactive"_a = false);

  col.def_rw("body1", &Col::body1, "First body in the constraint")
      .def_rw("body2", &Col::body2, "Second body in the constraint")
      .def_rw("iDist", &Col::iDist, "Interaction distance")
      .def_rw("sDist", &Col::sDist, "Security distance")
      .def_rw("damping", &Col::damping, "Damping (0 is automatic)")
      .def_rw("r1Joints", &Col::r1Joints,
              R"(
  Active/Inactive joints in the first robot:

  * no value specified = all joints selected
  * value specified:
    * if r1JointsInactive = false : specified joints are treated as active
    * if r1JointsInactive = true : specified joints are treated as inactive)")
      .def_rw("r2Joints", &Col::r2Joints, "See :py:attr:`r1Joints`")
      .def_rw("r1JointsInactive", &Col::r1JointsInactive,
              "When true the selected joints in r1ActiveJoints are considered inactive")
      .def_rw("r2JointsInactive", &Col::r2JointsInactive,
              "When true the selected joints in r2ActiveJoints are considered inactive")
      .def("isNone", &Col::isNone)
      .def(nb::self == nb::self)
      .def(nb::self != nb::self)
      .def("__repr__",
           [](const Col & col)
           {
             std::ostringstream os;
             os << col;
             return os.str();
           });
}

void bind_FrameDescription(nb::class_<RobotModule> & rm)
{
  using FrameDescription = RobotModule::FrameDescription;
  auto frameDescription = nb::class_<FrameDescription>(rm, "FrameDescription");
  frameDescription.doc() = R"(
        A lightweight frame description

        This will be used when creating extra frames in the robot

        These frames shouldn't:

        * share a name with bodies in the robot
        * share a name with surfaces in the robot
        )";

  frameDescription.def(nb::init<const std::string &, const std::string &, const sva::PTransformd &, bool>(), "name"_a,
                       "parent"_a, "X_parentToFrame"_a, "baked"_a);
  frameDescription.def_rw("name", &FrameDescription::name, "Name of the frame")
      .def_rw("parent", &FrameDescription::parent, "Parent of the frame")
      .def_rw("X_p_f", &FrameDescription::X_p_f, "Transformation from the parent frame to this one")
      .def_rw("baked", &FrameDescription::baked, "When true the frame is baked");
}

void bind_Gripper(nb::class_<RobotModule> & rm)
{
  using Gripper = RobotModule::Gripper;
  using Safety = Gripper::Safety;
  using Mimic = mc_rbdyn::Mimic;

  auto gripper = nb::class_<mc_rbdyn::RobotModule::Gripper>(rm, "Gripper");
  gripper.doc() = "Holds necessary information to create a gripper";

  auto safety = nb::class_<mc_rbdyn::RobotModule::Gripper::Safety>(gripper, "Safety");
  safety
      .def_ro_static("DEFAULT_PERCENT_VMAX", &Safety::DEFAULT_PERCENT_VMAX,
                     "Percentage of max velocity of active joints in the gripper")
      .def_ro_static("DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER", &Safety::DEFAULT_ACTUAL_COMMAND_DIFF_TRIGGER,
                     "Difference between the command and the reality that triggers the safety")
      .def_ro_static("DEFAULT_RELEASE_OFFSET", &Safety::DEFAULT_RELEASE_OFFSET, "Release offset [gripper units]")
      .def_ro_static("DEFAULT_OVER_COMMAND_LIMIT_ITER_N", &Safety::DEFAULT_OVER_COMMAND_LIMIT_ITER_N,
                     "Number of iterations before the security is triggered");

  safety.def(nb::init())
      .def(nb::init<double, double, double, unsigned int>(), "percentVMax"_a, "actualCommandDiffTrigger"_a,
           "releaseSafetyOffset"_a, "overCommandLimitIterN"_a);

  safety.def("load", &Safety::load, "config"_a, "Load safety parameters from a configuration object");
  safety.def("save", &Safety::save, "Save safety parameters");

  safety.def_rw("percentVMax", &Safety::percentVMax, "Percentage of max velocity of active joints in the gripper")
      .def_rw("actualCommandDiffTrigger", &Safety::actualCommandDiffTrigger,
              "Difference between the command and the reality that triggers the safety")
      .def_rw("releaseSafetyOffset", &Safety::releaseSafetyOffset,
              "Offset by which the gripper is released when safety is triggered")
      .def_rw("overCommandLimitIterN", &Safety::overCommandLimitIterN,
              "Number of iterations before the security is triggered");

  gripper
      .def(nb::init<const std::string &, const std::vector<std::string> &, bool>(), "name"_a, "joints"_a,
           "reverse_limits"_a, "Constructor with no mimics and no safety information")
      .def(nb::init<const std::string &, const std::vector<std::string> &, bool, const Safety &>(), "name"_a,
           "joints"_a, "reverse_limits"_a, "safety"_a, "Constructor with safety parameters but not mimics information")
      .def(nb::init<const std::string &, const std::vector<std::string> &, bool, const Safety &,
                    const std::vector<Mimic> &>(),
           "name"_a, "joints"_a, "reverse_limits"_a, "safety"_a, "mimics"_a,
           "Constructor with mimics and safety information");
  gripper.def_rw("name", &Gripper::name, "Gripper's name")
      .def_rw("joints", &Gripper::joints, "Active joints in the gripper")
      .def_rw("reverse_limits", &Gripper::reverse_limits,
              "Wheter the limits should be reversed, see :py:class:`mc_rtc.mc_control.Gripper`");
}

void bind_RobotModule(nanobind::module_ & m)
{
  bind_Mimic(m);
  bind_Flexibility(m);
  bind_Springs(m);
  bind_Collision(m);

  auto rm = nb::class_<mc_rbdyn::RobotModule>(m, "RobotModule");
  rm.doc() = R"(
    A robot module contains all information needed to represent a robot. This can be used to construct an actual :py:class:`mc_rtc.mc_rbdyn.Robot` instance. It also contains additional information that can be used by interface (e.g :py:attr:`ref_joint_order`) and simulators.
    )";

  bind_FrameDescription(rm);
  bind_Gripper(rm);

  rm.def(nb::init<const std::string &, const std::string &>(), "path"_a, "name"_a,
         R"(Construct from a provided path and name

As a result:

* name is defined as \p name
* path is defined as \p path
* urdf_path is path + /urdf/ + name + .urdf
* rsdf_dir is path + /rsdf/ + name
* calib_dir is path + /calib/ + name:q

No further action is taken. This constructor is useful to inherit from

:param path: Path to the robot description
:param name: Name of the robot)"),
      rm.def(nb::init<const std::string &, const std::string &, const std::string &>(), "path"_a, "name"_a,
             "urdf_path"_a,
             R"(Construct from a provided path, name and urdf_path

  See: RobotModule(const std::string &, const std::string &)

  The difference is that urdf_path is defined to urdf_path

:param path: Path to the robot description
:param name: Name of the robot
:param urdf_path: Path to the robot URDF)");

  // RobotModule(const std::string & name, const rbd::parsers::ParserResult & res);
  // void init(const rbd::parsers::ParserResult & res);

  // const std::vector<std::map<std::string, std::vector<double>>> & bounds() const { return _bounds; }
  rm.def_rw("bounds", &RobotModule::_bounds,
            R"(
  Returns the robot's bounds obtained from parsing a urdf

  The vector should hold 6 string -> vector<double> map

  Each map's keys are joint names and values are joint limits.

  They should be provided in the following order:

  * joint limits (lower/upper)
  * velocity limits (lower/upper)
  * torque limits (lower/upper))");

  // const std::vector<std::map<std::string, std::vector<double>>> & accelerationBounds() const
  rm.def_rw("accelerationBounds", &RobotModule::_accelerationBounds,
            R"(
   The robot's acceleration bounds

   The vector should hold 2 string -> vector<double> map

   Each map's keys are joint names and values are joint limits.

   They should be provided in the following order:

   * acceleration limits (lower/upper)
           )");

  // const std::vector<std::map<std::string, std::vector<double>>> & jerkBounds() const { return _jerkBounds; }
  rm.def_rw("jerkBounds", &RobotModule::_jerkBounds,
            R"(
  The robot's jerk bounds

  The vector should hold 2 string -> vector<double> map

  Each map's keys are joint names and values are joint limits.

  They should be provided in the following order:

  * jerk limits (lower/upper))");

  // const std::vector<std::map<std::string, std::vector<double>>> & torqueDerivativeBounds() const
  rm.def_rw("torqueDerivativeBounds", &RobotModule::_torqueDerivativeBounds,
            R"(
  The robot's torque-derivative bounds

  The vector should hold 2 string -> vector<double> map

  Each map's keys are joint names and values are joint limits.

  They should be provided in the following order:

  * torque-derivative limits (lower/upper)

          )");

  // const std::map<std::string, std::vector<double>> & stance() const { return _stance; }
  rm.def_rw("stance", &RobotModule::_stance,
            R"(
  A default configuration for the robot

  Keys are joint names and values are joint configurations.

  It should be ok to include joints that are not in the robot (e.g. generate
  the same default stance for variants of a robot. However, each actuated
  joint in the robot should have a valid entry, see :py:func:`expand_stance`

  For the floating base see :py:attr:`default_attitude`
  )");

  // const std::map<std::string, std::pair<std::string, std::string>> & convexHull() const { return _convexHull; }
  rm.def_rw("convexHull", &RobotModule::_convexHull,
            R"(
  A map describing the convex hulls for the robot

  A key defines a valid collision name, there should be no collision with the names in :py:attr:`collisionObjects`

  A value is composed of two strings:

  1. the name of the body the convex is attached to
  2. the path to the file containing the convex description

  The transformation between the convex and the body it's attached to are
  provided in a separate map see :py:attr:`collisionTransforms`
  )");

  // TODO:
  // const std::map<std::string, std::pair<std::string, S_ObjectPtr>> & collisionObjects() const

  // const std::map<std::string, std::pair<std::string, std::string>> & stpbvHull() const { return _stpbvHull; }
  rm.def_rw("stpbvHull", &RobotModule::_stpbvHull,
            R"(
  Returns a map describing the STPBV hulls for the robot

  A key defines a valid collision name, a value is composed of two strings:

  1. the name of the body the convex is attached to

  2. the path to the file containing the STPBV description

  The transformation between the STPBV and the body it's attached to are
  provided in a separate map see :py:attr:`collisionTransforms`
  )");

  // const std::map<std::string, sva::PTransformd> & collisionTransforms() const { return _collisionTransforms; }
  rm.def_rw("collisionTransforms", &RobotModule::_collisionTransforms,
            R"(
  Returns a map describing the transformation between convex/STPBV hulls
  and their parent bodies

  A key defines the collision name. The value is the transformation between
  this collision object and its parent body
  )");

  rm.def_rw("flexibility", &RobotModule::_flexibility,
            R"(
  Flexibilities of the robot

  See :py:class:`Flexibility` for details on the expected data
                  )");

  rm.def_rw("springs", &RobotModule::_springs,
            R"(
  Springs of a robot

  See :py:class:`Spring` for details on the expected data
                  )");

  // const std::vector<mc_rbdyn::Collision> & minimalSelfCollisions() const { return _minimalSelfCollisions; }
  // const std::vector<mc_rbdyn::Collision> & commonSelfCollisions() const { return _commonSelfCollisions; }
  rm.def_rw("minimalSelfCollisions", &RobotModule::_minimalSelfCollisions,
            R"(
  Minimal self-collision set.

  This set of collision describe self-collisions that you always want to
  enable regardless of the application

  See :py:class:`Collision` for details on the expected data
  See also :py:attr:commonSelfCollisions:
          )");
  rm.def_rw("commonSelfCollisions", &RobotModule::_commonSelfCollisions,
            R"(
  common self-collision set.

  This set of collision describe self-collisions that you always want to
  enable regardless of the application

  See :py:class:`Collision` for details on the expected data
  See also :py:attr:minimalSelfCollisions:
          )");

  // TODO:
  // rm.def_rw("forceSensors", &RobotModule::forceSensors,
  //                 R"(
  // Force sensors of the robot
  //
  // See :py:class:`ForceSensor` for details on the expected data
  //                 )");
  // const BodySensorVector & bodySensors() const { return _bodySensors; }
  // const std::vector<JointSensor> & jointSensors() const { return _jointSensors; }
  rm.def_rw("grippers", &RobotModule::_grippers,
            R"(
  :returns: the grippers in the robot

  See :py:class:`mc_rtc.mc_rbdyn.Gripper` for details on the expected data)");
  rm.def_rw("grippersSafety", &RobotModule::_gripperSafety,
            R"(
  :returns: default gripper safety parameters if one is not provided by a gripper.

  This can also be used to provide identical settings for every grippers in a robot

  See :py:class:`mc_rtc.mc_rbdyn.Gripper.Safety` for details on the safety parameters)");

  // const std::vector<std::string> & ref_joint_order() const { return _ref_joint_order; }
  rm.def_rw("ref_joint_order", &RobotModule::_ref_joint_order,
            R"(
  :return: the reference (native controller) joint order of the robot

  If it is empty, :py:func:`make_default_ref_joint_order` will be used to
  generate one
          )");

  // const std::array<double, 7> & default_attitude() const { return _default_attitude; }
  rm.def_rw("default_attitude", &RobotModule::_default_attitude,
            R"(
  :return: the default attitude of the floating base

  This attitute is associated to the :py:attr:`stance` configuration

          )");

  // TODO:
  // const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & defaultLIPMStabilizerConfiguration() const
  // void boundsFromURDF(const rbd::parsers::Limits & limits);

  // void expand_stance();
  rm.def("expand_stance", &RobotModule::expand_stance,
         R"(
  Add missing elements to the current module stance

  If joints are present in the MultiBody but absent from the default stance,
  this will add a default value for this joint to the stance (the joint's
  zero configuration).
          )");

  // void make_default_ref_joint_order();
  rm.def("make_default_ref_joint_order", &RobotModule::make_default_ref_joint_order,
         R"(
  Make a valid ref_joint_order

  If :py:attr:`ref_joint_order` is empty, this will generate a list of actuated
  joints in the order they appear in the kinematic tree
          )");

  // TODO:
  // inline const CompoundJointConstraintDescriptionVector & compoundJoints() const { return _compoundJoints; }
  //

  // inline const std::vector<std::string> & parameters() const { return _parameters; }
  rm.def_rw("parameters", &RobotModule::_parameters,
            R"(
  List of parameters passed to mc_rbdyn::RobotLoader::get_robot_module to obtain this module)");

  // inline const std::vector<std::string> & canonicalParameters() const { return _canonicalParameters; }
  rm.def_rw("canonicalParameters", &RobotModule::_canonicalParameters,
            R"(
  List of parameters to get a RobotModule that is a canonical representation of this module
  )");

  // TODO:
  // RobotConverterConfig controlToCanonicalConfig;
  // std::function<void(const mc_rbdyn::Robot & control, mc_rbdyn::Robot & canonical)> controlToCanonicalPostProcess =

  // std::string real_urdf() const { return _real_urdf; }
  rm.def_rw("real_urdf", &RobotModule::_real_urdf,
            R"(
  Path to a "real" URDF file

  This will be used to show a visually distinct robot for displaying the
  control and observed models simulatenously.

  This defaults to urdf_path
          )");

  // FIXME(nanobind): returns a DevicePtrVector, which is not a support type in python
  // rm.def_rw("devices", &RobotModule::_devices,
  //           R"(Returns a list of non standard sensors supported by this module)");

  rm.def_rw("frames", &RobotModule::_frames, R"(List of robot frames supported by this module)");

  rm.def_rw("path", &RobotModule::path, "Path to the robot's description package")
      .def_rw("name", &RobotModule::name, "(default) Name of the robot")
      .def_rw("urdf_path", &RobotModule::path, "Path to the robot's urdf file")
      .def_rw("rsdf_dir", &RobotModule::rsdf_dir, "Path to the robot's RSDF folder (surfaces)")
      .def_rw("calib_dir", &RobotModule::calib_dir, "Path to the robot's calib folder");

  // TODO:
  // All properties that depend on custom types

  bind_RobotLoader(m);
}
} // namespace mc_rtc_python
