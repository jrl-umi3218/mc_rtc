#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
// #include <RBDyn/MultiBodyConfig.h>

#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>

#include <algorithm>

namespace nb = nanobind;
using namespace nb::literals;
using Robot = mc_rbdyn::Robot;
using RobotModule = mc_rbdyn::RobotModule;
using RobotFrame = mc_rbdyn::RobotFrame;
using BodySensor = mc_rbdyn::BodySensor;
using ForceSensor = mc_rbdyn::ForceSensor;
using JointSensor = mc_rbdyn::JointSensor;
using Device = mc_rbdyn::Device;

namespace mc_rtc_python
{

template<typename Class, typename Ret, typename... Args>
void bind_ref_cref_accessor(nb::class_<Class> & c,
                            const std::string & name,
                            Ret & (Class::*fun)(),
                            const Ret & (Class::*cfun)() const,
                            Args &&... args)
{
  c.def((name + "_const").c_str(), [&](Class & self) -> const Ret & { return std::invoke(cfun, self); },
        std::forward<Args>(args)...);
  c.def(name.c_str(), [&](Class & self) -> Ret & { return std::invoke(fun, self); }, std::forward<Args>(args)...);
}

void bind_Robot(nb::module_ & m)
{
  auto robot = nb::class_<mc_rbdyn::Robot>(m, "Robot");
  robot.def(
      "name", [](Robot & self) { return self.name(); },
      R"(
   Returns the name of the robot

   To rename a robot, use :py:func:rename:
                                )");
  robot
      .def("module", static_cast<const RobotModule & (Robot::*)() const>(&Robot::module),
           "Retrieve the asociated RobotModule")
      .def("bodySensor", static_cast<const BodySensor & (Robot::*)() const>(&Robot::bodySensor),
           ":returns: the first BodySensor in the robot")
      .def("bodySensor", static_cast<const BodySensor & (Robot::*)(const std::string &) const>(&Robot::bodySensor),
           "name"_a,
           R"(
  :returns: a specific BobySensor by name

  :param name: Name of the sensor

  :throws: If the sensor does not exist
                    )")
      .def("addBodySensor", &Robot::addBodySensor,
           R"(
   Add BodySensor to the robot

  :param: sensor Body to add)")
      .def("hasBodySensor", &Robot::hasBodySensor, "name"_a,
           R"(
   :returns: true if the robot has a body sensor named name

  :param name: Name of the body sensor
                                                )")
      .def("bodyHasBodySensor", &Robot::bodyHasBodySensor, "body"_a,
           R"(
  :returns: true if the specified body has a body sensor attached to it

  :param body: Body to query
                                                )")
      .def("bodyBodySensor",
           static_cast<const BodySensor & (Robot::*)(const std::string &) const>(&Robot::bodyBodySensor), "name"_a,
           R"(
  :returns: a specific BodySensor by body name

  :param name: Name of the body

  :throws: If there is no sensor attached to the body
                    )");

  robot
      .def("jointHasJointSensor", &Robot::jointHasJointSensor, "joint"_a,
           R"(
  :returns: true if the specified joint has a joint sensor attached to it

  :param joint: Joint to query
                                                )")
      .def("jointJointSensor",
           static_cast<const JointSensor & (Robot::*)(const std::string &) const>(&Robot::jointJointSensor), "name"_a,
           R"(
  :returns: a specific JointSensor by joint name

  :param name: Name of the joint

  :throws: If there is no sensor attached to the joint
                    )")
      .def("jointSensors", static_cast<const std::vector<JointSensor> & (Robot::*)() const>(&Robot::jointSensors),
           ":returns: all joint sensors");

  robot.def("hasJoint", &Robot::hasJoint, "name"_a, ":returns: True if the robot has a joint named **name**")
      .def("hasBody", &Robot::hasBody, "name"_a, ":returns: True if the robot has a body named **name**")
      .def("hasFrame", &Robot::hasFrame, "name"_a, ":returns: True if the robot has a frame named **name**");

  // TODO: bind frames
  robot.def(
      "frame", [](Robot & self, const std::string & name) -> const RobotFrame & { return self.frame(name); },
      nb::rv_policy::reference, "name"_a, R"(
   Access the frame named **name** (const)

  :throws: If the frame does not exist)");
  robot.def(
      "frame", [](Robot & self, const std::string & name) -> RobotFrame & { return self.frame(name); },
      nb::rv_policy::reference, "name"_a, R"(
   Access the frame named **name**

  :throws: If the frame does not exist)");
  robot.def("frames", &Robot::frames, "Returns the list of available frames in this robot");

  robot
      .def("jointIndexByName", &Robot::jointIndexByName, "name"_a,
           R"(
  :returns: the joint index of joint named \name

  :throws: If the joint does not exist within the robot.
          )")
      .def("jointIndexInMBC", &Robot::jointIndexInMBC, "refJointOrderIndex"_a,
           R"(
  :returns: the joint index in the mbc of the joint with index jointIndex in
  refJointOrder
  :note: Joint indices can be -1 for joints present in refJointOrder but not
  in the robot's mbc (such as filtered joints in some robot modules)
  :param refJointOrderIndex: Joint index in refJointOrder
  :throws: If jointIndex >= refJointOrder.size()
              )")
      .def("bodyIndexByName", &Robot::bodyIndexByName, "name"_a,
           R"(
  :returns: the body index of joint named **name**

  :throws: If the body does not exist within the robot.
              )");

  // TODO: bind mb/mbc/mbg
  robot.def("mbc", nb::overload_cast<>(&Robot::mbc), nb::rv_policy::reference,
            "Access the MultiBodyConfig of the robot's mb");

  robot
      .def("q_const", nb::overload_cast<>(&Robot::q, nb::const_),
           R"(Equivalent to :py:attr:`rbdyn.MultiBodyConfig.q` returned by :py:func:`mbc` (const))")
      .def("q", nb::overload_cast<>(&Robot::q),
           R"(Equivalent to :py:attr:`rbdyn.MultiBodyConfig.q` returned by :py:func:`mbc`)")
      .def("alpha_const", nb::overload_cast<>(&Robot::alpha, nb::const_),
           R"(Equivalent to :py:attr:`mbc().alpha` (const))")
      .def("alpha", nb::overload_cast<>(&Robot::alpha), R"(Equivalent to :py:attr:`mbc().alpha`)")
      .def("alphaD_const", nb::overload_cast<>(&Robot::alphaD, nb::const_),
           R"(Equivalent to :py:attr:`mbc().alphaD` (const))")
      .def("alphaD", nb::overload_cast<>(&Robot::alphaD), R"(Equivalent to :py:attr:`mbc().alphaD`)")
      .def("jointTorque_const", nb::overload_cast<>(&Robot::jointTorque, nb::const_),
           R"(Equivalent to :py:attr:`mbc().jointTorque` (const))")
      .def("jointTorque", nb::overload_cast<>(&Robot::jointTorque), R"(Equivalent to :py:attr:`mbc().jointTorque`)")
      .def("controlTorque_const", nb::overload_cast<>(&Robot::controlTorque, nb::const_),
           R"(Access the desired control torque (const))")
      .def("controlTorque", nb::overload_cast<>(&Robot::controlTorque), R"(Access the desired control torque)");

  robot
      .def("bodyPosW_const", nb::overload_cast<>(&Robot::bodyPosW, nb::const_),
           R"(Equivalent to :py:attr:`mbc().bodyPosW` (const))")
      .def("bodyPosW", nb::overload_cast<>(&Robot::bodyPosW), R"(Equivalent to :py:attr:`mbc().bodyPosW`)")
      .def("bodyVelW_const", nb::overload_cast<>(&Robot::bodyPosW, nb::const_),
           R"(Equivalent to :py:attr:`mbc().bodyVelW` (const))")
      .def("bodyVelW", nb::overload_cast<>(&Robot::bodyPosW), R"(Equivalent to :py:attr:`mbc().bodyVelW`)")
      .def("bodyVelB_const", nb::overload_cast<>(&Robot::bodyVelB, nb::const_),
           R"(Equivalent to :py:attr:`mbc().bodyVelB` (const))")
      .def("bodyVelB", nb::overload_cast<>(&Robot::bodyVelB), R"(Equivalent to :py:attr:`mbc().bodyVelB`)")
      .def("bodyAccW_const", nb::overload_cast<>(&Robot::bodyPosW, nb::const_),
           R"(Equivalent to :py:attr:`mbc().bodyAccW` (const))")
      .def("bodyAccW", nb::overload_cast<>(&Robot::bodyPosW), R"(Equivalent to :py:attr:`mbc().bodyAccW`)")
      .def("bodyAccB_const", nb::overload_cast<>(&Robot::bodyPosW, nb::const_),
           R"(Equivalent to :py:attr:`mbc().bodyAccB` (const))")
      .def("bodyAccB", nb::overload_cast<>(&Robot::bodyPosW), R"(Equivalent to :py:attr:`mbc().bodyAccB`)");
  ;

  bind_ref_cref_accessor<>(robot, "bodyPosW_Test", &Robot::bodyPosW, &Robot::bodyPosW,
                           R"(Equivalent to :py:attr:`mbc().bodyPosW`)");

  robot.def(
      "devices",
      [](Robot & self) -> std::vector<Device *>
      { // converts from DevicePtrVector
        auto & rd = self.devices();
        std::vector<Device *> devices(rd.size());
        std::transform(rd.begin(), rd.end(), devices.begin(), [](const auto & device) { return device.get(); });
        std::cout << "r devices size: " << rd.size() << std::endl;
        std::cout << "devices size: " << devices.size() << std::endl;
        return devices;
      },
      "Get all devices attached to a robot");

  // XXX: This is not stricly speaking a binding issue,
  // but robot().grippers() is empty until the MCController is created
  // This is at best unexpected from the user perspective.
  // My guess is that this was done because the grippers implementation is in mc_control while the robot is in mc_rbdyn
  robot.def("hasGripper",
          &Robot::hasGripper,
          "gripper"_a,
          R"(
   Access a gripper by name

  :param gripper: Gripper name

  :throws: If the gripper does not exist within this robot

  :note: The grippers does not exist until :py:class:`mc_control.MCController` has been created. Use :py:attr:`RobotModule.gripper` instead.
          )")
      .def("grippersByName",
              [](Robot & self)
              {
                std::map<std::string, mc_control::Gripper *> gout;
                // convert as raw pointer
                for(const auto & [name, gripperPtr] : self.grippersByName())
                {
                    gout.emplace(std::make_pair(name, gripperPtr.get()));
                }
                return gout;
              }
              )
      .def("grippers",
              [](Robot & self)
              { // convert as vector of pointers to remove use of reference_wrapper
                const auto & gin = self.grippers();
                std::vector<mc_control::Gripper *> gout(gin.size());
                std::transform(gin.begin(), gin.end(), gout.begin(),
                        [](const auto & g) { return &g.get(); });
                return gout;
              },
              R"(:returns: all grippers

  :note: The grippers does not exist until :py:class:`mc_control.MCController` has been created. Use :py:attr:`RobotModule.gripper` instead.
              )");
}

} // namespace mc_rtc_python
