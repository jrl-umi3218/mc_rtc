/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "gui_TestServer.h"

#include "samples_Schema.h"

struct ComposeSchema
{
  MC_RTC_NEW_SCHEMA(ComposeSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ComposeSchema, __VA_ARGS__))
  MEMBER(int, integer, "Integer value")
  MEMBER(SimpleSchema, simple, "Simple schema")
  MEMBER(std::vector<SimpleSchema>, many, "Multiple simple schema")
#undef MEMBER
};

struct InteractiveSchema
{
  MC_RTC_NEW_SCHEMA(InteractiveSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(InteractiveSchema, __VA_ARGS__))
  MEMBER(Eigen::Vector3d,
         point,
         "3D point",
         mc_rtc::schema::Required | mc_rtc::schema::Interactive,
         Eigen::Vector3d::Zero())
  MEMBER(Eigen::Vector3d, gain, "3D gain", mc_rtc::schema::Required, Eigen::Vector3d::Zero())
  MEMBER(Eigen::Vector3d, gainOpt, "3D optional gain", mc_rtc::schema::None, Eigen::Vector3d::Zero())
#undef MEMBER
};

struct SimpleVariant
{
  MC_RTC_NEW_SCHEMA(SimpleVariant)
  using gain_t = std::variant<double, Eigen::Vector3d>;
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleVariant, __VA_ARGS__))
  MEMBER(gain_t, stiffness, "Task stiffness", mc_rtc::schema::Choices({"scalar", "dimensional"}))
#undef MEMBER
};

struct ChildSchema : public SimpleSchema
{
  MC_RTC_SCHEMA(ChildSchema, SimpleSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ChildSchema, __VA_ARGS__))
  MEMBER(double, gain, "Some gain")
#undef MEMBER
};

struct MCCContactConstraint
{
  MC_RTC_NEW_SCHEMA(MCCContactConstraint)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCContactConstraint, __VA_ARGS__))
  MEMBER(std::string, type, "Type of contact constraint", mc_rtc::schema::Choices({"Empty", "Surface", "Grasp"}))
  MEMBER(double, fricCoeff, "Contact friction coefficient")
#undef MEMBER
};

struct MCCSwingCommandConfig
{
  MC_RTC_NEW_SCHEMA(MCCSwingCommandConfig)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(MCCSwingCommandConfig, __VA_ARGS__))
  MEMBER(Eigen::Vector3d, approachOffset, "Offset for approach")
  MEMBER(Eigen::Vector3d, withdrawOffset, "Offset for withdraw")
#undef MEMBER
};

struct MCCSwingCommand
{
  MC_RTC_NEW_SCHEMA(MCCSwingCommand)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCSwingCommand, __VA_ARGS__))
  MEMBER(std::string, type, "Type of motion", mc_rtc::schema::Choices({"Add", "Remove"}))
  MEMBER(double, startTime, "Start time")
  MEMBER(double, endTime, "End time")
  MEMBER(MCCSwingCommandConfig, config, "Configuration for swing command")
#undef MEMBER
};

struct MCCContactCommand
{
  MC_RTC_NEW_SCHEMA(MCCContactCommand)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCContactCommand, __VA_ARGS__))
  MEMBER(double, time, "Time to enable contact")
  MEMBER(MCCContactConstraint, constraint, "Contact constraint")
#undef MEMBER
};

struct MCCGripperCommandConfig
{
  MC_RTC_NEW_SCHEMA(MCCGripperCommandConfig)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCGripperCommandConfig, __VA_ARGS__))
  MEMBER(double, opening, "Target opening")
#undef MEMBER
};

struct MCCGripperCommand
{
  MC_RTC_NEW_SCHEMA(MCCGripperCommand)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCGripperCommand, __VA_ARGS__))
  MEMBER(double, time, "Time to trigger gripper")
  MEMBER(std::string, name, "Gripper's name")
  MEMBER(MCCGripperCommandConfig, config, "Gripper's motion configuration")
#undef MEMBER
};

struct MCCStepCommand
{
  MC_RTC_NEW_SCHEMA(MCCStepCommand)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCStepCommand, __VA_ARGS__))
  MEMBER(std::string, limb, "Contact limb")
  MEMBER(sva::PTransformd, pose, "Contact position")
#undef MEMBER
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(MCCStepCommand, __VA_ARGS__))
  MEMBER(std::string, type, "Type of motion", mc_rtc::schema::Choices({"Add", "Remove"}))
  MEMBER(double, startTime, "Start time")
  MEMBER(double, endTime, "End time")
  MEMBER(MCCContactConstraint, constraint, "Contact constraint")
  MEMBER(MCCSwingCommand, swingCommand, "Swing command")
  MEMBER(std::vector<MCCContactCommand>, contactCommandList, "List of contact commands")
  MEMBER(std::vector<MCCGripperCommand>, gripperCommandList, "List of gripper commands")
#undef MEMBER
};

struct MCCNominalCentroidalPose
{
  MC_RTC_NEW_SCHEMA(MCCNominalCentroidalPose)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCNominalCentroidalPose, __VA_ARGS__))
  MEMBER(double, time, "Time to trigger")
  MEMBER(sva::PTransformd, pose, "Centroidal pose")
#undef MEMBER
};

struct MCCCollision
{
  MC_RTC_NEW_SCHEMA(MCCCollision)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCCollision, __VA_ARGS__))
  MEMBER(std::string, body1, "Body in r1")
  MEMBER(std::string, body2, "Body in r2")
#undef MEMBER
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_MEMBER(MCCCollision, __VA_ARGS__))
  MEMBER(double, iDist, "Interaction distance", 0.05)
  MEMBER(double, sDist, "Safety distance", 0.01)
#undef MEMBER
};

struct MCCCollisionConfig
{
  MC_RTC_NEW_SCHEMA(MCCCollisionConfig)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MCCCollisionConfig, __VA_ARGS__))
  MEMBER(double, time, "Time to trigger")
  MEMBER(std::string, type, "Type of operation", mc_rtc::schema::Choices({"Add", "Remove"}))
  MEMBER(std::string, r1, "First robot")
  MEMBER(std::string, r2, "First robot")
  MEMBER(std::vector<MCCCollision>, collisions, "Collisions")
#undef MEMBER
};

struct MultiContactMotionConfiguration
{
  MC_RTC_NEW_SCHEMA(MultiContactMotionConfiguration)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(MultiContactMotionConfiguration, __VA_ARGS__))
  MEMBER(std::string, baseTime, "How time should be interpreted", mc_rtc::schema::Choices({"Relative", "Absolute"}))
  MEMBER(std::vector<MCCStepCommand>, stepCommandList, "Multi-contact steps")
  MEMBER(std::vector<MCCNominalCentroidalPose>, nominalCentroidalPoseList, "List of nominal centroidal poses")
  MEMBER(std::vector<MCCCollisionConfig>, collisionConfigList, "List of collision in the motion")
#undef MEMBER
};

struct SchemaServer : public TestServer
{
  SchemaServer()
  {
    simple_.addToGUI(builder, {"Simple"}, "Update simple", [this]() { simple_updated(); });
    compose_.addToGUI(builder, {"Compose"}, "Update compose", [this]() { compose_updated(); });
    interactive_.addToGUI(builder, {"Interactive"}, "Update interactive", [this]() { interactive_updated(); });
    simple_variant_.addToGUI(builder, {"Simple variant"}, "Update simple", [this]() { simple_variant_updated(); });
  }

  SimpleSchema simple_;
  ComposeSchema compose_;
  InteractiveSchema interactive_;
  SimpleVariant simple_variant_;

  void simple_updated() { mc_rtc::log::info("simple_ updated:\n{}", simple_.dump(true, true)); }
  void compose_updated() { mc_rtc::log::info("compose_ updated:\n{}", compose_.dump(true, true)); }
  void interactive_updated() { mc_rtc::log::info("interactive_ updated:\n{}", interactive_.dump(true, true)); }
  void simple_variant_updated() { mc_rtc::log::info("simple_variant_ updated:\n{}", simple_variant_.dump(true, true)); }
};

int main()
{
  SchemaServer server;
  server.loop();
  return 0;
}
