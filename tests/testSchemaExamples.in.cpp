/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <boost/filesystem.hpp>
#include <boost/mpl/list.hpp>
#include <boost/test/unit_test.hpp>
namespace bfs = boost::filesystem;

#include <mc_tasks/AddRemoveContactTask.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/BSplineTrajectoryTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/ExactCubicTrajectoryTask.h>
#include <mc_tasks/GazeTask.h>
#include <mc_tasks/LookAtSurfaceTask.h>
#include <mc_tasks/LookAtTask.h>
#include <mc_tasks/MomentumTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PositionBasedVisServoTask.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/SurfaceTransformTask.h>
#include <mc_tasks/VectorOrientationTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include "utils.h"

static bool configured = configureRobotLoader();
/* Create Robots with one robot and an environment for the purpose of the test */
static auto rm = mc_rbdyn::RobotLoader::get_robot_module("JVRC1");
static auto em =
    mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"));
static auto robots = mc_rbdyn::loadRobotAndEnv(*rm, *em);
static std::unique_ptr<mc_solver::QPSolver> solver_ptr = [](std::shared_ptr<mc_rbdyn::Robots> robots) {
  std::unique_ptr<mc_solver::QPSolver> solver(new mc_solver::QPSolver(robots, 0.005));
  return solver;
}(robots);
static mc_solver::QPSolver & solver = *solver_ptr;

static const bfs::path EXAMPLE_PATH = "@EXAMPLE_PATH@";
static const bfs::path JSON_EXAMPLES = EXAMPLE_PATH / "json" / "MetaTask";
static const bfs::path YAML_EXAMPLES = EXAMPLE_PATH / "yaml" / "MetaTask";

template<typename T>
struct TaskExamples
{
  static_assert(sizeof(T) == 0, "This must be specialized before being used");
};

#define TEST_TASK(TaskT, TaskN)                                         \
  template<>                                                            \
  struct TaskExamples<TaskT>                                            \
  {                                                                     \
    static const bfs::path json()                                       \
    {                                                                   \
      return JSON_EXAMPLES / #TaskN ".json";                            \
    }                                                                   \
    static const bfs::path yaml()                                       \
    {                                                                   \
      return YAML_EXAMPLES / #TaskN ".yaml";                            \
    }                                                                   \
  };                                                                    \
  BOOST_AUTO_TEST_CASE(TaskN)                                           \
  {                                                                     \
    {                                                                   \
      mc_rtc::Configuration json(TaskExamples<TaskT>::json().string()); \
      auto task = mc_tasks::MetaTaskLoader::load<TaskT>(solver, json);  \
    }                                                                   \
    {                                                                   \
      mc_rtc::Configuration yaml(TaskExamples<TaskT>::yaml().string()); \
      auto task = mc_tasks::MetaTaskLoader::load<TaskT>(solver, yaml);  \
    }                                                                   \
  }

TEST_TASK(mc_tasks::AddContactTask, AddContactTask)
TEST_TASK(mc_tasks::RemoveContactTask, RemoveContactTask)
TEST_TASK(mc_tasks::force::AdmittanceTask, AdmittanceTask)
TEST_TASK(mc_tasks::BSplineTrajectoryTask, BSplineTrajectoryTask)
TEST_TASK(mc_tasks::force::ComplianceTask, ComplianceTask)
TEST_TASK(mc_tasks::CoMTask, CoMTask)
TEST_TASK(mc_tasks::force::CoPTask, CoPTask)
TEST_TASK(mc_tasks::EndEffectorTask, EndEffectorTask)
TEST_TASK(mc_tasks::ExactCubicTrajectoryTask, ExactCubicTrajectoryTask)
TEST_TASK(mc_tasks::GazeTask, GazeTask)
TEST_TASK(mc_tasks::lipm_stabilizer::StabilizerTask, LIPMStabilizerTask)
TEST_TASK(mc_tasks::LookAtSurfaceTask, LookAtSurfaceTask)
TEST_TASK(mc_tasks::LookAtTask, LookAtTask)
TEST_TASK(mc_tasks::MomentumTask, MomentumTask)
TEST_TASK(mc_tasks::OrientationTask, OrientationTask)
TEST_TASK(mc_tasks::PositionBasedVisServoTask, PBVSTask)
TEST_TASK(mc_tasks::PositionTask, PositionTask)
TEST_TASK(mc_tasks::PostureTask, PostureTask)
TEST_TASK(mc_tasks::RelativeEndEffectorTask, RelativeEndEffectorTask)
TEST_TASK(mc_tasks::SurfaceTransformTask, SurfaceTransformTask)
TEST_TASK(mc_tasks::VectorOrientationTask, VectorOrientationTask)
