#include <boost/test/unit_test.hpp>

#include <mc_control/mc_completion_criteria.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_tasks/CoMTask.h>

const double dt = 0.005;

mc_rbdyn::Robots & get_robots()
{
  static std::shared_ptr<mc_rbdyn::Robots> robots_ptr = nullptr;
  if(robots_ptr)
  {
    return *robots_ptr;
  }
  mc_rbdyn::RobotLoader::clear();
  mc_rbdyn::RobotLoader::update_robot_module_path({"@CMAKE_CURRENT_BINARY_DIR@/../src/mc_robots"});
  auto rm = mc_rbdyn::RobotLoader::get_robot_module("HRP2DRC");
  robots_ptr = mc_rbdyn::loadRobot(*rm);
  return *robots_ptr;
}

struct MockTask : public mc_tasks::CoMTask
{
  MockTask() : mc_tasks::CoMTask(get_robots(), 0) {}

  Eigen::VectorXd eval() const override { return eval_; }
  Eigen::VectorXd speed() const override { return speed_; }

  Eigen::Vector3d eval_;
  Eigen::Vector3d speed_;
};

BOOST_AUTO_TEST_CASE(TestDefault)
{
  MockTask task;
  mc_control::CompletionCriteria criteria;
  BOOST_REQUIRE(criteria.completed(task));
}

BOOST_AUTO_TEST_CASE(TestTimeout)
{
  MockTask task;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  config.add("timeout", timeout);
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  unsigned int ticks = timeout / dt;
  for(size_t i = 0; i < ticks; ++i)
  {
    BOOST_REQUIRE(!criteria.completed(task));
  }
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "timeout");
}

BOOST_AUTO_TEST_CASE(TestEval)
{
  MockTask task;
  double norm = 1e-3;
  mc_rtc::Configuration config;
  config.add("eval", norm);
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  task.eval_ = Eigen::Vector3d::UnitZ();
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_.z() = norm; // task.eval().norm() == 1e-3 == norm
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "eval");
}

BOOST_AUTO_TEST_CASE(TestSpeed)
{
  MockTask task;
  double norm = 1e-3;
  mc_rtc::Configuration config;
  config.add("speed", norm);
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  task.speed_ = Eigen::Vector3d::UnitZ();
  BOOST_REQUIRE(!criteria.completed(task));
  task.speed_.z() = norm; // task.speed().norm() == 1e-3 == norm
  BOOST_REQUIRE(!criteria.completed(task));
  task.speed_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "speed");
}

BOOST_AUTO_TEST_CASE(TestEvalAndSpeed)
{
  MockTask task;
  double norm = 1e-3;
  mc_rtc::Configuration config;
  auto AND = config.array("AND", 2);
  AND.push([norm](){ mc_rtc::Configuration c; c.add("eval", norm); return c; }());
  AND.push([norm](){ mc_rtc::Configuration c; c.add("speed", norm); return c; }());
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  task.eval_ = Eigen::Vector3d::UnitZ();
  task.speed_ = Eigen::Vector3d::UnitZ();
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_ = Eigen::Vector3d::UnitZ();
  task.speed_.z() = norm*1e-1;
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "eval AND speed");
}

BOOST_AUTO_TEST_CASE(TestEvalOrSpeed)
{
  MockTask task;
  double norm = 1e-3;
  mc_rtc::Configuration config;
  auto OR = config.array("OR", 2);
  OR.push([norm](){ mc_rtc::Configuration c; c.add("eval", norm); return c; }());
  OR.push([norm](){ mc_rtc::Configuration c; c.add("speed", norm); return c; }());
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  task.eval_ = Eigen::Vector3d::UnitZ();
  task.speed_ = Eigen::Vector3d::UnitZ();
  BOOST_REQUIRE(!criteria.completed(task));
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "eval");
  task.eval_ = Eigen::Vector3d::UnitZ();
  task.speed_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "speed");
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "eval");
}

BOOST_AUTO_TEST_CASE(TestEvalAndSpeedOrTimeout)
{
  MockTask task;
  double norm = 1e-3;
  double timeout = 5.0;
  mc_rtc::Configuration config;
  auto OR = config.array("OR", 2);
  OR.push([norm](){
          mc_rtc::Configuration c;
          auto AND = c.array("AND", 2);
          AND.push([norm](){ mc_rtc::Configuration c; c.add("eval", norm); return c; }());
          AND.push([norm](){ mc_rtc::Configuration c; c.add("speed", norm); return c; }());
          return c;
          }());
  OR.push([timeout](){ mc_rtc::Configuration c; c.add("timeout", timeout); return c; }());
  mc_control::CompletionCriteria criteria;
  criteria.configure(dt, config);
  // criteria <=> (eval().norm() < norm && speed().norm() < 1e-3) || timeout)
  task.eval_ = Eigen::Vector3d::UnitZ();
  task.speed_ = Eigen::Vector3d::UnitZ();
  BOOST_REQUIRE(!criteria.completed(task)); // every condition false
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(!criteria.completed(task)); // eval true, speed false, timeout false
  task.eval_.z() = 1.0;
  task.speed_.z() = norm*1e-1;
  BOOST_REQUIRE(!criteria.completed(task)); // eval false, speed true, timeout false
  task.eval_.z() = norm*1e-1;
  BOOST_REQUIRE(criteria.completed(task)); // eval true, speed true, timeout false
  BOOST_REQUIRE(criteria.output() == "eval AND speed");
  task.eval_.z() = 1.0; // from now, eval always false, speed always true
  unsigned int ticks = timeout / dt;
  for(size_t i = 3; i < ticks; ++i)
  {
    BOOST_REQUIRE(!criteria.completed(task));
  }
  BOOST_REQUIRE(criteria.completed(task));
  BOOST_REQUIRE(criteria.output() == "timeout");
}
