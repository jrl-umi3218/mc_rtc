#include <mc_planning/generator.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/time_utils.h>

using namespace mc_planning;
using Vector3 = Eigen::Vector3d;

constexpr double dt = 0.005;
const unsigned n_preview = static_cast<unsigned>(std::lround(1.6 / dt));

int main(void)
{

  generator com_traj(n_preview, dt);

  // Trajectory of size 2*n_preview+1
  std::vector<Eigen::Vector3d> steps;
  steps.push_back(Vector3((double)n_preview * dt, -0.2, 0.0));
  steps.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});
  steps.push_back(steps.back() + Vector3{0.1, 0.0, 0.0});
  steps.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  steps.push_back(steps.back() + Vector3{0.1, 0.2, 0.095});
  steps.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  steps.push_back(steps.back() + Vector3{0.1, 0.0, -0.19});
  steps.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  steps.push_back(steps.back() + Vector3{0.1, -0.2, 0.095});
  steps.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});
  // Repeat last step for computations
  steps.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});
  com_traj.steps(steps);
  mc_rtc::log::info(
      "Desired steps:\nTime\tCoM X\tCoM Y\n{}",
      mc_rtc::io::to_string(steps, [](const Eigen::Vector3d & v) -> Eigen::RowVector3d { return v; }, "\n"));

  mc_rtc::Logger logger(mc_rtc::Logger::Policy::NON_THREADED, "/tmp", "mc_rtc-test");
  logger.start("CoMGenerator", dt);
  com_traj.addToLogger(logger);

  unsigned n_loop = static_cast<unsigned>(std::lround(com_traj.steps().back()(0) / dt) - n_preview);
  mc_rtc::log::info("n_loop {}", n_loop);
  /*
   * Generate the CoM trajectory based on the parameters in com_traj
   */
  auto comGeneration = [&]() {
    for(unsigned loop = 0; loop <= n_loop; loop++)
    {
      com_traj.generate(loop);
      logger.log();
    }
  };

  auto duration = mc_rtc::measure_ms::execution(comGeneration).count();
  mc_rtc::log::info("calc time = {} (ms)", duration);
  mc_rtc::log::info("ave. calc time = {} (ms)", duration / static_cast<double>(n_loop));
  mc_rtc::log::success("end of com trajectory generation");

  com_traj.removeFromLogger(logger);
  return 0;
}
