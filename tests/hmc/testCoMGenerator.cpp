#include <mc_planning/generator.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/time_utils.h>

using namespace mc_planning;
using Vector3 = Eigen::Vector3d;

int main(void)
{
  double dt = 0.005;
  CenteredPreviewWindow preview(1.6, dt);

  // Trajectory of size 2*n_preview+1
  generator com_traj(preview);

  auto prev_time = preview.halfDuration();
  PreviewSteps<Eigen::Vector2d> steps;
  steps.add({prev_time, {-0.2, 0.0}});
  steps.addRelative({prev_time, {0.0, 0.0}});
  steps.addRelative({0.1, {0.0, 0.0}});
  steps.addRelative({1.6, {0.0, 0.0}});
  steps.addRelative({0.1, {0.2, 0.095}});
  steps.addRelative({1.6, {0.0, 0.0}});
  steps.addRelative({0.1, {0.0, -0.19}});
  steps.addRelative({1.6, {0.0, 0.0}});
  steps.addRelative({0.1, {-0.2, 0.095}});
  steps.addRelative({prev_time, {0.0, 0.0}});
  steps.initialize();

  com_traj.steps(steps);
  mc_rtc::log::info("Desired steps:\nTime\tCoM X\tCoM Y\n{}", mc_rtc::io::to_string(steps.steps(), "\n"));

  mc_rtc::Logger logger(mc_rtc::Logger::Policy::NON_THREADED, "/tmp", "mc_rtc-test");
  logger.start("CoMGenerator", dt);
  com_traj.addToLogger(logger);

  unsigned n_loop = preview.indexFromTime(com_traj.steps().back().t()) - preview.halfSize();
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
  mc_rtc::log::info("Desired steps:\nTime\tCoM X\tCoM Y\n{}", mc_rtc::io::to_string(com_traj.steps().steps(), "\n"));
  mc_rtc::log::info("calc time = {} (ms)", duration);
  mc_rtc::log::info("ave. calc time = {} (ms)", duration / static_cast<double>(n_loop));
  mc_rtc::log::success("end of com trajectory generation");

  com_traj.removeFromLogger(logger);
  return 0;
}
