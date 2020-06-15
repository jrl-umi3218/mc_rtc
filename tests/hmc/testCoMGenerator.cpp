#include <mc_rtc/log/Logger.h>
#include <mc_rtc/time_utils.h>
#include "generator.h"

using namespace mc_planning;
using Vector3 = Eigen::Vector3d;

constexpr double dt = 0.005;
const int n_preview = lround(1.6 / dt);

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

int main(void)
{

  generator com_traj(n_preview, dt);

  // Trajectory of size 2*n_preview+1
  auto steps = com_traj.Steps();
  com_traj.push_back(Vector3((double)n_preview * dt, -0.2, 0.0));
  com_traj.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{0.1, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{0.1, 0.2, 0.095});
  com_traj.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{0.1, 0.0, -0.19});
  com_traj.push_back(steps.back() + Vector3{1.6, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{0.1, -0.2, 0.095});
  com_traj.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});
  com_traj.push_back(steps.back() + Vector3{(double)n_preview * dt, 0.0, 0.0});

  mc_rtc::Logger logger(mc_rtc::Logger::Policy::NON_THREADED, "/tmp", "mc_rtc-test");
  logger.addLogEntry("IdealCOGPosition", [&com_traj]() { return com_traj.IdealCOGPosition(); });
  logger.addLogEntry("CompensatedCOGPosition", [&com_traj]() { return com_traj.CompensatedCOGPosition(); });
  logger.addLogEntry("OutputCOGPosition", [&com_traj]() { return com_traj.OutputCOGPosition(); });
  logger.addLogEntry("IdealZMPPosition", [&com_traj]() { return com_traj.IdealZMPPosition(); });
  logger.addLogEntry("CompensatedZMPPosition", [&com_traj]() { return com_traj.CompensatedZMPPosition(); });
  logger.addLogEntry("OutputZMPPosition", [&com_traj]() { return com_traj.OutputZMPPosition(); });
  logger.start("CoMGenerator", dt);

  int n_loop = lround(com_traj.Steps().back()(0) / dt) - n_preview;
  /*
   * Generate the CoM trajectory based on the parameters in com_traj
   */
  auto comGeneration = [&]() {
    for(int loop = 0; loop <= n_loop; loop++)
    {
      com_traj.generate(loop);
      logger.log();
    }
  };

  auto duration = mc_rtc::measure_ms::duration(comGeneration).count();
  mc_rtc::log::info("calc time = {} (ms)", duration);
  mc_rtc::log::info("ave. calc time = {} (ms)", duration / static_cast<double>(n_loop));
  mc_rtc::log::success("end of com trajectory generation");

  return 0;
}
