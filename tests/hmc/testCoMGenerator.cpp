#include "generator.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/time.h>

using namespace mc_planning;
using Vector3 = Eigen::Vector3d;

const double dt = 0.005;
const int n_preview = lround(1.6 / dt);

static const int X = 0;
static const int Y = 1;
static const int Z = 2;

int main(void)
{

  generator com_traj(n_preview, dt);

  com_traj.push_back(Vector3((double)n_preview * dt, -0.2, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3((double)n_preview * dt, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3(0.1, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3(1.6, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3(0.1, 0.2, 0.095));
  com_traj.push_back(com_traj.Steps().back() + Vector3(1.6, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3(0.1, 0.0, -0.19));
  com_traj.push_back(com_traj.Steps().back() + Vector3(1.6, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3(0.1, -0.2, 0.095));
  com_traj.push_back(com_traj.Steps().back() + Vector3((double)n_preview * dt, 0.0, 0.0));
  com_traj.push_back(com_traj.Steps().back() + Vector3((double)n_preview * dt, 0.0, 0.0));

  int n_loop = lround(com_traj.Steps().back()(0) / dt) - n_preview;
  std::vector<Vector3> log_com_ideal(n_loop + 1, Vector3::Zero());
  std::vector<Vector3> log_com_cmp(n_loop + 1, Vector3::Zero());
  std::vector<Vector3> log_com_out(n_loop + 1, Vector3::Zero());
  std::vector<Vector3> log_zmp_ideal(n_loop + 1, Vector3::Zero());
  std::vector<Vector3> log_zmp_cmp(n_loop + 1, Vector3::Zero());
  std::vector<Vector3> log_zmp_out(n_loop + 1, Vector3::Zero());

  struct timeval tv;
  gettimeofday(&tv, NULL);
  double t_start = tv.tv_sec + tv.tv_usec * 1.0e-6;

  int n_steps = 0;
  for(int loop = 0; loop <= n_loop; loop++)
  {
    com_traj.generate(loop);

    log_com_ideal[loop] = com_traj.IdealCOGPosition();
    log_com_cmp[loop] = com_traj.CompensatedCOGPosition();
    log_com_out[loop] = com_traj.OutputCOGPosition();

    log_zmp_ideal[loop] = com_traj.IdealZMPPosition();
    log_zmp_cmp[loop] = com_traj.CompensatedZMPPosition();
    log_zmp_out[loop] = com_traj.OutputZMPPosition();
  }

  gettimeofday(&tv, NULL);
  double t_end = tv.tv_sec + tv.tv_usec * 1.0e-6;
  std::cout << "calc time = " << t_end - t_start << std::endl;
  std::cout << "ave. calc time = " << (t_end - t_start) / (double)n_loop << std::endl;

  std::ofstream ofs("test.dat");
  for(unsigned int k = 0; k < n_loop; k++)
  {
    ofs << (double)k * dt << " " << log_com_ideal[k](X) << " " << log_com_ideal[k](Y) << " " << log_com_ideal[k](Z)
        << " "
        << " " << log_com_cmp[k](X) << " " << log_com_cmp[k](Y) << " " << log_com_cmp[k](Z) << " "
        << " " << log_com_out[k](X) << " " << log_com_out[k](Y) << " " << log_com_out[k](Z) << " "
        << " " << log_zmp_ideal[k](X) << " " << log_zmp_ideal[k](Y) << " " << log_zmp_ideal[k](Z) << " "
        << " " << log_zmp_cmp[k](X) << " " << log_zmp_cmp[k](Y) << " " << log_zmp_cmp[k](Z) << " "
        << " " << log_zmp_out[k](X) << " " << log_zmp_out[k](Y) << " " << log_zmp_out[k](Z) << " " << std::endl;
  }
  ofs.close();

  std::cout << "end of com trajectory generation" << std::endl;

  return 0;
}
