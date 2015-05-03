#ifndef _H_STANCECONFIG_H_
#define _H_STANCECONFIG_H_

/* This struct holds the configuration for stances in a seq plan */

/* TODO Fill-it up */
/* TODO Build a tool to import from python */
/* TODO Build a tool to read from configuration file */

#include <Eigen/Core>

namespace mc_rbdyn
{

struct StanceConfig
{
public:
  struct CoMTask
  {
    double stiffness;
    double extraStiffness;
    double weight;
  };
  struct CoMObj
  {
    Eigen::Vector3d comOffset;
  };
  struct PostureTask
  {
    double stiffness;
    double weight;
  };
public:
  CoMTask comTask;
  CoMObj comObj;
  PostureTask postureTask;
};

}

#endif
