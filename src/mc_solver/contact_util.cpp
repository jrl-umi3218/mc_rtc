#include <mc_solver/contact_util.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/robot.h>

namespace mc_solver
{

std::vector<mc_control::MRContactMsg> mrContactsMsgFromMrContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::vector<mc_control::MRContactMsg> res;

  for(const auto & c : contacts)
  {
    const auto & r1 = robots.robots[c.r1Index()];
    const auto & r2 = robots.robots[c.r2Index()];

    unsigned int r1BodyIndex = r1.bodyIndexByName(c.r1Surface()->bodyName());
    unsigned int r2BodyIndex = r2.bodyIndexByName(c.r2Surface()->bodyName());

    sva::PTransformd X_0_b1 = r1.mbc->bodyPosW[r1BodyIndex];
    sva::PTransformd X_0_b2 = r2.mbc->bodyPosW[r2BodyIndex];
    sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();

    mc_control::MRContactMsg msg;
    msg.r1_index = static_cast<uint16_t>(c.r1Index());
    msg.r2_index = static_cast<uint16_t>(c.r2Index());
    msg.r1_body = c.r1Surface()->bodyName();
    msg.r2_body = c.r2Surface()->bodyName();
    msg.r1_points = const_cast<const mc_rbdyn::Surface&>(*(c.r1Surface())).points();
    msg.X_b1_b2 = X_b1_b2;
    msg.nr_generators = static_cast<uint16_t>(mc_rbdyn::Stance::nrConeGen);
    msg.mu = mc_rbdyn::Stance::defaultFriction;
    res.push_back(msg);
  }

  return res;
}

}
