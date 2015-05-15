#include <mc_control/mc_solver/contact_util.h>

#include <mc_rbdyn/stance.h>

namespace mc_solver
{

QPContactPtr mrTasksContactFromMcContact(const mc_rbdyn::Robots & robots, const mc_rbdyn::MRContact & contact)
{
  QPContactPtr res;

  const mc_rbdyn::Robot & r1 = robots.robots[contact.r1Index];
  const mc_rbdyn::Robot & r2 = robots.robots[contact.r2Index];

  const std::shared_ptr<mc_rbdyn::Surface> & r1Surface = contact.r1Surface;
  const std::shared_ptr<mc_rbdyn::Surface> & r2Surface = contact.r2Surface;

  unsigned int r1BodyId = r1.bodyIdByName(r1Surface->bodyName);
  unsigned int r2BodyId = r2.bodyIdByName(r2Surface->bodyName);
  unsigned int r1BodyIndex = r1.bodyIndexByName(r1Surface->bodyName);
  unsigned int r2BodyIndex = r2.bodyIndexByName(r2Surface->bodyName);

  sva::PTransformd X_0_b1 = r1.mbc->bodyPosW[r1BodyIndex];
  sva::PTransformd X_0_b2 = r2.mbc->bodyPosW[r2BodyIndex];
  sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();
  const sva::PTransformd & X_b_s = contact.X_b_s;

  const std::vector<sva::PTransformd> & svaPoints = r1Surface->points;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Matrix3d> frames;
  for(const auto & p : svaPoints)
  {
    points.push_back(p.translation());
    frames.push_back(p.rotation());
  }

  const int & ambId = contact.ambiguityId;

  if(dynamic_cast<const mc_rbdyn::PlanarSurface*>(r1Surface.get()))
  {
    res.unilateralContact = new tasks::qp::UnilateralContact(contact.r1Index, contact.r2Index,
                                                             r1BodyId, r2BodyId, ambId, points,
                                                             frames[0], X_b1_b2, mc_rbdyn::Stance::nrConeGen,
                                                             mc_rbdyn::Stance::defaultFriction, X_b_s);
  }
  else if(dynamic_cast<const mc_rbdyn::GripperSurface*>(r1Surface.get()))
  {
    res.bilateralContact = new tasks::qp::BilateralContact(contact.r1Index, contact.r2Index,
                                                             r1BodyId, r2BodyId, ambId, points,
                                                             frames, X_b1_b2, mc_rbdyn::Stance::nrConeGen,
                                                             mc_rbdyn::Stance::defaultFriction, X_b_s);
  }
  else
  {
    std::cerr << "Surface is neither planar nor gripper" << std::endl;
  }

  return res;
}

std::pair<QPContactPtr, std::vector<sva::PTransformd> >
  tasksContactFromMcContact(const mc_rbdyn::Robots & robots, const mc_rbdyn::Contact & contact, const sva::PTransformd * X_es_rs)
{
  QPContactPtr res;

  const mc_rbdyn::Robot & robot = robots.robot();
  const mc_rbdyn::Robot & env = robots.env();

  const std::shared_ptr<mc_rbdyn::Surface> & robotSurface = contact.robotSurface;
  const std::shared_ptr<mc_rbdyn::Surface> & envSurface = contact.envSurface;
  unsigned int robotBodyId = robot.bodyIdByName(robotSurface->bodyName);
  unsigned int envBodyId = env.bodyIdByName(envSurface->bodyName);

  std::vector<sva::PTransformd> svas = robotSurface->points;
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Matrix3d> frames;
  sva::PTransformd X_b1_b2 = sva::PTransformd::Identity();
  if(X_es_rs)
  {
    X_b1_b2 = X_es_rs->inv();
    svas = computePoints(*robotSurface, *envSurface, *X_es_rs);
  }
  else
  {
    unsigned int robotBodyIndex = robot.bodyIndexByName(robotSurface->bodyName);
    unsigned int envBodyIndex = env.bodyIndexByName(envSurface->bodyName);
    sva::PTransformd X_0_b1 = robot.mbc->bodyPosW[robotBodyIndex];
    sva::PTransformd X_0_b2 = env.mbc->bodyPosW[envBodyIndex];
    X_b1_b2 = X_0_b2*X_0_b1.inv();
  }
  for(const auto & p : svas)
  {
    points.push_back(p.translation());
    frames.push_back(p.rotation());
  }

  if(dynamic_cast<const mc_rbdyn::PlanarSurface*>(robotSurface.get()))
  {
    res.unilateralContact = new tasks::qp::UnilateralContact(robots.robotIndex, robots.envIndex,
                                                             robotBodyId, envBodyId, points,
                                                             frames[0], X_b1_b2, mc_rbdyn::Stance::nrConeGen,
                                                             mc_rbdyn::Stance::defaultFriction);
  }
  else if(dynamic_cast<const mc_rbdyn::GripperSurface*>(robotSurface.get()))
  {
    res.bilateralContact = new tasks::qp::BilateralContact(robots.robotIndex, robots.envIndex,
                                                             robotBodyId, envBodyId, points,
                                                             frames, X_b1_b2, mc_rbdyn::Stance::nrConeGen,
                                                             mc_rbdyn::Stance::defaultFriction);
  }
  else
  {
    std::cerr << "Surface is neither planar nor gripper" << std::endl;
  }

  return std::pair<QPContactPtr, std::vector<sva::PTransformd> >(res, svas);
}

std::vector<tasks::qp::BilateralContact> mrTasksContactFromMcContactMsg
  (const mc_rbdyn::Robots & robots, const std::vector<mc_control::MRContactMsg> & msgContacts)
{
  std::vector<tasks::qp::BilateralContact> res;

  for(const auto & c : msgContacts)
  {
    const mc_rbdyn::Robot & r1 = robots.robots[c.r1_index];
    const mc_rbdyn::Robot & r2 = robots.robots[c.r2_index];

    unsigned int r1BodyId = r1.bodyIdByName(c.r1_body);
    unsigned int r2BodyId = r2.bodyIdByName(c.r2_body);

    const std::vector<sva::PTransformd> svaPoints = c.r1_points;
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Matrix3d> frames;
    for(const auto & p : svaPoints)
    {
      points.push_back(p.translation());
      frames.push_back(p.rotation());
    }
    const sva::PTransformd X_b1_b2 = c.X_b1_b2;

    res.push_back(tasks::qp::BilateralContact(c.r1_index, c.r2_index, r1BodyId, r2BodyId,
                                              points, frames, X_b1_b2, c.nr_generators, c.mu));
  }

  return res;
}

std::vector<mc_control::MRContactMsg> mrContactsMsgFromMrContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::MRContact> & contacts)
{
  std::vector<mc_control::MRContactMsg> res;

  for(const auto & c : contacts)
  {
    const auto & r1 = robots.robots[c.r1Index];
    const auto & r2 = robots.robots[c.r2Index];

    unsigned int r1BodyIndex = r1.bodyIndexByName(c.r1Surface->bodyName);
    unsigned int r2BodyIndex = r2.bodyIndexByName(c.r2Surface->bodyName);

    sva::PTransformd X_0_b1 = r1.mbc->bodyPosW[r1BodyIndex];
    sva::PTransformd X_0_b2 = r2.mbc->bodyPosW[r2BodyIndex];
    sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();

    mc_control::MRContactMsg msg;
    msg.r1_index = static_cast<uint16_t>(c.r1Index);
    msg.r2_index = static_cast<uint16_t>(c.r2Index);
    msg.r1_body = c.r1Surface->bodyName;
    msg.r2_body = c.r2Surface->bodyName;
    msg.r1_points = c.r1Surface->points;
    msg.X_b1_b2 = X_b1_b2;
    msg.nr_generators = static_cast<uint16_t>(mc_rbdyn::Stance::nrConeGen);
    msg.mu = mc_rbdyn::Stance::defaultFriction;
    res.push_back(msg);
  }

  return res;
}

}
