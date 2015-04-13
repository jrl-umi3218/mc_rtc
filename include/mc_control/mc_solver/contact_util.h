#ifndef _H_MCSOLVERCONTACTUTIL_H_
#define _H_MCSOLVERCONTACTUTIL_H_

#include <Tasks/QPContacts.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/contact.h>
#include <mc_rbdyn/robot.h>

#include <mc_control/msg/MRContact.h>

namespace mc_solver
{

struct QPContactPtr
{
  QPContactPtr() : unilateralContact(0), bilateralContact(0) {}
  tasks::qp::UnilateralContact * unilateralContact;
  tasks::qp::BilateralContact * bilateralContact;
};

QPContactPtr mrTasksContactFromMcContact
  (const mc_rbdyn::Robots & robots, const mc_rbdyn::MRContact & contact);

std::pair<QPContactPtr, std::vector<sva::PTransformd> > tasksContactFromMcContact
  (const mc_rbdyn::Robots & robots, const mc_rbdyn::Contact & contact, const sva::PTransformd * X_es_rs = 0);

std::vector<tasks::qp::BilateralContact> mrTasksContactFromMcContactMsg
  (const mc_rbdyn::Robots & robots, const std::vector<mc_control::MRContactMsg> & msgContacts);

std::vector<mc_control::MRContactMsg> mrContactsMsgFromMrContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::MRContact> & contacts);

}

#endif
