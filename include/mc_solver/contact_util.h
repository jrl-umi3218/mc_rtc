#ifndef _H_MCSOLVERCONTACTUTIL_H_
#define _H_MCSOLVERCONTACTUTIL_H_

#include <Tasks/QPContacts.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_solver/msg/Contact.h>

#include <memory>
#include <vector>

#include <mc_solver/api.h>

namespace mc_rbdyn
{
  struct Contact;
  struct Robots;
}

namespace mc_solver
{

MC_SOLVER_DLLAPI std::vector<mc_solver::ContactMsg> contactsMsgFromContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts);

}

#endif
