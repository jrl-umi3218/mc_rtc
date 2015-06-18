#ifndef _H_MCSOLVERCONTACTUTIL_H_
#define _H_MCSOLVERCONTACTUTIL_H_

#include <Tasks/QPContacts.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <mc_control/msg/MRContact.h>

#include <memory>

namespace mc_rbdyn
{
  struct Contact;
  struct Robots;
}

namespace mc_solver
{

std::vector<mc_control::MRContactMsg> mrContactsMsgFromMrContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts);

}

#endif
