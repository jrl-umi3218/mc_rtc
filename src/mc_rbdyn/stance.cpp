#include <mc_rbdyn/stance.h>

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>

#include <mc_rbdyn/Robots.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/CoM.h>

#include "../mc_rtc/internals/json.h"

#include <fstream>

namespace mc_rbdyn
{

namespace
{
  std::vector<PolygonInterpolator::tuple_pair_t> tpvFromJson(const mc_rtc::Configuration & conf)
  {
    std::vector<PolygonInterpolator::tuple_pair_t> tuple_pairs;
    if(conf.has("tuple_pairs") && conf("tuple_pairs").size())
    {
      for(const auto tpv : conf("tuple_pairs"))
      {
        if(tpv.has("p1") && tpv.has("p2"))
        {
          const auto p1 = tpv("p1");
          const auto p2 = tpv("p2");
          if(p1.size() == 2 && p2.size() == 2)
          {
            tuple_pairs.push_back({
              {{p1[0], p1[1]}},
              {{p2[0], p2[1]}}
            });
          }
        }
      }
    }
    return tuple_pairs;
  }
}


struct StanceImpl
{
public:
  std::vector< std::vector<double> > q;
  std::vector< Contact > geomContacts;
  std::vector< Contact > stabContacts;
};

Stance::Stance(const std::vector< std::vector<double> > & q, const std::vector<Contact> & geomContacts, const std::vector<Contact> stabContacts)
: impl(new StanceImpl({q, geomContacts, stabContacts}))
{
}

Stance::~Stance()
{
}

Stance::Stance(Stance && rhs)
: impl(std::move(rhs.impl))
{
}

Stance& Stance::operator=(Stance&& rhs)
{
  impl = std::move(rhs.impl);
  return *this;
}

const std::vector<Contact> & Stance::contacts() const
{
  return impl->geomContacts;
}

const std::vector< std::vector<double> > Stance::q() const
{
  return impl->q;
}

void Stance::q(const std::vector<std::vector<double>> & q_in)
{
  impl->q = q_in;
}

const std::vector<Contact> & Stance::geomContacts() const
{
  return impl->geomContacts;
}

void Stance::geomContacts(const std::vector<Contact> & contacts)
{
  impl->geomContacts = contacts;
}

const std::vector<Contact> & Stance::stabContacts() const
{
  return impl->stabContacts;
}

void Stance::stabContacts(const std::vector<Contact> & contacts)
{
  impl->stabContacts = contacts;
}

void Stance::updateContact(const Contact & oldContact, const Contact & newContact)
{
  for(Contact & c : impl->geomContacts)
  {
    if(c == oldContact) { c = newContact; }
  }
  for(Contact & c : impl->stabContacts)
  {
    if(c == oldContact) { c = newContact; }
  }
}

Eigen::Vector3d Stance::com(const Robot & robot) const
{
  rbd::MultiBodyConfig mbc(robot.mbc());
  mbc.q = impl->q;
  rbd::forwardKinematics(robot.mb(), mbc);
  return rbd::computeCoM(robot.mb(), mbc);
}

std::vector<std::string> Stance::robotSurfacesInContact()
{
  std::vector<std::string> res;
  for(const Contact & c : impl->geomContacts)
  {
    res.push_back(c.r1Surface()->name());
  }
  return res;
}

apply_return_t IdentityContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts(), stance.stabContacts());
  contact_vector_pair_t p2(stance.geomContacts(), stance.stabContacts());
  return apply_return_t(p1,p2);
}

void IdentityContactAction::update(const Stance &)
{
}

std::string IdentityContactAction::toStr()
{
  return "Identity";
}

std::string IdentityContactAction::type()
{
  return "identity";
}

const Contact & IdentityContactAction::contact() const
{
  std::string err = "Tried to access contact on IdentityContactAction";
  LOG_ERROR(err)
  throw(err.c_str());
}

Contact & IdentityContactAction::contact()
{
  std::string err = "Tried to access contact on IdentityContactAction";
  LOG_ERROR(err)
  throw(err.c_str());
}

AddContactAction::AddContactAction(const Contact & contact)
: _contact(contact)
{
}

apply_return_t AddContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts(), stance.stabContacts());
  p1.first.push_back(_contact);

  contact_vector_pair_t p2(p1.first, stance.stabContacts());
  p2.second.push_back(_contact);

  return apply_return_t(p1,p2);
}

void AddContactAction::update(const Stance & stance)
{
  auto contactit = std::find(stance.geomContacts().begin(), stance.geomContacts().end(), _contact);
  if(contactit != stance.geomContacts().end())
  {
    const Contact & contactIn = *contactit;
    _contact.X_r2s_r1s(contactIn.X_r2s_r1s());
  }
}

std::string AddContactAction::toStr()
{
  std::stringstream ss;
  ss << "Add " << _contact.toStr();
  return ss.str();
}

std::string AddContactAction::type()
{
  return "add";
}

const Contact & AddContactAction::contact() const
{
  return _contact;
}

Contact & AddContactAction::contact()
{
  return _contact;
}

RemoveContactAction::RemoveContactAction(const Contact & contact)
: _contact(contact)
{
}

apply_return_t RemoveContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts(), stance.stabContacts());
  auto contactit = std::find(p1.second.begin(), p1.second.end(), _contact);
  if(contactit != p1.second.end())
  {
    p1.second.erase(contactit);
  }

  contact_vector_pair_t p2(p1.first, p1.second);
  contactit = std::find(p2.first.begin(), p2.first.end(), _contact);
  if(contactit != p2.first.end())
  {
    p2.first.erase(contactit);
  }

  return apply_return_t(p1,p2);
}

void RemoveContactAction::update(const Stance & stance)
{
  auto contactit = std::find(stance.geomContacts().begin(), stance.geomContacts().end(), _contact);
  if(contactit != stance.geomContacts().end())
  {
    const Contact & contactIn = *contactit;
    _contact.X_r2s_r1s(contactIn.X_r2s_r1s());
  }
}

std::string RemoveContactAction::toStr()
{
  std::stringstream ss;
  ss << "Remove " << _contact.toStr();
  return ss.str();
}

std::string RemoveContactAction::type()
{
  return "remove";
}

const Contact & RemoveContactAction::contact() const
{
  return _contact;
}

Contact & RemoveContactAction::contact()
{
  return _contact;
}

sva::PTransformd svaPTransformdFromJSON(const mc_rtc::Configuration & conf)
{
  auto rotation = conf("rotation");
  Eigen::Matrix3d rot;
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      rot(i,j) = rotation[3*i+j];
    }
  }
  return sva::PTransformd(rot, conf("translation"));
}

const Surface& surfaceFromJSON(const mc_rbdyn::Robot & robot, const mc_rtc::Configuration & conf)
{
  if(robot.hasSurface(conf("name")))
  {
    return robot.surface(conf("name"));
  }
  LOG_ERROR("Surface stored in JSON " << static_cast<std::string>(conf("name")) << " does not exist in robot " << robot.name())
  throw(std::string("invalid json"));
}

Contact contactFromJSON(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & conf)
{
  const Surface & robotSurface = surfaceFromJSON(robots.robot(), conf("robotSurface"));
  const Surface & envSurface = surfaceFromJSON(robots.env(), conf("envSurface"));
  sva::PTransformd X_es_rs = svaPTransformdFromJSON(conf("X_es_rs"));
  bool is_fixed = conf("is_fixed");
  if(is_fixed)
  {
    return Contact(robots, robotSurface.name(), envSurface.name(), X_es_rs);
  }
  else
  {
    return Contact(robots, robotSurface.name(), envSurface.name());
  }
}

inline void addStanceFromJSON(const mc_rbdyn::Robots & robots, std::vector<Stance> & stances, const mc_rtc::Configuration & conf)
{
  std::vector< std::vector<double> > q = conf("q");
  std::vector<Contact> geomContacts;
  for(const auto vc : conf("geomContacts"))
  {
    geomContacts.push_back(contactFromJSON(robots, vc));
  }
  std::vector<Contact> stabContacts;
  for(const auto vc : conf("stabContacts"))
  {
    stabContacts.push_back(contactFromJSON(robots, vc));
  }
  stances.emplace_back(q, geomContacts, stabContacts);
}

std::shared_ptr<StanceAction> stanceActionFromJSON(const mc_rbdyn::Robots & robots, const mc_rtc::Configuration & conf)
{
  std::string type = conf("type");
  if(type == "Identity")
  {
    return std::shared_ptr<StanceAction>(new IdentityContactAction());
  }
  else
  {
    Contact contact = contactFromJSON(robots, conf("contact"));
    if(type == "Add")
    {
      return std::shared_ptr<StanceAction>(new AddContactAction(contact));
    }
    else if(type == "Remove")
    {
      return std::shared_ptr<StanceAction>(new RemoveContactAction(contact));
    }
  }
  throw(std::string("Invalid StanceAction saved in JSON"));
}

void loadStances(const mc_rbdyn::Robots & robots, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions, std::vector<PolygonInterpolator> & interpolators)
{
  mc_rtc::Configuration conf(filename);
  for(const auto sv : conf("stances"))
  {
    addStanceFromJSON(robots, stances, sv);
  }
  for(const auto sav : conf("actions"))
  {
    actions.push_back(stanceActionFromJSON(robots, sav));
  }
  if(conf.has("polygon_interpolators"))
  {
    for(const auto piv : conf("polygon_interpolators"))
    {
      interpolators.emplace_back(tpvFromJson(piv));
    }
  }
}

mc_rtc::Configuration svaPTransformdToJSON(const sva::PTransformd & X)
{
  mc_rtc::Configuration conf;
  mc_rtc::Configuration rotation = conf.array("rotation");
  const Eigen::Matrix3d & rot = X.rotation();
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      rotation.push(rot(i,j));
    }
  }
  conf.add("translation", X.translation());
  return conf;
}

mc_rtc::Configuration surfaceToJSON(const std::shared_ptr<Surface> & surface)
{
  mc_rtc::Configuration conf;
  // Save surfaces common part, points doesn't have to be saved
  conf.add("name", surface->name());
  conf.add("bodyName", surface->bodyName());
  conf.add("X_b_s", svaPTransformdToJSON(surface->X_b_s()));
  conf.add("materialName", surface->materialName());
  if(dynamic_cast<PlanarSurface*>(surface.get()))
  {
    mc_rtc::Configuration planarPoints = conf.array("planarPoints");
    for(const std::pair<double, double> & p : (dynamic_cast<PlanarSurface*>(surface.get()))->planarPoints())
    {
      mc_rtc::Configuration pJSON;
      pJSON.add("x", p.first);
      pJSON.add("y", p.second);
      planarPoints.push(pJSON);
    }
  }
  else if(dynamic_cast<CylindricalSurface*>(surface.get()))
  {
    CylindricalSurface * s = dynamic_cast<CylindricalSurface*>(surface.get());
    conf.add("radius", s->radius());
    conf.add("width", s->width());
  }
  else if(dynamic_cast<GripperSurface*>(surface.get()))
  {
    GripperSurface * s = dynamic_cast<GripperSurface*>(surface.get());
    auto pfo = conf.array("pointsFromOrigin");
    for(const sva::PTransformd & p : s->pointsFromOrigin())
    {
      pfo.push(svaPTransformdToJSON(p));
    }
    conf.add("X_b_motor", svaPTransformdToJSON(s->X_b_motor()));
    conf.add("motorMaxTorque", s->motorMaxTorque());
  }
  return conf;
}

mc_rtc::Configuration contactToJSON(const Contact & contact)
{
  mc_rtc::Configuration conf;
  conf.add("robotSurface", surfaceToJSON(contact.r1Surface()));
  conf.add("envSurface", surfaceToJSON(contact.r2Surface()));
  conf.add("X_es_rs", svaPTransformdToJSON(contact.X_r2s_r1s()));
  conf.add("is_fixed", contact.isFixed());
  return conf;
}

mc_rtc::Configuration stanceToJSON(Stance & stance)
{
  mc_rtc::Configuration conf;
  conf.add("q", stance.q());
  auto geomContacts = conf.array("geomContacts");
  for(const Contact & c : stance.geomContacts())
  {
    geomContacts.push(contactToJSON(c));
  }
  auto stabContacts = conf.array("stabContacts");
  for(const Contact & c : stance.stabContacts())
  {
    stabContacts.push(contactToJSON(c));
  }
  return conf;
}

mc_rtc::Configuration stanceActionToJSON(StanceAction & action)
{
  mc_rtc::Configuration conf;
  if(dynamic_cast<IdentityContactAction*>(&action))
  {
    conf.add("type", "Identity");
  }
  else if(dynamic_cast<AddContactAction*>(&action))
  {
    conf.add("type", "Add");
    conf.add("contact", contactToJSON(action.contact()));
  }
  else if(dynamic_cast<RemoveContactAction*>(&action))
  {
    conf.add("type", "Remove");
    conf.add("contact", contactToJSON(action.contact()));
  }
  return conf;
}

void saveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(stances[i]);
  }

  mc_rtc::Configuration conf;

  auto stancesJSON = conf.array("stances");
  for(Stance & stance : stances)
  {
    stancesJSON.push(stanceToJSON(stance));
  }

  auto stanceActionsJSON = conf.array("actions");
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.push(stanceActionToJSON(*sa));
  }

  conf.save(filename);
}

void pSaveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance*> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(*stances[i]);
  }

  mc_rtc::Configuration conf;

  auto stancesJSON = conf.array("stances");
  for(Stance * stance : stances)
  {
    stancesJSON.push(stanceToJSON(*stance));
  }

  auto stanceActionsJSON = conf.array("actions");
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.push(stanceActionToJSON(*sa));
  }

  conf.save(filename);
}

}
