#include <mc_rbdyn/stance.h>

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>

#include <mc_rbdyn/robot.h>

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/CoM.h>

#include <json/json.h>
#include <fstream>

namespace mc_rbdyn
{

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

sva::PTransformd svaPTransformdFromJSON(Json::Value & v)
{
  Json::Value rotation = v["rotation"];
  Eigen::Matrix3d rot;
  for(Json::Value::ArrayIndex i = 0; i < 3; ++i)
  {
    for(Json::Value::ArrayIndex j = 0; j < 3; ++j)
    {
      rot(i,j) = rotation[3*i+j].asDouble();
    }
  }
  Json::Value translation = v["translation"];
  Eigen::Vector3d t;
  for(Json::Value::ArrayIndex i = 0; i < 3; ++i)
  {
    t(i) = translation[i].asDouble();
  }
  return sva::PTransformd(rot, t);
}

const Surface& surfaceFromJSON(const mc_rbdyn::Robot & robot, Json::Value & v)
{
  if(robot.hasSurface(v["name"].asString()))
  {
    return robot.surface(v["name"].asString());
  }
  LOG_ERROR("Surface stored in JSON " << v["name"].asString() << " does not exist in robot " << robot.name())
  throw(std::string("invalid json"));
}

Contact contactFromJSON(const mc_rbdyn::Robots & robots, Json::Value & v)
{
  const Surface & robotSurface = surfaceFromJSON(robots.robot(), v["robotSurface"]);
  const Surface & envSurface = surfaceFromJSON(robots.env(), v["envSurface"]);
  sva::PTransformd X_es_rs = svaPTransformdFromJSON(v["X_es_rs"]);
  bool is_fixed = v["is_fixed"].asBool();
  if(is_fixed)
  {
    return Contact(robots, robotSurface.name(), envSurface.name(), X_es_rs);
  }
  else
  {
    return Contact(robots, robotSurface.name(), envSurface.name());
  }
}

inline void addStanceFromJSON(const mc_rbdyn::Robots & robots, std::vector<Stance> & stances, Json::Value & v)
{
  std::vector< std::vector<double> > q;
  for(Json::Value & vq : v["q"])
  {
    std::vector<double> qi;
    for(Json::Value & vqi : vq)
    {
      qi.push_back(vqi.asDouble());
    }
    q.push_back(qi);
  }
  std::vector<Contact> geomContacts;
  for(Json::Value & vc : v["geomContacts"])
  {
    geomContacts.push_back(contactFromJSON(robots, vc));
  }
  std::vector<Contact> stabContacts;
  for(Json::Value & vc : v["stabContacts"])
  {
    stabContacts.push_back(contactFromJSON(robots, vc));
  }
  stances.emplace_back(q, geomContacts, stabContacts);
}

std::shared_ptr<StanceAction> stanceActionFromJSON(const mc_rbdyn::Robots & robots, Json::Value & v)
{
  std::string type = v["type"].asString();
  if(type == "Identity")
  {
    return std::shared_ptr<StanceAction>(new IdentityContactAction());
  }
  else
  {
    Contact contact = contactFromJSON(robots, v["contact"]);
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
  Json::Value v;
  std::ifstream ifs(filename);
  if(!ifs.is_open())
  {
    LOG_ERROR("Could not open stance file " << filename)
    throw("Unable to open stance file");
  }
  ifs >> v;
  for(Json::Value & sv : v["stances"])
  {
    //stances.emplace_back(std::move(stanceFromJSON(sv)));
    addStanceFromJSON(robots, stances, sv);
  }
  for(Json::Value & sav : v["actions"])
  {
    actions.push_back(stanceActionFromJSON(robots, sav));
  }
  if(v.isMember("polygon_interpolators"))
  {
    for(const auto & piv : v["polygon_interpolators"])
    {
      interpolators.emplace_back(piv);
    }
  }
}

Json::Value svaPTransformdToJSON(const sva::PTransformd & X)
{
  Json::Value ret(Json::objectValue);
  const Eigen::Matrix3d & rot = X.rotation();
  Json::Value rotation(Json::arrayValue);
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      rotation.append(Json::Value(rot(i,j)));
    }
  }
  ret["rotation"] = rotation;
  const Eigen::Vector3d & t = X.translation();
  Json::Value translation(Json::arrayValue);
  for(int i = 0; i < 3; ++i)
  {
    translation.append(Json::Value(t(i)));
  }
  ret["translation"] = translation;
  return ret;
}

Json::Value surfaceToJSON(const std::shared_ptr<Surface> & surface)
{
  Json::Value ret(Json::objectValue);
  // Save surfaces common part, points doesn't have to be saved
  ret["name"] = Json::Value(surface->name());
  ret["bodyName"] = Json::Value(surface->bodyName());
  ret["X_b_s"] = svaPTransformdToJSON(surface->X_b_s());
  ret["materialName"] = Json::Value(surface->materialName());
  if(dynamic_cast<PlanarSurface*>(surface.get()))
  {
    Json::Value planarPoints(Json::arrayValue);
    for(const std::pair<double, double> & p : (dynamic_cast<PlanarSurface*>(surface.get()))->planarPoints())
    {
      Json::Value pJSON(Json::objectValue);
      pJSON["x"] = Json::Value(p.first);
      pJSON["y"] = Json::Value(p.second);
      planarPoints.append(pJSON);
    }
    ret["planarPoints"] = planarPoints;
  }
  else if(dynamic_cast<CylindricalSurface*>(surface.get()))
  {
    CylindricalSurface * s = dynamic_cast<CylindricalSurface*>(surface.get());
    ret["radius"] = Json::Value(s->radius());
    ret["width"] = Json::Value(s->width());
  }
  else if(dynamic_cast<GripperSurface*>(surface.get()))
  {
    GripperSurface * s = dynamic_cast<GripperSurface*>(surface.get());
    Json::Value pfo(Json::arrayValue);
    for(const sva::PTransformd & p : s->pointsFromOrigin())
    {
      pfo.append(svaPTransformdToJSON(p));
    }
    ret["pointsFromOrigin"] = pfo;
    ret["X_b_motor"] = svaPTransformdToJSON(s->X_b_motor());
    ret["motorMaxTorque"] = Json::Value(s->motorMaxTorque());
  }
  return ret;
}

Json::Value contactToJSON(const Contact & contact)
{
  Json::Value ret(Json::objectValue);
  ret["robotSurface"] = surfaceToJSON(contact.r1Surface());
  ret["envSurface"] = surfaceToJSON(contact.r2Surface());
  ret["X_es_rs"] = svaPTransformdToJSON(contact.X_r2s_r1s());
  ret["is_fixed"] = Json::Value(contact.isFixed());
  return ret;
}

Json::Value stanceToJSON(Stance & stance)
{
  Json::Value ret(Json::objectValue);
  ret["q"] = Json::Value(Json::arrayValue);
  for(const std::vector<double> & qi : stance.q())
  {
    Json::Value jqi = Json::Value(Json::arrayValue);
    for(double v : qi)
    {
      jqi.append(Json::Value(v));
    }
    ret["q"].append(jqi);
  }
  ret["geomContacts"] = Json::Value(Json::arrayValue);
  for(const Contact & c : stance.geomContacts())
  {
    ret["geomContacts"].append(contactToJSON(c));
  }
  ret["stabContacts"] = Json::Value(Json::arrayValue);
  for(const Contact & c : stance.stabContacts())
  {
    ret["stabContacts"].append(contactToJSON(c));
  }
  return ret;
}

Json::Value stanceActionToJSON(StanceAction & action)
{
  Json::Value ret(Json::objectValue);
  if(dynamic_cast<IdentityContactAction*>(&action))
  {
    ret["type"] = Json::Value("Identity");
  }
  else if(dynamic_cast<AddContactAction*>(&action))
  {
    ret["type"] = Json::Value("Add");
    ret["contact"] = contactToJSON(action.contact());
  }
  else if(dynamic_cast<RemoveContactAction*>(&action))
  {
    ret["type"] = Json::Value("Remove");
    ret["contact"] = contactToJSON(action.contact());
  }
  return ret;
}

void saveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(stances[i]);
  }

  Json::Value stancesAndActionsJSON(Json::objectValue);

  Json::Value stancesJSON(Json::arrayValue);
  for(Stance & stance : stances)
  {
    stancesJSON.append(stanceToJSON(stance));
  }

  Json::Value stanceActionsJSON(Json::arrayValue);
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.append(stanceActionToJSON(*sa));
  }

  stancesAndActionsJSON["stances"] = stancesJSON;
  stancesAndActionsJSON["actions"] = stanceActionsJSON;

  std::ofstream ofs(filename);
  ofs << stancesAndActionsJSON;
  ofs.close();
}

void pSaveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance*> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(*stances[i]);
  }

  Json::Value stancesAndActionsJSON(Json::objectValue);

  Json::Value stancesJSON(Json::arrayValue);
  for(Stance * stance : stances)
  {
    stancesJSON.append(stanceToJSON(*stance));
  }

  Json::Value stanceActionsJSON(Json::arrayValue);
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.append(stanceActionToJSON(*sa));
  }

  stancesAndActionsJSON["stances"] = stancesJSON;
  stancesAndActionsJSON["actions"] = stanceActionsJSON;

  std::ofstream ofs(filename);
  ofs << stancesAndActionsJSON;
  ofs.close();
}

}
