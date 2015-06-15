#include <mc_rbdyn/stance.h>

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>

#include <mc_rbdyn/robot.h>

#include <RBDyn/FK.h>
#include <RBDyn/CoM.h>

#include <jsoncpp/json/json.h>
#include <fstream>

namespace mc_rbdyn
{

unsigned int Stance::nrConeGen = 4;
double Stance::defaultFriction = 0.7;
unsigned int Stance::nrBilatPoints = 4;

Stance::Stance(const std::vector< std::vector<double> > & q, const std::vector<Contact> & geomContacts, const std::vector<Contact> stabContacts)
: q(q), geomContacts(geomContacts), stabContacts(stabContacts)
{
}

const std::vector<Contact> & Stance::contacts() const
{
  return geomContacts;
}

void Stance::updateContact(const Contact & oldContact, const Contact & newContact)
{
  for(Contact & c : geomContacts)
  {
    if(c == oldContact) { c = newContact; }
  }
  for(Contact & c : stabContacts)
  {
    if(c == oldContact) { c = newContact; }
  }
}

Eigen::Vector3d Stance::com(const Robot & robot) const
{
  rbd::MultiBodyConfig mbc(*(robot.mbc));
  mbc.q = q;
  rbd::forwardKinematics(*(robot.mb), mbc);
  return rbd::computeCoM(*(robot.mb), mbc);
}

std::vector<std::string> Stance::robotSurfacesInContact()
{
  std::vector<std::string> res;
  for(const Contact & c : geomContacts)
  {
    res.push_back(c.r1Surface()->name());
  }
  return res;
}

apply_return_t IdentityContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts, stance.stabContacts);
  contact_vector_pair_t p2(stance.geomContacts, stance.stabContacts);
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

AddContactAction::AddContactAction(const Contact & contact)
: contact(contact)
{
}

apply_return_t AddContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts, stance.stabContacts);
  p1.first.push_back(contact);

  contact_vector_pair_t p2(p1.first, stance.stabContacts);
  p2.second.push_back(contact);

  return apply_return_t(p1,p2);
}

void AddContactAction::update(const Stance & stance)
{
  auto contactit = std::find(stance.geomContacts.begin(), stance.geomContacts.end(), contact);
  if(contactit != stance.geomContacts.end())
  {
    const Contact & contactIn = *contactit;
    contact.X_r2s_r1s(contactIn.X_r2s_r1s());
  }
}

std::string AddContactAction::toStr()
{
  std::stringstream ss;
  ss << "Add " << contact.toStr();
  return ss.str();
}

std::string AddContactAction::type()
{
  return "add";
}

RemoveContactAction::RemoveContactAction(const Contact & contact)
: contact(contact)
{
}

apply_return_t RemoveContactAction::apply(const Stance & stance)
{
  contact_vector_pair_t p1(stance.geomContacts, stance.stabContacts);
  auto contactit = std::find(p1.second.begin(), p1.second.end(), contact);
  if(contactit != p1.second.end())
  {
    p1.second.erase(contactit);
  }

  contact_vector_pair_t p2(p1.first, p1.second);
  contactit = std::find(p2.first.begin(), p2.first.end(), contact);
  if(contactit != p2.first.end())
  {
    p2.first.erase(contactit);
  }

  return apply_return_t(p1,p2);
}

void RemoveContactAction::update(const Stance & stance)
{
  auto contactit = std::find(stance.geomContacts.begin(), stance.geomContacts.end(), contact);
  if(contactit != stance.geomContacts.end())
  {
    const Contact & contactIn = *contactit;
    contact.X_r2s_r1s(contactIn.X_r2s_r1s());
  }
}

std::string RemoveContactAction::toStr()
{
  std::stringstream ss;
  ss << "Remove " << contact.toStr();
  return ss.str();
}

std::string RemoveContactAction::type()
{
  return "remove";
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

std::shared_ptr<Surface> surfaceFromJSON(Json::Value & v)
{
  std::string name = v["name"].asString();
  std::string bodyName = v["bodyName"].asString();
  sva::PTransformd X_b_s = svaPTransformdFromJSON(v["X_b_s"]);
  std::string materialName = v["materialName"].asString();
  if(v.isMember("planarPoints"))
  {
    std::vector< std::pair<double, double> > planarPoints;
    for(Json::Value & pv : v["planarPoints"])
    {
      planarPoints.push_back(std::pair<double,double>(pv["x"].asDouble(), pv["y"].asDouble()));
    }
    return std::shared_ptr<Surface>(new PlanarSurface(name, bodyName, X_b_s, materialName, planarPoints));
  }
  else if(v.isMember("radius"))
  {
    double radius = v["radius"].asDouble();
    double width = v["width"].asDouble();
    return std::shared_ptr<Surface>(new CylindricalSurface(name, bodyName, X_b_s, materialName, radius, width));
  }
  else if(v.isMember("X_b_motor"))
  {
    std::vector<sva::PTransformd> pfo;
    for(Json::Value & pv : v["pointsFromOrigin"])
    {
      pfo.push_back(svaPTransformdFromJSON(pv));
    }
    sva::PTransformd X_b_motor = svaPTransformdFromJSON(v["X_b_motor"]);
    double motorMaxTorque = v["motorMaxTorque"].asDouble();
    return std::shared_ptr<Surface>(new GripperSurface(name, bodyName, X_b_s, materialName, pfo, X_b_motor, motorMaxTorque));
  }
  std::cerr << "Cannot restore surface from JSON value" << std::endl;
  throw(std::string("invalid json"));
}

Contact contactFromJSON(Json::Value & v)
{
  std::shared_ptr<Surface> robotSurface = surfaceFromJSON(v["robotSurface"]);
  std::shared_ptr<Surface> envSurface = surfaceFromJSON(v["envSurface"]);
  sva::PTransformd X_es_rs = svaPTransformdFromJSON(v["X_es_rs"]);
  bool is_fixed = v["is_fixed"].asBool();
  if(is_fixed)
  {
    return Contact(*robotSurface, *envSurface, X_es_rs);
  }
  else
  {
    return Contact(*robotSurface, *envSurface);
  }
}

Stance stanceFromJSON(Json::Value & v)
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
    geomContacts.push_back(contactFromJSON(vc));
  }
  std::vector<Contact> stabContacts;
  for(Json::Value & vc : v["stabContacts"])
  {
    stabContacts.push_back(contactFromJSON(vc));
  }
  return Stance(q, geomContacts, stabContacts);
}

std::shared_ptr<StanceAction> stanceActionFromJSON(Json::Value & v)
{
  std::string type = v["type"].asString();
  if(type == "Identity")
  {
    return std::shared_ptr<StanceAction>(new IdentityContactAction());
  }
  else
  {
    Contact contact = contactFromJSON(v["contact"]);
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

void loadStances(const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  Json::Value v;
  std::ifstream ifs(filename);
  ifs >> v;
  for(Json::Value & sv : v["stances"])
  {
    stances.push_back(stanceFromJSON(sv));
  }
  for(Json::Value & sav : v["actions"])
  {
    actions.push_back(stanceActionFromJSON(sav));
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

Json::Value contactToJSON(Contact & contact)
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
  for(std::vector<double> & qi : stance.q)
  {
    Json::Value jqi = Json::Value(Json::arrayValue);
    for(double v : qi)
    {
      jqi.append(Json::Value(v));
    }
    ret["q"].append(jqi);
  }
  ret["geomContacts"] = Json::Value(Json::arrayValue);
  for(Contact & c : stance.geomContacts)
  {
    ret["geomContacts"].append(contactToJSON(c));
  }
  ret["stabContacts"] = Json::Value(Json::arrayValue);
  for(Contact & c : stance.stabContacts)
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
    Contact & contact = (reinterpret_cast<AddContactAction&>(action)).contact;
    ret["type"] = Json::Value("Add");
    ret["contact"] = contactToJSON(contact);
  }
  else if(dynamic_cast<RemoveContactAction*>(&action))
  {
    Contact & contact = (reinterpret_cast<RemoveContactAction&>(action)).contact;
    ret["type"] = Json::Value("Remove");
    ret["contact"] = contactToJSON(contact);
  }
  return ret;
}

void saveStances(const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  saveStancesJSON(filename, stances, actions);
}

void saveStancesJSON(const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
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

}
