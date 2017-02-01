#include <mc_rbdyn/stance.h>

#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>

#include <mc_rbdyn/Robots.h>

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>
#include <RBDyn/CoM.h>

#include "../mc_rtc/internals/json.h"

#include <fstream>

namespace mc_rbdyn
{

namespace
{
  std::vector<PolygonInterpolator::tuple_pair_t> tpvFromJson(const rapidjson::Value & jsv)
  {
    std::vector<PolygonInterpolator::tuple_pair_t> tuple_pairs;
    if(jsv.HasMember("tuple_pairs") && jsv["tuple_pairs"].IsArray())
    {
      for(const auto & tpv : jsv["tuple_pairs"].GetArray())
      {
        if(tpv.HasMember("p1") && tpv.HasMember("p2"))
        {
          const auto & p1 = tpv["p1"];
          const auto & p2 = tpv["p2"];
          if(p1.IsArray() && p1.Capacity() == 2 &&
             p2.IsArray() && p2.Capacity() == 2)
          {
            tuple_pairs.push_back({
              {{p1[0].GetDouble(), p1[1].GetDouble()}},
              {{p2[0].GetDouble(), p2[1].GetDouble()}}
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

sva::PTransformd svaPTransformdFromJSON(rapidjson::Value & v)
{
  rapidjson::Value & rotation = v["rotation"];
  Eigen::Matrix3d rot;
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      rot(i,j) = rotation[3*i+j].GetDouble();
    }
  }
  rapidjson::Value & translation = v["translation"];
  Eigen::Vector3d t;
  for(int i = 0; i < 3; ++i)
  {
    t(i) = translation[i].GetDouble();
  }
  return sva::PTransformd(rot, t);
}

const Surface& surfaceFromJSON(const mc_rbdyn::Robot & robot, rapidjson::Value & v)
{
  if(robot.hasSurface(v["name"].GetString()))
  {
    return robot.surface(v["name"].GetString());
  }
  LOG_ERROR("Surface stored in JSON " << v["name"].GetString() << " does not exist in robot " << robot.name())
  throw(std::string("invalid json"));
}

Contact contactFromJSON(const mc_rbdyn::Robots & robots, rapidjson::Value & v)
{
  const Surface & robotSurface = surfaceFromJSON(robots.robot(), v["robotSurface"]);
  const Surface & envSurface = surfaceFromJSON(robots.env(), v["envSurface"]);
  sva::PTransformd X_es_rs = svaPTransformdFromJSON(v["X_es_rs"]);
  bool is_fixed = v["is_fixed"].GetBool();
  if(is_fixed)
  {
    return Contact(robots, robotSurface.name(), envSurface.name(), X_es_rs);
  }
  else
  {
    return Contact(robots, robotSurface.name(), envSurface.name());
  }
}

inline void addStanceFromJSON(const mc_rbdyn::Robots & robots, std::vector<Stance> & stances, rapidjson::Value & v)
{
  std::vector< std::vector<double> > q;
  for(rapidjson::Value & vq : v["q"].GetArray())
  {
    std::vector<double> qi;
    for(rapidjson::Value & vqi : vq.GetArray())
    {
      qi.push_back(vqi.GetDouble());
    }
    q.push_back(qi);
  }
  std::vector<Contact> geomContacts;
  for(rapidjson::Value & vc : v["geomContacts"].GetArray())
  {
    geomContacts.push_back(contactFromJSON(robots, vc));
  }
  std::vector<Contact> stabContacts;
  for(rapidjson::Value & vc : v["stabContacts"].GetArray())
  {
    stabContacts.push_back(contactFromJSON(robots, vc));
  }
  stances.emplace_back(q, geomContacts, stabContacts);
}

std::shared_ptr<StanceAction> stanceActionFromJSON(const mc_rbdyn::Robots & robots, rapidjson::Value & v)
{
  std::string type = v["type"].GetString();
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
  rapidjson::Document v;
  if(!mc_rtc::internal::loadDocument(filename, v)) { return; }
  for(rapidjson::Value & sv : v["stances"].GetArray())
  {
    addStanceFromJSON(robots, stances, sv);
  }
  for(rapidjson::Value & sav : v["actions"].GetArray())
  {
    actions.push_back(stanceActionFromJSON(robots, sav));
  }
  if(v.HasMember("polygon_interpolators"))
  {
    for(const auto & piv : v["polygon_interpolators"].GetArray())
    {
      interpolators.emplace_back(tpvFromJson(piv));
    }
  }
}

rapidjson::Value svaPTransformdToJSON(const sva::PTransformd & X, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  const Eigen::Matrix3d & rot = X.rotation();
  rapidjson::Value rotation(rapidjson::kArrayType);
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      rotation.PushBack(rot(i,j), allocator);
    }
  }
  ret["rotation"] = rotation;
  const Eigen::Vector3d & t = X.translation();
  rapidjson::Value translation(rapidjson::kArrayType);
  for(int i = 0; i < 3; ++i)
  {
    translation.PushBack(t(i), allocator);
  }
  ret["translation"] = translation;
  return ret;
}

rapidjson::Value surfaceToJSON(const std::shared_ptr<Surface> & surface, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  // Save surfaces common part, points doesn't have to be saved
  rapidjson::Value name(rapidjson::kStringType);
  name.SetString(surface->name().c_str(), allocator);
  ret.AddMember("name", name, allocator);
  rapidjson::Value bodyName(rapidjson::kStringType);
  bodyName.SetString(surface->bodyName().c_str(), allocator);
  ret.AddMember("bodyName", bodyName, allocator);
  rapidjson::Value X_b_s = svaPTransformdToJSON(surface->X_b_s(), allocator);
  ret.AddMember("X_b_s", X_b_s, allocator);
  rapidjson::Value materialName(rapidjson::kStringType);
  materialName.SetString(surface->materialName().c_str(), allocator);
  ret.AddMember("materialName", materialName, allocator);
  if(dynamic_cast<PlanarSurface*>(surface.get()))
  {
    rapidjson::Value planarPoints(rapidjson::kArrayType);
    for(const std::pair<double, double> & p : (dynamic_cast<PlanarSurface*>(surface.get()))->planarPoints())
    {
      rapidjson::Value pJSON(rapidjson::kObjectType);
      pJSON.AddMember("x", rapidjson::Value(p.first).Move(), allocator);
      pJSON.AddMember("y", rapidjson::Value(p.second).Move(), allocator);
      planarPoints.PushBack(pJSON, allocator);
    }
    ret["planarPoints"] = planarPoints;
  }
  else if(dynamic_cast<CylindricalSurface*>(surface.get()))
  {
    CylindricalSurface * s = dynamic_cast<CylindricalSurface*>(surface.get());
    ret.AddMember("radius", rapidjson::Value(s->radius()).Move(), allocator);
    ret.AddMember("width", rapidjson::Value(s->width()).Move(), allocator);
  }
  else if(dynamic_cast<GripperSurface*>(surface.get()))
  {
    GripperSurface * s = dynamic_cast<GripperSurface*>(surface.get());
    rapidjson::Value pfo(rapidjson::kArrayType);
    for(const sva::PTransformd & p : s->pointsFromOrigin())
    {
      pfo.PushBack(svaPTransformdToJSON(p, allocator).Move(), allocator);
    }
    ret.AddMember("pointsFromOrigin", pfo, allocator);
    ret.AddMember("X_b_motor", svaPTransformdToJSON(s->X_b_motor(), allocator).Move(), allocator);
    ret.AddMember("motorMaxTorque", rapidjson::Value(s->motorMaxTorque()).Move(), allocator);
  }
  return ret;
}

rapidjson::Value contactToJSON(const Contact & contact, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  ret.AddMember("robotSurface", surfaceToJSON(contact.r1Surface(), allocator).Move(), allocator);
  ret.AddMember("envSurface", surfaceToJSON(contact.r2Surface(), allocator).Move(), allocator);
  ret.AddMember("X_es_rs", svaPTransformdToJSON(contact.X_r2s_r1s(), allocator).Move(), allocator);
  ret.AddMember("is_fixed", rapidjson::Value(contact.isFixed()).Move(), allocator);
  return ret;
}

rapidjson::Value stanceToJSON(Stance & stance, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  rapidjson::Value q(rapidjson::kArrayType);
  for(const std::vector<double> & qi : stance.q())
  {
    rapidjson::Value jqi = rapidjson::Value(rapidjson::kArrayType);
    for(double v : qi)
    {
      jqi.PushBack(v, allocator);
    }
    q.PushBack(jqi, allocator);
  }
  ret.AddMember("q", q, allocator);
  rapidjson::Value geomContacts = rapidjson::Value(rapidjson::kArrayType);
  for(const Contact & c : stance.geomContacts())
  {
    geomContacts.PushBack(contactToJSON(c, allocator), allocator);
  }
  ret.AddMember("geomContacts", geomContacts, allocator);
  rapidjson::Value stabContacts = rapidjson::Value(rapidjson::kArrayType);
  for(const Contact & c : stance.stabContacts())
  {
    stabContacts.PushBack(contactToJSON(c, allocator), allocator);
  }
  ret.AddMember("stabContacts", stabContacts, allocator);
  return ret;
}

rapidjson::Value stanceActionToJSON(StanceAction & action, rapidjson::Document::AllocatorType & allocator)
{
  rapidjson::Value ret(rapidjson::kObjectType);
  if(dynamic_cast<IdentityContactAction*>(&action))
  {
    ret.AddMember("type", "Identity", allocator);
  }
  else if(dynamic_cast<AddContactAction*>(&action))
  {
    ret.AddMember("type", "Add", allocator);
    ret.AddMember("contact", contactToJSON(action.contact(), allocator).Move(), allocator);
  }
  else if(dynamic_cast<RemoveContactAction*>(&action))
  {
    ret.AddMember("type", "Remove", allocator);
    ret.AddMember("contact", contactToJSON(action.contact(), allocator).Move(), allocator);
  }
  return ret;
}

void saveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(stances[i]);
  }

  rapidjson::Document document;
  rapidjson::Document::AllocatorType & allocator = document.GetAllocator();
  rapidjson::Value stancesAndActionsJSON(rapidjson::kObjectType);

  rapidjson::Value stancesJSON(rapidjson::kArrayType);
  for(Stance & stance : stances)
  {
    stancesJSON.PushBack(stanceToJSON(stance, allocator).Move(), allocator);
  }

  rapidjson::Value stanceActionsJSON(rapidjson::kArrayType);
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.PushBack(stanceActionToJSON(*sa, allocator).Move(), allocator);
  }

  stancesAndActionsJSON.AddMember("stances", stancesJSON, allocator);
  stancesAndActionsJSON.AddMember("actions", stanceActionsJSON, allocator);

  mc_rtc::internal::saveDocument(filename, document);
}

void pSaveStances(const mc_rbdyn::Robots &/*robots*/, const std::string & filename, std::vector<Stance*> & stances, std::vector< std::shared_ptr<StanceAction> > & actions)
{
  for(size_t i = 0; i < std::min(stances.size(), actions.size()); ++i)
  {
    actions[i]->update(*stances[i]);
  }

  rapidjson::Document document;
  rapidjson::Document::AllocatorType & allocator = document.GetAllocator();
  rapidjson::Value stancesAndActionsJSON(rapidjson::kObjectType);

  rapidjson::Value stancesJSON(rapidjson::kArrayType);
  for(Stance * stance : stances)
  {
    stancesJSON.PushBack(stanceToJSON(*stance, allocator).Move(), allocator);
  }

  rapidjson::Value stanceActionsJSON(rapidjson::kArrayType);
  for(std::shared_ptr<StanceAction> & sa : actions)
  {
    stanceActionsJSON.PushBack(stanceActionToJSON(*sa, allocator).Move(), allocator);
  }

  stancesAndActionsJSON.AddMember("stances", stancesJSON, allocator);
  stancesAndActionsJSON.AddMember("actions", stanceActionsJSON, allocator);

  mc_rtc::internal::saveDocument(filename, document);
}

}
