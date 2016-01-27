#ifndef _H_MCRBDYNSTANCE_H_
#define _H_MCRBDYNSTANCE_H_

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/PolygonInterpolator.h>

namespace mc_rbdyn
{

struct Robot;

struct StanceImpl;

struct MC_RBDYN_DLLAPI Stance
{
public:
  Stance(const std::vector< std::vector<double> > & q, const std::vector<Contact> & geomContacts, const std::vector<Contact> stabContacts);

  ~Stance();

  /* Allow move operations to work on Stance vectors */
  Stance(Stance &&);
  Stance& operator=(Stance&&);

  const std::vector<Contact> & contacts() const;

  const std::vector< std::vector<double> > q() const;

  void q(const std::vector<std::vector<double>> & q_in);

  const std::vector<Contact> & geomContacts() const;

  const std::vector<Contact> & stabContacts() const;

  void updateContact(const Contact & oldContact, const Contact & newContact);

  Eigen::Vector3d com(const Robot & robot) const;

  std::vector<std::string> robotSurfacesInContact();
public:
  std::unique_ptr<StanceImpl> impl;
};

typedef std::pair<std::vector<Contact>, std::vector<Contact> > contact_vector_pair_t;
typedef std::pair< contact_vector_pair_t, contact_vector_pair_t > apply_return_t;

struct MC_RBDYN_DLLAPI StanceAction
{
  virtual ~StanceAction() {}

  virtual apply_return_t apply(const Stance & stance) = 0;

  virtual void update(const Stance & stance) = 0;

  virtual std::string toStr() = 0;

  virtual std::string type() = 0;

  virtual const Contact & contact() const = 0;

  virtual Contact & contact() = 0;
};

struct MC_RBDYN_DLLAPI IdentityContactAction : public StanceAction
{
public:
  virtual apply_return_t apply(const Stance & stance) override;

  virtual void update(const Stance & stance) override;

  virtual std::string toStr() override;

  virtual std::string type() override;

  virtual const Contact & contact() const override;

  virtual Contact & contact() override;
};

struct MC_RBDYN_DLLAPI AddContactAction : public StanceAction
{
public:
  AddContactAction(const Contact & contact);

  virtual apply_return_t apply(const Stance & stance) override;

  virtual void update(const Stance & stance) override;

  virtual std::string toStr() override;

  virtual std::string type() override;

  virtual const Contact & contact() const override;

  virtual Contact & contact() override;
private:
  Contact _contact;
};

struct MC_RBDYN_DLLAPI RemoveContactAction : public StanceAction
{
public:
  RemoveContactAction(const Contact & contact);

  virtual apply_return_t apply(const Stance & stance) override;

  virtual void update(const Stance & stance) override;

  virtual std::string toStr() override;

  virtual std::string type() override;

  virtual const Contact & contact() const override;

  virtual Contact & contact() override;
private:
  Contact _contact;
};

MC_RBDYN_DLLAPI void loadStances(const mc_rbdyn::Robots & robots, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions, std::vector<PolygonInterpolator> & interpolators);

MC_RBDYN_DLLAPI void saveStances(const mc_rbdyn::Robots & robots, const std::string & filename, std::vector<Stance> & stances, std::vector< std::shared_ptr<StanceAction> > & actions);

/* For pyhon bindings */
MC_RBDYN_DLLAPI void pSaveStances(const mc_rbdyn::Robots & robots, const std::string & filename, std::vector<Stance*> & stances, std::vector<std::shared_ptr<StanceAction>> & actions);

}

#endif
