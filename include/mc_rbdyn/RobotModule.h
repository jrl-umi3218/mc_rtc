#ifndef _H_MCRBDYNROBOTMODULE_H_
#define _H_MCRBDYNROBOTMODULE_H_

#include <mc_rbdyn/robot.h>

#include <map>
#include <vector>

/* This is an interface designed to provide additionnal information about a robot */

namespace mc_rbdyn
{

struct Robot;

struct RobotModule
{
  /* If implemented, returns limits in this order:
      - joint limits (lower/upper)
      - velocity limits (lower/upper)
      - torque limits (lower/upper)
  */
  virtual const std::vector< std::map<unsigned int, std::vector<double> > > & bounds() const { return _bounds; }

  /* return the initial configuration of the robot */
  virtual const std::map< unsigned int, std::vector<double> > & stance() const { return _stance; }

  /* return a map (name, (bodyName, PolyhedronURL)) */
  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const { return _convexHull; }

  /* return a map (name, (bodyName, STPBVURL)) */
  virtual const std::map<std::string, std::pair<std::string, std::string> > & stpbvHull() const { return _stpbvHull; }

  /* return a map (unsigned int, sva::PTransformd) */
  virtual const std::map<unsigned int, sva::PTransformd> & collisionTransforms() const { return _collisionTransforms; }

  /* return flexibilities */
  virtual const std::vector<Flexibility> & flexibility() const { return _flexibility; }

  /* return force sensors */
  virtual const std::vector<ForceSensor> & forceSensors() const { return _forceSensors; }

  const std::string & accelerometerBody() const { return _accelerometerBody; }

  virtual const Springs & springs() const { return _springs; }

  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::vector< std::map<unsigned int, std::vector<double> > > _bounds;
  std::map< unsigned int, std::vector<double> > _stance;
  std::map<std::string, std::pair<std::string, std::string> > _convexHull;
  std::map<std::string, std::pair<std::string, std::string> > _stpbvHull;
  std::map<unsigned int, sva::PTransformd> _collisionTransforms;
  std::vector<Flexibility> _flexibility;
  std::vector<ForceSensor> _forceSensors;
  std::string _accelerometerBody;
  Springs _springs;
};

}

#endif
