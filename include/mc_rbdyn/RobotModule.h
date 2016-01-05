#pragma once

#include <mc_rbdyn/robot.h>

#include <map>
#include <vector>

#include <mc_rbdyn/api.h>

/* This is an interface designed to provide additionnal information about a robot */

namespace mc_rbdyn
{

struct Robot;

/* TODO Functions are declared const here but most implementations will likely not respect the constness */
struct MC_RBDYN_DLLAPI RobotModule
{
  RobotModule(const std::string & path, const std::string & name) : path(path), name(name) {}

  virtual ~RobotModule() {}

  /* If implemented, returns limits in this order:
      - joint limits (lower/upper)
      - velocity limits (lower/upper)
      - torque limits (lower/upper)
  */
  virtual const std::vector< std::map<int, std::vector<double> > > & bounds() const { return _bounds; }

  /* return the initial configuration of the robot */
  virtual const std::map< unsigned int, std::vector<double> > & stance() const { return _stance; }

  /* return a map (name, (bodyName, PolyhedronURL)) */
  virtual const std::map<std::string, std::pair<std::string, std::string> > & convexHull() const { return _convexHull; }

  /* return a map (name, (bodyName, STPBVURL)) */
  virtual const std::map<std::string, std::pair<std::string, std::string> > & stpbvHull() const { return _stpbvHull; }

  /* return a map (unsigned int, sva::PTransformd) */
  virtual const std::map<int, sva::PTransformd> & collisionTransforms() const { return _collisionTransforms; }

  /* return flexibilities */
  virtual const std::vector<Flexibility> & flexibility() const { return _flexibility; }

  /* return force sensors */
  virtual const std::vector<ForceSensor> & forceSensors() const { return _forceSensors; }

  const std::string & accelerometerBody() const { return _accelerometerBody; }

  virtual const Springs & springs() const { return _springs; }

  std::string path;
  std::string name;
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  std::vector< std::map<int, std::vector<double> > > _bounds;
  std::map< unsigned int, std::vector<double> > _stance;
  std::map<std::string, std::pair<std::string, std::string> > _convexHull;
  std::map<std::string, std::pair<std::string, std::string> > _stpbvHull;
  std::map< int, sva::PTransformd> _collisionTransforms;
  std::vector<Flexibility> _flexibility;
  std::vector<ForceSensor> _forceSensors;
  std::string _accelerometerBody;
  Springs _springs;
};

} // namespace mc_rbdyn

/* Set of macros to assist with the writing of a RobotModule */

/*! ROBOT_MODULE_COMMON
 * Declare a destroy symbol and CLASS_NAME symbol
 * Constructor should be declared by the user
*/
#define ROBOT_MODULE_COMMON(NAME)\
  const char * CLASS_NAME() { return NAME; }\
  void destroy(mc_rbdyn::RobotModule * ptr) { delete ptr; }

/*! ROBOT_MODULE_DEFAULT_CONSTRUCTOR
 * Declare an external symbol for creation using a default constructor
 * Also declare destruction symbol
 * Exclusive of ROBOT_MODULE_CANONIC_CONSTRUCTOR
*/
#define ROBOT_MODULE_DEFAULT_CONSTRUCTOR(NAME, TYPE)\
extern "C"\
{\
  ROBOT_MODULE_COMMON(NAME)\
  mc_rbdyn::RobotModule * create() { return new TYPE(); }\
}

/*! ROBOT_MODULE_CANONIC_CONSTRUCTOR
 * Declare an external symbol for creation using the cannonical constructor (const string &, const string &)
 * Also declare destruction symbol
 * Exclusive of ROBOT_MODULE_DEFAULT_CONSTRUCTOR
*/
#define ROBOT_MODULE_CANONIC_CONSTRUCTOR(NAME, TYPE)\
extern "C"\
{\
  ROBOT_MODULE_COMMON(NAME)\
  mc_rbdyn::RobotModule * create(const std::string & path, const std::string & name) { return new TYPE(path, name); }\
}
