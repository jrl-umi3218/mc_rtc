#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct SurfaceImpl
{
public:
  std::string name;
  std::string bodyName;
  sva::PTransformd _X_b_s;
  std::string materialName;
  std::vector<sva::PTransformd> points;
};

Surface::Surface(const std::string & name,
                 const std::string & bodyName,
                 const sva::PTransformd & X_b_s,
                 const std::string & materialName)
: impl(new SurfaceImpl({name, bodyName, X_b_s, materialName, {}}))
{
}

Surface::~Surface() {}

const std::string & Surface::name() const
{
  return impl->name;
}

void Surface::name(const std::string & name)
{
  impl->name = name;
}

const std::string & Surface::bodyName() const
{
  return impl->bodyName;
}

const std::string & Surface::materialName() const
{
  return impl->materialName;
}

unsigned int Surface::bodyIndex(const mc_rbdyn::Robot & robot) const
{
  return robot.bodyIndexByName(impl->bodyName);
}

sva::PTransformd Surface::X_0_s(const mc_rbdyn::Robot & robot) const
{
  return X_0_s(robot, robot.mbc());
}

sva::PTransformd Surface::X_0_s(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc) const
{
  sva::PTransformd X_0_b = mbc.bodyPosW[bodyIndex(robot)];
  return impl->_X_b_s * X_0_b;
}

const sva::PTransformd & Surface::X_b_s() const
{
  return impl->_X_b_s;
}

void Surface::X_b_s(const sva::PTransformd & X_b_s)
{
  impl->_X_b_s = X_b_s;
  computePoints();
}

std::string Surface::toStr()
{
  std::stringstream ss;
  ss << impl->bodyName << ":" << impl->name;
  return ss.str();
}

const std::vector<sva::PTransformd> & Surface::points() const
{
  return impl->points;
}

std::vector<sva::PTransformd> & Surface::points()
{
  return impl->points;
}

bool Surface::operator==(const Surface & rhs)
{
  return this->name() == rhs.name();
}

bool Surface::operator!=(const Surface & rhs)
{
  return !(*this == rhs);
}

} // namespace mc_rbdyn
