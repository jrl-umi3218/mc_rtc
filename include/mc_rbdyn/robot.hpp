#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch_T> > > & schById)
{
  for(std::pair<const std::string, std::pair<unsigned int, std::shared_ptr<sch_T> > > & p : schById)
  {
    int index = mb.bodyIndexById(static_cast<int>(p.second.first));
    sch::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
  }
}

}
