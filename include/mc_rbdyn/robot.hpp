#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<int, std::shared_ptr<sch_T> > > & schById)
{
  for(auto & p : schById)
  {
    unsigned int index = static_cast<unsigned int>(mb.bodyIndexById(p.second.first));
    sch::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
  }
}

}
