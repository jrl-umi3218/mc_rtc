#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<std::string, std::shared_ptr<sch_T> > > & schByName)
{
  for(auto & p : schByName)
  {
    unsigned int index = static_cast<unsigned int>(mb.bodyIndexByName(p.second.first));
    sch::mc_rbdyn::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
  }
}

}
