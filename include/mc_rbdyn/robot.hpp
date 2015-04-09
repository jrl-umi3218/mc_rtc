#include <mc_rbdyn/SCHAddon.h>

namespace mc_rbdyn
{

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<unsigned int, sch_T> > & schById)
{
  for(std::pair<const std::string, std::pair<unsigned int, sch_T> > & p : schById)
  {
    unsigned int index = mb.bodyIndexById(p.second.first);
    sch::transform(p.second.second, mbc.bodyPosW[index]);
  }
}

}
