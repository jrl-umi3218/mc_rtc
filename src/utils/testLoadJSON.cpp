#include <iostream>
#include <memory>
#include <mc_rbdyn/stance.h>

int main(int argc, char * argv[])
{
  if(argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " [.json file]" << std::endl;
    return 1;
  }

  std::vector<mc_rbdyn::Stance> stances;
  std::vector< std::shared_ptr<mc_rbdyn::StanceAction> > actions;

  loadStances(argv[1], stances, actions);
  std::cout << "Loaded " << stances.size() << " stances and " << actions.size() << " actions" << std::endl;
  for(const auto & s : stances)
  {
    std::cout << s.q.size() << std::endl;
    std::cout << "geomContacts" << std::endl;
    for(const auto & c : s.geomContacts)
    {
      std::cout << c.toStr() << std::endl;
    }
    std::cout << "stabContacts" << std::endl;
    for(const auto & c : s.stabContacts)
    {
      std::cout << c.toStr() << std::endl;
    }
  }
  for(const auto & a : actions)
  {
    std::cout << a->toStr() << std::endl;
  }
  return 0;
}
