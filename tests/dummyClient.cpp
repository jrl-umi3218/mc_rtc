#include <mc_control/ControllerClient.h>

#include <mc_rtc/logging.h>

#include <unistd.h>
#include <iostream>

namespace
{

std::string cat2str(const std::vector<std::string> & cat)
{
  std::string ret;
  for(size_t i = 0; i < cat.size(); ++i)
  {
    ret += cat[i];
    if(i != cat.size() - 1) { ret += "/"; }
  }
  return ret;
}

}

struct DummyControllerClient : public mc_control::ControllerClient
{
  DummyControllerClient();

  void join();

  void category(const std::vector<std::string> & category, const std::string & name) override;
};

DummyControllerClient::DummyControllerClient()
: mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc")
{
}

void DummyControllerClient::join()
{
  while(run_)
  {
    sleep(1);
  }
}

void DummyControllerClient::category(const std::vector<std::string> & category,
                                     const std::string & name)
{
  LOG_INFO("Create new category " << name << " in " << cat2str(category))
}

int main()
{
  DummyControllerClient client;
  client.join();
  return 0;
}
