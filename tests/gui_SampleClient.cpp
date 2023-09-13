/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/ControllerClient.h>
#include <mc_rtc/logging.h>

#include <chrono>
#include <iostream>
#include <thread>

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

} // namespace

struct SampleControllerClient : public mc_control::ControllerClient
{
  SampleControllerClient();

  void join();

  void category(const std::vector<std::string> & category, const std::string & name) override;
};

SampleControllerClient::SampleControllerClient()
: mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc")
{
  start();
}

void SampleControllerClient::join()
{
  while(run_) { std::this_thread::sleep_for(std::chrono::seconds(1)); }
  stop();
}

void SampleControllerClient::category(const std::vector<std::string> & category, const std::string & name)
{
  mc_rtc::log::info("Create new category {} in {}", name, cat2str(category));
}

int main()
{
  SampleControllerClient client;
  client.join();
  return 0;
}
