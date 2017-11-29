#include <mc_control/ControllerClient.h>

#include <unistd.h>
#include <iostream>

struct DummyControllerClient : public mc_control::ControllerClient
{
  DummyControllerClient();

  void handle_gui_state(const char * data, size_t size) override;

  void join();
};

DummyControllerClient::DummyControllerClient()
: mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc")
{
}

void DummyControllerClient::handle_gui_state(const char * data, size_t size)
{
  std::cout << "Received message (size: " << size << ")\n";
  std::cout << data << std::endl;
}

void DummyControllerClient::join()
{
  while(run_)
  {
    sleep(1);
  }
}

int main()
{
  DummyControllerClient client;
  client.join();
  return 0;
}
