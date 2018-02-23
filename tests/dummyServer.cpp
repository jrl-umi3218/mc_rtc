#include <mc_rtc/GUIState.h>
#include <mc_control/ControllerServer.h>

struct DummyProvider
{
  double value = 42.0;
  Eigen::Vector3d point = Eigen::Vector3d(0., 1., 2.);
};

struct TestServer
{
  TestServer();

  void publish();

  mc_control::ControllerServer server {1.0, 1.0, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_ret.ipc"}};
  DummyProvider provider;
  mc_rtc::gui::StateBuilder builder;
};

TestServer::TestServer()
{
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::Label("value", [this]{ return provider.value; }));
  builder.addElement({"dummy", "provider"}, mc_rtc::gui::ArrayLabel("point", [this]{ return provider.point; }));
}

void TestServer::publish()
{
  server.publish(builder);
}

int main()
{
  TestServer server;
  while(1)
  {
    server.publish();
    sleep(1);
  }
  return 0;
}
