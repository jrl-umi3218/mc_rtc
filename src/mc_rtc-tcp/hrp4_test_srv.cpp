// includes
#include <iostream>

// boost
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

// tcp-com
#include <tcp_com/client_server.h>

// tmp
#include <tcp_control/controlStructures.h>


void srvThread(boost::asio::io_service& io_service)
{
  // add a work to avoid io_service to finish when it has nothing to do
  boost::asio::io_service::work work(io_service);
  io_service.run();
}


void initTab(double* tab, int size, int start)
{
  for(int i = 0; i < size; ++i)
  {
    tab[i] = start + i;
  }
}


void printTab(const double* tab, int size)
{
  std::copy(tab, tab + size, std::ostream_iterator<double>(std::cout, ", "));
  std::cout << std::endl;
}


int main(int argc, char **argv)
{
  if(argc < 3)
  {
    std::cerr << "usage: " << argv[0] << " port_sensor port_control" << std::endl;
    return 1;
  }

  short port_sensor = boost::lexical_cast<short>(argv[1]);
  short port_control = boost::lexical_cast<short>(argv[2]);

  // sensor thread
  HRP4OpenHRPSensors sensor_data_init;
  initTab(sensor_data_init.accelerometer, 3, 0);
  initTab(sensor_data_init.gyrometer, 3, 3);
  initTab(sensor_data_init.forceLH, 6, 3 + 6*1);
  initTab(sensor_data_init.forceRH, 6, 3 + 6*2);
  initTab(sensor_data_init.forceLF, 6, 3 + 6*3);
  initTab(sensor_data_init.forceRF, 6, 3 + 6*4);
  initTab(sensor_data_init.position, sensors_traits<HRP4OpenHRPSensors>::dof, 100);
  boost::asio::io_service io_sensor_srv;
  WriteAndAck<HRP4OpenHRPSensors> sensor_proto(io_sensor_srv, sensor_data_init);
  Server<WriteAndAck<HRP4OpenHRPSensors> > sensor_server(io_sensor_srv,
                                                         port_sensor, sensor_proto);
  boost::thread sensor_thread(srvThread, boost::ref(io_sensor_srv));

  // control thread
  HRP4OpenHRPControl control_data_init;
  boost::asio::io_service io_control_srv;
  ReadAndAck<HRP4OpenHRPControl> control_proto(io_control_srv, control_data_init);
  Server<ReadAndAck<HRP4OpenHRPControl> > control_server(io_control_srv, port_control,
                                                         control_proto);
  boost::thread control_thread(srvThread, boost::ref(io_control_srv));

  while(true)
  {
    std::cout << "sensors:" << std::endl;
    printTab(sensor_data_init.accelerometer, 3);
    printTab(sensor_data_init.gyrometer, 3);
    printTab(sensor_data_init.forceLH, 6);
    printTab(sensor_data_init.forceRH, 6);
    printTab(sensor_data_init.forceLF, 6);
    printTab(sensor_data_init.forceRF, 6);
    printTab(sensor_data_init.position, sensors_traits<HRP4OpenHRPSensors>::dof);
    std::cout << std::endl;

    std::cout << "control: " << std::endl;
    HRP4OpenHRPControl control_data = control_proto.data();
    std::cout << "ZMP: " << control_data.hasZmp << std::endl;
    std::cout << "FF: " << control_data.hasBaseFF << std::endl;
    printTab(control_data.control, control_traits<HRP4OpenHRPControl>::dof);
    std::cout << std::endl;

    sensor_proto.send();
    boost::this_thread::sleep(boost::posix_time::seconds(1.));
  }

  io_sensor_srv.stop();
  io_control_srv.stop();
  sensor_thread.join();
  control_thread.join();
  return 0;
}
