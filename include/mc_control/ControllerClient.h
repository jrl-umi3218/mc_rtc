#pragma once

#include <mc_control/api.h>

#include <nanomsg/nn.h>
#include <nanomsg/pubsub.h>
#include <nanomsg/reqrep.h>

#include <string>
#include <thread>
#include <vector>

namespace mc_control
{

  /** Receives data and interact with a ControllerServer
   *
   * - Uses a SUB socket to receive the data stream
   *
   * - Uses a REQ socket to send requests
   */
  struct MC_CONTROL_DLLAPI ControllerClient
  {

    /** Constructor
     *
     * \param sub_conn_uri URI the SUB socket should connect to
     *
     * \param req_conn_uri URI the REQ socket should connect to
     *
     * Check nanomsg documentation for supported protocols
     */
    ControllerClient(const std::string & sub_conn_uri,
                     const std::string & req_conn_uri);

    ControllerClient(const ControllerClient &) = delete;
    ControllerClient & operator=(const ControllerClient &) = delete;

    ~ControllerClient();
  protected:
    /** Overriden in derived class to handle the GUI state provided by the
     * SUB socket */
    virtual void handle_gui_state(const char * data, size_t size) = 0;

    bool run_ = true;
    int sub_socket_;
    std::thread sub_th_;
    int req_socket_;
  };


}
