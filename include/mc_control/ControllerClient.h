#pragma once

#include <mc_control/api.h>

#include <mc_rtc/Configuration.h>

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
     * \param push_conn_uri URI the PUSH socket should connect to
     *
     * \param timeout After timeout has elapsed without receiving messages from
     * the SUB socket, pass an empty message to handle_gui_state. It should be
     * expressed in secondd. If timeout <= 0, this is ignored.
     *
     * Check nanomsg documentation for supported protocols
     */
    ControllerClient(const std::string & sub_conn_uri,
                     const std::string & push_conn_uri,
                     double timeout = 0);

    ControllerClient(const ControllerClient &) = delete;
    ControllerClient & operator=(const ControllerClient &) = delete;

    ~ControllerClient();

    /** Send a request to the given element in the given category using data */
    void send_request(const std::vector<std::string> & category, const std::string & element, const mc_rtc::Configuration & data);

    /** Set the timeout of the SUB socket */
    double timeout(double t);

  protected:
    /** Overriden in derived class to handle the GUI state provided by the
     * SUB socket */
    virtual void handle_gui_state(const char * data, size_t size) = 0;

    bool run_ = true;
    int sub_socket_;
    std::thread sub_th_;
    int push_socket_;
    double timeout_;
  };


}
