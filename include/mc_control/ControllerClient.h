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
    void handle_gui_state(const char * data);

    void handle_category(const std::vector<std::string> & parent, const std::string & category, const mc_rtc::Configuration & data);

    void handle_widget(const std::vector<std::string> & category, const std::string & name, const mc_rtc::Configuration & data);

    /** Called when a message starts being processed, can be used to lock the GUI */
    virtual void started() {}

    /** Called when a message has been processed */
    virtual void stopped() {}

    /** Should be implemented to create a new category container */
    virtual void category(const std::vector<std::string> & parent, const std::string & category) = 0;

    /** Should be implemented to create a label for data that can be displayed as string */
    virtual void label(const std::vector<std::string> & category,
                       const std::string & label, const std::string &)
    {
      default_impl("Label", category, label);
    }

    /** Should be implemented to create a label for a numeric array
     *
     * \p category Category under which the label appears
     * \p label Name of the data
     * \p labels Per-dimension label (can be empty)
     * \p data Data to display
     */
    virtual void array_label(const std::vector<std::string> & category,
                             const std::string & label,
                             const std::vector<std::string> &,
                             const Eigen::VectorXd &)
    {
      default_impl("ArrayLabel", category, label);
    }

    /** Should be implemented to create a button */
    virtual void button(const std::vector<std::string> & category,
                        const std::string & label)
    {
      default_impl("Button", category, label);
    }

    /* Network elements */
    bool run_ = true;
    int sub_socket_;
    std::thread sub_th_;
    int push_socket_;
    double timeout_;

    /* Hold data from the server */
    mc_rtc::Configuration data_;
  private:
    /** Default implementations for widgets' creations display a warning message to the user */
    void default_impl(const std::string & type,
                      const std::vector<std::string> & category,
                      const std::string & label);
  };


}
