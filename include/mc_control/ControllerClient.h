/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/ControllerServer.h>
#include <mc_control/client_api.h>

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/plot/types.h>
#include <mc_rtc/gui/types.h>

#include <string>
#include <thread>
#include <vector>

namespace mc_control
{

/** Used to uniquely identify an element */
struct MC_CONTROL_CLIENT_DLLAPI ElementId
{
  ElementId() = default;
  ElementId(const ElementId &) = default;
  ElementId(ElementId &&) = default;
  ElementId & operator=(const ElementId &) = default;
  ElementId & operator=(ElementId &&) = default;

  ElementId(const std::vector<std::string> & category, const std::string & name, int sid)
  : category(category), name(name), sid(sid)
  {
  }

  ElementId(const std::vector<std::string> & category, const std::string & name) : ElementId(category, name, -1) {}

  /** Category the element belongs to */
  std::vector<std::string> category = {};
  /** Name of the element */
  std::string name = {};
  /** Stack id, the elements that share the same stack id should be displayed on the same line */
  int sid = -1;
};

/** Receives data and interact with a ControllerServer
 *
 * - Uses a SUB socket to receive the data stream
 *
 * - Uses a REQ socket to send requests
 */
struct MC_CONTROL_CLIENT_DLLAPI ControllerClient
{

  /** Constructor
   *
   * Default (disconnected) client
   */
  ControllerClient();

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
  ControllerClient(const std::string & sub_conn_uri, const std::string & push_conn_uri, double timeout = 0);

  /** Constructor
   *
   * \param server In-memory ControllerServer instance
   *
   * \param gui GUI updated by the server
   *
   */
  ControllerClient(ControllerServer & server, mc_rtc::gui::StateBuilder & gui);

  ControllerClient(const ControllerClient &) = delete;
  ControllerClient & operator=(const ControllerClient &) = delete;

  ~ControllerClient();

  /** Connect to the provided uris */
  void connect(const std::string & sub_conn_uri, const std::string & push_conn_uri);

  /** Connect to an in-memory server */
  void connect(ControllerServer & server, mc_rtc::gui::StateBuilder & gui);

  /** Send a request to the given element in the given category using data */
  void send_request(const ElementId & id, const mc_rtc::Configuration & data);

  /** Helper for send_request in simple cases */
  template<typename T>
  void send_request(const ElementId & id, const T & data)
  {
    mc_rtc::Configuration c;
    c.add("data", data);
    send_request(id, c("data"));
  }

  /** Helper for the void case */
  void send_request(const ElementId & id);

  /** Get the raw request data
   *
   * out.c_str() can be used to send requests to the raw data interface of ControllerServer
   */
  void raw_request(const ElementId & id, const mc_rtc::Configuration & data, std::string & out);

  /** Helper for raw request in simple cases */
  template<typename T>
  void raw_request(const ElementId & id, const T & data, std::string & out)
  {
    mc_rtc::Configuration c;
    c.add("data", data);
    raw_request(id, c("data"), out);
  }

  /** Helper for the void case */
  void raw_request(const ElementId & id, std::string & out);

  /** Set the timeout of the SUB socket */
  void timeout(double t);

  /** Get the current timeout */
  double timeout();

  /** Check if there is an available message from the server and process it
   *
   * This is the synchronous pendant to \ref start()
   *
   * \param buffer Buffer to receive data from the server, if it is too small,
   * the buffer is resized and the message is discarded
   *
   * \param t_last_received Time when the last message was received, it is
   * updated if the client receives a message. If a message has not been
   * received since \ref timeout() then act as if we received an empty message
   * (server offline).
   *
   */
  void run(std::vector<char> & buffer, std::chrono::system_clock::time_point & t_last_received);

  /** Run with raw data received from any possible way
   *
   * \param buffer Data to be processed
   *
   * \param bufferSize Size of data
   */
  void run(const char * buffer, size_t bufferSize);

protected:
  /** Should be called when the client is ready to receive data */
  void start();

  /** Connect/Reconnect the client and start receiving data asap
   *
   * \param sub_conn_uri URI the SUB socket should connect to
   *
   * \param push_conn_uri URI the PUSH socket should connect to
   *
   * Check nanomsg documentation for supported protocols
   */
  void reconnect(const std::string & sub_conn_uri, const std::string & push_conn_uri);

  /** Kill the connection and data flow */
  void stop();

  void handle_gui_state(mc_rtc::Configuration state);

  void handle_category(const std::vector<std::string> & parent,
                       const std::string & category,
                       const mc_rtc::Configuration & data);

  void handle_widget(const ElementId & id, const mc_rtc::Configuration & data);

  /** Called when a message starts being processed, can be used to lock the GUI */
  virtual void started() {}

  /** Called when a message has been processed */
  virtual void stopped() {}

  /** Should be implemented to create a new category container */
  virtual void category(const std::vector<std::string> & parent, const std::string & category) = 0;

  /** Should be implemented to create a label for data that can be displayed as string */
  inline virtual void label(const ElementId & id, const std::string &)
  {
    default_impl("Label", id);
  }

  /** Should be implemented to create a label for a numeric array
   *
   * \p category Category under which the label appears
   * \p label Name of the data
   * \p labels Per-dimension label (can be empty)
   * \p data Data to display
   */
  inline virtual void array_label(const ElementId & id, const std::vector<std::string> &, const Eigen::VectorXd &)
  {
    default_impl("ArrayLabel", id);
  }

  /** Should be implemented to create a button */
  virtual void button(const ElementId & id)
  {
    default_impl("Button", id);
  }

  /** Should be implemented to create a checkbox */
  virtual void checkbox(const ElementId & id, bool /*state */)
  {
    default_impl("Checkbox", id);
  }

  /** Should be implemented to create a widget able to input strings */
  virtual void string_input(const ElementId & id, const std::string & /*data*/)
  {
    default_impl("StringInput", id);
  }

  /** Should be implemented to create a widget able to input integers */
  virtual void integer_input(const ElementId & id, int /*data*/)
  {
    default_impl("IntegerInput", id);
  }

  /** Should be implemented to create a widget able to input numbers */
  virtual void number_input(const ElementId & id, double /*data*/)
  {
    default_impl("NumberInput", id);
  }

  virtual void number_slider(const ElementId & id, double /*data*/, double /*min*/, double /*max*/)
  {
    default_impl("NumberSlider", id);
  }

  /** Should be implemented to create a widget able to input array of numbers */
  virtual void array_input(const ElementId & id,
                           const std::vector<std::string> & /*labels*/,
                           const Eigen::VectorXd & /*data*/)
  {
    default_impl("ArrayInput", id);
  }

  /** Should be implemented to create a widget able to select one string among many */
  virtual void combo_input(const ElementId & id,
                           const std::vector<std::string> & /*values*/,
                           const std::string & /*data*/)
  {
    default_impl("ComboInput", id);
  }

  /** Should be implemented to create a widget able to select one string
   * among entries available in the data part of the GUI message */
  virtual void data_combo_input(const ElementId & id,
                                const std::vector<std::string> & /*data_ref*/,
                                const std::string & /*data*/)
  {
    default_impl("DataComboInput", id);
  }

  /** Should display an interactive point in 3D environment
   *
   * \p requestId should be in requests instead of \p id
   *
   * bool \p ro indicates whether this point is interactive or not
   */
  virtual void point3d(const ElementId & id,
                       const ElementId & /*requestId*/,
                       bool /*ro */,
                       const Eigen::Vector3d & /*pos*/,
                       const mc_rtc::gui::PointConfig & /* config */)
  {
    default_impl("Point3D", id);
  }

  /** Should display a trajectory of 3d points in 3D environment
   *
   * \p points Vector of 3D points
   */
  virtual void trajectory(const ElementId & id,
                          const std::vector<Eigen::Vector3d> & /* points */,
                          const mc_rtc::gui::LineConfig & /* config */)
  {
    default_impl("Point3DTrajectory", id);
  }

  /** Should display a trajectory of transforms in 3D environment
   *
   * \p points Vector of poses
   */
  virtual void trajectory(const ElementId & id,
                          const std::vector<sva::PTransformd> & /* points */,
                          const mc_rtc::gui::LineConfig & /* config */)
  {
    default_impl("PoseTrajectory", id);
  }

  /** Should display a trajectory of points, points are added one by one by the server
   *
   * \p point New point added to the trajectory
   */
  virtual void trajectory(const ElementId & id,
                          const Eigen::Vector3d & /* point */,
                          const mc_rtc::gui::LineConfig & /* config */)
  {
    default_impl("Point3DRealTimeTrajectory", id);
  }

  /** Should display a trajectory of points, points are added one by one by the server
   *
   * \p point New point added to the trajectory
   */
  virtual void trajectory(const ElementId & id,
                          const sva::PTransformd & /* point */,
                          const mc_rtc::gui::LineConfig & /* config */)
  {
    default_impl("PoseRealTimeTrajectory", id);
  }

  /** Should display a list of polygons of 3d points in 3D environment
   *
   * For backward compatibility, this version has the default implementation
   * and the version that accepts a LineConfig configuration instead of a Color
   * is implemented in terms of this one. The opposite makes more sense in
   * actual implementation of ControllerClient.
   *
   * \p points Each entry in the vector is a polygon described by its 3D points
   *
   * \p color Color of the polygon
   */
  virtual void polygon(const ElementId & id,
                       const std::vector<std::vector<Eigen::Vector3d>> & /* points */,
                       const mc_rtc::gui::Color & /* color */)
  {
    default_impl("PolygonArray", id);
  }

  /** Should display a list of polygons of 3D points in 3D environment
   *
   * For backward compatibility, this version is implemented to forward the
   * call to the color variant, the concrete implementation makes more sense
   * the other way around.
   *
   * \p points Each entry in the vector is a polygon described by its 3D points
   *
   * \p config Describe how to display the polygon
   */
  virtual void polygon(const ElementId & id,
                       const std::vector<std::vector<Eigen::Vector3d>> & points,
                       const mc_rtc::gui::LineConfig & config)
  {
    polygon(id, points, config.color);
  }

  /** Should display a force in 3D environment
   */
  virtual void force(const ElementId & id,
                     const ElementId & /*requestId*/,
                     const sva::ForceVecd & /* force */,
                     const sva::PTransformd & /* application point for the force */,
                     const mc_rtc::gui::ForceConfig & /* forceConfig */,
                     bool /* ro */)
  {
    default_impl("Force", id);
  }

  /** Should display an arrow in 3D environment
   */
  virtual void arrow(const ElementId & id,
                     const ElementId & /* requestId */,
                     const Eigen::Vector3d & /* start */,
                     const Eigen::Vector3d & /* end */,
                     const mc_rtc::gui::ArrowConfig & /* config */,
                     bool /* ro */)
  {
    default_impl("Arrow", id);
  }

  /** Should display a rotation in 3D environment
   *
   * \p requestId should be in requests instead of \p id
   *
   * bool \p ro indicates whether this point is interactive or not
   */
  virtual void rotation(const ElementId & id,
                        const ElementId & /*requestId*/,
                        bool /*ro */,
                        const sva::PTransformd & /*pos*/)
  {
    default_impl("Rotation", id);
  }

  /** Should display a PTransform in 3D environment
   *
   * \p requestId should be in requests instead of \p id
   *
   * bool \p ro indicates whether this point is interactive or not
   */
  virtual void transform(const ElementId & id,
                         const ElementId & /*requestId*/,
                         bool /*ro */,
                         const sva::PTransformd & /*pos*/)
  {
    default_impl("Transform", id);
  }

  /** Should display X,Y,theta in a 3D environment
   *
   * \p requestId should be in requests instead of \p id
   *
   * bool \p ro indicates whether this point is interactive or not
   */
  virtual void xytheta(const ElementId & id,
                       const ElementId & /*requestId*/,
                       bool /*ro */,
                       const Eigen::Vector3d & /*xytheta*/,
                       double /* altitude */)
  {
    default_impl("XYTheta", id);
  }

  /** Called when a table starts */
  virtual void table_start(const ElementId & id, const std::vector<std::string> & /*header*/)
  {
    default_impl("Table", id);
  }

  /** Called for each element appearing in the table identified by \p id */
  virtual void table_row(const ElementId & /*id*/, const std::vector<std::string> & /*data*/) {}

  /** Called to close a table identified by \p id */
  virtual void table_end(const ElementId & /*id*/) {}

  /** Should display a robot model, the RobotModule can be created using \p parameters, its configuration is in \q and
   * its world position is in \p posW */
  virtual void robot(const ElementId & id,
                     const std::vector<std::string> & /*parameters*/,
                     const std::vector<std::vector<double>> & /*q*/,
                     const sva::PTransformd & /*posW*/)
  {
    default_impl("Robot", id);
  }

  /** Should display the visual element \p visual at the position \p pose */
  virtual void visual(const ElementId & id,
                      [[maybe_unused]] const rbd::parsers::Visual & visual,
                      [[maybe_unused]] const sva::PTransformd & pose)
  {
    default_impl("Visual", id);
  }

  /** Should display a form to send schema-based request to the server
   *
   * \p schema is the schema directory relative to mc_rtc JSON schema installation
   */
  virtual void schema(const ElementId & id, const std::string & /*schema*/)
  {
    default_impl("Schema", id);
  }

  /** Create a form */
  virtual void form(const ElementId & id)
  {
    default_impl("Form", id);
  }

  /** A checkbox within a form */
  virtual void form_checkbox(const ElementId & /*formId*/,
                             const std::string & /*name*/,
                             bool /*required*/,
                             bool /*default*/)
  {
  }

  /** An integer input within a form */
  virtual void form_integer_input(const ElementId & /*formId*/,
                                  const std::string & /*name*/,
                                  bool /*required*/,
                                  int /*default*/)
  {
  }

  /** A number input within a form */
  virtual void form_number_input(const ElementId & /*formId*/,
                                 const std::string & /*name*/,
                                 bool /*required*/,
                                 double /*default*/)
  {
  }

  /** A string input within a form */
  virtual void form_string_input(const ElementId & /*formId*/,
                                 const std::string & /*name*/,
                                 bool /*required*/,
                                 const std::string & /*default*/)
  {
  }

  /** An array input within a form */
  virtual void form_array_input(const ElementId & /*formId*/,
                                const std::string & /*name*/,
                                bool /*required*/,
                                const Eigen::VectorXd & /*default*/,
                                bool /*fixed_size*/)
  {
  }

  /** A combo input within a form
   *
   * \p formId Identifier of the form
   *
   * \p name Name of the entry
   *
   * \p required If true, it must hold a value when the form is sent
   *
   * \p values Possible values
   *
   * \p send_index If true, the implementation should send back the index
   * rather than the value
   */
  virtual void form_combo_input(const ElementId & /*formId*/,
                                const std::string & /*name*/,
                                bool /*required*/,
                                const std::vector<std::string> & /*values*/,
                                bool /*send_index*/)
  {
  }

  /** A data combo input within a form
   *
   * \p formId Identifier of the form
   *
   * \p name Name of the entry
   *
   * \p required If true, it must hold a value when the form is sent
   *
   * \p ref Reference to GUI data store, can reference other fields of the
   * form, e.g. {"$R0", "surfaces"} depends on the value of the R0 entry in
   * the form
   *
   * \p send_index If true, the implementation should send back the index in
   * the list rather than the value
   */
  virtual void form_data_combo_input(const ElementId & /*formId*/,
                                     const std::string & /*name*/,
                                     bool /*required*/,
                                     const std::vector<std::string> & /*ref*/,
                                     bool /*send_index*/)
  {
  }

  /** Called when new plot data arrives
   *
   * This should open a new plotting window with the provided title.
   *
   * \p id The plot id, this serves to disambiguate plots with the same title that are started right after closing the
   * previous one
   *
   * \p title Title of the plot
   *
   */
  virtual void start_plot(uint64_t /*id*/, const std::string & /*title*/) {}

  /** Setup the X-axis
   *
   * \p id Plot id
   *
   * \p legend Legend on the X-axis
   *
   * \p range Range on the X-axis plot
   *
   */
  virtual void plot_setup_xaxis(uint64_t /*id*/,
                                const std::string & /*legend*/,
                                const mc_rtc::gui::plot::Range & /*range*/)
  {
  }

  /** Setup the Y-axis on the left side
   *
   * \p id Plot id
   *
   * \p legend Legend on the Y-axis
   *
   * \p range Range on the Y-axis plot
   *
   */
  virtual void plot_setup_yaxis_left(uint64_t /*id*/,
                                     const std::string & /*legend*/,
                                     const mc_rtc::gui::plot::Range & /*range*/)
  {
  }

  /** Setup the Y-axis on the right side
   *
   * \p id Plot id
   *
   * \p legend Legend on the Y-axis
   *
   * \p range Range on the Y-axis plot
   *
   */
  virtual void plot_setup_yaxis_right(uint64_t /*id*/,
                                      const std::string & /*legend*/,
                                      const mc_rtc::gui::plot::Range & /*range*/)
  {
  }

  /** Add data to be displayed on a plot
   *
   * \p id Plot id
   *
   * \p did Id for this data, this is strictly increasing from 0 to the
   * number of plots - 1 and you can assume that only one data point is added
   * in one iteration
   *
   * \p legend Legend for this data
   *
   * \p x X value
   *
   * \p y Y value
   *
   * \p style How to link data points together
   *
   * \p side Add on the Y left or right side
   *
   */
  virtual void plot_point(uint64_t /* id */,
                          uint64_t /* did */,
                          const std::string & /*legend*/,
                          double /*x*/,
                          double /*y*/,
                          mc_rtc::gui::Color /*color*/,
                          mc_rtc::gui::plot::Style /*style*/,
                          mc_rtc::gui::plot::Side /*side*/)
  {
  }

  /** Plot a polygon
   *
   * \p id Id for the plot
   *
   * \p did Id for this polygon
   *
   * \p legen Legend to describe this polygon
   *
   * \p polygon Description of the polygon
   *
   * \p side Side where the polygon should be displayed
   *
   */
  virtual void plot_polygon(uint64_t /*id*/,
                            uint64_t /* did */,
                            const std::string & /*legend*/,
                            const mc_rtc::gui::plot::PolygonDescription & /*polygon*/,
                            mc_rtc::gui::plot::Side /*side*/)
  {
  }

  /** Plot several polygons under the same legend
   *
   * \p id Id for the plot
   *
   * \p did Id for these polygons
   *
   * \p legen Legend to describe these polygons
   *
   * \p polygons Description of the polygons
   *
   * \p side Side where the polygons should be displayed
   *
   */
  virtual void plot_polygons(uint64_t /*id*/,
                             uint64_t /* did */,
                             const std::string & /*legend*/,
                             const std::vector<mc_rtc::gui::plot::PolygonDescription> & /*polygons*/,
                             mc_rtc::gui::plot::Side /*side*/)
  {
  }

  /** Called when no more data for a plot will come this iteration
   *
   * This typically should refresh the log display
   *
   */
  virtual void end_plot(uint64_t /*id*/) {}

  /* Network elements */
  bool run_ = true;
  int sub_socket_ = -1;
  std::thread sub_th_;
  int push_socket_ = -1;
  double timeout_;

  /* Hold data from the server */
  mc_rtc::Configuration data_;

  /* Pointer to the server if connected in-memory */
  ControllerServer * server_ = nullptr;
  /* Pointer to the GUI if connected in-memory */
  mc_rtc::gui::StateBuilder * gui_ = nullptr;

private:
  /** Default implementations for widgets' creations display a warning message to the user */
  virtual void default_impl(const std::string & type, const ElementId & id);

  /** Handle details of Point3D elements */
  void handle_point3d(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle Trajectory and dispatch to 3D or Pose case */
  void handle_trajectory(const ElementId & id, const mc_rtc::Configuration & data);

  /** Hand details of DisplayPolygon elemets */
  void handle_polygon(const ElementId & id, const mc_rtc::Configuration & data);

  /** Hand details of DisplayForce elemets */
  void handle_force(const ElementId & id, const mc_rtc::Configuration & data);

  /** Hand details of DisplayArrow elemets */
  void handle_arrow(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle details of Rotation elements */
  void handle_rotation(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle details of Transform elements */
  void handle_transform(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle details of XYTheta elements */
  void handle_xytheta(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle details of Form elements */
  void handle_form(const ElementId & id, const mc_rtc::Configuration & data);

  /** Handle details of a plot */
  void handle_plot(const mc_rtc::Configuration & plot);

  /** Handle standard plot */
  void handle_standard_plot(const mc_rtc::Configuration & plot);

  /** Handle XY plot */
  void handle_xy_plot(const mc_rtc::Configuration & plot);

  /** Handle Table details */
  void handle_table(const ElementId & id,
                    const std::vector<std::string> & header,
                    const std::vector<std::string> & format,
                    const std::vector<mc_rtc::Configuration> & data);
};

} // namespace mc_control
