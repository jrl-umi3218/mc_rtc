The `DataStore` is a facility used to efficiently store and retrieve arbitrary C++ objects, and is intended as a way to conveniently and efficiently share arbitrary objects within the framework. It is strongly typed (only objects of the same type as the stored type can be retrieved), and only relies on standard C++ mechanisms.

The `DataStore` may for instance be used to:

- **Share data between `States` in the FSM**.
  - For instance, it makes it easy to achieve a stabilized FSM simply by putting a `StabilizerStandingState` (see the [LIPM Stabilizer tutorial](lipm-stabilizer.html)) in parallel with an existing FSM. Targets to the `StabilizerStandingState` may then be provided through the `DataStore`. For instance to change the CoM target, you can simply set the `StabilizerStandingState::com` property.
- **Provide additional information to [observers](observers.html)** (such as the `anchorFrame` required by the `KinematicInertial` observer).
- **Share data between plugins and the controller**. As an example, consider the interaction between a walking controller and a footstep planner plugin. As an input, the footstep planner requires information about *(i) the current feet contacts*, *(ii) a desired goal*, and might also use additional information (such as obstacles, etc.). In return, it should provide the *(iii) next footsteps* that the walking controller should follow. Additionally in the case of an online footstep planner, it is needed to request updates to the footstep plan (when the goal or robot state changes, new obstacles are detected, etc.)

  - (i) Could be provided by the controller, or alternatively another vision plugin that estimates the state of the robot
  - (ii) Could be provided by the controller, or another plugin such as object detection
  - (iii) The footstep plan can then be used by the controller

These are of course just a few examples, this feature is intended to simplify sharing data across the framework. Use it sparingly, as adding objects to the datastore effectively makes them global, and thus modifiable from anywhere, including external plugins.

## Usage

A default `DataStore` instance is available in the `mc_control::MCController` class, and can be accessed using `mc_control::MCController::datastore()`. Any `c++` object can be created and retrieved from this `DataStore` instance from anywhere with access to the controller's instance.

- Objects can be directly created on the datastore through the use of `make` or `make_initializer` as follows:

  ```cpp
  // Create a vector of double of size 4 initialized with value 42
  datastore().make<std::vector<double>>("key", 4, 42.0);
  // Create a vector of double of size 2 initialized with value {4, 42} (using list initialization)
  datastore().make_initializer<std::vector<double>>("key", 4, 42.0);
  ```

- If an object with the same name already exists, an exception will be raised. To avoid this, you can either check whether the datastore already has such an element with
  ```cpp
  if(!datastore().has("key"))
  {
    datastore().make<Eigen::Vector3d>("key", 1, 2, 3);
  }
  ```

  Note that these functions also return a reference to the created object that you can use to access and modify it

  ```cpp
  auto & vec = datastore().make<Eigen::Vector3d>("EigenVector", Eigen::Vector3d::Zero());
  vec.x() = 42; // vec is now: 42, 0, 0
  ```

- A reference to the stored value can be retrieved at any time using `get<Type>`:

  ```cpp
  auto & vec = datastore().get<Eigen::Vector3d>("EigenVector");
  // vec: 42, 0, 0
  ```

  An exception will be thrown if the key is not present in the datastore, or the requested `Type` differs from the one used upon creation. Similarly to `mc_rtc::Configuration` objects, you may also use one of the convenience overloads of `get` to specify a default value to use in case the key does not exist:

  ```cpp
  Eigen::Vector3d vec{1,2,3};
  datastore().get<Eigen::Vector3d>("NotAKey", vec); // vec remains unchanged: 1, 2, 3
  datastore().get<Eigen::Vector3d>("EigenVector", vec); // vec is now 42, 0, 0

  bool hasFeature = datastore().get<bool>("HasFeature", false); // hasFeature will be assigned the value of "HasFeature" if that key exists, false otherwise
  ```


## Advanced usage

- **Inheritance** is also supported, but you need to explicitly provide the type of your object's parent classes to be able to retrieve them later. Here is a very simple example showing how to store and use objects using inheritance and polymorphic member functions.

  ```cpp
  struct A
  {
    virtual std::string hello() const
    {
      return "A";
    }
    std::string type() const
    {
      return "A";
    }
  };

  struct B : public A
  {
    std::string hello() const override
    {
      return "B";
    }
    std::string type() const
    {
      return "B";
    }
  };
  datastore().make<B, A>("ObjectB");
  auto & parent = store.get<A>("ObjectB");
  auto & derived = store.get<B>("ObjectB");
  parent.type();   // "A"
  parent.hello();  // "B" because hello is virtual
  derived.type();  // "B"
  derived.hello(); // "B"
  ```

- **Lambda functions** may also be stored but you must use the special `make_call`

  ```cpp
  // Create a lambda function and store it as an std::function
  datastore().make_call("lambda", [](double t) {});
  // Retrieve the lambda
  auto & lambdaFun = datastore().get<std::function<void(double)>("lambda");
  // Call function
  lambdaFun(42);
  // Call directly through the datastore (the function return type and arguments type must be repeated)
  datastore().call<void, double>("lambda", 42);
  ```
