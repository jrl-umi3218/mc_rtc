---
layout: tutorials
---

The `DataStore` is a facility used to efficiently store and retrieve arbitrary C++ objects, and is intended as a way to conveniently and efficiently share arbitrary objects within the framework. It is strongly typed (only objects of the same type as the stored type can be retrieved), and only relies on standard C++ mechanisms.

The `DataStore` may for instance be used to:

- **Share data between `States` in the FSM**.
  - For instance, it makes it easy to achieve a stabilized FSM simply by putting a `StabilizerStandingState` (see the [LIPM Stabilizer tutorial](lipm-stabilizer.html)) in parallel with an existing FSM. Targets to the `StabilizerStandingState` may then be provided through the `DataStore`. For instance to change the CoM target, you can simply set the `StabilizerStandingState::com` property.
- **Provide additional information to [observers](observers.html)** (such as the `anchorFrame` required by the `KinematicInertial` observer).
- **Share data between plugins and the controller**. As an example, consider the interaction between a walking controller and a footstep planner plugin. As an input, the footstep planner requires information about *(i) the current feet contacts*, *(ii) a desired goal*, and might also use additional information (such as obstacles, etc.). In return, it should provide the *(iii) next footsteps* that the walking controller should follow. Additionally in the case of an online footstep planner, it is needed to request updates to the footstep plan (when the goal or robot state changes, new obstacles are detected, etc.)

  - (i) Could be provided by the controller, or alternatively another vision plugin that estimates the state of the robot
  - (ii) Could be provided by the controller, or another plugin such as object detection
  - (iii) The footstep plan can then be used by the controller

These are of course just a few example, this feature is intended to simplify sharing data across the framework. Use it sparingly, as adding objects to the datastore effectively makes them global, and thus modifiable from anywhere, including external plugins.

## Usage

A default `DataStore` instance is available in the `mc_control::MCController` class, and can be accessed using `mc_control::MCController::datastore()`. Any `c++` object can be created and retrieved from this `DataStore` instance from anywhere with access to the controller's instance.

- Objects can be directly created on the datastore throught the use of `make` or `make_initializer` as follows:

  ```cpp
  // Create a vector of double of size 4 initialized with value 42
  datastore().make<std::vector<double>>("key", 4, 42.0);
  // Create a vector of double of size 2 initialized with value {4, 42} (using list initialization)
  datastore().make_initializer<std::vector<double>>("key", 4, 42.0);
  ```

- If an object with the same name already exists, an exception will be raised. To avoid this, you can either check whether the datastore already has such an element with
  ```cpp
  if(datastore().has("key"))
  {
    datastore().make<Eigen::Vector3d>("key", 1, 2, 3);
  }
  ```
  or by using the more compact `make_or_assign` helper function that does the same in a single operattion

  ```cpp
  datastore().make_or_assign<Eigen::Vector3d>("EigenVectorScope", 1,2,3);
  ```

- You can also add an existing object to the datastore

  ```cpp
  Eigen::Vector3d vec{1, 2, 3};
  // Creates a copy of vec on the datastore
  store.make_or_assign<Eigen::Vector3d>("EigenVector", vec); // EigenVector is now {1,2,3}
  vec.x() = 2; // The datastore object is a copy of vec, so modifying vec will not modify the datastore's value
  // EigenVector: {1,2,3}
  // vec: {1,2,2}
  ```

  Note that all of these functions also return a reference to the created object that you can use to access and modify it

  ```cpp
  auto & vec = datastore().make_or_assign<Eigen::Vector3d>("EigenVector", Eigen::Vector3d::Zero());
  vec.x() = 42; // vec is now: 42, 0, 0
  ```

- A reference to the stored value can be retrieved at any time using `get<Type>`. Note that the retrived `Type` must match the one used when creating the object.

  ```cpp
  auto & vec = datastore().get<Eigen::Vector3d>("EigenVector");
  // vec: 42, 0, 0
  ```

  An exception will be thrown if the key is not present in the datastore, or the requested `Type` differs from the one used upon creation. Similarely to `mc_rtc::Configuration` objects, you may also use one of the convenience overloads of `get` to specify a default value to use in case the key does not exist:

  ```cpp
  Eigen::Vector3d vec{1,2,3};
  datastore().get<Eigen::Vector3d>("NotAKey", vec); // vec remains unchanged: 1, 2, 3
  datastore().get<Eigen::Vector3d>("EigenVector", vec); // vec is now 42, 0, 0

  bool hasFeature = datastore().get<bool>("HasFeature", false); // hasFeature will be assigned the value of "HasFeature" if that key exists, false otherwise
  ```


## Advanced usage

- **Inheritance** is also supported, but you need to explitely provide the type of your object's parent classes to be able to retrive them later. Here is a very simple example showing how to store and use objects using inheritance and polymorphic member functions.

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

- **Lambda functions** may also be stored, but their type must be explitely specified as an `std::function<...>`. Indeed, the inferred type for a lambda is a composite type and cannot be used for later retrieving it:

  ```cpp
  datastore().make("lambda", [](double t) {} ); // valid but the inferred lambda type is unknown
  datastore().get<???>("lambda");
  ```

  Instead, store the lambda as

  ```
  // Create a lambda function and store it as an std::function
  datastore().make<std::function<void(double)>("lambda", [](double t) {});
  // Retrieve the lambda
  auto & lambdaFun = datastore().get<std::function<void(double)>("lambda") {});
  // Call function
  lambdaFun(42);
  ```


# Main datastore objects

This section provides a list of the main datastore objects used within the framework. In some cases it is required to use datastore objects to ensure proper use of some componenents. Other components may use datastore objects if they are defined but do not require them, this is for instance the case of the `StabilizerStandingState` to which CoM targets may be provided using the datastore. In the table below, required elements are displayed in *bold*.

<table class="table">
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Type</th>
      <th scope="col">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr class="table-active">
      <th scope="row">
        StabilizerStandingState
      </th>
      <td colspan="2">These entries concern the StabilizerStandingState (see the <a href="lipm-stabilizer.html">stabilizer tutorial</a>)</td>
    </tr>

    <tr>
      <td>StabilizerStandingState::com</td>
      <td>Eigen::Vector3d</td>
      <td>Modify the CoM target used by the stabilizer.</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::stiffness</td>
      <td>double</td>
      <td>Modify the CoM tracking stiffness</td>
    </tr>
    <tr>
      <td>StabilizerStandingState::damping</td>
      <td>double</td>
      <td>Modify the CoM damping used by the stabilizer. If stiffness is provided but not damping, the default damping of 2*sqrt(stiffness) will be used</td>
    </tr>
</table>
