---
layout: tutorials
---

The `DataStore` is a facility used to efficiently store and retrieve arbitrary C++ objects. It is used to conveniently share objects between various parts of the framework. For instance, it may be used to [change the CoM target](lipm-stabilizer.html) for the `StabilizerStandingState`, provide [anchor frames](observers.html) for the `KinematicInertial` observer, or share any arbitrary data you wish between various part of your controller.

## Usage 
A default `DataStore` instance is available in the `mc_control::MCController` class, and can be accessed with `MCController::datastore()`.

Its use is quite intuitive, first a shared object is created in the datastore and assigned a unique name, which can later be used to retrieve the object anywhere within the framework.

```cpp
// Create a vector of double of size 4 initialized with value 42
datastore().make<std::vector<double>>("data", 4, 42.0);

// ...
// Somewhere else in the framework (other state, plugin, etc)
// ...

// Get a reference to the object
auto & data = store.get<std::vector<double>>("data");
// Modify the vector
data[0] = 0;
data.push_back(10);
// Vector now contains [0,42,42,42,10]
```


### Inheritance

Storing of inherited objects is also supported, but you need to explitely provide the type of your object's parent classes to be able to retrive them later. Here is a very simple example showing how to store and use objects using inheritance and polymorphic member functions.

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

# Main datastore objects 

This section provides a list of the main datastore objects used within the framework. In some cases it is required to use datastore objects to ensure proper use of some componenents. For instance, the [KinematicInertial](observers.html) observer requires the `KinematicInertial::anchorFrame` and `KinematicInertial::anchorFrameReal` properties to be updated at every iteration. Other components may use datastore objects if they are defined but do not require them, this is for instance the case of the `StabilizerStandingState` to which CoM targets may be provided using the datastore. In the table below, required elements are displayed in *bold*.

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

    <tr class="table-active">
      <th scope="row">
        KinematicInertialObserver 
      </th>
      <td colspan="2">These entries are used to configure the KinematicInertial Observer (<b>bold</b> entries are required)<br/> The anchorFrame and anchorFrameReal values must be provided at every iteration to use this observer (see the <a href="observers.html">observers tutorial</a>)</td>
    </tr>

    <tr>
      <th scope="row">KinematicInertialObserver::anchorFrame</th>
      <td>sva::PTransformd</td>
      <td>Sets the required anchorFrame for the control robot</td>
    </tr>
    <tr>
      <th scope="row">KinematicInertialObserver::anchorFrameReal</th>
      <td>sva::PTransformd</td>
      <td>Sets the required anchorFrame for the real robot</td>
    </tr>
  </tbody>
</table>
