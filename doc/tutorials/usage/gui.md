---
layout: tutorials
---

The GUI feature allows to build a dynamic graphical interface to interact with your controller remotely. In some regards, the feature implementation and the way you will use it in your controller is close to the approach to [logging]({{site.baseurl}}/tutorials/usage/logging.html).

This page covers the following topic:
- Access the GUI from the controller
- Add elements to the GUI
- Remove elements from the GUI

### Access to the GUI

The GUI works with a server/client model. The controller being the server and the client being remote interfaces. As such, on the controller side, your code will only provide data that is sent by the GUI server to the clients. This data is added through the `mc_rtc::gui::StateBuilder` interface.

To access the GUI builder, you simply call `gui()` from your controller code. Much like the logging interface, the GUI builder interface is always enabled so you don't have to worry about the state of the feature.

### Add elements to the GUI

The GUI is organized into category so that when you add multiple elements to the same category they appear together in the GUI.

Categories are represented by vector of strings (list of strings in Python) and elements are represented by a simple object that wraps a description of the element's type and accesors to the data your controller is providing.

To add an element one simply calls `gui()->addElement(...)` with a category an one or more elements, e.g.:

```cpp
// Add a single button named "Push" in the a/b category
gui()->addElement({"a", "b"},
  mc_rtc::gui::Button("Push", []() { std::cout << "Hello!" << std::endl; })
);

// Add multiple elements at once, they will appear in the order they are added in the function
gui()->addElement({"a", "b"},
  mc_rtc::gui::Button("Button 1", []() { return; }),
  mc_rtc::gui::Button("Button 2", []() { return; }),
  mc_rtc::gui::Button("Button 3", []() { return; })
);
```

##### A note on data

The state builder also holds some "static" data that can be used to reference commonly refered elements. By default, the following entries are already into data:
- `robots` is a list of robots' names in the controller
- `surfaces` is a map of robot's names to robot's surfaces
- `bodies` is a map of robot's names to robot's bodies

The data can be accessed by calling `gui()->data()` and modifying it. It is strongly advised not to modify the default data as it is used by the GUI clients to display some elements nicely.

#### Simple elements

In this section we present every element that can be added to the GUI, every sample assumes they are included as part of a `gui()->addElement(...)` call. For every functions you provide to the GUI to read data from your controller or pass data back to your controller you don't need to worry about exporting/importing the data to/from a specific format, this will be handled by the GUI.

##### `Label`/`ArrayLabel`

These elements can be used to display simple text or numeric data.

Examples

```cpp
// String label
Label("LabelText", []() { return "some text"; });

// Display a value
Label("Some value", [this]() { return this->value_; });

// Display a vector of strings
ArrayLabel("Some strings", []() { return {"a", "b", "c"}; });

// Display a vector with labels
ArrayLabel("Vector", {"x", "y", "z"}, []() { return Eigen::Vector3d::Zero(); });

// Some elements have associated labels, e.g. Force also adds an ArrayLabel
Force("LeftFoot", [this]() { return this->robot().surfaceWrench("LFullSole"); }, [this]() { return this->robot().surfacePose("LFullSole"); }),
Force("RightFoot", [this]() { return this->robot().surfaceWrench("RFullSole"); }, [this]() { return this->robot().surfacePose("RFullSole"); }),
Force("LeftHand", [this]() { return this->robot().surfaceWrench("LeftFingers"); }, [this]() { return this->robot().surfacePose("LeftFingers"); }),
Force("RightHand", [this]() { return this->robot().surfaceWrench("RightFingers"); }, [this]() { return this->robot().surfacePose("RightFingers"); })
```

Note that in Python, the labels go at the end of the parameter list:
```python
ArrayLabel("Vector", lambda: self.v3d, ["x", "y", "z"])
```

##### `Button`

This element simply display a button. Clicking the button triggers a call to the callback you provided.

```cpp
Button("Push me", []() { std::cout << "Hello!" << std::endl; });
```

##### `Checkbox`

This element display a checkbox. Its status depends on the value you provide, when you click it that value will be flipped.

```cpp
Checkbox("Check me", [this]() { return status_; }, [this](bool b) { status_ = b; });
```

##### `StringInput`/`IntegerInput`/`Number`/`ArrayInput`

These elements provide inputs for the user. You should use the representation that is most relevant to your data.

```cpp
StringInput("Your name", [this]() { return name_; }, [this](const std::string & n){ name_ = n; });

NumberInput("Weight", [this]() { return w_; }, [this](double w){ w_ = w; });
```

`ArrayInput` accepts an additional parameter to add labels to the data. It also assumes that the array size is fixed.

```cpp
ArrayInput("Your array", {"x", "y", "z"},
           [this]() { return v3_; },
           [this](const Eigen::Vector3d & v) { v3_ = v; });
```

In Python, the labels go at the end of the parameter list.

##### `ComboInput`/`DataComboInput`

These elements create a dialog to choose a value from a list of string, in `ComboInput` you provide this list of strings, in `DataComboInput`, you provide a reference to an entry in the GUI state data map.

```cpp
ComboInput("Choose from list", {"a", "b", "c"},
           [this]() { return choice_; },
           [this](const std::string & c) { choice_ = c; });

DataComboInput("Choose a robot", {"robots"},
               [this]() { return robot_; }
               [this](const std::string & r) { robot_ = r; });

DataComboInput("Choose a surface", {"surfaces", robot().name()},
               [this]() { return surface_; }
               [this](const std::string & s) { surface_ = s; });
```

##### `Point3D`/`Rotation`/`Transform`

These elements will display two things:
- an `ArrayInput` suited for the element you are adding
- an (optionally) interactive element to display the data in a 3D environment such as RViZ

```cpp
// Read-only variant does not provide a callback to set the data
Point3d("Point", [this]() { return v3_; });

// Read-write variant

// Note that even though we want to display a rotation, we need to provide a
// full transform otherwise the GUI could not guess where to display the element
Rotation("Rot",
         [this]() { return sva::PTransformd{rot_, pos_} },
         [this](const Eigen::Matrix3d & rot) { rot_ = rot; });
```

##### `Trajectory`

This element displays a trajectory in the 3D environment: depending on the type of data it returns it can be interpreted as a real-time trajectory display (e.g. to show the path taken by a surface) or a pre-planned trajectory.


```cpp
// Real-time trajectory, returns a single point (either Eigen::Vector3d or sva::PTransformd)
Trajectory("RealTimeTrajectory",
           [this]() { return robot().surfacePose("LeftGripper"); });

// Pre-planned trajectory returns a vector of points
Trajectory("Trajectory",
           [this]() { return traj_; });
```

##### `Polygon`

This element displays a single polygon (e.g. a planar surface) or list of polygon (e.g. a walking steps-plan).

```cpp
// A single polygon is a vector of Eigen::Vector3d
Polygon("Polygon"
        [this]() -> std::vector<Eigen::Vector3d> { return {p0, p1, p2, p3}; });

// A list of polygon is a vector of vector of Eigen::Vector3d
Polygon("Step plan",
        [this]]() { return step_plan_display_; });
```

##### `Force`

This element displays a force vector in the 3D environment. You need to provide the force value as an `sva::ForceVecd` as well as the frame where the force is applied as an `sva::PTransformd`.

```cpp
Force("LeftFoot", [this]() { return this->robot().surfaceWrench("LFullSole"); }, [this]() { return this->robot().surfacePose("LFullSole"); }),
```

##### `Arrow`

This element displays an arrow in the 3D environment. You need to provide the starting point and the end point of the arrow. It is possible to provide an editable arrow.

```cpp
// Read-only
Arrow("ArrowRO", [this]() { return start_; }, [this]() { return end_; });

// Editable arrow
Arrow("Arrow",
      [this](){ return start_; },
      [this](const Eigen::Vector3d & start) { start_ = start; },
      [this](){ return end_; },
      [this](const Eigen::Vector3d & end) { end_ = end; });
```

##### `XYTheta`

This element is a special case of the `Transform` element where only X/Y translation and Z rotation are editable.

```cpp
// Returns a vector with 3 elements representing the X/Y position and the theta angle
XYTheta("XYThetaOnGround", [this] -> std::array<double, 3> { return {x, y, theta}; });

// Specify the height with a 4th element
XYTheta("XYTheta", [this] -> std::array<double, 4> { return {x, y, theta, z}; });
```

##### `Table`

This element allows to display a table with arbitrary data in it. The data callback must return an object that could be represented as an array of array such as an `std::vector<std::vector<double>>` or an `std::vector<std::tuple<X, Y, Z>>`

```cpp
// Table with a fixed header
Table("Simple table", {"X", "Y", "Z"}, [this]() -> const std::vector<Eigen::Vector3d> & { return points_; });
// Table with a fixed header and formatting information
Table("Simple with format", {"X", "Y", "Z"}, {"{:0.3f}", "{:0.3f}", "{:0.3f}"}, [this]() { return data_; });
// Table with a dynamic header
Table("Dynamic table", [this]() { return header_; }, [this]() { return data_; });
// Table with a dynamic header + formatting
Table("Dynamic with format", [this]() { return header_; }, [this]() { return format_; }, [this]() { return data_; });
```

It is your reponsibility to ensure that the header, format and data sizes are coherent although the client should be relatively resilient when they are not. Format strings must follow the rules of [{fmt}](https://fmt.dev/latest/syntax.html).

##### `Form`

The `Form` element allows to build a more complexe dialog for interaction with the user. A form is itself composed of elements, the `Form` creation looks like the following:

```cpp
Form("Push to send", // This will be shown on the "send" button for the form
     [this](const mc_rtc::Configuration & data) {},
     ... // list of element
);
```

The available elements are:
- `FormCheckbox(name, required, default)`
- `FormIntegerInput(name, required, default)`
- `FormNumberInput(name, required, default)`
- `FormStringInput(name, required, default)`
- `FormArrayInput(name, required, default, fixed_size (true))`
- `FormComboInput(name, required, values)`
- `FormDataComboInput(name, required, reference)`

In all those, `name` will be used to display the name of the element and to retrieve the data in the callback, `required` indicates whether the user must provide a value for this field or not and `default` is optional and provides a default value for the field.

In `FormArrayInput`, `fixed_size` indicates whether the default value you are providing is of a fixed size or can be expanded/shrinked by the user.

In `FormComboInput`, `values` is the list of values provided to the user in the combo box.

In `FormDataComboInput`, `reference` is a reference to the data entry. Additionally, it can reference another field of the form by using the `$` symbol. For example:

```cpp
Form("Button name",
     [this](const mc_rtc::Configuration & data) {},
     FormDataComboInput("Robot", true, {"robots"}),
     FormDataComboInput("Surface", true, {"surfaces", "$Robot"})
);
```

In that example, the content of the `Surface` combo box will change depending on the selection the user makes in `Robot`.

##### A note on callbacks

Every callback call triggered by the user interaction happens after an iteration of the controller and before the next. You don't have to worry about concurrency here.

### Remove elements from the GUI

Two functions are provided to remove elements from the GUI:

```cpp
// Remove a single element by name
gui()->removeElement({"a", "b"}, "element");
// Remove a category and all its sub-categories
gui()->removeCategory({"a", "b"});
```
