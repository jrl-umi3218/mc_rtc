---
layout: tutorials
toc: true
---

This page provides information about the following topics:

1. How to read data from an `mc_rtc::Configuration` object
2. How to write data to an `mc_rtc::Configuration` object
3. Which C++/Python types are supported out of the box
4. How to write your own function to load/save C++ objects
   from/to an `mc_rtc::Configuration` object

In every example on this page, we assume that `config` is a C++
object of type `mc_rtc::Configuration` in C++ examples and a
Python object of type `mc_rtc.Configuration` in Python
examples.

### Data source

`mc_rtc::Configuration` provides an abstraction over a JSONor a YAML object to easily manipulate the object and retrieve C++/Python data from it.

To load data into the configuration object you can use the following:

```cpp
// Load from a file, assume it's JSON data
auto config = mc_rtc::Configuration("myfile.conf");
// Load from a YAML file, the extension must be yml or yaml
auto config = mc_rtc::Configuration("myfile.yaml");
// Load from already loaded JSON data
auto config = mc_rtc::Configuration::fromData(data);
// Load from already loaded YAML data
auto config = mc_rtc::Configuration::fromYAMLData(data);
```

### Read data from the configuration

Several methods are proposed to access the data in the
configuration, you can mix them in your program depending on
the most appropriate. The three methods are:

1. Strict access: the key must exist and match the type you
   are retrieving
2. Optional strict access: a way to work around the existence
   rquirement of the strict access approach
3. Default access: completely relax the constraints of strict
   access

#### Strict access

The first method we present will throw if the configuration
entries you are accessing do not exist in the configuration
file that was loaded or if the stored type does not fit the
value you are trying to retrieve.

Accessing an entry is then done in this way:
```cpp
Eigen::Vector3d v = config("MyVector3d");
```

You can also access a sub-section of the configuration:
```cpp
Eigen::VectorXd v = config("section")("MyVectorXd");
```

And those can be nested:
```cpp
std::vector<std::string> v = config("section")("sub1")("sub2")("SomeStrings");
```

#### Optional strict access

This is simply done as so:
```cpp
if(conf.has("MyQuaternion"))
{
  Eigen::Quaterniond q = config("MyQuaternion");
}
```

The above code only throws if the `MyQuaternion` entry is part
of the configuration but does not contain the required data to
obtain an `Eigen::Quaterniond` object (see the previous
section).

#### Default access method

This method can be used safely to retrieve configuration entries. It will not
throw if the configuration entry does not exist or does not match the required
type.

Accessing an entry is done in this way:
```cpp
bool b = false;
config("MyBool", b);
```

You can also simplify the code above like so:
```cpp
bool b = config("MyBool", false);
```

If the entry does not exist or does not match the required type, the provided variable is simply not modified. Otherwise, the value stored in the configuration is copied into the provided variable. This is particularly important to understand when retrieving a vector:
```cpp
std::vector<double> v = {1., 2., 3.};
config("MyVector", v); // if MyVector is a valid entry, the initial content of v is lost
```

Similarly to the previous method, you can access sub-sections of the configuration:
```cpp
double d = 1.0;
config("section")("sub1")("sub2")("MyDouble", d);
```

However, the previous code **will throw** if any of the section/sub-section does not exist in the configuration file.

#### Sub-section access

The strict/default access methods presented above can be applied to access a sub-section of an `mc_rtc::Configuration` object, e.g.

```cpp
// throws if conf is not an object or section does not exist in conf
auto sub = config("section");

// returns an empty object if the call above would fail
auto sub = config("section", mc_rtc::Configuration{});
```

Note that copies are shallow so any changes you make to a
sub-section can be seen from the parent.

#### Python differences

In Python, we cannot deduce the type of the requested element so one has to provide a Python type or value to retrieve data:

```python
# c is an mc_rtc.Configuration object
c = config("key")
# retrieve a bool
b = config("b", bool)
# v is an eigen.Vector3d object
v = config("v", eigen.Vector3d)
# retrieve a bool with a default value of False
b = config("b", False)
# retrieve a list of eigen.Vector3d objects
vlist = config("vlist", [eigen.Vector3d])
# retrieve a list of float with a default value
flist = config("flist", [0., 1., 2.])
```

### Write data to an `mc_rtc::Configuration` object

Here are some writing examples that demonstrate the difference in object or array APIs.

```cpp
// -- Objects --

// Create a new empty object with the section key
auto section = conf.add("section");

// Add data to the section
section.add("v1", Eigen::Vector3d{1,2,3});
section.add("isFixed", true);
// Here data is of a type supported by mc_rtc::Configuration
section.add("data", data);

// -- Arrays --

// Creates a new empty array
auto array = conf.array("array");
// Creates a new empty array and reserves memory for 10 elements
auto array10 = conf.array("array10", 10);

// Add data to the array
array.push(Eigen::Vector3d{1,2,3});
array.push(true);
array.push(data);
```

The Python API is identical.

### Supported types

As you saw in the previous examples, the `Configuration` object can be used to
retrieve various types of object. The following table shows the supported types
and JSON requirements.

<table class="table">
  <thead>
    <tr>
      <th scope="col">Type</th>
      <th scope="col">JSON requirement</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th scope="row"><code>bool</code></th>
      <td>Either a boolean (<code>true</code>/<code>false</code>) or an integer (0 is false, anything else is true)</td>
    </tr>
    <tr>
      <th scope="row"><code>int</code></th>
      <td>An integer</td>
    </tr>
    <tr>
      <th scope="row"><code>unsigned int</code></th>
      <td>An unsigned integer or an integer that is &ge; 0</td>
    </tr>
    <tr>
      <th scope="row"><code>double</code></th>
      <td>A numeric entry</td>
    </tr>
    <tr>
      <th scope="row"><code>std::string</code></th>
      <td>A string entry</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Vector3d</code></th>
      <td>An array of size 3 composed of numeric elements</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Quaterniond</code></th>
      <td>An array of size 4 composed of numeric elements, the returned quaternion is normalized.</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Vector6d</code></th>
      <td>An array of size 6 composed of numeric elements</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::VectorXd</code></th>
      <td>An array of any size composed of numeric elements</td>
    </tr>
    <tr>
      <th scope="row"><code>Eigen::Matrix3d</code></th>
      <td>An array of size 9 composed of numeric elements. Two alternatives representations are supported to support rotation It can also be an array of size 3 representing RPY angles or an array of size 4 representing a quaternion</td>
    </tr>
    <tr>
      <th scope="row"><code>std::vector&lt;T&gt;</code> where <code>T</code> is any of the above types</th>
      <td>An array for which each element satisfies the requirements of type <code>T</code></td>
    </tr>
    <tr>
      <th scope="row"><code>std::array&lt;T, N&gt;</code> where <code>T</code> is any of the above types and `N` a fixed size</th>
      <td>An array of size <code>N</code> for which each elements satisfies the requirements of type <code>T</code></td>
    </tr>
    <tr>
      <th scope="row"><code>std::pair&lt;U, V&gt;</code> where <code>U</code> and <code>V</code> are any of the above types</th>
      <td>An array of size 2 whose elements satisfy the requirements of types <code>U</code> and `V` respectively</td>
    </tr>
    <tr>
      <th scope="row"><code>std::map&lt;std::string, T&gt;</code> where <code>T</code> is any of the above types</th>
      <td>A map indexed by string whose elements satisfy the requirements of types <code>T</code></td>
    </tr>
  </tbody>
</table>

We support a lot more types, including tasks and constraints, for each object that can be loaded by mc\_rtc you can find a documentation [on this website]({{site.baseurl}}/json.html).

#### Types' composability

The generic types supported by the configuration can be composed in any way you can imagine. For example, the following example is perfectly valid:

```cpp
std::vector<std::pair<std::vector<Eigen::Vector3d>, std::vector<double>> inequalities = config("inequalities");
```

### Support for your own types

This feature allows you to specialize `mc_rtc::Configuration`
to support loading/save to/from your own type. This feature is
a C++ only feature for now.

It works by adding a specialization template in the `mc_rtc`
namespace of the following form:

```cpp
template<>
struct ConfigurationLoader<MyType>
{
  static MyType load(const mc_rtc::Configuration & config);

  static mc_rtc::Configuration save(const MyType & object);
};
```

This feature is used extensively in `mc_rtc` to provide
support for many `SpaceVecAlg`, `RBDyn` and `mc_rbdyn` types.
All declarations can be found in
`mc_rbdyn/configuration_io.h`.

The saving function supports optional arguments so, the
following is a valid specialization:

```cpp
template<>
struct ConfigurationLoader<MyType>
{
  static MyType load(const mc_rtc::Configuration & config);

  static mc_rtc::Configuration save(const MyType & object, bool verbose = false);
};
```
