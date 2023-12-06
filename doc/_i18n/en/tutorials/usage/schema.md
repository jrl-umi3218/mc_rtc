## Introducing `mc_rtc::Schema`

JSON Schema is a [specification](https://json-schema.org/) that allows to document the expected data format to go from a JSON object to a given data structure.

It is used in [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/json.html) to document the expected data for loading objects -- e.g. tasks or constraints -- into the framework.

As you write code within mc_rtc you will likely encounter a scenarion where you load a number of parameters from an {% include link_tutorial.html category="usage" tutorial="mc_rtc_configuration" %} object.

In addition to loading this configuration object into your data structure, you will also likely be interested in:

- Saving the current state of the object to an `mc_rtc::Configuration` file -- and then to disk;
- Allow to edit the parameters online;
- Document the configuration format;

Writing all this code by hand is possible but tedious and error-prone, especially when adding or removing fields from the structure.

This is where `mc_rtc::Schema` comes in. Its objective is to allow you to write a simple structure that behaves like a simple structure but comes packed with extra features.

## Our first schema-based structure

The following example is a simple schema structure that illustrates the basic usage of the feature:

```cpp
#include <mc_rtc/Schema.h>

struct SimpleSchema
{
  MC_RTC_NEW_SCHEMA(SimpleSchema)
#define MEMBER(...) MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__)
  MEMBER(bool, useFeature, "Use magic feature")
  MEMBER(double, weight, "Task weight")
  MEMBER(std::vector<std::string>, names, "Some names")
  using MapType = std::map<std::string, double>
  MEMBER(MapType, jointValues, "Map of joint names and values")
  MEMBER(sva::ForceVecd, wrench, "Target wrench")
  MEMBER(sva::PTransformd, pt, "Some transform")
#undef MEMBER
};
```

<h5 class="no_toc">Note on the MEMBER macro</h5>

If you intend to use the `MEMBER` macro, beware of the following pre-processor limitations.

<h6 class="no_toc">Templated types</h6>

Due to limitations of the pre-processor, you cannot directly pass types depending on multiple template parameters to the macro. For example, the following will give a compilation error.

```cpp
MEMBER(std::map<std::string, double>, jointValues, "Map of joint names and values")
```

This occurs because the pre-processor splits the arguments as follows:

- `std::map<std::string`
- `double>`
- `jointValues`
- `"Map of joint names and values"`

To avoid this you can explicitly alias the type

```cpp
using MapType = std::map<std::string, double>;
MEMBER(MapType, jointValues, "Map of joint names and values")
```

If this is not an option, you may use `BOOST_IDENTIY_TYPE` instead.


<h6 class="no_toc">Note for MSVC users</h6>

If you intend the above code to be used with the Microsoft Visual C++ Compiler (MSVC), the `MEMBER` definition should be changed to:

```cpp
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleSchema, __VA_ARGS__))
```

This works around a pre-processor issue in MSVC.

### Usage

The structure that we defined this way is used a regular C++ structure:

```cpp
void do_foo(const sva::ForceVecd &);

// The structure can be default constructed (more on the defaults later)
SimpleSchema simple;
// Access a member as a simple structure
do_foo(simple.wrench);
// The members are of the type you specified so this is also possible
do_foo(simple.pt * simple.wrench);
```

However, this structure also has a number of extra functionalities built-in:

```cpp
// Load from an mc_rtc::Configuration value
simple.load(config);
// Save to an mc_rtc::Configuration object
simple.save(config);
// We can also take advantage of mc_rtc::Configuration user defined conversions
config.add("simple", simple);
SimpleSchema simple2 = config("simple");
// Print to the console
mc_rtc::log::info("simple:\n{}", simple.dump(true, true));
// Add a form to the GUI that will edit simple in place
simple.addToGUI(*controller.gui(), {"Form category"}, "Form name",
                [this]() {
                  // The simple object has been updated with new values at this point
                  on_simple_update();
                });
```

You can also use aggregate initialization or [designated initialization](https://en.cppreference.com/w/cpp/language/aggregate_initialization#Designated_initializers) (from C++20) to initialize the structure:

```cpp
SimpleSchema simple{true, 42.42, {"a", "b"}, wrench, pt};
```

Finally the structure we created here has the same size as the simpler structure we could have created by hand.

## The `MC_RTC_SCHEMA_MEMBER` macro

In the previous example, we have used the `MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER` macro. It is actually a wrapper around the more general `MC_RTC_SCHEMA_MEMBER` macro which has the following siganture:

```cpp
MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, FLAGS, DEFAULT, ...)
```

The parameters have the following usage:

- `T` is the type of the Schema object where this member appears;
- `TYPE` is the type of the member variable;
- `NAME` is the name of the member variable;
- `DESCRIPTION` is a documentation string for the member variable, it is also used in the generated Form;
- `FLAGS` is used to add additional information about the member, most notably whether the member is required or not;
- `DEFAULT` is the default value used for this member;
- `...`/`NAMES` these extra parameters are used for two purposes:
  - if `TYPE` is `std::string` this can be used to provide valid values;
  - if `TYPE` is `std::variant<T...>` this is used to assign meaningful names to the variant altneratives;

### About `TYPE`

The intent is for any `TYPE` to be usable -- at least for the load/save scenario, not all types may be supported for the form-based edition.

If a given `TYPE` is not supported you will get a compilation error, please report the issue to mc_rtc developpers.

As of now, most types that can be loaded/saved through an `mc_rtc::Configuration` object are supported as well as schema-based types and `std::vector` and `std::map` of such types.

The following is an example using such types:

```cpp
struct ComposeSchema
{
  MC_RTC_NEW_SCHEMA(ComposeSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ComposeSchema, __VA_ARGS__))
  MEMBER(int, integer, "Integer value")
  MEMBER(SimpleSchema, simple, "Simple schema")
  MEMBER(std::vector<SimpleSchema>, many, "Multiple simple schema")
#undef MEMBER
};
```

### About `FLAGS`

There is current two flags:

- `mc_rtc::schema::Required`
- `mc_rtc::schema::Interactive`

#### `mc_rtc::schema::Required`

When a member is required:

- the member must be present in the configuration;
- the member must correctly de-serialize to its type;

Otherwise the member is only read if it is present in the configuration.

Note: members are always saved into a Configuration object, whether they are required or not

#### `mc_rtc::schema::Interactive`

The schema can treat the following types as interactive:

- `Eigen::Vector3d`
- `sva::PTransformd`

If members of these types are added to a form then interactive form elements will be used.

However, it does not always make sense. For example, a 3D gain could be represented with an `Eigen::Vector3d` and there is no point to use a 3D marker to edit this value.

You can control this behavior by setting the interactive flag accordingly. In all macros, except the basic `MC_RTC_SCHEMA_MEMBER`, the flag is set by default.

#### Example

The following example illustrates the use of these flags in the `MC_RTC_SCHEMA_MEMBER` macro call:

```cpp
struct InteractiveSchema
{
  MC_RTC_NEW_SCHEMA(InteractiveSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(InteractiveSchema, __VA_ARGS__))
  MEMBER(Eigen::Vector3d,
         point,
         "3D point",
         mc_rtc::schema::Required | mc_rtc::schema::Interactive,
         Eigen::Vector3d::Zero())
  MEMBER(Eigen::Vector3d, gain, "3D gain", mc_rtc::schema::Required, Eigen::Vector3d::Ones())
  MEMBER(Eigen::Vector3d, gainOpt, "3D optional gain", mc_rtc::schema::None, Eigen::Vector3d::Zero())
#undef MEMBER
};
```

### About `DEFAULT`

This lets you specify the default value of the member. Anything that would be valid on the right-hand side of the assignment operator is ok here.

If you are using a `_DEFAULT_` macro, this is `mc_rtc::Default<T>` which is:

- `0` for arithmetic types (thus `false` for `bool`)
- Zero for Eigen vectors
- Identity for Eigen square matrix
- Identity for `sva::PTransformd`
- Zero for `sva::ForceVecd`, `sva::MotionVecd`, `sva::ImpedanceVecd`, `sva::AdmittanceVecd`
- Empty string for `std::string`
- Empty vector for `std::vector<T>`
- Empty map for `std::map<K, V>`
- `mc_rtc::Default<T>` for `std::variant<T, Others...>`
- Default values for a schema-based structure

### About `NAMES`

These extra parameter has two possible use-case:

- a list of possible values when `TYPE` is `std::string`
- a description of the types when `TYPE` is `std::variant<T...>`

#### Example

In this example we have a gain represented as variant. It can be a `double` scalar or an `Eigen::Vector3d`. The `mc_rtc::schema::Choices` is passed to the `MEMBER` macro and those names will be used when displaying the choice in the GUI.

```cpp
struct SimpleVariant
{
  MC_RTC_NEW_SCHEMA(SimpleVariant)
  using gain_t = std::variant<double, Eigen::Vector3d>;
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(SimpleVariant, __VA_ARGS__))
  MEMBER(gain_t, stiffness, "Task stiffness", mc_rtc::schema::Choices({"scalar", "dimensional"}))
#undef MEMBER
};
```

## The `MC_RTC_NEW_SCHEMA` and `MC_RTC_SCHEMA` macros

In all examples so far we have seen usage of `MC_RTC_NEW_SCHEMA`. As the name suggests, this macro is used to introduce a new schema declaration.

It takes a single argument which is the name of the structure we are creating.

However, it is sometimes useful to extend a schema with additional members. In such cases one should use the `MC_RTC_SCHEMA` macro instead:

```cpp
struct ExtendedSchema : public SimpleSchema
{
  MC_RTC_SCHEMA(ExtendedSchema, SimpleSchema)
#define MEMBER(...) MC_RTC_PP_ID(MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(ExtendedSchema, __VA_ARGS__))
  MEMBER(double, extendedGain, "Gain for the extended algorithm")
#undef MEMBER
};
```

This macro simply takes the schema name as well as the base class the schema is using.
