#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/gui/Form.h>
#include <mc_rtc/gui/StateBuilder.h>

namespace mc_rtc::schema
{

namespace details
{

/** Helper to pass otherwise non deducible template parameters to a constructor */
template<auto ptr>
struct MemberPointerWrapper
{
};

/** Helper when a Schema value has a finite number of choices */
template<bool HasChoices>
struct Choices
{
  Choices() = default;
  Choices(const std::vector<std::string> & choices) : choices(choices) {}
  std::vector<std::string> choices;
};

template<>
struct Choices<false>
{
  Choices() = default;
};

/** Type-trait to detect a Schema type */
struct is_schema
{
  template<typename T>
  static auto test(T *) -> typename T::is_schema_t;

  template<typename T>
  static std::false_type test(...);
};

template<typename T>
inline constexpr bool is_schema_v = decltype(is_schema::test<T>(nullptr))::value;

/** Type-trait to detect an std::vector */
template<typename T>
struct is_std_vector : std::false_type
{
};

template<typename T, typename Allocator>
struct is_std_vector<std::vector<T, Allocator>> : std::true_type
{
};

template<typename T>
inline constexpr bool is_std_vector_v = is_std_vector<T>::value;

/** Type-trait to detect an std::vector<T> where T is a Schema-based type */
template<typename T>
inline constexpr bool is_std_vector_schema_v = []()
{
  if constexpr(is_std_vector_v<T>) { return is_schema_v<typename T::value_type>; }
  else { return false; }
}();

/** Helper to get a default value for a given type */
template<typename T, typename Enable = void>
struct Default
{
  static_assert(!std::is_same_v<T, T>, "Must be specialized");
};

template<typename T>
struct Default<T, std::enable_if_t<is_schema_v<T>>>
{
  inline static const T value = {};
};

template<typename T>
struct Default<T, std::enable_if_t<is_std_vector_v<T>>>
{
  inline static const T value = {};
};

template<typename T>
struct Default<T, std::enable_if_t<std::is_arithmetic_v<T>>>
{
  inline static constexpr T value = 0;
};

template<typename Scalar, int N, int Options, int MaxRows, int MaxCols>
struct Default<Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols>, std::enable_if_t<(N > 0)>>
{
  inline static const Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols> value =
      Eigen::Matrix<Scalar, N, 1, Options, MaxRows, MaxCols>::Zero();
};

template<typename Scalar, int N, int Options, int MaxRows, int MaxCols>
struct Default<Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols>, std::enable_if_t<(N > 1)>>
{
  inline static const Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols> value =
      Eigen::Matrix<Scalar, N, N, Options, MaxRows, MaxCols>::Identity();
};

template<>
struct Default<sva::PTransformd>
{
  inline static const sva::PTransformd value = sva::PTransformd::Identity();
};

template<>
struct Default<sva::MotionVecd>
{
  inline static const sva::MotionVecd value = sva::MotionVecd::Zero();
};

template<>
struct Default<sva::ForceVecd>
{
  inline static const sva::ForceVecd value = sva::ForceVecd::Zero();
};

template<>
struct Default<sva::ImpedanceVecd>
{
  inline static const sva::ImpedanceVecd value = sva::ImpedanceVecd::Zero();
};

template<>
struct Default<sva::AdmittanceVecd>
{
  inline static const sva::AdmittanceVecd value = sva::AdmittanceVecd::Zero();
};

template<>
struct Default<std::string>
{
  inline static const std::string value;
};

template<typename T, typename... Others>
struct Default<std::variant<T, Others...>> : public Default<T>
{
};

template<typename T, bool IsRequired, bool IsInteractive, bool HasChoices = false, bool IsStatic = false>
void addValueToForm(const T & value,
                    const std::string & description,
                    const details::Choices<HasChoices> & choices,
                    gui::details::FormElements & form);

template<bool IsRequired, bool IsInteractive, bool HasChoices, typename... Args>
void variantToForm(const std::variant<Args...> &,
                   gui::details::FormElements & form,
                   const Choices<HasChoices> & choices)
{
  auto get_choice = [&](size_t idx)
  {
    if constexpr(HasChoices)
    {
      if(idx < choices.choices.size()) { return choices.choices[idx]; }
      return std::to_string(idx);
    }
    else { return std::to_string(idx); }
  };
  size_t i = 0;
  (addValueToForm<Args, IsRequired, IsInteractive, false, true>(Default<Args>::value, get_choice(i++), {}, form), ...);
}

template<typename T, bool IsRequired, bool IsInteractive, bool HasChoices, bool IsStatic>
void addValueToForm(const T & value,
                    const std::string & description,
                    const details::Choices<HasChoices> & choices,
                    gui::details::FormElements & form)
{
  const auto & get_value = [&value]() -> decltype(auto)
  {
    if constexpr(IsStatic) { return value; }
    else
    {
      return [&value]() -> const T & { return value; };
    }
  }();
  if constexpr(details::is_schema_v<T>)
  {
    mc_rtc::gui::FormObjectInput input(description, IsRequired);
    value.buildForm(input);
    form.addElement(input);
  }
  else if constexpr(details::is_std_vector_v<T>)
  {
    mc_rtc::gui::FormGenericArrayInput input(description, IsRequired, get_value);
    using value_type = typename T::value_type;
    static value_type default_{};
    addValueToForm<value_type, true, IsInteractive>(default_, description, {}, input);
    form.addElement(input);
  }
  else if constexpr(gui::details::is_variant_v<T>)
  {
    auto input = mc_rtc::gui::FormOneOfInput(description, IsRequired, get_value);
    variantToForm<IsRequired, IsInteractive>(value, input, choices);
    form.addElement(input);
  }
  else if constexpr(std::is_same_v<T, bool>)
  {
    form.addElement(mc_rtc::gui::FormCheckbox(description, IsRequired, get_value));
  }
  else if constexpr(std::is_integral_v<T>)
  {
    form.addElement(mc_rtc::gui::FormIntegerInput(description, IsRequired, get_value));
  }
  else if constexpr(std::is_floating_point_v<T>)
  {
    form.addElement(mc_rtc::gui::FormNumberInput(description, IsRequired, get_value));
  }
  else if constexpr(std::is_same_v<T, std::string>)
  {
    if constexpr(HasChoices)
    {
      auto it = std::find(choices.choices.begin(), choices.choices.end(), value);
      long int idx = it != choices.choices.end() ? std::distance(choices.choices.begin(), it) : -1;
      form.addElement(
          mc_rtc::gui::FormComboInput(description, IsRequired, choices.choices, false, static_cast<int>(idx)));
    }
    else { form.addElement(mc_rtc::gui::FormStringInput(description, IsRequired, get_value)); }
  }
  else if constexpr(std::is_same_v<T, Eigen::Vector3d>)
  {
    form.addElement(mc_rtc::gui::FormPoint3DInput(description, IsRequired, get_value, IsInteractive));
  }
  else if constexpr(std::is_same_v<T, sva::PTransformd>)
  {
    form.addElement(mc_rtc::gui::FormTransformInput(description, IsRequired, get_value, IsInteractive));
  }
  else if constexpr(std::is_same_v<T, sva::ForceVecd>)
  {
    form.addElement(mc_rtc::gui::FormArrayInput(description, IsRequired, get_value));
  }
  else { static_assert(!std::is_same_v<T, T>, "addValueToForm must be implemented for this value type"); }
}

} // namespace details

/** Feature flags for Schema values */
enum class ValueFlag
{
  /** No flags */
  None = 0,
  /** The value is required */
  Required = 2 << 0,
  /** Use an interactive display */
  Interactive = 2 << 1,
  /** All flags enabled */
  All = Required | Interactive
};

inline constexpr ValueFlag operator|(ValueFlag lhs, ValueFlag rhs) noexcept
{
  using int_t = std::underlying_type_t<ValueFlag>;
  return static_cast<ValueFlag>(static_cast<int_t>(lhs) | static_cast<int_t>(rhs));
}

inline constexpr ValueFlag operator&(ValueFlag lhs, ValueFlag rhs) noexcept
{
  using int_t = std::underlying_type_t<ValueFlag>;
  return static_cast<ValueFlag>(static_cast<int_t>(lhs) & static_cast<int_t>(rhs));
}

inline constexpr bool HasFeature(ValueFlag flag, ValueFlag feature) noexcept
{
  using int_t = std::underlying_type_t<ValueFlag>;
  return (static_cast<int_t>(flag) & static_cast<int_t>(feature)) != 0;
}

/** Helper to implement a Schema methods
 *
 * Inside a schema-enabled struct, an Operations singleton should be created
 *
 * When a member variable is added to the struct, it should be statically registered to the Operations singleton
 *
 * When deriving a schema-enabled struct, an Operations singleton specific to this derived struct should be created and
 * it should refer the base operations
 *
 * It is advised to use the provided macros to deal with that boilerplate, namely:
 * - MC_RTC_DECLARE_SCHEMA(SchemaT, BaseT) for declaring a schema-enabled struct of type SchemaT that inherits BaseT
 * - MC_RTC_SCHEMA_MEMBER_*(SchemaT, ...) to declare members inside a schema-enable struct of type SchemaT
 *
 */
struct MC_RTC_UTILS_DLLAPI Operations
{
  /** Name of members in the schema */
  size_t values_count = 0;

  /** Save to a configuration object */
  std::function<void(const void * self, Configuration & out)> save = [](const void *, Configuration &) {};

  /** Write to a MessagePackBuilder */
  std::function<void(const void * self, MessagePackBuilder & builder)> write = [](const void *, MessagePackBuilder &) {
  };

  /** Load from a configuration object */
  std::function<void(void * self, const Configuration & in)> load = [](void *, const Configuration &) {};

  using FormElements = gui::details::FormElements;

  /** Build a Form to load the object */
  std::function<void(const void * self, FormElements & form)> buildForm = [](const void *, FormElements &) {};

  /** Convert a form configuration to the configuration expected by load */
  std::function<void(const Configuration & in, Configuration & out)> formToStd = [](const Configuration &,
                                                                                    Configuration &) {};

  /** Compare two objects */
  std::function<bool(const void * lhs, const void * rhs)> areEqual = [](const void *, const void *) { return true; };

  /** FIXME Add a saveSchema function */

  /** FIXME Add logging operation?*/

  /** Register a value
   *
   * \tparam Schema Schema this value belongs to
   *
   * \tparam ptr Member pointer in the Schema object
   *
   * \tparam Flags Control some features of the value in the schema
   *
   * \tparam HasChoices If true, we expect choices to be provided for the value instead of an open form
   *
   * \param name Name of the value
   *
   * \param description Description for the value
   *
   * \param choices Possible choices when \tparam HasChoices is true
   */
  template<typename T, typename Schema, T Schema::*ptr, ValueFlag Flags = ValueFlag::All, bool HasChoices = false>
  bool registerValue(details::MemberPointerWrapper<ptr>,
                     const std::string & name,
                     const std::string & description,
                     const std::integral_constant<ValueFlag, Flags> & = {},
                     const details::Choices<HasChoices> & choices = {})
  {
    constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
    constexpr bool IsInteractive = HasFeature(Flags, ValueFlag::Interactive);
    values_count += 1;
    save = [save = save, name](const void * self, mc_rtc::Configuration & out)
    {
      save(self, out);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      out.add(name, value);
    };
    write = [write = write, description](const void * self, mc_rtc::MessagePackBuilder & builder)
    {
      write(self, builder);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      builder.write(description);
      builder.write(value);
    };
    load = [load = load, name](void * self, const mc_rtc::Configuration & in)
    {
      load(self, in);
      T & value = static_cast<Schema *>(self)->*ptr;
      if(in.has(name)) { value = in(name).operator T(); }
      else if constexpr(IsRequired) { mc_rtc::log::error_and_throw("{} is required"); }
    };
    formToStd = [formToStd = formToStd, name, description](const Configuration & in, Configuration & out)
    {
      formToStd(in, out);
      if(IsRequired || in.has(description))
      {
        if constexpr(details::is_schema_v<T>)
        {
          auto out_ = out.add(name);
          T::formToStd(in(description), out_);
        }
        else if constexpr(details::is_std_vector_schema_v<T>)
        {
          using SchemaT = typename T::value_type;
          std::vector<Configuration> in_ = in(description);
          auto out_ = out.array(name, in_.size());
          for(size_t i = 0; i < in_.size(); ++i)
          {
            auto out_i = out_.object();
            SchemaT::formToStd(in_[i], out_i);
          }
        }
        else { out.add(name, in(description)); }
      }
    };
    buildForm = [buildForm = buildForm, description, choices](const void * self, Operations::FormElements & form)
    {
      buildForm(self, form);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      details::addValueToForm<T, IsRequired, IsInteractive>(value, description, choices, form);
    };
    areEqual = [areEqual = areEqual](const void * lhs, const void * rhs)
    {
      const T & lhs_value = static_cast<const Schema *>(lhs)->*ptr;
      const T & rhs_value = static_cast<const Schema *>(rhs)->*ptr;
      return areEqual(lhs, rhs) && (lhs_value == rhs_value);
    };
    return true;
  }
};

namespace details
{

/** Gets a default value:
 * - if IsRequired is true, HasChoices is true and the choices are not empty this returns the first choice
 * - otherwise returns the default value provided
 */
template<typename T, ValueFlag Flags = ValueFlag::All, bool HasChoices = false>
const T & get_default(const T & default_,
                      const std::integral_constant<ValueFlag, Flags> & = {},
                      const details::Choices<HasChoices> & choices = {})
{
  constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
  if constexpr(IsRequired && HasChoices && std::is_same_v<T, std::string>)
  {
    if(choices.choices.size()) { return choices.choices[0]; }
  }
  return default_;
}

/** Empty schema used to simplify writing of the MC_RTC_SCHEMA macro, you do not need to inherit from that */
struct EmptySchema
{
  /** Empty ops used to simplify code generation */
  inline static mc_rtc::schema::Operations ops_;
};

} // namespace details

#define MC_RTC_SCHEMA(SchemaT, BaseT)                                                                        \
  /** Tag to detect Schema objects */                                                                        \
  using is_schema_t = std::true_type;                                                                        \
                                                                                                             \
protected:                                                                                                   \
  inline static mc_rtc::schema::Operations ops_;                                                             \
                                                                                                             \
  inline static size_t schema_size() noexcept                                                                \
  {                                                                                                          \
    return ops_.values_count + BaseT::ops_.values_count;                                                     \
  }                                                                                                          \
  inline void write_impl(mc_rtc::MessagePackBuilder & builder) const                                         \
  {                                                                                                          \
    BaseT::ops_.write(this, builder);                                                                        \
    ops_.write(this, builder);                                                                               \
  }                                                                                                          \
                                                                                                             \
public:                                                                                                      \
  inline void save(mc_rtc::Configuration & out) const                                                        \
  {                                                                                                          \
    BaseT::ops_.save(this, out);                                                                             \
    ops_.save(this, out);                                                                                    \
  }                                                                                                          \
  inline mc_rtc::Configuration toConfiguration() const                                                       \
  {                                                                                                          \
    mc_rtc::Configuration out;                                                                               \
    save(out);                                                                                               \
    return out;                                                                                              \
  }                                                                                                          \
  inline void write(mc_rtc::MessagePackBuilder & builder) const                                              \
  {                                                                                                          \
    builder.start_map(schema_size());                                                                        \
    write_impl(builder);                                                                                     \
    builder.finish_map();                                                                                    \
  }                                                                                                          \
  inline std::string dump(bool pretty, bool yaml) const                                                      \
  {                                                                                                          \
    mc_rtc::Configuration out;                                                                               \
    save(out);                                                                                               \
    return out.dump(pretty, yaml);                                                                           \
  }                                                                                                          \
  inline void load(const mc_rtc::Configuration & in)                                                         \
  {                                                                                                          \
    BaseT::ops_.load(this, in);                                                                              \
    ops_.load(this, in);                                                                                     \
  }                                                                                                          \
  inline static SchemaT fromConfiguration(const mc_rtc::Configuration & in)                                  \
  {                                                                                                          \
    SchemaT out;                                                                                             \
    out.load(in);                                                                                            \
    return out;                                                                                              \
  }                                                                                                          \
  inline void buildForm(mc_rtc::schema::Operations::FormElements & form) const                               \
  {                                                                                                          \
    BaseT::ops_.buildForm(this, form);                                                                       \
    ops_.buildForm(this, form);                                                                              \
  }                                                                                                          \
  static inline void formToStd(const mc_rtc::Configuration & in, mc_rtc::Configuration & out)                \
  {                                                                                                          \
    BaseT::ops_.formToStd(in, out);                                                                          \
    ops_.formToStd(in, out);                                                                                 \
  }                                                                                                          \
  template<typename Callback = std::function<void()>>                                                        \
  inline void addToGUI(                                                                                      \
      mc_rtc ::gui::StateBuilder & gui, const std::vector<std::string> & category, const std::string & name, \
      Callback callback = []() {})                                                                           \
  {                                                                                                          \
    auto form = mc_rtc::gui::Form(name,                                                                      \
                                  [this, callback](const mc_rtc::Configuration & in)                         \
                                  {                                                                          \
                                    mc_rtc::Configuration cfg;                                               \
                                    formToStd(in, cfg);                                                      \
                                    /** FIXME If callback takes SchemaT do a copy **/                        \
                                    load(cfg);                                                               \
                                    callback();                                                              \
                                  });                                                                        \
    buildForm(form);                                                                                         \
    gui.addElement(category, form);                                                                          \
  }                                                                                                          \
  inline bool operator==(const SchemaT & rhs) const                                                          \
  {                                                                                                          \
    return BaseT::ops_.areEqual(this, &rhs) && ops_.areEqual(this, &rhs);                                    \
  }                                                                                                          \
  inline bool operator!=(const SchemaT & rhs) const                                                          \
  {                                                                                                          \
    return !(*this == rhs);                                                                                  \
  }

#define MC_RTC_NEW_SCHEMA(SchemaT) MC_RTC_SCHEMA(SchemaT, mc_rtc::schema::details::EmptySchema)

/** Declare a T member of type TYPE, specify REQUIRED and DEFAULT value */
#define MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, DEFAULT, ...)                           \
public:                                                                                                    \
  TYPE NAME = mc_rtc::schema::details::get_default(                                                        \
      DEFAULT, std::integral_constant<mc_rtc::schema::ValueFlag, REQUIRED>{}, ##__VA_ARGS__);              \
                                                                                                           \
private:                                                                                                   \
  inline static const bool NAME##_registered_ =                                                            \
      T::ops_.registerValue(mc_rtc::schema::details::MemberPointerWrapper<&T::NAME>{}, #NAME, DESCRIPTION, \
                            std::integral_constant<mc_rtc::schema::ValueFlag, REQUIRED>{}, ##__VA_ARGS__); \
                                                                                                           \
public:

/** Declare a required Schema<T> member of type TYPE, only specify DEFAULT */
#define MC_RTC_SCHEMA_REQUIRED_MEMBER(T, TYPE, NAME, DESCRIPTION, DEFAULT, ...) \
  MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::All, DEFAULT, ##__VA_ARGS__)

/** Declare an optional Schema<T> member of type TYPE, only specify DEFAULT */
#define MC_RTC_SCHEMA_OPTIONAL_MEMBER(T, TYPE, NAME, DESCRIPTION, DEFAULT, ...) \
  MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::Interactive, DEFAULT, ##__VA_ARGS__)

/** Declare a Schema<T> member of type TYPE with a default value, only specify REQUIRED */
#define MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, ...)                             \
  MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, mc_rtc::schema::details::Default<TYPE>::value, \
                       ##__VA_ARGS__)

/** Declare a required Schema<T> member of type TYPE with a default value */
#define MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::All, ##__VA_ARGS__)

/** Declare an optional Schema<T> member of type TYPE with a default value */
#define MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::Interactive, ##__VA_ARGS__)

/** The following alias make working with schema a little simpler */
static inline constexpr ValueFlag Required = ValueFlag::Required;

static inline constexpr ValueFlag Interactive = ValueFlag::Interactive;

static inline constexpr ValueFlag None = ValueFlag::None;

using Choices = details::Choices<true>;

} // namespace mc_rtc::schema
