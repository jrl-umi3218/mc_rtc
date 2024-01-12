#pragma once

#include <mc_rtc/Configuration.h>
#include <mc_rtc/Default.h>
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

/** Type-trait to detect an std::map */
template<typename T>
struct is_std_map : std::false_type
{
};

template<typename T>
struct is_std_map<std::map<std::string, T>> : std::true_type
{
};

template<typename T>
inline constexpr bool is_std_map_v = is_std_map<T>::value;

/** Type-trait to detect an std::map<Key, ValueT> where ValueT is a Schema-based type */
template<typename T>
inline constexpr bool is_std_map_schema_v = []()
{
  if constexpr(is_std_map_v<T>) { return is_schema_v<typename T::value_type>; }
  else { return false; }
}();

/** Type-trait to detect if something is an Eigen::VectorNd */
template<typename T>
struct is_eigen_vector : public std::false_type
{
};

template<typename Scalar, int Rows, int Options, int MaxRows>
struct is_eigen_vector<Eigen::Matrix<Scalar, Rows, 1, Options, MaxRows, 1>> : public std::true_type
{
};

template<typename T>
inline constexpr bool is_eigen_vector_v = is_eigen_vector<T>::value;

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
  else if constexpr(details::is_std_map_v<T>)
  {
    // We currently do not support map in forms
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
  else if constexpr(std::is_same_v<T, sva::ForceVecd> || details::is_eigen_vector_v<T>)
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
struct Operations
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
      constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
      load(self, in);
      T & value = static_cast<Schema *>(self)->*ptr;
      if(in.has(name)) { value = in(name).operator T(); }
      else if constexpr(IsRequired) { mc_rtc::log::error_and_throw("{} is required"); }
    };
    formToStd = [formToStd = formToStd, name, description](const Configuration & in, Configuration & out)
    {
      constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
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
        else if constexpr(details::is_std_map_schema_v<T>) {}
        else { out.add(name, in(description)); }
      }
    };
    buildForm = [buildForm = buildForm, description, choices](const void * self, Operations::FormElements & form)
    {
      constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
      constexpr bool IsInteractive = HasFeature(Flags, ValueFlag::Interactive);
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

/** The following alias make working with schema a little simpler */
static inline constexpr ValueFlag Required = ValueFlag::Required;

static inline constexpr ValueFlag Interactive = ValueFlag::Interactive;

static inline constexpr ValueFlag None = ValueFlag::None;

using Choices = details::Choices<true>;

} // namespace mc_rtc::schema

namespace mc_rtc
{
template<typename T>
struct Default<T, std::enable_if_t<schema::details::is_schema_v<T>>>
{
  inline static const T value = {};
};

template<typename T>
struct Default<T, std::enable_if_t<schema::details::is_std_vector_v<T>>>
{
  inline static const T value = {};
};

template<typename T>
struct Default<T, typename std::enable_if_t<schema::details::is_std_map_v<T>>>
{
  inline static const T value = {};
};
} // namespace mc_rtc

#include <mc_rtc/SchemaMacros.h>
