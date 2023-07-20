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

template<typename Scalar, int N, int _Options, int _MaxRows, int _MaxCols>
struct Default<Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols>, std::enable_if_t<(N > 0)>>
{
  inline static const Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols> value =
      Eigen::Matrix<Scalar, N, 1, _Options, _MaxRows, _MaxCols>::Zero();
};

template<typename Scalar, int N, int _Options, int _MaxRows, int _MaxCols>
struct Default<Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols>, std::enable_if_t<(N > 1)>>
{
  inline static const Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols> value =
      Eigen::Matrix<Scalar, N, N, _Options, _MaxRows, _MaxCols>::Identity();
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
  inline static const std::string value = "";
};

template<typename T, typename... Others>
struct Default<std::variant<T, Others...>> : public Default<T>
{
};

} // namespace details

/** Operations implemented by an object that can be represented with a Schema */
struct MC_RTC_UTILS_DLLAPI Operations
{
  /** Number of elements in the schema */
  size_t schema_size = 0;

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

  /** Load from a Form */
  std::function<void(void * self, const Configuration & in)> loadForm = [](void *, const Configuration &) {};

  /** Compare two objects */
  std::function<bool(const void * lhs, const void * rhs)> areEqual = [](const void *, const void *) { return true; };

  /** FIXME Add a saveSchema function */

  /** FIXME Add logging operation?*/
};

namespace details
{

template<typename T, bool IsRequired, bool IsInteractive, bool HasChoices = false, bool IsStatic = false>
void addValueToForm(const T & value,
                    const std::string & description,
                    const details::Choices<HasChoices> & choices,
                    Operations::FormElements & form);

template<bool IsRequired, bool IsInteractive, bool HasChoices, typename... Args>
void variantToForm(const std::variant<Args...> &, Operations::FormElements & form, const Choices<HasChoices> & choices)
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
                    Operations::FormElements & form)
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
    value.buildForm(&value, input);
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
  else
  {
    if constexpr(std::is_same_v<T, bool>)
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
    else if constexpr(gui::details::is_variant_v<T>)
    {
      auto input = mc_rtc::gui::FormOneOfInput(description, IsRequired, get_value);
      variantToForm<IsRequired, IsInteractive>(value, input, choices);
      form.addElement(input);
    }
    else { static_assert(!std::is_same_v<T, T>, "addValueToForm must be implemented for this value type"); }
  }
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

/** A simple value in a schema
 *
 * This comprises of:
 * - a value of type \tparam T
 * - an indentifier/name given at construction
 * - a description given at construction
 */
template<typename T>
struct alignas(T) Value
{
  /** Create the value
   *
   * \tparam Schema Schema this value belongs to
   *
   * \tparam ptr Member pointer in the Schema object
   *
   * \tparam Flags Control some features of the value in the schema
   *
   * \tparam HasChoices If true, we expect choices to be provided for the value instead of an open form
   *
   * \param ops Operations that we should setup
   *
   * \param name Name of the value
   *
   * \param description Description for the value
   *
   * \param default_ Default value
   *
   * \param required Pass the IsRequired value
   *
   * \param combo Possible choices when \tparam HasChoices is true
   */
  template<typename Schema, T Schema::*ptr, ValueFlag Flags = ValueFlag::All, bool HasChoices = false>
  Value(Operations & ops,
        details::MemberPointerWrapper<ptr>,
        const std::string & name,
        const std::string & description,
        const T & default_ = details::Default<T>::value,
        const std::integral_constant<ValueFlag, Flags> & = {},
        const details::Choices<HasChoices> & choices = {})
  : value_(default_)
  {
    constexpr bool IsRequired = HasFeature(Flags, ValueFlag::Required);
    constexpr bool IsInteractive = HasFeature(Flags, ValueFlag::Interactive);
    ops.schema_size += 1;
    ops.save = [save = ops.save, name](const void * self, mc_rtc::Configuration & out)
    {
      save(self, out);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      if constexpr(details::is_schema_v<T>)
      {
        auto out_ = out.add(name);
        value.save(out_);
      }
      else if constexpr(details::is_std_vector_schema_v<T>)
      {
        auto out_ = out.array(name, value.size());
        for(const auto & v : value)
        {
          auto obj_ = out_.object();
          v.save(obj_);
        }
      }
      else { out.add(name, value); }
    };
    ops.write = [write = ops.write, description](const void * self, mc_rtc::MessagePackBuilder & builder)
    {
      write(self, builder);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      builder.write(description);
      builder.write(value);
    };
    ops.load = [load = ops.load, name](void * self, const mc_rtc::Configuration & in)
    {
      load(self, in);
      T & value = static_cast<Schema *>(self)->*ptr;
      if constexpr(details::is_schema_v<T>) { value.load(in(name)); }
      else if constexpr(details::is_std_vector_schema_v<T>)
      {
        std::vector<mc_rtc::Configuration> in_ = in(name);
        value.resize(in_.size());
        for(size_t i = 0; i < in.size(); ++i) { value[i].load(in_[i]); }
      }
      else { value = in(name).operator T(); }
    };
    ops.loadForm = [loadForm = ops.loadForm, description](void * self, const mc_rtc::Configuration & in)
    {
      loadForm(self, in);
      T & value = static_cast<Schema *>(self)->*ptr;
      if(IsRequired || in.has(description))
      {
        if constexpr(details::is_schema_v<T>) { value.loadForm(&value, in(description)); }
        else if constexpr(details::is_std_vector_schema_v<T>)
        {
          std::vector<mc_rtc::Configuration> in_ = in(description);
          value.resize(in_.size());
          for(size_t i = 0; i < in_.size(); ++i) { value[i].loadForm(&value[i], in_[i]); }
        }
        else { value = in(description).operator T(); }
      }
    };
    ops.buildForm =
        [buildForm = ops.buildForm, description, choices](const void * self, Operations::FormElements & form)
    {
      buildForm(self, form);
      const T & value = static_cast<const Schema *>(self)->*ptr;
      details::addValueToForm<T, IsRequired, IsInteractive>(value, description, choices, form);
    };
    ops.areEqual = [areEqual = ops.areEqual](const void * lhs, const void * rhs)
    {
      const T & lhs_value = static_cast<const Schema *>(lhs)->*ptr;
      const T & rhs_value = static_cast<const Schema *>(rhs)->*ptr;
      return areEqual(lhs, rhs) && (lhs_value == rhs_value);
    };
  }

  inline operator const T &() const noexcept { return value_; }

  T value_;
};

/** Schema for a given type
 *
 * Members are added at the declaration site
 */
template<typename T>
struct Schema : public Operations
{
  /** Tag to detect Schema objects */
  using is_schema_t = std::true_type;

  void save(mc_rtc::Configuration & out) const { Operations::save(this, out); }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_map(schema_size);
    Operations::write(this, builder);
    builder.finish_map();
  }

  std::string dump(bool pretty, bool yaml) const
  {
    mc_rtc::Configuration out;
    save(out);
    return out.dump(pretty, yaml);
  }

  void load(const Configuration & in) { Operations::load(this, in); }

  template<typename Callback = std::function<void()>>
  void addToGUI(
      mc_rtc::gui::StateBuilder & gui,
      const std::vector<std::string> & category,
      const std::string & name,
      Callback callback = []() {})
  {
    auto form = mc_rtc::gui::Form(name,
                                  [this, callback](const mc_rtc::Configuration & cfg)
                                  {
                                    // FIXME If callback takes a Schema<T> do a copy instead of self modification
                                    Operations::loadForm(this, cfg);
                                    callback();
                                  });
    Operations::buildForm(this, form);
    gui.addElement(category, form);
  }

  bool operator==(const Schema<T> & rhs) const { return Operations::areEqual(this, &rhs); }
  bool operator!=(const Schema<T> & rhs) const { return !Operations::areEqual(this, &rhs); }
};

/** Declare a Schema<T> member of type TYPE, specify REQUIRED and DEFAULT value */
#define SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, DEFAULT, ...)                          \
  TYPE NAME = mc_rtc::schema::Value<TYPE>                                                          \
  {                                                                                                \
    *this, mc_rtc::schema::details::MemberPointerWrapper<&T::NAME>{}, #NAME, DESCRIPTION, DEFAULT, \
        std::integral_constant<mc_rtc::schema::ValueFlag, REQUIRED>{}, ##__VA_ARGS__               \
  }

/** Declare a required Schema<T> member of type TYPE, only specify DEFAULT */
#define SCHEMA_REQUIRED_MEMBER(T, TYPE, NAME, DESCRIPTION, DEFAULT, ...) \
  SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, true, ##__VA_ARGS__)

/** Declare an optional Schema<T> member of type TYPE, only specify DEFAULT */
#define SCHEMA_OPTIONAL_MEMBER(T, TYPE, NAME, DESCRIPTION, DEFAULT, ...) \
  SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, false, ##__VA_ARGS__)

/** Declare a Schema<T> member of type TYPE with a default value, only specify REQUIRED */
#define SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, ...) \
  SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, mc_rtc::schema::details::Default<TYPE>::value, ##__VA_ARGS__)

/** Declare a required Schema<T> member of type TYPE with a default value */
#define SCHEMA_REQUIRED_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, true, ##__VA_ARGS__)

/** Declare an optional Schema<T> member of type TYPE with a default value */
#define SCHEMA_OPTIONAL_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, false, ##__VA_ARGS__)

/** The following alias make working with schema a little simpler */
static inline constexpr ValueFlag Required = ValueFlag::Required;

static inline constexpr ValueFlag Interactive = ValueFlag::Interactive;

static inline constexpr ValueFlag None = ValueFlag::None;

using Choices = details::Choices<true>;

} // namespace mc_rtc::schema

template<typename T>
struct fmt::formatter<mc_rtc::schema::Value<T>> : formatter<T>
{
};
