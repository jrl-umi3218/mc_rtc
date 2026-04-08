#include <mc_rtc/macros/pp_id.h>

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
  MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::All, DEFAULT, ##__VA_ARGS__))

/** Declare an optional Schema<T> member of type TYPE, only specify DEFAULT */
#define MC_RTC_SCHEMA_OPTIONAL_MEMBER(T, TYPE, NAME, DESCRIPTION, DEFAULT, ...)                                  \
  MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::Interactive, DEFAULT, \
                                    ##__VA_ARGS__))

/** Declare a Schema<T> member of type TYPE with a default value, only specify REQUIRED */
#define MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, ...) \
  MC_RTC_PP_ID(MC_RTC_SCHEMA_MEMBER(T, TYPE, NAME, DESCRIPTION, REQUIRED, mc_rtc::Default<TYPE>::value, ##__VA_ARGS__))

/** Declare a required Schema<T> member of type TYPE with a default value */
#define MC_RTC_SCHEMA_REQUIRED_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  MC_RTC_PP_ID(MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::All, ##__VA_ARGS__))

/** Declare an optional Schema<T> member of type TYPE with a default value */
#define MC_RTC_SCHEMA_OPTIONAL_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, ...) \
  MC_RTC_PP_ID(                                                                \
      MC_RTC_SCHEMA_DEFAULT_MEMBER(T, TYPE, NAME, DESCRIPTION, mc_rtc::schema::ValueFlag::Interactive, ##__VA_ARGS__))
