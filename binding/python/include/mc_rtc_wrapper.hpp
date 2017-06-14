#include <functional>

#include <Eigen/Core>
#include <SpaceVecAlg/SpaceVecAlg>

template<typename T>
using function = std::function<T()>;

template<typename retT, typename T, typename U>
std::function<retT()> make_log_callback(T fn, U arg)
{
  return [fn, arg]{ return fn(arg); };
};

#define MAKE_LOG_HELPER(NAME, TYPE)\
  template<typename T, typename U>\
  std::function<TYPE()> NAME(T fn, U arg)\
  {\
    return make_log_callback<TYPE>(fn ,arg);\
  }
MAKE_LOG_HELPER(make_v3d_log_callback, Eigen::Vector3d)
MAKE_LOG_HELPER(make_double_log_callback, double)
MAKE_LOG_HELPER(make_doublev_log_callback, std::vector<double>)
MAKE_LOG_HELPER(make_quat_log_callback, Eigen::Quaterniond)
MAKE_LOG_HELPER(make_pt_log_callback, sva::PTransformd)
MAKE_LOG_HELPER(make_fv_log_callback, sva::ForceVecd)
MAKE_LOG_HELPER(make_string_log_callback, std::string)
