#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/contact_transform.h>

namespace mc_rbdyn
{

/* Does not implement signum to mimic numpy copysign function */
template<typename T>
inline T sgn(const T & lhs)
{
  return lhs < 0 ? -1 : 1;
}

sva::PTransformd planar(const double & T, const double & B, const double & N_rot)
{
  return sva::PTransformd(sva::RotZ(N_rot), Eigen::Vector3d(T, B, 0));
}

sva::PTransformd cylindrical(const double & T, const double & T_rot)
{
  return sva::PTransformd(sva::RotX(T_rot), Eigen::Vector3d(T, 0, 0));
}

void planarParam(const sva::PTransformd & X_es_rs, double & T, double & B, double & N_rot)
{
  Eigen::Vector3d TEnv = Eigen::Vector3d::UnitX();
  Eigen::Vector3d BEnv = Eigen::Vector3d::UnitY();

  Eigen::Vector3d TRob = X_es_rs.rotation().row(0).transpose();

  T = TEnv.dot(X_es_rs.translation());
  B = BEnv.dot(X_es_rs.translation());

  double normalizedTeTr = std::min(std::max(TEnv.dot(TRob), -1.0), 1.0);
  N_rot = acos(normalizedTeTr) * sgn(TRob.dot(BEnv));
}

void cylindricalParam(const sva::PTransformd & X_es_rs, double & T, double & T_rot)
{
  Eigen::Vector3d TEnv = Eigen::Vector3d::UnitX();
  Eigen::Vector3d BEnv = Eigen::Vector3d::UnitY();

  Eigen::Vector3d BRob = X_es_rs.rotation().row(1).transpose();
  Eigen::Vector3d NRob = X_es_rs.rotation().row(2).transpose();

  T = TEnv.dot(X_es_rs.translation());

  double normalizedBeBr = std::min(std::max(BEnv.dot(BRob), -1.0), 1.0);
  T_rot = -1 * acos(normalizedBeBr) * sgn(NRob.dot(BEnv));
}

MC_RBDYN_DLLAPI std::vector<double> jointParam(const Surface & r1Surface,
                                               const Surface & r2Surface,
                                               const sva::PTransformd & X_es_rs)
{
  if(r1Surface.type() == "planar" && r2Surface.type() == "planar")
  {
    double T, B, N_rot;
    planarParam(X_es_rs, T, B, N_rot);
    auto rz = sva::RotZ(N_rot);
    Eigen::Vector3d trans(T, B, 0);
    auto transJoint = rz * trans;
    return {N_rot, transJoint.x(), transJoint.y()};
  }
  else if((r1Surface.type() == "planar" || r1Surface.type() == "gripper") && r2Surface.type() == "cylindrical")
  {
    double T, T_rot;
    cylindricalParam(X_es_rs, T, T_rot);
    return {T, T_rot};
  }
  return {};
}

} // namespace mc_rbdyn
