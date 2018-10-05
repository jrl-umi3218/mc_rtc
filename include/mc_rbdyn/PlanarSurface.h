#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct Robot;

struct PlanarSurfaceImpl;

struct MC_RBDYN_DLLAPI PlanarSurface : public Surface
{
  PlanarSurface(const std::string & name,
                const std::string & bodyName,
                const sva::PTransformd & X_b_s,
                const std::string & materialName,
                const std::vector<std::pair<double, double>> & planarPoints);

  ~PlanarSurface();

  void computePoints() override;

  void planarTransform(const double & T, const double & B, const double & N_rot);

  const std::vector<std::pair<double, double>> & planarPoints() const;

  void planarPoints(const std::vector<std::pair<double, double>> & planarPoints);

  std::shared_ptr<Surface> copy() const override;

  std::string type() const override;

public:
private:
  std::unique_ptr<PlanarSurfaceImpl> impl;
};

} // namespace mc_rbdyn
