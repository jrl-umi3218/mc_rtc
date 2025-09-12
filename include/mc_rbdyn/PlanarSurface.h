/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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

  /**
   * @brief Construct a PlanarSurface from an XML element.
   *
   * Example XML:
   * @code{.xml}
   *  <planar_surface name="Top" link="box_table">
   *    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.3632499873638153" />
   *    <points>
   *      <point xy="-0.17499999701976776 -0.2750000059604645" />
   *      <point xy="0.17499999701976776 -0.2750000059604645" />
   *      <point xy="0.17499999701976776 0.2750000059604645" />
   *      <point xy="-0.17499999701976776 0.2750000059604645" />
   *    </points>
   *    <material name="plastic" />
   *  </planar_surface>
   * @endcode
   *
   * @param elem The XML element describing the planar surface.
   * @return Unique pointer to the constructed PlanarSurface.
   */
  static std::unique_ptr<PlanarSurface> fromXML(const tinyxml2::XMLElement & elem);

  ~PlanarSurface() override;

  void computePoints() override;

  void planarTransform(const double & T, const double & B, const double & N_rot);

  const std::vector<std::pair<double, double>> & planarPoints() const;

  void planarPoints(const std::vector<std::pair<double, double>> & planarPoints);

  tinyxml2::XMLElement * toXML(tinyxml2::XMLDocument & doc) const override;

  std::shared_ptr<Surface> copy() const override;

  std::string type() const override;

public:
private:
  std::unique_ptr<PlanarSurfaceImpl> impl;
};

} // namespace mc_rbdyn
