/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Surface.h>

namespace mc_rbdyn
{

struct Robot;

struct CylindricalSurfaceImpl;

struct MC_RBDYN_DLLAPI CylindricalSurface : public Surface
{
  CylindricalSurface(const std::string & name,
                     const std::string & bodyName,
                     const sva::PTransformd & X_b_s,
                     const std::string & materialName,
                     const double & radius,
                     const double & width);

  ~CylindricalSurface() override;

  /**
   * @brief Construct a CylindricalSurface from an XML element.
   *
   * Example XML:
   * @code{.xml}
   *  <cylindrical_surface name="Cylinder" link="some_link" radius="0.05" width="0.2">
   *    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0" />
   *    <material name="plastic" />
   *  </cylindrical_surface>
   * @endcode
   *
   * @param elem The XML element describing the cylindrical surface.
   * @return Unique pointer to the constructed CylindricalSurface.
   */
  static std::unique_ptr<CylindricalSurface> fromXML(const tinyxml2::XMLElement & elem);

  void computePoints() override;

  const double & radius() const;

  const double & width() const;

  void width(const double & width);

  std::shared_ptr<Surface> copy() const override;

  std::string type() const override;

  tinyxml2::XMLElement * toXML(tinyxml2::XMLDocument & doc) const override;

private:
  std::unique_ptr<CylindricalSurfaceImpl> impl;
};

} // namespace mc_rbdyn
