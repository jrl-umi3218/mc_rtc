/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <memory>
#include <string>
#include <tinyxml2.h>
#include <vector>

namespace mc_rbdyn
{

struct Surface;

/**
 * @brief Create an <origin> XML element from a spatial transform.
 *
 * @param doc The XML document to which the element will belong.
 * @param X The spatial transform (position and orientation). It will be represented as rpy and xyz attributes.
 * @param tagName The tag name for the element (default: "origin").
 * @return Pointer to the created tinyxml2::XMLElement.
 */
MC_RBDYN_DLLAPI tinyxml2::XMLElement * tfToOriginDom(tinyxml2::XMLDocument & doc,
                                                     const sva::PTransformd & X,
                                                     const std::string_view & tagName = "origin");
MC_RBDYN_DLLAPI sva::PTransformd tfFromOriginDom(const tinyxml2::XMLElement & dom);

/**
 * @brief Add a <robot> element and its surfaces to an XML document.
 *
 * @param doc The XML document to populate.
 * @param robotName The name of the robot.
 * @param surfaces The surfaces to add as children of the robot element.
 */
MC_RBDYN_DLLAPI void surfacesToXML(tinyxml2::XMLDocument & doc,
                                   const std::string & robotName,
                                   const std::vector<std::shared_ptr<Surface>> & surfaces);

/**
 * @brief Read all RSDF files from a directory and parse them into Surface objects.
 *
 * @param dirname The directory containing .rsdf files.
 * @return Vector of shared pointers to parsed Surface objects.
 */
MC_RBDYN_DLLAPI std::vector<std::shared_ptr<Surface>> readRSDFFromDir(const std::string & dirname);

} // namespace mc_rbdyn
