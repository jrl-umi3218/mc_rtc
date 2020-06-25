/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/PreviewWindow.h>

namespace mc_planning
{

double PreviewElement::time() const noexcept
{
  return window_.timeFromIndex(index_);
}

} // namespace mc_planning
