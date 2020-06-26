/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/PreviewWindow.h>

namespace mc_planning
{

unsigned PreviewElement::index() const noexcept
{
  return index_;
}

double PreviewElement::time() const noexcept
{
  return window_.timeFromIndex(index_);
}

unsigned PreviewElement::localIndex() const noexcept
{
  return index_ - window_.startIndex();
}

double PreviewElement::localTime() const noexcept
{
  return window_.timeFromIndex(index_ - window_.startIndex());
}

} // namespace mc_planning
