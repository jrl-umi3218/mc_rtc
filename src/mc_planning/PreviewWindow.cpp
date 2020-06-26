/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/PreviewWindow.h>

namespace mc_planning
{

PreviewWindowView CenteredPreviewWindow::all(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, Index{0}, fullSize_};
}

PreviewWindowView CenteredPreviewWindow::all(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), Index{0}, fullSize_};
}

// PreviewWindowView CenteredPreviewWindow::past(Index startIndex) const noexcept
// {
//   return PreviewWindowView{*this, startIndex, halfSize_};
// }

// PreviewWindowView CenteredPreviewWindow::future(Index startIndex) const noexcept
// {
//   return PreviewWindowView{*this, startIndex+halfSize_, startIndex+fullSize_};
// }

Index PreviewElement::index() const noexcept
{
  return index_;
}

Time PreviewElement::time() const noexcept
{
  return window_.time(Index(index_));
}

Index PreviewElement::localIndex() const noexcept
{
  return Index{index_ - window_.startIndex()};
}

Time PreviewElement::localTime() const noexcept
{
  return window_.time(Index(index_ - window_.startIndex()));
}

} // namespace mc_planning
