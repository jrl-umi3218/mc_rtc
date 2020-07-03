/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_planning/PreviewWindow.h>

namespace mc_planning
{

PreviewWindowView CenteredPreviewWindow::all(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, Index{0}, fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::all(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), Index{0}, fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::all() const noexcept
{
  return all(Index{0});
}

PreviewWindowView CenteredPreviewWindow::past(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, Index{0}, halfSize() - 1u};
}

PreviewWindowView CenteredPreviewWindow::past(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), Index{0}, halfSize() - 1u};
}

PreviewWindowView CenteredPreviewWindow::past() const noexcept
{
  return past(Index{0});
}

PreviewWindowView CenteredPreviewWindow::pastInclusive(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, Index{0}, Index{halfSize()}};
}

PreviewWindowView CenteredPreviewWindow::pastInclusive(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), Index{0}, Index{halfSize()}};
}

PreviewWindowView CenteredPreviewWindow::pastInclusive() const noexcept
{
  return pastInclusive(Index{0});
}

PreviewWindowView CenteredPreviewWindow::future(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, halfSize() + 1u, fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::future(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), halfSize() + 1u, fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::future() const noexcept
{
  return future(Index{0});
}

PreviewWindowView CenteredPreviewWindow::futureInclusive(Index startIndex) const noexcept
{
  return PreviewWindowView{*this, startIndex, halfSize(), fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::futureInclusive(Time startTime) const noexcept
{
  return PreviewWindowView{*this, index(startTime), halfSize(), fullSize_ - 1u};
}

PreviewWindowView CenteredPreviewWindow::futureInclusive() const noexcept
{
  return futureInclusive(Index{0});
}

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
