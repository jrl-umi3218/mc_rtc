/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>
#include <mc_rtc/gui/types.h>

namespace mc_rtc
{

namespace gui
{

/** Table should display a table of heterogeneous data
 *
 * The user is responsible for making sure the header size and data size match
 *
 * \tparam GetHeader should return the header information for the table
 *
 * \tparam GetData should return data that can be converted to a JSON array of
 * array (e.g. vector<vector<T>> or vector<tuple<...>>)
 */
template<typename GetHeader, typename GetData>
struct TableImpl : public Element
{
  static constexpr auto type = Elements::Table;

  TableImpl(const std::string & name, GetHeader get_header_fn, GetData get_data_fn)
  : Element(name), get_header_fn_(get_header_fn), get_data_fn_(get_data_fn)
  {
  }

  static constexpr size_t write_size()
  {
    return Element::write_size() + 2;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    Element::write(builder);
    builder.write(get_header_fn_());
    builder.write(get_data_fn_());
  }

private:
  GetHeader get_header_fn_;
  GetData get_data_fn_;
};

/** Table should display a table of heterogeneous data
 *
 * The user is responsible for making sure the header size and data size match
 *
 * In this variant, the header data is static
 *
 * \tparam GetData should return data that can be converted to a JSON array of
 * array (e.g. vector<vector<T>> or vector<tuple<...>>)
 */
template<typename GetData>
struct StaticTableImpl : public Element
{
  static constexpr auto type = Elements::Table;

  StaticTableImpl(const std::string & name, std::vector<std::string> header, GetData get_data_fn)
  : Element(name), header_(std::move(header)), get_data_fn_(get_data_fn)
  {
  }

  static constexpr size_t write_size()
  {
    return Element::write_size() + 2;
  }

  void write(mc_rtc::MessagePackBuilder & builder)
  {
    Element::write(builder);
    builder.write(header_);
    builder.write(get_data_fn_());
  }

private:
  std::vector<std::string> header_;
  GetData get_data_fn_;
};

/** Helper function to a get a StaticTableImpl */
template<typename GetData>
StaticTableImpl<GetData> Table(const std::string & name, std::vector<std::string> header, GetData get_data_fn)
{
  return StaticTableImpl<GetData>(name, std::move(header), get_data_fn);
}

/** Helper function to get a TableImpl */
template<typename GetHeader, typename GetData>
TableImpl<GetHeader, GetData> Table(const std::string & name, GetHeader get_header_fn, GetData get_data_fn)
{
  return TableImpl<GetHeader, GetData>(name, get_header_fn, get_data_fn);
}

} // namespace gui

} // namespace mc_rtc
