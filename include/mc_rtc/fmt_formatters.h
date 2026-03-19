/*
 * Copyright 2026 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <SpaceVecAlg/PTransform.h>
#include <fmt/format.h>

// fmt 9.0.0 removed automated operator<< discovery we use fmt::streamed instead when needed through a macro
#if FMT_VERSION >= 9 * 10000
#  define MC_FMT_STREAMED(X) fmt::streamed(X)
#  include <boost/filesystem.hpp>
#  include <eigen-fmt/fmt.h>
#  include <filesystem>
#  include <fmt/ostream.h>

// Formatter for boost::filesystem::path
template<>
struct fmt::formatter<boost::filesystem::path> : fmt::formatter<std::string>
{
  template<typename FormatContext>
  auto format(const boost::filesystem::path & p, FormatContext & ctx)
  {
    return fmt::formatter<std::string>::format(p.string(), ctx);
  }
};

// Formatter for std::filesystem::path
template<>
struct fmt::formatter<std::filesystem::path> : fmt::formatter<std::string>
{
  template<typename FormatContext>
  auto format(const std::filesystem::path & p, FormatContext & ctx)
  {
    return fmt::formatter<std::string>::format(p.string(), ctx);
  }
};

/**
 * @brief Custom formatter for sva::PTransformd using fmtlib.
 *
 * Usage:
 *   - Default formatting:
 *       fmt::print("{}", ptransform);
 *     Uses built-in vector and matrix formatting options.
 *
 *   - Custom formatting:
 *       fmt::print("T: {:{}{}}", ptransform, vector_fmt, matrix_fmt);
 *     Where:
 *       - vector_fmt: format string for vectors (e.g. translation, rpy)
 *       - matrix_fmt: format string for matrices (e.g. rotation)
 *     Example:
 *       std::string vector_fmt = "t;noal;p{4};csep{, };rpre{[};rsuf{]}";
 *       std::string matrix_fmt = "p{4};csep{, };rpre{      [};rsuf{]};rsep{\n}";
 *       fmt::print("T: {:{}{}}", ptransform, vector_fmt, matrix_fmt);
 *
 *   - Supported format string options (see eigen-fmt docs):
 *       - p{N}: precision
 *       - csep{sep}: coefficient separator
 *       - rsep{sep}: row separator
 *       - rpre{prefix}: row prefix
 *       - rsuf{suffix}: row suffix
 *       - t: transpose
 *       - noal: no column alignment
 *
 * The formatter will use the provided format strings for vectors and matrices,
 * or fall back to sensible defaults if not provided.
 */
template<>
struct fmt::formatter<sva::PTransformd>
{
  int arg_id_vec = -1, arg_id_mat = -1;
  // Default format strings
  const std::string default_vec_fmt = "t;noal;p{3};csep{, };rpre{[};rsuf{]}";
  const std::string default_mat_fmt = "p{3};csep{, };rpre{    [};rsuf{]};rsep{\n}";

  // Parse for up to two nested arguments: {:{}} or {:{}{}}
  // arg(1) is formatting for vectors
  // arg(2) is formatting for matrices
  auto parse(format_parse_context & ctx)
  {
    auto it = ctx.begin(), end = ctx.end();
    if(it == end || *it == '}') return it;
    if(*it == '{')
    {
      ++it;
      arg_id_vec = ctx.next_arg_id();
      if(it != end && *it == '}')
      {
        ++it;
        // Check for a second nested argument
        if(it != end && *it == '{')
        {
          ++it;
          arg_id_mat = ctx.next_arg_id();
          if(it != end && *it == '}') { ++it; }
        }
      }
      return it;
    }
    throw format_error("invalid format");
  }

  template<typename FormatContext>
  auto format(const sva::PTransformd & pt, FormatContext & ctx) const
  {
    // Get user format strings or use defaults
    std::string vec_fmt = default_vec_fmt;
    std::string mat_fmt = default_mat_fmt;

    if(arg_id_vec != -1)
    {
      vec_fmt = fmt::visit_format_arg(
          [](auto val) -> std::string
          {
            if constexpr(std::is_same_v<std::decay_t<decltype(val)>, const char *>) { return std::string(val); }
            else
            {
              return "";
            }
          },
          ctx.arg(arg_id_vec));
      if(vec_fmt.empty()) vec_fmt = default_vec_fmt;
    }
    if(arg_id_mat != -1)
    {
      mat_fmt = fmt::visit_format_arg(
          [](auto && val) -> std::string
          {
            if constexpr(std::is_same_v<std::decay_t<decltype(val)>, const char *>) { return std::string(val); }
            else
            {
              return "";
            }
          },
          ctx.arg(arg_id_mat));
      if(mat_fmt.empty()) mat_fmt = default_mat_fmt;
    }

    const auto & rotation_ft = pt.rotation();
    auto rotation_std = pt.rotation().transpose();

    auto rpy_ft = mc_rbdyn::rpyFromMat(rotation_ft);
    auto rpy_ft_deg = rpy_ft * 180. / mc_rtc::constants::PI;

    auto rpy_std = mc_rbdyn::rpyFromMat(rotation_std);
    auto rpy_std_deg = rpy_std * 180. / mc_rtc::constants::PI;

    return fmt::format_to(ctx.out(),
                          "translation: {:{}}\n"
                          "rotation:\n"
                          "  - matrix (Featherstone, left-handed convention):\n{:{}}\n"
                          "  - matrix (Standard,     right-handed convention):\n{:{}}\n"
                          "  - rpy (Featherstone): {:{}} (rad), {:{}} (deg)\n"
                          "  - rpy (standard):     {:{}} (rad), {:{}} (deg)\n",
                          pt.translation(), vec_fmt, rotation_ft, mat_fmt, rotation_std, mat_fmt, rpy_ft, vec_fmt,
                          rpy_ft_deg, vec_fmt, rpy_std, vec_fmt, rpy_std_deg, vec_fmt);
  }
};

#else
#  define MC_FMT_STREAMED(X) X
#endif
