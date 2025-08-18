#pragma once

#if FMT_VERSION >= 9 * 10000

#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <fmt/ostream.h>

// Formatter for Eigen dense types (like Eigen::Matrix, Eigen::Array)
template<typename T>
struct fmt::formatter<
    T,
    std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<T>, T>, char>
> : fmt::ostream_formatter
{
};

// Formatter for Eigen::Transpose of a dense matrix type
template<typename MatrixType>
struct fmt::formatter<
    Eigen::Transpose<MatrixType>,
    std::enable_if_t<std::is_base_of_v<Eigen::DenseBase<MatrixType>, MatrixType>, char>
> : fmt::ostream_formatter
{
};

// Formatter for Eigen::Block
template<typename Scalar, int Rows, int Cols, int BlockRows, int BlockCols, bool InnerPanel>
struct fmt::formatter<
    Eigen::Block<Eigen::Matrix<Scalar, Rows, Cols>, BlockRows, BlockCols, InnerPanel>
> : fmt::ostream_formatter
{
};

// Formatter for Eigen::Diagonal
template<typename Scalar, int Rows, int Cols>
struct fmt::formatter<
    Eigen::Diagonal<Eigen::Matrix<Scalar, Rows, Cols>>
> : fmt::ostream_formatter
{
};

// Formatter for boost::filesystem::path
template <>
struct fmt::formatter<boost::filesystem::path> : fmt::formatter<std::string>
{
    template <typename FormatContext>
    auto format(const boost::filesystem::path& p, FormatContext& ctx)
    {
        return fmt::formatter<std::string>::format(p.string(), ctx);
    }
};

#endif
