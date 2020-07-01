/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/eigen_traits.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/type_name.h>
#include <iostream>
#include <type_traits>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <boost/program_options.hpp>
namespace po = boost::program_options;

void usage(char * name)
{
  std::cerr << name << " log1.bin log2.bin [entry1 entry2 ...]\n";
}

template<typename DerivedA, typename DerivedB>
bool allclose(
    const Eigen::DenseBase<DerivedA> & a,
    const Eigen::DenseBase<DerivedB> & b,
    const typename DerivedA::RealScalar & rtol = Eigen::NumTraits<typename DerivedA::RealScalar>::dummy_precision(),
    const typename DerivedA::RealScalar & atol = Eigen::NumTraits<typename DerivedA::RealScalar>::epsilon())
{
  return ((a.derived() - b.derived()).array().abs() <= (atol + rtol * b.derived().array().abs())).all();
}

template<
    typename T,
    typename std::enable_if<!std::is_arithmetic<T>::value && !mc_rtc::internal::is_eigen_matrix<T>::value
                                && !std::is_same<T, sva::MotionVecd>::value && !std::is_same<T, sva::ForceVecd>::value,
                            int>::type = 0>
bool compare_elem(const T &, const T &, double)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("Comparison for type {} hasn't been implemented",
                                                   mc_rtc::type_name<T>());
}

template<typename T,
         typename std::enable_if<std::is_arithmetic<T>::value && !mc_rtc::internal::is_eigen_matrix<T>::value,
                                 int>::type = 0>
bool compare_elem(const T & e1, const T & e2, double tolerance)
{
  return std::fabs(e1 - e2) < tolerance;
}

template<typename T,
         typename std::enable_if<mc_rtc::internal::is_eigen_matrix<T>::value && !std::is_arithmetic<T>::value,
                                 int>::type = 0>
bool compare_elem(const T & e1, const T & e2, double tolerance)
{
  return allclose(e1, e2, tolerance, tolerance);
}

template<>
bool compare_elem(const Eigen::Quaterniond & e1, const Eigen::Quaterniond & e2, double tolerance)
{
  return allclose(e1.matrix(), e2.matrix(), tolerance, tolerance);
}

template<>
bool compare_elem(const bool & e1, const bool & e2, double)
{
  return e1 == e2;
}

template<>
bool compare_elem(const std::string & e1, const std::string & e2, double)
{
  return e1 == e2;
}

template<typename T,
         typename std::enable_if<std::is_same<T, sva::MotionVecd>::value || std::is_same<T, sva::ForceVecd>::value,
                                 int>::type = 0>
bool compare_elem(const T & e1, const T & e2, double tolerance)
{
  return allclose(e1.vector(), e2.vector(), tolerance, tolerance);
}

template<>
bool compare_elem(const sva::PTransformd & e1, const sva::PTransformd & e2, double tolerance)
{
  return sva::transformError(e1, e2).vector().norm() < 6 * tolerance;
}

template<>
bool compare_elem(const std::vector<double> & e1, const std::vector<double> & e2, double tolerance)
{
  if(e1.size() != e2.size())
  {
    return false;
  }
  for(size_t i = 0; i < e1.size(); ++i)
  {
    if(!compare_elem(e1[i], e2[i], tolerance))
    {
      return false;
    }
  }
  return true;
}

template<typename T>
bool compare(const std::string & entry,
             const std::vector<const T *> & v1,
             const std::vector<const T *> & v2,
             double tolerance = 1e-10)
{
  if(v1.size() != v2.size())
  {
    mc_rtc::log::error("Entry {} requires the same number of elements", entry);
    return false;
  }
  bool res = true;
  for(size_t i = 0; i < v1.size(); ++i)
  {
    const T * elem1 = v1[i];
    const T * elem2 = v2[i];
    if(elem1 == nullptr || elem2 == nullptr)
    {
      res = res && (elem1 == nullptr && elem2 == nullptr);
    }
    else
    {
      bool equal = compare_elem<T>(*elem1, *elem2, tolerance);
      if(!equal)
      {
        // TODO implement fmt::formatter for all types including eigen to
        // display the element value generically
        // mc_rtc::log::info("Comparation failed for entry {} at element {}: {} != {} (tolerance {})", entry, i, *elem1,
        // *elem2, tolerance);
        mc_rtc::log::info("Comparaison failed for entry {} at index {} (tolerance {})", entry, i, tolerance);
      }
      res = res && equal;
    }
    if(res == false) return false;
  }
  return res;
}

bool compare_entry(const mc_rtc::log::FlatLog & log1, const mc_rtc::log::FlatLog & log2, const std::string & entry)
{
  if(!log1.has(entry) || !log2.has(entry))
  {
    mc_rtc::log::error("Entry {} must exist in both log files", entry);
    return false;
  }
  auto types1 = log1.type(entry);
  auto types2 = log2.type(entry);
  if(types1 != types2)
  {
    mc_rtc::log::error("Types for entry {} must match", entry);
    return false;
  }

  bool res = false;
#define COMPARE(LOGT)                                                         \
  case mc_rtc::log::LogType::LOGT:                                            \
  {                                                                           \
    using type = mc_rtc::log::GetType<mc_rtc::log::LogType::LOGT>::type;      \
    res = compare(entry, log1.getRaw<type>(entry), log2.getRaw<type>(entry)); \
    break;                                                                    \
  }
  switch(types1)
  {
    COMPARE(Quaterniond)
    COMPARE(Vector2d)
    COMPARE(Vector3d)
    COMPARE(Vector6d)
    COMPARE(VectorXd)
    COMPARE(Int8_t)
    COMPARE(Int16_t)
    COMPARE(Int32_t)
    COMPARE(Int64_t)
    COMPARE(Uint8_t)
    COMPARE(Uint16_t)
    COMPARE(Uint32_t)
    COMPARE(Uint64_t)
    COMPARE(Float)
    COMPARE(Double)
    COMPARE(Bool)
    COMPARE(String)
    COMPARE(MotionVecd)
    COMPARE(ForceVecd)
    COMPARE(PTransformd)
    COMPARE(VectorDouble)
    case mc_rtc::log::LogType::None:
    default:
      mc_rtc::log::error("Comparison of unsupported type {} requested for entry {}", mc_rtc::log::LogTypeName(types1),
                         entry);
      res = false;
      break;
  }
#undef COMPARE

  if(res == false)
  {
    mc_rtc::log::error("Comparison for entry {} failed", entry);
  }
  return res;
}

int main(int argc, char * argv[])
{
  po::variables_map vm;
  po::options_description tool("mc_bin_compare");
  // clang-format off
  tool.add_options()
    ("help", "Produce this message")
    ("log1", po::value<std::string>(), "First log file")
    ("log2", po::value<std::string>(), "Second log file")
    ("entries", po::value<std::vector<std::string>>(), "Entries")
    ("tolerance", po::value<double>()->default_value(1e-10), "Tolerance threshold");
  // clang-format on
  po::positional_options_description pos;
  pos.add("log1", 1);
  pos.add("log2", 1);
  pos.add("entries", -1);
  po::store(po::command_line_parser(argc, argv).options(tool).positional(pos).run(), vm);
  po::notify(vm);
  if(vm.count("help") || !vm.count("log1") || !vm.count("log2"))
  {
    std::cout << "Usage: mc_bin_compare [options] log1 log2 [entries...]\n\n";
    std::cout << tool << "\n";
    return 1;
  }
  std::vector<std::string> entries;
  if(vm.count("entries"))
  {
    entries = vm["entries"].as<std::vector<std::string>>();
  }
  auto file1 = vm["log1"].as<std::string>();
  auto file2 = vm["log2"].as<std::string>();
  auto tolerance = vm["tolerance"].as<double>();

  if(!bfs::exists(file1))
  {
    mc_rtc::log::error("File {} does not exist", file1);
    return 1;
  }
  if(!bfs::exists(file2))
  {
    mc_rtc::log::error("File {} does not exist", file2);
    return 1;
  }
  mc_rtc::log::FlatLog log1(file1);
  mc_rtc::log::FlatLog log2(file2);
  if(entries.empty())
  {
    mc_rtc::log::info("No entry specified, assuming comparison between all entries");
    for(const auto & entry : log1.entries())
    {
      entries.push_back(entry);
    }
  }
  mc_rtc::log::info("Comparing:\n"
                    "- Logs     : {} and {} with:\n"
                    "- Tolerance: {}\n"
                    "- Entries  : {}",
                    file1, file2, tolerance, mc_rtc::io::to_string(entries));

  bool success = true;
  for(const auto & entry : entries)
  {
    success = success && compare_entry(log1, log2, entry);
  }

  if(success)
  {
    mc_rtc::log::success("Comparison successful");
    return 0;
  }
  else
  {
    mc_rtc::log::error("Comparison failed: one or more log entries differ");
    return 1;
  }
}
