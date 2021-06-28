# This script is meant to be sourced from other scripts

# Check for clang-format, prefer 10 if available
if [[ -x "$(command -v clang-format-10)" ]]; then
  clang_format=clang-format-10
elif [[ -x "$(command -v clang-format)" ]]; then
  clang_format=clang-format
else
  echo "clang-format or clang-format-10 must be installed"
  exit 1
fi

# Find all source files in the project minus those that are auto-generated or we do not maintain
src_files=`find benchmarks binding/python/include include src tests utils plugins -name '*.cpp' -or -name '*.h' -or -name '*.hpp'|grep -Ev "utils/mc_bin_flatbuffers/"|grep -Ev "include/mc_rtc/robin_hood/robin_hood.h"`
