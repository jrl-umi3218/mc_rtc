mc\_rtc also provides some console logging utility which are available in
`<mc_rtc/logging.h>`. Those are simple macros that allows to highlight
information in the console in a cross-platform way. The following macros are
available:
- `mc_rtc::log::success(...)` displays a message on `std::cout` indicating success
- `mc_rtc::log::info(...)` display a message on `std::cout` to provide information to the user
- `mc_rtc::log::warning(...)` display a message on `std::cerr` to warn the user of a non-fatal error
- `mc_rtc::log::error(...)` display a message on `std::cerr` to show an error message
- `mc_rtc::log::error_and_throw<T = std::runtime_errror>(...)` display a message on `std::cerr` to show a critical error message and throw an exception of type `T` (defaults to `std::runtime_error`)

You are not forced to use these functions to use `std::cout` and `std::cerr`
but this helps to highlight important messages on the console.

Example:
```cpp
mc_rtc::log::success("Great success");
mc_rtc::log::info("You don't need a ; after those instructions");
mc_rtc::log::warning("The value of d is greater than expected: {} (d_limit: {})", d, d_limit);
mc_rtc::log::error("This is very wrong");
// Throw std::runtime_error("Abort")
mc_rtc::log::error_and_throw("Abort");
// Throw std::domain_error("Abort")
mc_rtc::log::error_and_throw<std::domain_error>("Abort");
```

The formatting syntax is the syntax supported by the [{fmt} library](https://fmt.dev/dev/syntax.html).

## Note on Matrix types

Matrix-like types (`Eigen::Matrix`, `Eigen::Vector`, `sva::PTransformd`, etc) are formatted with custom `{fmt}` formatters based on the [{eigen-fmt}](https://gite.lirmm.fr/rpc/utils/eigen-fmt) library.

### Eigen

`Eigen` types (matrices, vectors, etc) are formatted using the `{eigen-fmt}` formatters, which provides modifiers to control how they are displayed. Please refer to [{eigen-fmt} documentation](https://rpc.lirmm.net/rpc-framework/packages/eigen-fmt/index.html) for details.

For example:

```cpp
auto m1 = Eigen::Matrix3d::Random();

// Either use fmt type modifiers to control how to display expressions
mc_rtc::log::info("Matrix m1:\n{:noal;csep{, };rsep{, };mpre{ << };msuf{;}}", m1);
```

For quick reference, the following modifiers are available for `Eigen` types (as of `eigen-fmt v1.0.4`). All parameters must be separated by a semicolon. The possible parameters are:

- `t`: transpose (default = `false`)
- `noal`: don't align columns (default = `false`)
- `p{int/str}`:
  - `int`: fixed precision (default = `Eigen::StreamPrecision`)
  - `'f'` or `'s'`: full precision (`Eigen::FullPrecision`) or stream precision (`Eigen::StreamPrecision`)
- `csep{str}`: coefficient separator (default = `" "`)
- `rsep{str}`: row separator (default = `"\n"`)
- `rpre{str}`: row prefix (default = `""`)
- `rsuf{str}`: row suffix (default = `""`)
- `mpre{str}`: matrix prefix (default = `""`)
- `msuf{str}`: matrix suffix (default = `""`)



```
// Or use the more explicit EigenFmt::FormatSpec struct
EigenFmt::FormatSpec clean_format;
clean_format.precision = 4;
clean_format.coeff_sep = ", ";
clean_format.row_prefix = "[";
clean_format.row_suffix = "]";

// The str() function has some runtime cost, consider caching its result
const auto octave_format_str = clean_format.str();
mc_rtc::log::info("Matrix m1:\n{:{}}", m1, octave_format_str);
```

### PTransformd

We provide a somewhat opinionated formatter for `sva::PTransformd` that details individual data fields using `{eigen-fmt}` formatters:
- translation
- rotation as:
  - matrix in `sva`'s convention (Featherstone, left-handed rotation matrices)
  - matrix in `standard` convention (right-handed rotation matrices)
  - RPY in both degree/radian in both featherstone and standard convention

`{eigen-fmt}` modifiers can be used to control how the matrices and vectors are displayed, but the overall structure of the output is fixed. However one can always display the raw data fields manually using `.matrix()`, `.vector()`, `.translation()`, `.rotation()` (which for eigen types benefit from the flexibility of `{eigen-fmt}`).

For example:

```cpp
  sva::PTransformd T(sva::RotZ(M_PI / 4), Eigen::Vector3d(1, 2, 3));
  fmt::print("T:\n{}", T);
```

displays a `PTransformd` with the following default style:

```
T:
  translation: [1, 2, 3]
  rotation:
    - matrix (Featherstone, left-handed convention):
      [              0.707,               0.707,                   0]
      [             -0.707,               0.707,                   0]
      [                  0,                   0,                   1]
    - matrix (Standard,     right-handed convention):
      [              0.707,              -0.707,                   0]
      [              0.707,               0.707,                   0]
      [                  0,                   0,                   1]
    - rpy (Featherstone): [0, -0, 0.785] (rad), [0, -0, 45] (deg)
    - rpy (standard):     [0, -0, -0.785] (rad), [0, -0, -45] (deg)
```

or you can customize it as follows:

```cpp
  EigenFmt::FormatSpec clean_format;
  clean_format.precision = 7;
  clean_format.coeff_sep = ", ";
  clean_format.row_prefix = "      ";
  clean_format.row_suffix = " ";
  clean_format.mat_prefix = "    [";
  clean_format.mat_suffix = "]";
  clean_format.dont_align_cols = true;
  const auto eigen_matrix_str = clean_format.str();

  clean_format.row_prefix = "";
  clean_format.row_suffix = "";
  clean_format.transpose = true;
  std::string eigen_vector_str = clean_format.str();
  fmt::print("T:\n{:{}{}}", T, eigen_vector_str.c_str(), eigen_matrix_str.c_str());
```

gives

```
T:
translation:     [1, 2, 3]
rotation:
  - matrix (Featherstone, left-handed convention):
    [      0.7071068, 0.7071068, 0
      -0.7071068, 0.7071068, 0
      0, 0, 1 ]
  - matrix (Standard,     right-handed convention):
    [      0.7071068, -0.7071068, 0
      0.7071068, 0.7071068, 0
      0, 0, 1 ]
  - rpy (Featherstone):     [0, -0, 0.7853982] (rad),     [0, -0, 45] (deg)
  - rpy (standard):         [0, -0, -0.7853982] (rad),     [0, -0, -45] (deg)
```
