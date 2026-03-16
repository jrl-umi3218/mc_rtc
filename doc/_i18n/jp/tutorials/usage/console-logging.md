mc\_rtcでは、いくつかのロギングユーティリティも用意されています。これらのユーティリティは、 `<mc_rtc/logging.h>`で定義されています。これらは、コンソールに特定の情報を表示するシンプルなマクロで、プラットフォームにかかわらず使用できます。以下のマクロが用意されています。
- `mc_rtc::log::success(...)`は、成功を示すメッセージを`std::cout`に表示します。
- `mc_rtc::log::info(...)`は、ユーザーへの情報を`std::cout`に表示します。
- `mc_rtc::log::warning(...)`は、致命的ではないエラーが発生したことを示す警告を`std::cerr`に表示します。
- `mc_rtc::log::error(...)`は、エラーメッセージを`std::cerr`に表示します。
- `mc_rtc::log::error_and_throw<T = std::runtime_errror>(...)`は`std::cerr`に致命的なエラーが発生した旨のメッセージを表示し、`T`型 (デフォルトは `std::runtime_error`)の例外を発生させます。

これらの関数では`std::cout`と`std::cerr`以外も使用できますが、重要なメッセージをコンソールに表示する際はこれらを使用すると便利です。

例:
```cpp
mc_rtc::log::success("Great success");
mc_rtc::log::info("You don't need a ; after those instructions");
mc_rtc::log::warning("The value of d is greater than expected: {} (d_limit: {})", d, d_limit);
mc_rtc::log::error("This is very wrong");
//  std::runtime_error("Abort")を発生させる
mc_rtc::log::error_and_throw("Abort");
// std::domain_error("Abort")を発生させる
mc_rtc::log::error_and_throw<std::domain_error>("Abort");
```

[{fmt}ライブラリ](https://fmt.dev/dev/syntax.html)でサポートされている書式設定構文を使用できます。

## 行列型についての注意

行列型（`Eigen::Matrix`, `Eigen::Vector`, `sva::PTransformd` など）は、[{eigen-fmt}](https://gite.lirmm.fr/rpc/utils/eigen-fmt) ライブラリに基づいたカスタム `{fmt}` フォーマッタでフォーマットされます。

### Eigen

`Eigen` 型（行列、ベクトルなど）は `{eigen-fmt}` フォーマッタを使用してフォーマットされ、表示方法を制御するための修飾子を提供します。詳細は [{eigen-fmt} ドキュメント](https://rpc.lirmm.net/rpc-framework/packages/eigen-fmt/index.html) を参照してください。

例：

```cpp
auto m1 = Eigen::Matrix3d::Random();

// 式の表示方法を制御するために fmt 型の修飾子を使用できます
mc_rtc::log::info("Matrix m1:\n{:noal;csep{, };rsep{, };mpre{ << };msuf{;}}", m1);
```

参考までに、`Eigen` 型で利用可能な修飾子（`eigen-fmt v1.0.4` 時点）は以下の通りです。すべてのパラメータはセミコロンで区切ります。利用可能なパラメータは：

- `t`: 転置（デフォルト = `false`）
- `noal`: 列を揃えない（デフォルト = `false`）
- `p{int/str}`:
  - `int`: 固定精度（デフォルト = `Eigen::StreamPrecision`）
  - `'f'` または `'s'`: 完全精度（`Eigen::FullPrecision`）またはストリーム精度（`Eigen::StreamPrecision`）
- `csep{str}`: 要素の区切り文字（デフォルト = `" "`）
- `rsep{str}`: 行の区切り文字（デフォルト = `"\n"`）
- `rpre{str}`: 行の接頭辞（デフォルト = `""`）
- `rsuf{str}`: 行の接尾辞（デフォルト = `""`）
- `mpre{str}`: 行列の接頭辞（デフォルト = `""`）
- `msuf{str}`: 行列の接尾辞（デフォルト = `""`）

```cpp
// より明示的な EigenFmt::FormatSpec 構造体を使用することもできます
EigenFmt::FormatSpec clean_format;
clean_format.precision = 4;
clean_format.coeff_sep = ", ";
clean_format.row_prefix = "[";
clean_format.row_suffix = "]";

// str() 関数は実行時コストがあるため、結果をキャッシュすることを推奨します
const auto octave_format_str = clean_format.str();
mc_rtc::log::info("Matrix m1:\n{:{}}", m1, octave_format_str);
```

### PTransformd

`sva::PTransformd` には、個々のデータフィールドを `{eigen-fmt}` フォーマッタで詳細に表示する、やや独自のフォーマッタを提供しています：
- translation（並進）
- rotation（回転）:
  - `sva` の慣習（Featherstone、左手系回転行列）による行列
  - 標準的な慣習（右手系回転行列）による行列
  - Featherstone/標準の両方で度・ラジアンのRPY

`{eigen-fmt}` 修飾子を使って行列やベクトルの表示方法を制御できますが、出力の全体構造は固定されています。ただし、`.matrix()`, `.vector()`, `.translation()`, `.rotation()` などの生データフィールドを手動で表示することも可能です（Eigen型なので `{eigen-fmt}` の柔軟性を活用できます）。

例：

```cpp
  sva::PTransformd T(sva::RotZ(M_PI / 4), Eigen::Vector3d(1, 2, 3));
  fmt::print("T:\n{}", T);
```

`PTransformd` は以下のデフォルトスタイルで表示されます：

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

また、以下のようにカスタマイズすることもできます：

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

出力例：

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
