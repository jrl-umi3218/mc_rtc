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
