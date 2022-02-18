{% comment %} FIXME Documentation for log::error_and_throw out-of-date {% endcomment %}

mc\_rtcでは、いくつかのロギングユーティリティも用意されています。これらのユーティリティは、 `<mc_rtc/logging.h>`で定義されています。これらは、コンソールに特定の情報を表示するシンプルなマクロで、プラットフォームにかかわらず使用できます。以下のマクロが用意されています。
- `mc_rtc::log::success(...)`は、成功を示すメッセージを`std::cout`に表示します。
- `mc_rtc::log::info(...)`は、ユーザーへの情報を`std::cout`に表示します。
- `mc_rtc::log::warning(...)`は、致命的ではないエラーが発生したことを示す警告を`std::cerr`に表示します。
- `mc_rtc::log::error(...)`は、エラーメッセージを`std::cerr`に表示します。
- `mc_rtc::log::error_and_throw<T>(...)`は、致命的なエラーが発生してタイプ`T`の例外がスローされたことを示すメッセージを`std::cerr`に表示します。

これらの関数では`std::cout`と`std::cerr`以外も使用できますが、重要なメッセージをコンソールに表示する際はこれらを使用すると便利です。

例:
```cpp
mc_rtc::log::success("Great success");
mc_rtc::log::info("You don't need a ; after those instructions");
mc_rtc::log::warning("The value of d is greater than expected: {} (d_limit: {})", d, d_limit);
mc_rtc::log::error("This is very wrong");
mc_rtc::log::error_and_throw<std::runtime_exception>("Abort");
```

[{fmt}ライブラリ](https://fmt.dev/dev/syntax.html)でサポートされている書式設定構文を使用できます。
