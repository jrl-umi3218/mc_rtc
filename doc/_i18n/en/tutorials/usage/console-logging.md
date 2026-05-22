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
