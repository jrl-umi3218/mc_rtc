---
layout: tutorials
---

mc\_rtc also provides some console logging utility which are available in
`<mc_rtc/logging.h>`. Those are simple macros that allows to highlight
information in the console in a cross-platform way. The following macros are
available:
- `LOG_SUCCESS` displays a bright green message on `std::cout` indicating success
- `LOG_INFO` display a blue message on `std::cout` to provide information to the user
- `LOG_WARNING` display a purple message on `std::cerr` to warn the user of a non-fatal error
- `LOG_ERROR` display a bright red message on `std::cerr` to show a critical error message

You are not forced to use these functions to use `std::cout` and `std::cerr`
but this helps to highlight important messages on the console.

Example:
```cpp
LOG_SUCCESS("Great success")
LOG_INFO("You don't need a ; after those instructions")
LOG_WARNING("The value of d is greater than expected: " << d << " (d_limit: " << d_limit << ")")
LOG_ERROR("Bye bye")
```

Finally, you can also log an error and throw an exception. For example:

```cpp
LOG_ERROR_AND_THROW(std::runtime_error, "The world is on fire")
```
