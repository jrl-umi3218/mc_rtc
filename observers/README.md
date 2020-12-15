External observers
==

You can place your observer projects in there so they are built alongside mc\_rtc

Notes
--

- After adding a new observer make sure to run CMake in mc\_rtc build folder so that the new project is picked up
- You should not call `find_package(mc_rtc)` in these projects, you will have access to mc\_rtc libraries and CMake macros directly, you can easily check whether the project is built within mc\_rtc or not:

```cmake
# We are not building from mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
```
