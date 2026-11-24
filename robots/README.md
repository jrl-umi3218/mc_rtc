# External robot modules

You can place your robot module projects in there so they are built alongside mc_rtc

## Notes

- After adding a new robot module make sure to run CMake in mc_rtc build folder so that the new project is picked up
- You should not call `find_package(mc_rtc)` in these projects, you will have access to mc_rtc libraries and CMake macros directly, you can easily check whether the project is built within mc_rtc or not:

```cmake
# We are not building from mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
```
