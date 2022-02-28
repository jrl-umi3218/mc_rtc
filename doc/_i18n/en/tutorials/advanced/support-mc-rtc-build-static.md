`MC_RTC_BUILD_STATIC` is an mc_rtc build option. Its main purpose is to enable the [online demonstration](https://mc-rtc-demo.netlify.app/). This feature entirely disable dynamic loading of controllers, FSM states, robot modules, observers and plugins. Instead, these components must be built-in within mc_rtc libraries.

On this page we first list some of the changes that are required to get your projet to compile with this option. We then give an guideline on how to port your own demonstration to the web.

## Project adaptation

First of all you must build your controller and all other needed components within mc_rtc source tree. This only requires putting your project in either the `controllers`, `observers`, `plugins` or `robots` folder of mc_rtc source tree. They do not have any functional difference, they merely act as a way to organize your projects.

### Detecting you are inside mc_rtc source tree

The usual way to find mc_rtc is as follows:

```cmake
find_package(mc_rtc REQUIRED)
```

The way to detect if you are inside mc_rtc source tree is simply to check whether you need to find the package at all:

```cmake
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
```

If you use the [jrl-cmakemodules](https://github.com/jrl-umi3218/jrl-cmakemodules) you should also conditionally include them if building within mc_rtc (all functionalities are avaiable via mc_rtc anyway).

### Finding dependencies

Let us say, you need the [copra](https://github.com/jrl-umi3218/copra) package in your controller. You would normally use the following CMake code to find the dependency:

```cmake
find_package(copra REQUIRED)
```

Now that your controller is built with mc_rtc, `copra` becomes a dependency of `mc_rtc` so it becomes:

```cmake
if(NOT MC_RTC_BUILD_STATIC)
  add_required_dependency(copra REQUIRED)
else()
  find_package(copra REQUIRED)
endif()
```

### Adding mc_rtc components

If you are using the macros provided by `find_package(mc_rtc)` (e.g. `add_controller(NAME ...)`, `add_robot(NAME ...)` and so-on) then most of the hard-work is done for you. Otherwise, you are advised to look into `src/CMakeLists.txt` in mc_rtc source tree to understand what you should modify. The main difference is that source paths should be provided as absolute paths instead of relative paths.

There is one caveat to this. If your controller derives from `mc_control::fsm::Controller` then you should use `add_fsm_controller(NAME ...)` instead.

Furthermore, if you are linking your component with an external library this library should be linked to the related mc_rtc library instead.

For example:

```cmake
add_controller(myController "${myController_SRC}" "${myController_HDR}")
target_link_libraries(myController PUBLIC copra::copra)
```

Becomes:

```cmake
if(NOT MC_RTC_BUILD_STATIC)
  add_controller(myController "${myController_SRC}" "${myController_HDR}")
  target_link_libraries(myController PUBLIC copra::copra)
else()
  add_fsm_controller(myController "${myController_SRC}" "${myController_HDR}")
  target_link_libraries(mc_control_fsm PUBLIC copra::copra)
  # Would be:
  # - mc_control for a non-FSM controller
  # - mc_rbdyn for a RobotModule
  # - mc_observers for an Observer
  # - mc_control for a plugin
endif()
```

### Example

This [patch](https://github.com/gergondet/lipm_walking_controller/commit/f507f63de378a9d80917d9b3f1280a5843bb2b56) enables the support of `MC_RTC_BUILD_STATIC` in the [lipm_walking_controller](https://github.com/jrl-umi3218/lipm_walking_controller).

## Porting your mc_rtc demo to the web

The web demonstration is made possible by using the [emscripten](https://emscripten.org/index.html) project. This project enables the compilation of C++ code to [WebAssembly](http://webassembly.org/) which can then run in any compatible browser. Note that, to the best of our knowledge, the entire mc_rtc API is available.

<div class="row">
  <div class="offset-2 col-8">
    <div class="alert alert-warning" role="alert">
      If you choose to deploy an online demonstration of your controller, be aware that both the code (in compiled form) and the data will be de-facto publicly available. If confidentiality is a concern you should refrain from deploying such a demonstration.
    </div>
  </div>
</div>

### General emscripten limitations

Please refer to [emscripten Porting guide](https://emscripten.org/docs/porting/index.html) for a complete overview of emscripten limitations. In general this should not affect you much.

The most important limitation concerns file system issues. As the web application access files via a virtual filesystem, you must take care of either:
1. Reproducing the build system filesystem hierarchi in the virtual filesystem
2. Taking care of configuring the build with "dummy" paths that are created for the online demonstration

In the case of mc_rtc online demonstration linked below, we use the second approach as it makes it a little easier to control what goes into the demonstration.

### Building your online demo

For now, the recommended approach is to use the existing [mc-rtc online demonstration](https://github.com/mc-rtc/demo/) as a basis to adapt your project. You can use this build script to build your demonstration locally before deploying to an online host.

### Firefox and GitHub pages caveats

Be aware that if you choose to host the demonstration on GitHub pages then it will not work on Firefox (as of 2020/11/27 when this tutorial is written) as the web server should provide specific HTTP headers that GitHub pages does not provide and there is currently no way to customize those headers.

The headers are:
```yaml
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
```
