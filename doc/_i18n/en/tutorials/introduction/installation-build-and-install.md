## Building from source with the build_and_install script

If you are building on Linux or macOS you can skip to the Building section. However, Windows users have to follow extra-instructions.

### Windows pre-requisites

If you are building on Windows, you need to have the following tools installed before starting:
- [Visual Studio 2019](https://visualstudio.microsoft.com/vs/) - when installing, ensure that you select the Python extension if you wish to build the Python bindings. Furthermore, you should make sure that Visual Studio's `python` and `pip` executables are in your `PATH` environment variable;
- [Git Bash](https://git-scm.com/download/win) - we will use this tool to clone mc\_rtc and start the installation script;
- [CMake](https://cmake.org/download/) - install the latest version available;
- [Boost](https://www.boost.org/) - install the latest binaries avaible from [sourceforce](https://sourceforge.net/projects/boost/files/boost-binaries/). Make sure to select the right version for your computer and Visual Studio version, for example, for Boost 1.72 and Visual Studio 2019 on a 64 bits computer it should be: [boost_1_72_0-msvc-14.2-64.exe](https://sourceforge.net/projects/boost/files/boost-binaries/1.72.0/boost_1_72_0-msvc-14.2-64.exe/download). After installing, make sure to set the environment variable `BOOST_ROOT` to the location where you installed Boost and add Boost DLLs path to your `PATH` environment. For example, if you installed into `C:\local\boost_1_72_0` then `BOOST_ROOT` should be `C:\local\boost_1_72_0` and `%BOOST_ROOT%\lib64-msvc-14.2` should be in your `PATH`;
- [mingw-w64](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) - this will be used to compile Fortran code on Windows. After downloading, you can run the executable. At one point, it will ask you to select several options: Version, Architecture, Threads, Exception and Build revision. For Version you can choose the most recent one (default), for Architecture you should choose x86\_64 if you are building on a 64 bits Windows (likely), for Threads you should choose win32, for Exceptions you should choose seh (default). After installing, you should make sure the `bin` folder of the installation is in your `PATH` environment, e.g. `C:\mingw-w64\x86_64-8.1.0-release-win32-seh-rt_v6-rev0\mingw64\bin`

Note: it should also work and compile with Visual Studio 2017. However, only Visual Studio 2019 is regularly tested

### Building

1. Clone the [mc\_rtc](https://github.com/jrl-umi3218/mc_rtc) repository;
2. Go into the mc\_rtc directory and update submodules `git submodule update --init`;
3. Go into the `utils` directory and locate the file named `build_and_install.sh`;
4. [optional] Create a custom configuration file `build_and_install_user_config.sh` (overrides the corresponding variables from the default configuration `build_and_install_default_config.sh`)
```sh
cp build_and_install_user_config.sample.sh build_and_install_user_config.sh
```
5. [optional] Edit the `build_and_install_user_config.sh` and edit the options to your liking: `INSTALL_PREFIX`, `WITH_ROS_SUPPORT`, `ROS_DISTRO`. On Ubuntu, ROS will be installed if you enable ROS support and it was not already installed. Otherwise, you are required to install ROS by yourself before attempting to install mc\_rtc with ROS support;
5. Run `./build_and_install.sh`

The script will take care of installing the required dependencies, clone all required source codes, build and install them. This may take a while.

If the script fails, please open up an issue on mc\_rtc issue tracker, providing the following information:
- System (compiler, distribution/OSX version)
- Script output
- Any detail you might think relevant

## Getting started

Once mc_rtc has been installed, you can jump to the next [section]({{site.baseurl}}/tutorials/introduction/configuration.html).
