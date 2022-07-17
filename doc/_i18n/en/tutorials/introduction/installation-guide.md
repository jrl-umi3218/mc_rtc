## Overview

mc\_rtc is an interface for simulation and robot control systems. These systems should provide the state of a given robot (joints' values, sensor readings...) and in return mc\_rtc will provide the desired robot's state (command). This is done through the `mc_control::MCGlobalController` class. This class does not perform control by itself but rather delegates this task to the `mc_control::MCController` derived objects that it holds. Writing a controller within the mc\_rtc framework is done by writing a class that inherits from the `mc_control::MCController` base class and implements the required functionnality. We implement such a controller in the following tutorials. The present tutorial simply explains how to build/install the framework on your machine.

<img src="{{site.baseurl_root}}/assets/tutorials/introduction/img/mc_rtc_architecture.jpg" alt="architecture_overview" class="img-fluid" />

## Installation instruction

We provide binaries for the current Ubuntu LTS releases and macOS via [Homebrew](https://brew.sh/). We also provide a source release using an easy-to-use script for Ubuntu, macOS and Windows and a [vcpkg](https://vcpkg.io/en/index.html) registry.

Binaries are recommended for Ubuntu users and macOS users. vcpkg is recommended for Windows users.

### Ubuntu LTS (18.04, 20.04)

{% assign install_apt=site.translations[site.lang].tutorials.introduction["installation-guide"].install_apt %}

{% include show_sources.html sources=install_apt id="install_apt" copy=true %}

*Note: the distributed version of mc\_rtc runs with the QLD QP solver through [eigen-qld](https://github.com/jrl-umi3218/eigen-qld). If you have access to the LSSOL solver and thus can install [eigen-lssol](https://gite.lirmm.fr/multi-contact/eigen-lssol) then you can build [Tasks](https://github.com/jrl-umi3218/Tasks) with LSSOL support and install it in `/usr`. The two versions are binary compatible.*

### Homebrew (macOS)

Follow the official instructions to install [Homebrew](https://brew.sh/). Then:

{% capture source %}
brew tap mc-rtc/mc-rtc
brew install mc_rtc
{% endcapture %}

{% include show_source.html id="brew" lang="bash" source=source copy=true %}

### vcpkg

Follow [vcpkg](https://vcpkg.io/) instruction to install vcpkg on your system.

You can then setup our registry by creating a `vcpkg-configuration.json` file either alongside the `vcpkg` binary or alongside your `vcpkg.json` manifest:

{% capture source %}
{
  "registries": [
    {
      "kind": "git",
      "baseline": "{see below}",
      "repository": "https://github.com/mc-rtc/vcpkg-registry",
      "packages": [ "spacevecalg", "rbdyn", "eigen-qld", "sch-core", "tasks",
                    "mc-rbdyn-urdf", "mc-rtc-data", "eigen-quadprog", "state-observation",
                    "hpp-spline", "ndcurves", "mc-rtc" ]
    }
  ]
}
{% endcapture %}

{% include show_source.html id="vcpkg-configuration" lang="json" source=source copy=true %}

Where `baseline` should be the latest commit sha1 on [mc-rtc/vcpkg-registry](https://github.com/mc-rtc/vcpkg-registry/)

You can then either:

- install mc_rtc via the `vcpkg` command: `vcpkg install mc_rtc`
- use the mc_rtc package in your manifest (`vcpkg.json` file), such as the following example:

{% capture source %}
{
  "name": "my-package",
  "version-string": "1.0.0",
  "homepage": "https://my.home",
  "description": "My package description",
  "dependencies": [
    "mc-rtc"
  ]
}
{% endcapture %}

{% include show_source.html id="vcpkg-json" lang="json" source=source copy=true %}

### Using mc-rtc-superbuild

[mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild/) is a CMake-based option to build mc_rtc and its dependencies. It also provides an extension mechanism to that you can build projects that require mc_rtc.

Please refer to the project's homepage for usage instructions.

### Deprecated options

The following options are also available but are deprecated:
- [Building from source using the build_and_install.sh script]({{site.baseurl}}/tutorials/introduction/installation-build-and-install.html)
- [Building from source (no script)]({{site.baseurl}}/tutorials/introduction/installation-build-no-script.html)

## Getting started

Once mc_rtc has been installed, you can jump to the next [section]({{site.baseurl}}/tutorials/introduction/configuration.html).
