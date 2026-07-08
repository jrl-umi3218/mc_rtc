{
  description = " mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    # mc-rtc-ros-compat.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mc_rtc_ros_compat";
    mc-rtc-ros-compat.url = "github:jrl-umi3218/mc_rtc_ros_compat";

    # To override dependencies according to a commit/pull request, add them to inputs
    # For example:
    # mc-force-shoe-plugin.url = "github:Hugo-L3174/mc_force_shoe_plugin/pull/16/head";
    # or use pull/N/merge to get the version merged with master, assuming there are no conflicts
    # mc-force-shoe-plugin.flake = false;
    # use true if the repository has a flake
  };
  nixConfig = {
    extra-substituters = [
      "https://mc-rtc-nix.cachix.org"
      "https://gepetto.cachix.org"
      "https://attic.iid.ciirc.cvut.cz/ros"
    ];
    extra-trusted-public-keys = [
      "mc-rtc-nix.cachix.org-1:5M3sLvHXJCep4wc1tQl7QuFWL2eH2I0jkuvWtqJDYQs="
      "gepetto.cachix.org-1:toswMl31VewC0jGkN6+gOelO2Yom0SOHzPwJMY2XiDY="
      "ros:JR95vUYsShSqfA1VTYoFt1Nz6uXasm5QrcOsGry9f6Q="
    ];
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports =
          let
            with-ros-default = true;
          in
          [
            inputs.mc-rtc-nix.flakeModule
            # or inputs.mc-rtc-nix.flakeModule if you don't need private repositories
            {
              mc-rtc-nix = {
                with-ros = with-ros-default;
                overlays.ccache = true;
              };
              mc-rtc-superbuild =
                { ... }:
                {
                  enable = true;
                  shells.defaultShells.release = true;
                  shells.defaultShells.devel = false;
                };
              flakoboros = {
                overrideAttrs.mc-rtc =
                  { drv-prev, pkgs-final, ... }:
                  {
                    src = lib.cleanSource ./.;
                    # FIXME: enable testing:
                    # - testing fails in nix build
                    # - testRobotModule fails in nix devel
                    cmakeFlags = (drv-prev.cmakeFlags or [ ]) ++ [ (lib.cmakeBool "BUILD_TESTING" true) ];
                    nativeBuildInputs = drv-prev.nativeBuildInputs ++ [ pkgs-final.ninja ];
                    propagatedBuildInputs = (drv-prev.propagatedBuildInputs or [ ]) ++ [ pkgs-final.mc-rtc-ros-compat ];
                    nativeCheckInputs = [
                      # workaround for some tests trying to write to /homeless-shelter
                      pkgs-final.writableTmpDirAsHomeHook
                    ];
                    doCheck = false;
                  };
                packages = {
                  mc-rtc-ros-compat =
                    {
                      stdenv,
                      lib,
                      cmake,
                      jrl-cmakemodulesv2,
                      buildRosPackage,
                      with-ros ? with-ros-default,
                      rclcpp,
                    }:

                    (if with-ros then buildRosPackage else stdenv.mkDerivation) {
                      pname = "mc-rtc-ros-compat";
                      version = "1.0.0";

                      src = inputs.mc-rtc-ros-compat;

                      buildInputs = [
                        jrl-cmakemodulesv2
                      ];
                      nativeBuildInputs = [
                        cmake
                      ];
                      propagatedBuildInputs = lib.optional with-ros rclcpp;

                      cmakeFlags = [
                        (lib.cmakeBool "DISABLE_ROS" (!with-ros))
                      ];

                      doCheck = true;

                      meta = with lib; {
                        description = "mc-rtc-ros-compat: small library to keep mc-rtc ros-agnostic";
                        homepage = "https://github.com/jrl-umi3218/mc_rtc_ros_compat";
                        license = licenses.bsd2;
                        platforms = platforms.all;
                      };
                    };
                };
              };
            }
          ];
        perSystem =
          { ... }:
          {
            treefmt = {
              settings.global.excludes = [
                ".envrc"
                ".git-blame-ignore-revs"
                ".jrl-ci"
                "3rd-party/*"
                "doc/*"
                "LICENSE"
              ];
              programs = {
                # keep-sorted start
                mdformat.enable = true;
                yamlfmt.enable = false;
                # keep-sorted end
              };
            };
          };
      }
    );
}
