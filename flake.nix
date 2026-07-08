{
  description = " mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    mc-rtc-ros-compat.url = "github:jrl-umi3218/mc_rtc_ros_compat";
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
                      catch2_3,
                      buildRosPackage,
                      with-ros ? with-ros-default,
                      human-description ? null,
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
                        catch2_3
                      ]
                      # for tests
                      ++ lib.optional (human-description != null) human-description;
                      propagatedBuildInputs = lib.optional with-ros rclcpp;

                      cmakeFlags = [
                        (lib.cmakeBool "DISABLE_ROS" (!with-ros))
                        (lib.cmakeBool "BUILD_TESTS_WITH_ROS_PACKAGES" (human-description != null))
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
