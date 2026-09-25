{
  description = " mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
    gepetto.follows = "mc-rtc-nix/gepetto";

    ccache-trigger.follows = "mc-rtc-nix/ccache-trigger";
    with-ros-trigger.follows = "mc-rtc-nix/with-ros-trigger";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          # or inputs.mc-rtc-nix.flakeModule if you don't need private repositories
          {
            mc-rtc-nix = {
              with-ros = inputs.with-ros-trigger.value;
              overlays.ccache = inputs.ccache-trigger.value;
            };
            mc-rtc-superbuild =
              { ... }:
              {
                enable = true;
                shells.defaultShells.release = true;
                shells.defaultShells.devel = false;
              };
            flakoboros = {
              overrideAttrs.tasks = { pkgs-final, ... }: {
                src = pkgs-final.fetchFromGitHub {
                  owner = "jrl-umi3218";
                  repo = "Tasks";
                  tag = "v1.9.1";
                  hash = "sha256-9eioOZus6LYE5WZCKeA3r2h2cqSbq8Qyg6qSHVLc0yM=";
                };
                version = "1.9.1";
              };
              overrideAttrs.tvm =
                { pkgs-final, ... }:
                {
                  src = pkgs-final.fetchgit {
                    url = "https://github.com/jrl-umi3218/TVM";
                    tag = "v0.9.7";
                    hash = "sha256-ohXrbyf1oobBIL1lMx6yxS3GtRsslJIus8fozSeTRxU=";
                    fetchSubmodules = true;
                  };
                  version = "0.9.7";
                };

              overrideAttrs.mc-rtc =
                { drv-prev, pkgs-final, ... }:
                {
                  src = lib.cleanSource ./.;
                  # FIXME: enable testing:
                  # - testing fails in nix build
                  # - testRobotModule fails in nix devel
                  cmakeFlags = (drv-prev.cmakeFlags or [ ]) ++ [ (lib.cmakeBool "BUILD_TESTING" true) ];
                  nativeBuildInputs = drv-prev.nativeBuildInputs ++ [ pkgs-final.ninja ];
                  nativeCheckInputs = [
                    # workaround for some tests trying to write to /homeless-shelter
                    pkgs-final.writableTmpDirAsHomeHook
                  ];
                  doCheck = false;
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
