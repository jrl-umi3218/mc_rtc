{
  description = " mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

    mesh-sampling.url = "github:jrl-umi3218/mesh_sampling/pull/12/head";
    # mesh-sampling.url = "path:/home/arnaud/devcontainers/volumes/mc-rtc-superbuild-jammy/devel/mesh-sampling";
    mesh-sampling.flake = true;
    # mesh-sampling.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mesh_sampling";

    mc-rtc-ros-compat.url = "path:/home/arnaud/devel/mc-rtc-nix/workspace/mc_rtc_ros_compat";

    # To override dependencies according to a commit/pull request, add them to inputs
    # For example:
    # mc-force-shoe-plugin.url = "github:Hugo-L3174/mc_force_shoe_plugin/pull/16/head";
    # or use pull/N/merge to get the version merged with master, assuming there are no conflicts
    # mc-force-shoe-plugin.flake = false;
    # use true if the repository has a flake
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
              # with-ros = false;
            };
            mc-rtc-superbuild =
              { ... }:
              {
                enable = true;
                shells.defaultShells.release = true;
                shells.defaultShells.devel = false;
              };
            flakoboros = {
              overrideAttrs.mesh-sampling = {
                src = inputs.mesh-sampling;
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
                  propagatedBuildInputs = (drv-prev.propagatedBuildInputs or [ ])
  ++ [ inputs.mc-rtc-ros-compat.packages.x86_64-linux.default ];
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
