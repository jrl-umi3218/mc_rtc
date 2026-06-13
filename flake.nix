{
  description = " mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";

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
            mc-rtc-superbuild =
              { ... }:
              {
                enable = true;
                shells.defaultShells.release = true;
                shells.defaultShells.devel = false;
              };
            flakoboros = {
              overrideAttrs.mc-rtc =
                { ... }:
                {
                  src = lib.cleanSource ./.;
                  # FIXME: enable testing:
                  # - testing fails in nix build
                  # - testRobotModule fails in nix devel
                  # cmakeFlags = (drv-prev.cmakeFlags or [ ]) ++ [ (lib.cmakeBool "BUILD_TESTING" true) ];
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
