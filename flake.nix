{
  description = "mc_rtc is an interface for simulated and real robotic systems suitable for real-time control";

  inputs = {
    # mc-rtc-nix.url = "github:mc-rtc/nixpkgs";
    # mc-rtc-nix.url = "path:/home/arnaud/devel/mc-rtc-nix/nixpkgs";
    mc-rtc-nix.url = "github:arntanguy/nixpkgs-1?ref=topic/flakoboros";
    flake-parts.follows = "mc-rtc-nix/flake-parts";
    systems.follows = "mc-rtc-nix/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.mc-rtc-nix.flakeModule
          {
            flakoboros = {
              extraPackages = [ "ninja" ];
              overrideAttrs.mc-rtc =
                _:
                (super: {
                  src = lib.cleanSource ./.;
                  cmakeFlags = super.cmakeFlags ++ [
                    # "-DPYTHON_BINDINGS=OFF"
                  ];
                });
            };
          }
        ];
      }
    );
}
