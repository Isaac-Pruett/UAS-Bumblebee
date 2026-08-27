{
  description = "Portable PX4 JSBSim SITL launcher";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs =
    { self, nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in
    {
      packages = forAllSystems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
          launcher = pkgs.writeShellApplication {
            name = "px4-jsbsim";
            runtimeInputs = [
              pkgs.bash
              pkgs.coreutils
              pkgs.git
              pkgs.docker-client
            ];
            text = ''
              export PX4_LAUNCHER=${./run-px4-jsbsim.sh}
              exec ${pkgs.bash}/bin/bash ${./bootstrap-px4-jsbsim.sh}
            '';
          };
        in
        {
          default = launcher;
          px4-jsbsim = launcher;
          simulation = launcher;
        }
      );

      apps = forAllSystems (system: {
        default = {
          type = "app";
          program = "${self.packages.${system}.default}/bin/px4-jsbsim";
        };
        px4-jsbsim = self.apps.${system}.default;
        simulation = self.apps.${system}.default;
      });

      devShells = forAllSystems (
        system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          default = pkgs.mkShell {
            packages = [
              pkgs.bash
              pkgs.coreutils
              pkgs.git
              pkgs.docker-client
            ];
            shellHook = ''
              export PX4_LAUNCHER="$PWD/run-px4-jsbsim.sh"
            '';
          };
        }
      );
    };
}
