{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";

    simulation = {
      url = "./simulation";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    mavlink = {
      url = "./mavlink";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    identity = {
      url = "./identity";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    zenohd = {
      url = "./zenohd";
      inputs.nixpkgs.follows = "nixpkgs";
    };

  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];

      imports = [
        # ./module.nix
        # inputs.foo.flakeModule
      ];

      perSystem =
        {
          config,
          self',
          inputs',
          pkgs,
          system,
          ...
        }:
        let
          # Allows definition of system-specific attributes
          # without needing to declare the system explicitly!
          #
          # Quick rundown of the provided arguments:
          # - config is a reference to the full configuration, lazily evaluated
          # - self' is the outputs as provided here, without system. (self'.packages.default)
          # - inputs' is the input without needing to specify system (inputs'.foo.packages.bar)
          # - pkgs is an instance of nixpkgs for your specific system
          # - system is the system this configuration is for

          default_drone_name = "bingus_the_drone";

          drones_in_sim = [
            {
              name = "luke";
              mavlinkPort = 14550;
            }
            {
              name = "akshay";
              mavlinkPort = 14551;
            }
            {
              name = "reagan";
              mavlinkPort = 14552;
            }
          ];

          drone_sim_names = builtins.map (x: x.name) drones_in_sim;

          # Generate shared Zenoh config (customize as needed; could derive from template)
          mkSharedConfig =
            drone_name:
            pkgs.writeText "zenoh-config.json5" ''
              {
                mode: "client",
                namespace: "${drone_name}",
                connect: {
                  endpoints: ["tcp/127.0.0.1:7447"]
                },
                scouting: {
                  multicast: {
                    enabled: false
                  }
                },
                transport: {
                  unicast: {
                    lowlatency: false,
                  },
                  shared_memory: {
                    enabled: true,
                  },
                },
              }
            '';

          routerCfg = pkgs.writeText "zenoh-router-cfg.json" ''
            {
              mode: "router",
              listen: {
                endpoints: ["tcp/127.0.0.1:7447"]
              }
            }
          '';

          mkDroneStack =
            drone:
            pkgs.writeShellApplication {
              name = "execUAS_${drone.name}";

              runtimeInputs = [
                self'.packages.mavlink
                self'.packages.identity
              ];

              text = ''
                export ZENOH_CONFIG=${mkSharedConfig drone.name}
                export MAVLINK_CONN="udpin:127.0.0.1:${toString drone.mavlinkPort}"

                echo "Launching drone: ${drone.name}"
                echo "Shared config: $ZENOH_CONFIG"
                echo "MAVLink endpoint: $MAVLINK_CONN"

                PIDS=()

                spawn() {
                  "$@" &
                  PIDS+=($!)
                }

                spawn identity
                spawn mavlink

                trap 'kill "''${PIDS[@]}" 2>/dev/null' EXIT INT TERM
                wait "''${PIDS[@]}"
              '';
            };

        in
        {

          # Expose subproject packages for composition
          packages = {
            rust_demo = inputs.rust-demo-sub.packages.${system}.default;
            python_demo = inputs.python-demo-sub.packages.${system}.default;
            mavlink = inputs.mavlink.packages.${system}.default;
            identity = inputs.identity.packages.${system}.default;

            zenohd = inputs.zenohd.packages.${system}.default;

            nuze = pkgs.rustPlatform.buildRustPackage rec {
              pname = "nuze";
              version = "0.3.0";

              src = pkgs.fetchCrate {
                inherit pname version;
                hash = "sha256-aaL01xlN0oLAuEcRMbkN3+JRcQ03/6VgZWs+y6M64K8=";
              };

              cargoHash = "sha256-Ik72sGNDeMh+YNWloErhFx+YLYHMEZ1aWBWmKWYUQNg=";
            };

            dev = mkDroneStack {
              name = default_drone_name;
              mavlinkPort = 14450;
            };

            default = self'.packages.dev;

            full_simulation = pkgs.writeShellApplication {
              name = "simulate";

              runtimeInputs = [
                self'.packages.zenohd
              ]
              ++ builtins.map mkDroneStack drones_in_sim
              ++ [
                (inputs.simulation.lib.mkSimSwarm {
                  inherit pkgs;
                  numSims = builtins.length drones_in_sim;
                })
              ];

              text = ''
                PIDS=()

                spawn() {
                  "$@" &
                  PIDS+=($!)
                }

                spawn simSwarm
                sleep 3.5


                # One shared Zenoh router for the entire simulation.
                spawn zenohd -c ${routerCfg}
                sleep 0.5

                ${builtins.concatStringsSep "" (map (drone: "spawn execUAS_${drone.name}\n") drones_in_sim)}

                trap 'kill "''${PIDS[@]}" 2>/dev/null' EXIT INT TERM
                wait "''${PIDS[@]}"
              '';
            };

          };

          devShells.default = pkgs.mkShell {
            packages = [
              self'.packages.zenohd
              self'.packages.dev
              pkgs.just
              pkgs.uv
            ];

            env.DRONE_NAME = default_drone_name;
            env.ZENOH_CONFIG = mkSharedConfig default_drone_name;

            shellHook = ''
              alias j="just"
              echo "just is aliased to j"
            '';
          };
        };

      flake = {
        # The usual flake attributes can be defined here, including
        # system-agnostic and/or arbitrary outputs.
      };
    };
}
