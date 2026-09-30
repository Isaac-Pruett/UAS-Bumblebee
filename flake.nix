{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";

    naersk.url = "github:nix-community/naersk/master";
    rust-overlay.url = "github:oxalica/rust-overlay";

    simulation = {
      url = "./simulation";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    mavlink = {
      url = "./mavlink";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
      inputs.naersk.follows = "naersk";
      inputs.rust-overlay.follows = "rust-overlay";
    };

    identity = {
      url = "./identity";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
      inputs.naersk.follows = "naersk";
      inputs.rust-overlay.follows = "rust-overlay";
    };

    zenohd = {
      url = "./zenohd";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    telemetry = {
      url = "./telemetry";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    demo_ctrl_node = {
      url = "./demo_ctrl_node";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
      inputs.naersk.follows = "naersk";
      inputs.rust-overlay.follows = "rust-overlay";
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

          mkMavUDPConn = port: "udpin:127.0.0.1:${toString port}";

          drones_in_sim = [
            {
              name = "luke";
              mavlinkConn = mkMavUDPConn 14540;

              offset = {
                north = 0.0;
                east = 0.0;
                up = 0.0;
              };
            }

            {
              name = "akshay";
              mavlinkConn = mkMavUDPConn 14541;

              offset = {
                north = 0.0;
                east = 4.0;
                up = 0.0;
              };
            }

            {
              name = "reagan";
              mavlinkConn = mkMavUDPConn 14542;

              offset = {
                north = 4.0;
                east = 0.0;
                up = 0.0;
              };
            }

            {
              name = "2luke";
              mavlinkConn = mkMavUDPConn 14543;

              offset = {
                north = 4.0;
                east = 4.0;
                up = 0.0;
              };
            }

            {
              name = "2akshay";
              mavlinkConn = mkMavUDPConn 14544;

              offset = {
                north = 0.0;
                east = -4.0;
                up = 0.0;
              };
            }

            {
              name = "2reagan";
              mavlinkConn = mkMavUDPConn 14545;

              offset = {
                north = -4.0;
                east = 0.0;
                up = 0.0;
              };
            }
          ];

          simulationOrigin = {
            lat = 35.32844;
            lon = -120.75255;
            alt = 300.0;
          };

          telemetryConfig = pkgs.writeText "zenoh-telemetry.json" (
            builtins.toJSON {
              mode = "client";
              connect.endpoints = [ "tcp/127.0.0.1:7447" ];
              scouting.multicast.enabled = false;
            }
          );

          # Generate shared Zenoh config (customize as needed; could derive from template)
          mkSharedConfig =
            droneName:
            pkgs.writeText "zenoh-${droneName}.json" (
              builtins.toJSON {
                mode = "client";
                namespace = droneName;

                connect.endpoints = [ "tcp/127.0.0.1:7447" ];
                scouting.multicast.enabled = false;

                transport = {
                  unicast.lowlatency = false;
                  shared_memory.enabled = true;
                };
              }
            );

          routerCfg = pkgs.writeText "zenoh-router-cfg.json" ''
            {
              mode: "router",
              listen: {
                endpoints: ["tcp/127.0.0.1:7447"]
              }
            }
          '';

          mkDroneProcesses =
            drone:
            let
              environment = [ "ZENOH_CONFIG=${mkSharedConfig drone.name}" ];
            in
            {
              "identity-${drone.name}" = {
                command = "exec ${self'.packages.identity}/bin/identity";
                inherit environment;
              };
              "mavlink-${drone.name}" = {
                command = "exec ${self'.packages.mavlink}/bin/mavlink";
                environment = environment ++ [ "MAVLINK_CONN=${drone.mavlinkConn}" ];
                ready_log_line = "mavlink: autopilot =";
                availability = {
                  restart = "on_failure";
                  backoff_seconds = 3;
                  max_restarts = 0; # Retry indefinitely if the connection is unavailable.
                };
              };
            };

          mkProcessStack =
            name: processes:
            let
              composeConfig = pkgs.writeText "${name}-process-compose.yaml" (
                builtins.toJSON {
                  version = "0.5";
                  shell = {
                    shell_command = "${pkgs.bash}/bin/bash";
                    shell_argument = "-c";
                  };
                  processes = builtins.mapAttrs (
                    _: process:
                    {
                      shutdown = {
                        signal = 15;
                        timeout_seconds = 10;
                        parent_only = false;
                      };
                    }
                    // process
                  ) processes;
                }
              );
            in
            pkgs.writeShellApplication {
              inherit name;
              runtimeInputs = [ pkgs.process-compose ];
              text = ''
                exec process-compose --config ${composeConfig} --no-server up "$@"
              '';
            };

          mkDroneStack = drone: mkProcessStack "execUAS_${drone.name}" (mkDroneProcesses drone);

          extractBin = name: name.packages.${system}.default;

        in
        {

          # Expose subproject packages for composition
          packages = {
            mavlink = extractBin inputs.mavlink;
            identity = extractBin inputs.identity;
            telemetry = extractBin inputs.telemetry;
            demo_ctrl_node = extractBin inputs.demo_ctrl_node;
            zenohd = extractBin inputs.zenohd;

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
              mavlinkConn = mkMavUDPConn 14540;
            };

            default = self'.packages.dev;

            full_simulation =
              let
                simSwarm = inputs.simulation.lib.mkSimSwarm {
                  inherit pkgs;
                  vehicles = drones_in_sim;
                  origin = simulationOrigin;
                  backend = "sih";
                };
                droneProcesses = builtins.foldl' (acc: drone: acc // mkDroneProcesses drone) { } drones_in_sim;
                routerDependency = {
                  zenohd.condition = "process_healthy";
                };
              in
              mkProcessStack "simulate" (
                {
                  simulator = {
                    command = "exec ${simSwarm}/bin/simSwarm";
                    availability.restart = "exit_on_failure";
                  };
                  zenohd = {
                    command = "exec ${self'.packages.zenohd}/bin/zenohd -c ${routerCfg}";
                    availability.restart = "exit_on_failure";
                    readiness_probe = {
                      exec.command = "${pkgs.bash}/bin/bash -c 'exec 3<>/dev/tcp/127.0.0.1/7447'";
                      period_seconds = 1;
                      timeout_seconds = 1;
                      success_threshold = 1;
                      failure_threshold = 30;
                    };
                  };
                  telemetry = {
                    command = "exec ${self'.packages.telemetry}/bin/telemetry";
                    environment = [
                      "TELEMETRY_ZENOH_CONFIG=${telemetryConfig}"

                    ];
                    availability = {
                      restart = "on_failure";
                      backoff_seconds = 3;
                    };
                    depends_on = routerDependency;
                  };
                  demo_ctrl_node = {
                    command = "exec ${self'.packages.demo_ctrl_node}/bin/demo_ctrl_node";
                    environment = [
                      "ZENOH_CONFIG=${telemetryConfig}"

                    ];

                    availability = {
                      restart = "on_failure";
                      backoff_seconds = 3;
                    };
                    depends_on = routerDependency;
                  };
                }
                // builtins.mapAttrs (
                  _: process:
                  process
                  // {
                    depends_on = routerDependency;
                  }
                ) droneProcesses
              );

          };

          devShells.default = pkgs.mkShell {
            packages = [
              self'.packages.zenohd
              self'.packages.dev
              self'.packages.telemetry
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
