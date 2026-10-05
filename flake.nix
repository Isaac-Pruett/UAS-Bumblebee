{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";

    crane.url = "github:ipetkov/crane";

    simulation = {
      url = "./simulation";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    zenohd = {
      url = "./zenohd";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    telemetry = {
      url = "./telemetry";
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

          nullNamespaceConfig = mkSharedConfig "";

          # Generate shared Zenoh config (customize as needed; could derive from template)
          mkSharedConfig =
            droneName:
            let
              filename = if droneName == "" then "null-namespace" else droneName;
            in
            pkgs.writeText "zenoh-${filename}.json" (
              builtins.toJSON (
                {
                  mode = "client";

                  connect.endpoints = [ "tcp/127.0.0.1:7447" ];
                  scouting.multicast.enabled = false;

                  transport = {
                    unicast.lowlatency = false;
                    shared_memory.enabled = true;
                  };
                }
                // pkgs.lib.optionalAttrs (droneName != "") {
                  namespace = droneName;
                }
              )
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
                ready_log_line = "Set drone name to:";

              };
              "formations-${drone.name}" = {
                command = "exec ${self'.packages.formations}/bin/formations";
                environment = environment ++ [ "NULL_ZENOH_CONFIG=${nullNamespaceConfig}" ];
                depends_on = {
                  "identity-${drone.name}".condition = "process_log_ready";
                };
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

          withDependencies =
            dependencies: process:
            process
            // {
              depends_on = (process.depends_on or { }) // dependencies;
            };

          craneLib = inputs.crane.mkLib pkgs;

          # Keep templates and other non-workspace Cargo manifests out of Crane's
          # recursive source scan.
          rustWorkspaceSrc = pkgs.lib.fileset.toSource {
            root = ./.;
            fileset = pkgs.lib.fileset.unions [
              ./Cargo.toml
              ./Cargo.lock
              ./identity
              ./mavlink
              ./gamepad
              ./formations
              ./gamepad_controlled_demo
            ];
          };

          commonRustArgs = {
            pname = "uas-rust-workspace";
            version = "0.1.0";
            src = craneLib.cleanCargoSource rustWorkspaceSrc;
            strictDeps = true;
          };

          # Build the workspace dependency graph once and reuse its artifacts for
          # each node package.
          rustWorkspaceDeps = craneLib.buildDepsOnly (
            commonRustArgs
            // {
              cargoExtraArgs = "--workspace";
            }
          );

          rustWorkspace = craneLib.buildPackage (
            commonRustArgs
            // {
              cargoArtifacts = rustWorkspaceDeps;
              cargoExtraArgs = "--workspace";
            }
          );

          # Each node is a thin package around the one workspace build. Building
          # the workspace with the same Cargo arguments as rustWorkspaceDeps
          # keeps Cargo's feature fingerprints compatible, so Zenoh is reused.
          mkRustNode =
            name:
            pkgs.runCommand name { } ''
              mkdir -p "$out/bin"
              ln -s ${rustWorkspace}/bin/${name} "$out/bin/${name}"
            '';

        in
        {

          packages = {
            mavlink = mkRustNode "mavlink";
            identity = mkRustNode "identity";
            gamepad = mkRustNode "gamepad";
            gamepad_controlled_demo = mkRustNode "gamepad_controlled_demo";
            formations = mkRustNode "formations";
            telemetry = inputs.telemetry.packages.${system}.default;
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
                routerDependency.zenohd.condition = "process_healthy";
                withRouterDependency = withDependencies routerDependency;
              in
              mkProcessStack "simulate" (
                {
                  simulator = {
                    command = "exec ${simSwarm}/bin/simSwarm";
                    ready_log_line = "Started all instances!";
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
                  telemetry = withRouterDependency {
                    command = "exec ${self'.packages.telemetry}/bin/telemetry";
                    environment = [ "TELEMETRY_ZENOH_CONFIG=${nullNamespaceConfig}" ];
                    availability = {
                      restart = "on_failure";
                      backoff_seconds = 3;
                    };
                  };
                  gamepad = withRouterDependency {
                    command = "exec ${self'.packages.gamepad}/bin/gamepad";
                    environment = [ "ZENOH_CONFIG=${nullNamespaceConfig}" ];
                    availability = {
                      restart = "on_failure";
                      backoff_seconds = 3;
                    };
                  };
                  gamepad_controlled_demo = withRouterDependency {
                    depends_on =
                      builtins.listToAttrs (
                        map (drone: {
                          name = "mavlink-${drone.name}";
                          value.condition = "process_log_ready";
                        }) drones_in_sim
                      )
                      // {
                        gamepad.condition = "process_started";
                      };
                    command = "exec ${self'.packages.gamepad_controlled_demo}/bin/gamepad_controlled_demo";
                    environment = [ "ZENOH_CONFIG=${nullNamespaceConfig}" ];
                    availability = {
                      restart = "on_failure";
                      backoff_seconds = 3;
                    };
                  };
                }
                // builtins.mapAttrs (
                  name: process:
                  withRouterDependency (
                    withDependencies (pkgs.lib.optionalAttrs (pkgs.lib.hasPrefix "mavlink-" name) {
                      simulator.condition = "process_log_ready";
                    }) process
                  )
                ) droneProcesses
              );

          };

          devShells.default = pkgs.mkShell {
            packages = [
              self'.packages.zenohd
              self'.packages.dev
              self'.packages.telemetry
              pkgs.cargo
              pkgs.rustc
              pkgs.rustfmt
              pkgs.rustPackages.clippy
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
