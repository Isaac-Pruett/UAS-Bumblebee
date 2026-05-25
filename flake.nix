{
  description = "Master Zenoh monorepo";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    rust-demo-sub.url = "./rust_demo";
    python-demo-sub.url = "./python_demo";

    mavlink.url = "./mavlink";

    identity.url = "./identity";

    sim_ctrl.url = ./sim_ctrl;

    zenohd.url = "./zenohd";

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

          # This is equivalent to packages.<system>.default

          # Generate shared Zenoh config (customize as needed; could derive from template)
          sharedConfig = pkgs.writeText "zenoh-config.json5" ''
            {
              mode: "client",
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
                  shm: {
                    enabled: true,
                  },
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

        in
        {
          # Expose subproject packages for composition
          packages = {
            rust_demo = inputs.rust-demo-sub.packages.${system}.default;
            python_demo = inputs.python-demo-sub.packages.${system}.default;
            mavlink = inputs.mavlink.packages.${system}.default;
            identity = inputs.identity.packages.${system}.default;
            sim_ctrl = inputs.sim_ctrl.packages.${system}.default;

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

            # Launcher: Spins up all with shared config
            demo = pkgs.writeShellApplication {
              name = "demo-ping-pong-zenoh";
              runtimeInputs = [
                self'.packages.rust_demo
                self'.packages.python_demo

                self'.packages.zenohd

              ];
              text = ''
                export ZENOH_CONFIG=${sharedConfig}

                echo "Launching with shared config: $ZENOH_CONFIG"

                zenohd -c ${routerCfg} 1>/dev/null &
                ZENOH_PID=$!

                sleep 0.5

                python_demo &
                PYTHON_PID=$!

                rust_demo &
                RUST_PID=$!

                trap 'kill $ZENOH_PID $PYTHON_PID $RUST_PID 2>/dev/null' EXIT INT TERM

                wait $PYTHON_PID $RUST_PID
              '';
            };

            dev = pkgs.writeShellApplication {
              name = "execUAS";
              runtimeInputs = [
                self'.packages.mavlink
                self'.packages.identity
                self'.packages.sim_ctrl

                self'.packages.zenohd
              ];
              text = ''
                export ZENOH_CONFIG=${sharedConfig}
                export DRONE_NAME=bingus
                echo "Launching with shared config: $ZENOH_CONFIG"

                PIDS=()

                spawn() { "$@" & PIDS+=($!); }

                spawn zenohd -c ${routerCfg} 1>/dev/null
                sleep 0.5
                spawn identity
                spawn mavlink
                spawn sim_ctrl

                trap 'kill "''${PIDS[@]}" 2>/dev/null' EXIT INT TERM
                wait "''${PIDS[@]:1}"
              '';
            };

            default = self'.packages.dev;

          };

          devShells.default = pkgs.mkShell {
            packages = [
              self'.packages.demo
              self'.packages.rust_demo
              self'.packages.python_demo
              self'.packages.zenohd

              pkgs.just
              pkgs.uv
              self'.packages.nuze

            ];
            env.ZENOH_CONFIG = sharedConfig;

            shellHook = ''
              alias j="just"
              echo "Run 'demo-ping-pong-zenoh' to start demo."
              echo "just is aliased to j"
              # zenohd -c ${routerCfg}
            '';
          };
        };
      flake = {
        # The usual flake attributes can be defined here, including
        # system-agnostic and/or arbitrary outputs.
      };
    };
}
