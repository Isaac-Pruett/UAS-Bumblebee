{

  description = "PolyUAS Swarm Simulation";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { self, nixpkgs, ... }:
    let
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      forAllSystems = f: nixpkgs.lib.genAttrs systems (system: f nixpkgs.legacyPackages.${system});

      # Configurable package constructor
      mkSimSwarm =
        {
          pkgs,
          numSims ? 2,
        }:
        assert builtins.isInt numSims && numSims > 0;
        pkgs.writeShellApplication {
          name = "simSwarm";

          excludeShellChecks = [ "SC1091" ];

          runtimeInputs = with pkgs; [
            python313

            cmake
            ninja
            gnumake
            ccache
            git
            pkg-config

            rsync
            zip
            unzip
          ];

          text = ''
            #
            # Find the root of the PolyUAS monorepo.
            #
            REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"

            if [ -z "$REPO_ROOT" ]; then
              echo "ERROR: Could not determine repository root."
              exit 1
            fi

            export POLYUAS_ROOT="$REPO_ROOT"
            export PX4_ROOT="$POLYUAS_ROOT/px4"
            export PX4_REQUIREMENTS="$PX4_ROOT/Tools/setup/requirements.txt"
            export PX4_VENV="$PX4_ROOT/.venv"


            echo "Initializing Git submodules..."

            git -C "$REPO_ROOT" submodule sync --recursive
            git -C "$REPO_ROOT" submodule update --init --recursive


            if [ ! -d "$PX4_ROOT" ]; then
              echo "ERROR: px4 submodule not found:"
              echo "  $PX4_ROOT"
              exit 1
            fi

            if [ ! -f "$PX4_REQUIREMENTS" ]; then
              echo "ERROR: PX4 requirements file not found:"
              echo "  $PX4_REQUIREMENTS"
              exit 1
            fi

            #
            # Create a Python venv using the Nix-provided Python.
            #
            if [ -d "$PX4_VENV" ]; then
              VENV_PYTHON="$PX4_VENV/bin/python"

              if [ ! -x "$VENV_PYTHON" ]; then
                echo "Existing PX4 venv is invalid; recreating it."
                rm -rf "$PX4_VENV"
              fi
            fi

            if [ ! -d "$PX4_VENV" ]; then
              echo "Creating PX4 Python virtual environment..."
              python -m venv "$PX4_VENV"
            fi

            source "$PX4_VENV/bin/activate"

            #
            # Only rerun pip when requirements.txt changes.
            #
            REQ_HASH="$(shasum -a 256 "$PX4_REQUIREMENTS" | cut -d' ' -f1)"
            HASH_FILE="$PX4_VENV/.px4-requirements-hash"

            if [ ! -f "$HASH_FILE" ] ||
               [ "$(cat "$HASH_FILE")" != "$REQ_HASH" ]; then

              echo "Installing PX4 Python dependencies..."

              python -m pip install --upgrade pip
              python -m pip install future
              python -m pip install -r "$PX4_REQUIREMENTS"

              echo "$REQ_HASH" > "$HASH_FILE"
            fi

            cd "$PX4_ROOT"

            echo "Starting ${toString numSims} PX4 instances..."


            ./Tools/simulation/sitl_multiple_run.sh \
              ${toString numSims} \
              sihsim_quadx \
              px4_sitl_sih
          '';
        };

    in
    {
      #
      # Export the constructor so other flakes can configure it.
      #
      lib.mkSimSwarm = mkSimSwarm;

      #
      # Standalone default: 2 drones.
      #
      packages = forAllSystems (pkgs: {
        default = mkSimSwarm {
          inherit pkgs;
          numSims = 2;
        };
      });
    };
}
