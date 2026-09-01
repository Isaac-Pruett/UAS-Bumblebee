{
  description = "PolyUAS PX4 SIH development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "aarch64-darwin"
        "x86_64-darwin"
        "x86_64-linux"
        "aarch64-linux"
      ];

      forAllSystems =
        f:
        nixpkgs.lib.genAttrs systems (
          system:
          f (
            import nixpkgs {
              inherit system;
            }
          )
        );
    in
    {
      devShells = forAllSystems (pkgs: {
        default = pkgs.mkShell {
          packages = with pkgs; [
            # Python for PX4's Python tooling.
            python313

            # PX4 build tools.
            cmake
            ninja
            gnumake
            ccache
            git
            pkg-config

            # Utilities used by PX4 tooling.
            rsync
            zip
            unzip
          ];

          shellHook = ''
            #
            # Find the root of the PolyUAS monorepo.
            #
            REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"

            if [ -z "$REPO_ROOT" ]; then
              echo "ERROR: Could not determine repository root."
              return 1
            fi

            export POLYUAS_ROOT="$REPO_ROOT"
            export PX4_ROOT="$POLYUAS_ROOT/PX4-Autopilot"
            export PX4_REQUIREMENTS="$PX4_ROOT/Tools/setup/requirements.txt"
            export PX4_VENV="$PX4_ROOT/.venv"

            if [ ! -d "$PX4_ROOT" ]; then
              echo "ERROR: PX4-Autopilot not found:"
              echo "  $PX4_ROOT"
              return 1
            fi

            if [ ! -f "$PX4_REQUIREMENTS" ]; then
              echo "ERROR: PX4 requirements file not found:"
              echo "  $PX4_REQUIREMENTS"
              return 1
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

            echo
            echo "PolyUAS PX4 SIH environment"
            echo "---------------------------"
            echo "Repository: $POLYUAS_ROOT"
            echo "PX4:        $PX4_ROOT"
            echo "Python:     $(which python)"
            echo
            echo "Build SIH:"
            echo "  cd \$PX4_ROOT"
            echo "  make px4_sitl_sih sihsim_quadx"
            echo
            echo "Run 2 SIH vehicles:"
            echo "  cd \$PX4_ROOT"
            echo "  ./Tools/simulation/sitl_multiple_run.sh 2 sihsim_quadx px4_sitl_sih"
            echo
          '';
        };
      });
    };
}
