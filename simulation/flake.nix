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
          vehicles,
          origin ? {
            lat = 33.748997;
            lon = -84.387985;
            alt = 300.0;
          },
          backend ? "sih",
        }:

        assert builtins.isList vehicles && builtins.length vehicles > 0;

        let
          indexedVehicles = builtins.genList (i: {
            index = i;
            vehicle = builtins.elemAt vehicles i;
          }) (builtins.length vehicles);

          vehicleLaunchers = builtins.concatStringsSep "\n" (
            map (
              entry:
              let
                i = entry.index;
                vehicle = entry.vehicle;
              in
              ''
                launch_vehicle \
                  ${toString i} \
                  ${toString vehicle.offset.north} \
                  ${toString vehicle.offset.east} \
                  ${toString vehicle.offset.up}
              ''
            ) indexedVehicles
          );

        in
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

                  echo "Killing existing PX4 instances..."
                  pkill -x px4 || true
                  sleep 1

                  BUILD_PATH="$PX4_ROOT/build/px4_sitl_sih"

                  launch_vehicle() {
                    instance="$1"
                    north_m="$2"
                    east_m="$3"
                    up_m="$4"

                    read -r lat lon alt <<EOF
            $(python - "$north_m" "$east_m" "$up_m" <<'PY'
            import math
            import sys

            north = float(sys.argv[1])
            east = float(sys.argv[2])
            up = float(sys.argv[3])

            lat0 = ${toString origin.lat}
            lon0 = ${toString origin.lon}
            alt0 = ${toString origin.alt}

            meters_per_degree = 111320.0

            lat = lat0 + north / meters_per_degree
            lon = lon0 + east / (
                meters_per_degree * math.cos(math.radians(lat0))
            )

            alt = alt0 + up

            print(lat, lon, alt)
            PY
            )
            EOF

                    working_dir="$BUILD_PATH/instance_$instance"
                    mkdir -p "$working_dir"

                    echo "Starting PX4 instance $instance"
                    echo "  offset N/E/U = $north_m / $east_m / $up_m"
                    echo "  home = $lat, $lon, $alt"

                    (
                      cd "$working_dir"

                      PX4_HOME_LAT="$lat" \
                      PX4_HOME_LON="$lon" \
                      PX4_HOME_ALT="$alt" \
                      PX4_SIM_MODEL="sihsim_quadx" \
                        "$BUILD_PATH/bin/px4" \
                          -i "$instance" \
                          -d "$BUILD_PATH/etc" \
                          >out.log 2>err.log
                    ) &
                  }

                  echo "Starting ${toString (builtins.length vehicles)} PX4 instances..."

                  ${vehicleLaunchers}

                  wait
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
