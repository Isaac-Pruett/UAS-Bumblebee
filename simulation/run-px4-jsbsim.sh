#!/usr/bin/env bash
set -Eeuo pipefail

IMAGE="${PX4_DOCKER_IMAGE:-px4io/px4-dev-simulation-jammy:latest}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-$PWD/PX4-Autopilot}"
PX4_TARGET="${PX4_TARGET:-px4_sitl}"
PX4_SIMULATOR="${PX4_SIMULATOR:-jsbsim_rascal}"
PX4_MAKE_ARGS="${PX4_MAKE_ARGS:-}"
PX4_DOCKER_NETWORK="${PX4_DOCKER_NETWORK:-auto}"

command -v docker >/dev/null ||
  { echo "Docker is required." >&2; exit 1; }

[[ -f "$PX4_SOURCE_DIR/Makefile" ]] ||
  { echo "Invalid PX4_SOURCE_DIR: $PX4_SOURCE_DIR" >&2; exit 1; }

docker pull "$IMAGE"

docker_os="$(docker info --format '{{.OperatingSystem}}' 2>/dev/null || true)"
case "$PX4_DOCKER_NETWORK" in
  host)
    network_args=(--network host)
    ;;
  bridge)
    network_args=(-p 14540:14540/udp -p 14550:14550/udp -p 4560:4560/tcp)
    ;;
  auto)
    if [[ "$(uname -s)" == Linux && "$docker_os" != *Docker\ Desktop* ]]; then
      network_args=(--network host)
    else
      network_args=(-p 14540:14540/udp -p 14550:14550/udp -p 4560:4560/tcp)
    fi
    ;;
  *)
    echo "PX4_DOCKER_NETWORK must be auto, host, or bridge." >&2
    exit 1
    ;;
esac

exec docker run --rm -it \
  "${network_args[@]}" \
  --ipc host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e HEADLESS=1 \
  -v "$PX4_SOURCE_DIR:/src/PX4-Autopilot" \
  -w /src/PX4-Autopilot \
  "$IMAGE" \
  bash -lc "make $PX4_TARGET $PX4_SIMULATOR $PX4_MAKE_ARGS"
