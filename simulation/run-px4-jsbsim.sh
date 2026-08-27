#!/usr/bin/env bash
# Run one PX4/JSBSim SITL instance in Docker with a host-reachable MAVLink GCS
# endpoint. Start this script once per vehicle, changing PX4_INSTANCE each time.

set -Eeuo pipefail

IMAGE="${PX4_DOCKER_IMAGE:-px4io/px4-dev-simulation-jammy:latest}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-$PWD/PX4-Autopilot}"
PX4_TARGET="${PX4_TARGET:-px4_sitl}"
PX4_SIMULATOR="${PX4_SIMULATOR:-jsbsim_rascal}"
PX4_MAKE_ARGS="${PX4_MAKE_ARGS:-}"
PX4_DOCKER_NETWORK="${PX4_DOCKER_NETWORK:-auto}"
PX4_INSTANCE="${PX4_INSTANCE:-0}"

if ! [[ "$PX4_INSTANCE" =~ ^[0-9]+$ ]]; then
  echo "PX4_INSTANCE must be a non-negative integer." >&2
  exit 1
fi

# PX4's POSIX startup script listens for GCS MAVLink on 18570 + instance.
# Docker exposes that listener on a different host port so QGC can use its own
# local UDP port. Do not publish 14540 or 14550: host-side QGC/MAVSDK listeners
# need those ports themselves.
PX4_GCS_LOCAL_PORT="${PX4_GCS_LOCAL_PORT:-$((18570 + PX4_INSTANCE))}"
PX4_GCS_HOST_PORT="${PX4_GCS_HOST_PORT:-$((14570 + PX4_INSTANCE))}"

command -v docker >/dev/null || {
  echo "Docker is required." >&2
  exit 1
}

[[ -f "$PX4_SOURCE_DIR/Makefile" ]] || {
  echo "Invalid PX4_SOURCE_DIR: $PX4_SOURCE_DIR" >&2
  exit 1
}

docker pull "$IMAGE"

docker_os="$(docker info --format '{{.OperatingSystem}}' 2>/dev/null || true)"
case "$PX4_DOCKER_NETWORK" in
  host)
    network_args=(--network host)
    ;;
  bridge)
    network_args=(-p "${PX4_GCS_HOST_PORT}:${PX4_GCS_LOCAL_PORT}/udp")
    ;;
  auto)
    if [[ "$(uname -s)" == Linux && "$docker_os" != *Docker\ Desktop* ]]; then
      network_args=(--network host)
    else
      network_args=(-p "${PX4_GCS_HOST_PORT}:${PX4_GCS_LOCAL_PORT}/udp")
    fi
    ;;
  *)
    echo "PX4_DOCKER_NETWORK must be auto, host, or bridge." >&2
    exit 1
    ;;
esac

echo "PX4 instance $PX4_INSTANCE GCS endpoint: udp://127.0.0.1:${PX4_GCS_HOST_PORT}"
echo "Maps host UDP $PX4_GCS_HOST_PORT to PX4 UDP $PX4_GCS_LOCAL_PORT in Docker."

exec docker run --rm -it \
  "${network_args[@]}" \
  --ipc host \
  --user "$(id -u):$(id -g)" \
  -e HOME=/tmp \
  -e HEADLESS=1 \
  -e PX4_INSTANCE="$PX4_INSTANCE" \
  -v "$PX4_SOURCE_DIR:/src/PX4-Autopilot" \
  -w /src/PX4-Autopilot \
  "$IMAGE" \
  bash -lc "make $PX4_TARGET $PX4_SIMULATOR $PX4_MAKE_ARGS"
