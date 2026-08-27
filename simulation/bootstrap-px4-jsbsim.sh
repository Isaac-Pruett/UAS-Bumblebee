#!/usr/bin/env bash
# Bootstrap a pinned PX4 checkout, then start the Docker-backed JSBSim SITL.

set -Eeuo pipefail

PX4_REPO_URL="${PX4_REPO_URL:-https://github.com/PX4/PX4-Autopilot.git}"
PX4_REF="${PX4_REF:-main}"
PX4_SOURCE_DIR="${PX4_SOURCE_DIR:-$PWD/.px4/PX4-Autopilot}"
PX4_LAUNCHER="${PX4_LAUNCHER:-$(dirname "${BASH_SOURCE[0]}")/run-px4-jsbsim.sh}"

if ! command -v git >/dev/null 2>&1; then
  echo 'git is required but was not found in PATH.' >&2
  exit 1
fi

if [[ ! -x "$PX4_LAUNCHER" ]]; then
  echo "PX4 launcher is missing or not executable: $PX4_LAUNCHER" >&2
  exit 1
fi

if [[ ! -e "$PX4_SOURCE_DIR/.git" ]]; then
  if [[ -e "$PX4_SOURCE_DIR" ]]; then
    echo "PX4_SOURCE_DIR exists but is not a PX4 git checkout: $PX4_SOURCE_DIR" >&2
    exit 1
  fi

  echo "Cloning PX4 ($PX4_REF) into $PX4_SOURCE_DIR"
  mkdir -p "$(dirname "$PX4_SOURCE_DIR")"
  git clone --recursive --branch "$PX4_REF" "$PX4_REPO_URL" "$PX4_SOURCE_DIR"
else
  echo "Reusing PX4 checkout: $PX4_SOURCE_DIR"
  git -C "$PX4_SOURCE_DIR" submodule update --init --recursive
fi

export PX4_SOURCE_DIR
exec "$PX4_LAUNCHER"
