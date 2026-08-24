alias r := run
alias b := build
alias c := clean
alias d := develop
alias n := new
alias mv := rename
alias rn := rename

# default target, lists all targets available
list:
    @just --list

# runs the default master nix flake target
run:
    @nix run

# builds the nix targets
build:
    @nix build

# removes all build artifacts and binaries
clean:
    @rm -rf result target
    @ ./.build_utils/clean.py

# runs clean and destroys the machine's nix cache (used to free all system storage memory related to this project)
nuke: clean
    @nix-collect-garbage
    @rm -rf ardupilot

# creates a new node from the script, with NAME as the node name, and TEMPLATE as the template name inside of .build_utils/templates
new NAME TEMPLATE:
    @ ./.build_utils/create.py {{ NAME }} {{ TEMPLATE }}

# opens the nix shell
develop:
    @nix develop



PX4_DIR := justfile_directory() + "/tmp/px4"
PX4_REPO := "https://github.com/PX4/PX4-Autopilot.git"
PX4_TARGET := "jsbsim"
PYTHON := "python3"

clone-px4:
  @if [ ! -e "{{ PX4_DIR }}" ]; then \
    git clone --recursive "{{ PX4_REPO }}" "{{ PX4_DIR }}"; \
  elif [ -d "{{ PX4_DIR }}/.git" ]; then \
    cd "{{ PX4_DIR }}" && git submodule update --init --recursive; \
  else \
    echo "{{ PX4_DIR }} exists but is not a Git repository"; \
    exit 1; \
  fi

clean-sitl:
    @rm -rf "{{ PX4_DIR }}/build/px4_sitl_default"

setup-sitl: clone-px4
  @cd "{{ PX4_DIR }}" && \
    "{{ PYTHON }}" -m venv .venv && \
    .venv/bin/python -m pip install --upgrade pip && \
    .venv/bin/python -m pip install -r Tools/setup/requirements.txt

sitl: setup-sitl
  @cd "{{ PX4_DIR }}" && \
    PATH="$PWD/.venv/bin:$PATH" \
    make px4_sitl "{{ PX4_TARGET }}"

# Examples:
# just sitl PX4_TARGET=gz_x500
# just sitl PX4_TARGET=jsbsim_quadrotor_x

rename OLD NEW:
    @ ./.build_utils/clean.py {{ OLD }} {{ NEW }}
