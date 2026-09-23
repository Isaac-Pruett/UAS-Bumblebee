# Running the full PX4 simulation

This guide installs the two host-side tools needed to run this repository's
full simulation:

- [Nix](https://nixos.org/download/) supplies the pinned build and runtime
  environment declared by `flake.nix`.
- [Foxglove Desktop](https://foxglove.dev/download) displays the simulation's
  telemetry and recorded/live data.

Use a shell in the environment where Nix and the repository are installed. Nix
runs natively on Linux and macOS; on Windows, use a WSL2 Linux distribution and
run every Nix and Git command inside that distribution. Foxglove Desktop runs
separately on its supported desktop platform and can connect to a data source
reachable from the simulation environment.

The top-level flake declares Linux (x86_64 and ARM64) and macOS (Intel and
Apple Silicon) systems. That declares evaluation support, not a guarantee that
the PX4 SIH toolchain builds or runs on each host. Treat the first run on a new
platform as the compatibility check, and retain its complete error output.

## 1. Install Foxglove Desktop

1. Download and install **Foxglove Desktop** from
   <https://foxglove.dev/download>.
2. Start Foxglove once so the operating system can complete its normal
   first-launch checks.
   Signing in is optional unless the project uses Foxglove Cloud.

The desktop application—not the `foxglove` command-line client—is the relevant
viewer. The flakes currently do **not** launch Foxglove, add it to their runtime
inputs, create an MCAP recording, or declare a Foxglove WebSocket endpoint.
Install it now, but obtain the telemetry connection details from the telemetry
node's own documentation or output before attempting to connect it.

## 2. Install Nix

Install Nix using the current instructions at <https://nixos.org/download/>.
Choose the installer path for the host environment:

- Linux: use the recommended multi-user installation when the system meets its
  prerequisites, otherwise use the documented single-user installation.
- macOS: use the official macOS installer path.
- Windows: install Nix **inside WSL2**, not in Command Prompt or PowerShell.
  Enable systemd in WSL first if using Nix's multi-user installer.

The multi-user installation runs a Nix daemon and gives stronger build
isolation; the single-user installation is a valid alternative where a daemon
cannot be used.

After the installer finishes, open a **new shell session** (inside WSL2 on
Windows) and confirm Nix is available:

```sh
nix --version
```

If `nix` is not found, follow the installer's shell-setup instructions and open
another new shell session. Do not run the simulation until this succeeds.

## 3. Enable the Nix command and flakes

`nix run .#full_simulation` uses the modern `nix` command and the experimental
flake feature. Enable both permanently in the Nix configuration for the
environment that runs the simulation. For standalone Nix, that is normally
`$XDG_CONFIG_HOME/nix/nix.conf` (by default, `~/.config/nix/nix.conf`); create
the directory and file if necessary:

```ini
experimental-features = nix-command flakes
```

Confirm it works:

```sh
nix flake --help
```

For a one-off run, the equivalent is:

```sh
nix --extra-experimental-features 'nix-command flakes' run .#full_simulation
```

Permanent configuration is preferred so every repository command behaves the
same way. NixOS users should set this declaratively instead:

```nix
nix.settings.experimental-features = [ "nix-command" "flakes" ];
```

Reference: the [NixOS flakes guide](https://wiki.nixos.org/wiki/Flakes).

## 4. PX4 submodule: handled automatically

This repository's `.gitmodules` defines `px4` as a submodule at
`https://github.com/PolyUAS/PX4-mods.git`. The `simSwarm` program that powers
`full_simulation` explicitly performs both of these commands before it checks
for PX4 or starts a build:

```sh
git submodule sync --recursive
git submodule update --init --recursive
```

Therefore, a normal `nix run .#full_simulation` recursively initializes the PX4
submodule automatically. You do not need to run the commands separately for
the usual first run. They remain useful as a quick manual recovery step if the
checkout was interrupted or PX4 is missing:

```sh
git submodule status --recursive
```

Every entry should show a commit SHA with no leading `-`. A leading `-` means
the submodule is not initialized; run the update command again. A leading `+`
means the checkout is at a different commit than the parent repository expects;
ask before discarding local work in that submodule.

## 5. Run the simulation

From the repository root—the directory containing `flake.nix`—run:

```sh
nix run .#full_simulation
```

What to expect:

1. Nix downloads/builds the flake's pinned dependencies. This can take a while
   and use substantial disk space; later runs reuse cached build outputs.
2. `simSwarm` synchronizes and recursively initializes Git submodules, creates
   `px4/.venv` when needed, and installs PX4's Python requirements with `pip`.
   The first run consequently also needs network access for PX4 Python packages.
3. It builds `px4_sitl_sih`, then launches three PX4 SIH vehicles (`luke`,
   `akshay`, and `reagan`) at the configured Atlanta-area origin and offsets.
4. The wrapper starts one local Zenoh router on `tcp/127.0.0.1:7447`, the
   telemetry node, and one identity/MAVLink stack per vehicle.
5. The current wrapper does not expose Foxglove itself. Inspect the telemetry
   node's log/output for its supported data source, then use Foxglove Desktop's
   matching connection type (for example, a WebSocket URL if it provides one).
6. Stop the run with `Ctrl-C`. The wrapper sends termination signals to the
   processes it directly launched; if child processes survive,
   use the project's documented cleanup command rather than deleting build
   outputs manually.

## Troubleshooting

- **`experimental Nix feature 'flakes' is disabled`** — add the `nix.conf`
  setting from step 3, open a new shell session, and retry.
- **`path ... does not contain a 'flake.nix'`** — `cd` to the repository root
  before running the command.
- **PX4 source/file missing** — the automatic submodule setup failed (usually
  network access or repository credentials). Run the two commands in step 4 to
  see the Git error directly.
- **Unsupported system / missing binary cache** — a flake system declaration
  does not guarantee the PX4 SIH toolchain is available on that host. Inspect
  the error before changing the lock file or substituting dependencies. On
  Windows, verify that the command is running inside WSL2.
- **Foxglove has no data** — confirm the simulator is still running, use the
  exact URL it printed, and check that the selected transport (for example
  Foxglove WebSocket) matches the project output.

## Update workflow

When pulling repository updates that modify `flake.lock` or `.gitmodules`, the
next full simulation run re-synchronizes the submodules automatically. To do it
ahead of time, run:

```sh
git submodule update --init --recursive
nix run .#full_simulation
```

Do not run `nix flake update` as part of ordinary setup: it changes the pinned
dependency versions and should be a deliberate repository maintenance change.
