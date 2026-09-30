{
<<<<<<<< HEAD:visualization/flake.nix
  description = "visualization — Python Zenoh Node";
========
  description = "telemetry — Python Zenoh Node";
>>>>>>>> main:telemetry/flake.nix
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    pyproject-nix = {
      url = "github:pyproject-nix/pyproject.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    uv2nix = {
      url = "github:pyproject-nix/uv2nix";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    pyproject-build-systems = {
      url = "github:pyproject-nix/build-system-pkgs";
      inputs.pyproject-nix.follows = "pyproject-nix";
      inputs.uv2nix.follows = "uv2nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };
  outputs =
    {
      nixpkgs,
      pyproject-nix,
      uv2nix,
      pyproject-build-systems,
      ...
    }:
    let
      inherit (nixpkgs) lib;
      forAllSystems = lib.genAttrs lib.systems.flakeExposed;
      workspace = uv2nix.lib.workspace.loadWorkspace { workspaceRoot = ./.; };
      overlay = workspace.mkPyprojectOverlay {
        sourcePreference = "wheel";
      };
      editableOverlay = workspace.mkEditablePyprojectOverlay {
        root = "$REPO_ROOT";
      };
      pythonSets = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          python = pkgs.python3;
        in
        (pkgs.callPackage pyproject-nix.build.packages {
          inherit python;
        }).overrideScope
          (
            lib.composeManyExtensions [
              pyproject-build-systems.overlays.wheel
              overlay
            ]
          )
      );
    in
    {
      devShells = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          pythonSet = pythonSets.${system}.overrideScope editableOverlay;
<<<<<<<< HEAD:visualization/flake.nix
          virtualenv = pythonSet.mkVirtualEnv "visualization-dev-env" workspace.deps.all;
========
          virtualenv = pythonSet.mkVirtualEnv "telemetry-dev-env" workspace.deps.all;
>>>>>>>> main:telemetry/flake.nix
        in
        {
          default = pkgs.mkShell {
            packages = [
              virtualenv
              pkgs.uv
            ];
            env = {
              UV_NO_SYNC = "1";
              UV_PYTHON = pythonSet.python.interpreter;
              UV_PYTHON_DOWNLOADS = "never";
            };
            shellHook = ''
              unset PYTHONPATH
<<<<<<<< HEAD:visualization/flake.nix
              export REPO_ROOT=$(git rev-parse --show-toplevel)/visualization
========
              export REPO_ROOT=$(git rev-parse --show-toplevel)/telemetry
>>>>>>>> main:telemetry/flake.nix
            '';
          };
        }
      );
<<<<<<<< HEAD:visualization/flake.nix
      packages = forAllSystems (system: {
        default = pythonSets.${system}.mkVirtualEnv "visualization-env" workspace.deps.default;
      });
========
      packages = forAllSystems (
        system:
        let
          pkgs = nixpkgs.legacyPackages.${system};
          pythonSet = pythonSets.${system};

          virtualenv = pythonSet.mkVirtualEnv "telemetry-env" workspace.deps.default;

          telemetry = pkgs.writeShellApplication {
            name = "telemetry";
            text = ''
              exec ${virtualenv}/bin/telemetry "$@"
            '';
          };
        in
        {
          default = telemetry;
          inherit telemetry;
        }
      );
>>>>>>>> main:telemetry/flake.nix
    };
}
