# Options config for drone physical characteristics
{ lib, hostname, ... }:

let
  inherit (lib) mkOption types;
in
{
  options.drone = {
    name = mkOption {
      type = types.str;
      default = hostname;
    };

    frame = {
      wheelbaseMm = mkOption {
        type = types.number;
      };

      dryMassGrams = mkOption {
        type = types.number;
      };
    };

    motors = {
      count = mkOption {
        type = types.int;
        default = 4;
      };

      kv = mkOption {
        type = types.number;
      };
    };

    propellers = {
      diameterInches = mkOption {
        type = types.number;
      };

      pitchInches = mkOption {
        type = types.number;
      };
    };

    battery = {
      cells = mkOption {
        type = types.int;
      };

      capacityMah = mkOption {
        type = types.number;
      };

      massGrams = mkOption {
        type = types.number;
      };
    };
  };
}
