# Autonomous navigation.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros = {
    moveBase = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Autonomous navigation.
        '';
      };
    };
  };

  config = mkIf config.services.ros.moveBase.enable {
    programs.ros.packages = [ "move-base" ];

    services.ros = {
      enable = true;
      runServices.moveBase = {
        packageName = "move_base";
        executable = "move_base";
      };
    };
  };
}
