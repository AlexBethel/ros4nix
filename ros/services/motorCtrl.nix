# Autonomous navigation.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros = {
    motorCtrl = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Custom motor controller.
        '';
      };
    };
  };

  config = mkIf config.services.ros.motorCtrl.enable {
    programs.ros.buildPackages.motor_ctrl = ./motor_ctrl;

    services.ros = {
      enable = true;
      runServices.motorCtrl = {
        packageName = "motor_ctrl";
        executable = "motor_ctrl_node.py";
      };
    };
  };
}
