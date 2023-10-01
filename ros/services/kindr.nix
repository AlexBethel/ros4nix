# Kinematics and Dynamics for Robotics.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros.libraries = {
    kindr = {
      enable = mkOption {
        type = types.bool;
        default = false;
      };
    };
  };

  config = (mkIf config.services.ros.libraries.kindr.enable {
    programs.ros = {
      enable = true;

      ubuntuPackages = [ "libeigen3-dev" ];
      buildPackages.kindr =
        pkgs.fetchFromGitHub {
          owner = "ANYbotics";
          repo = "kindr";
          rev = "e18abfc";
          sha256 = "sha256-HZwCaG5wMoOdABhNnI5ele7VTJ/chCE/o4BVJlvum5o=";
        };
    };
  });
}

