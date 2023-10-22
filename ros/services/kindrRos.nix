# Kinematics and Dynamics for Robotics, ROS binding.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros.libraries = {
    kindrRos = {
      enable = mkOption {
        type = types.bool;
        default = false;
      };
    };
  };

  config = (mkIf config.services.ros.libraries.kindrRos.enable {
    services.ros.libraries.kindr.enable = true;

    programs.ros = {
      enable = true;

      packages = [ "pcl-ros" "tf-conversions" ];
      ubuntuPackages = [ "libeigen3-dev" ];
      buildPackages.kindr_ros =
        pkgs.fetchFromGitHub {
          owner = "ANYbotics";
          repo = "kindr_ros";
          rev = "b5bf6e8";
          sha256 = "sha256-6v3sdXVKy6L8KKWHYmll0Lft6KMp3WQOe1/GV6FPE1M=";
        };
    };
  });
}

