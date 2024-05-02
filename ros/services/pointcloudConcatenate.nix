# Combining point clouds.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros = {
    pointcloudConcatenate = {
      build = mkOption {
        type = types.bool;
        default = false;
      };
    };
  };

  config =
    mkMerge [
      (mkIf config.services.ros.pointcloudConcatenate.build {
        programs.ros = {
          enable = true;
          packages = [ "pcl-ros" ];
          ubuntuPackages = [ "libpcl-dev" ];
          buildPackages.pointcloud_concatenate = pkgs.fetchFromGitHub {
            owner = "aseligmann";
            repo = "pointcloud_concatenate";
            rev = "c872586";
            sha256 = "sha256-RdMAvgoHQUZbOYCNCgdBf8Aa408q0/1bD5SbYT6NkP0=";
          };
        };
      })
    ];
}
