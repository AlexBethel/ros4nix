# Robot-centric elevation mapping for rough terrain navigation.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros = {
    elevationMapping = {
      build = mkOption {
        type = types.bool;
        default = false;
      };

      # TODO: we need support for actually *running*
      # elevation_mapping.
    };
  };

  config = (mkIf config.services.ros.elevationMapping.build {
    services.ros = {
      enable = true;

      libraries.kindrRos.enable = true;
      libraries.messageLogger.enable = true;
    };

    programs.ros = {
      packages = [
        "grid-map"
        "eigen-conversions"
      ];

      ubuntuPackages = [
        "libpcl-dev"
        "libeigen3-dev"
      ];

      buildPackages.elevation_mapping =
        let
          repo = pkgs.fetchFromGitHub {
            owner = "ANYbotics";
            repo = "elevation_mapping";
            # version 0.7.17; elevation_mapping doesn't use tags in
            # any meaningful way.
            rev = "e841b8b";
            sha256 = "sha256-DnaCICYEREnicZHccBTm9V4yb8tH5Rlw3A34R8ZS3IY=";
          };
        in
        "${repo}/elevation_mapping";
    };
  });
}
