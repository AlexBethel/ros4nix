# Message logger for ANYbotics's things.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros.libraries = {
    messageLogger = {
      enable = mkOption {
        type = types.bool;
        default = false;
      };
    };
  };

  config = (mkIf config.services.ros.libraries.messageLogger.enable {
    programs.ros = {
      enable = true;

      buildPackages.message_logger =
        pkgs.fetchFromGitHub {
          owner = "ANYbotics";
          repo = "message_logger";
          rev = "0.2.0";
          sha256 = "sha256-X3/ckoLIr9glWT+S28buM2F3ftd3Fy4EMR4Mm4N4+Ws=";
        };
    };
  });
}

