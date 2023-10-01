# Support for generic V4L USB cameras.
{ config, lib, ... }:

with lib;
{
  options.services.ros = {
    usbCamera = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          V4L USB camera.
        '';
      };

      device = mkOption {
        type = types.str;
        default = "/dev/video0";
        description = ''
          Device to stream video from.
        '';
      };
    };
  };

  config = (mkIf config.services.ros.usbCamera.enable {
    programs.ros.packages = [ "usb-cam" ];

    services.ros = {
      enable = true;
      runServices.usbCamera = {
        packageName = "usb_cam";
        executable = "usb_cam_node";
        remap._video_device = config.services.ros.usbCamera.device;
      };
    };
  });
}
