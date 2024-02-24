# A TCP server for allowing non-ROS programs to communicate with a ROS
# network.
{ config, lib, ... }:
with lib;
{
  options.services.ros = {
    rosbridge = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Enable rosbridge, a server for communication with ROS from
          outside processes.
        '';
      };

      mode = mkOption {
        type = types.enum [
          "tcp"
          "udp"
          "websocket"
        ];
        default = "tcp";
        description = ''
          Type of service to provide over the socket.
        '';
      };
    };
  };

  config = (mkIf config.services.ros.rosbridge.enable {
    services.ros.enable = true;

    programs.ros.packages = [ "rosbridge-server" ];

    services.ros.launchServices.rosbridge = {
      packageName = "rosbridge_server";
      launchFile = "rosbridge_${config.services.ros.rosbridge.mode}.launch";
    };
  });
}
