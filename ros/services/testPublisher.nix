# A simple service that publishes a string to a ROS topic.
{ config, lib, ... }:
with lib;
{
  options.services.ros = {
    testPublisher = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Enable a test publisher service, that sends a string to a
          ROS topic once per second.
        '';
      };
    };
  };

  config = (mkIf config.services.ros.testPublisher.enable) {
    services.ros.enable = true;

    systemd.services.ros-testPublisher = {
      wantedBy = [ "multi-user.target" ];
      after =
        if config.programs.ros.master != null
        then [ "rosMaster.service" ]
        else [ ];

      script = ''
        ${config.programs.ros.rootDir}/nixWrappers/rostopic pub \
            -r 1 /testPublisher std_msgs/String "testPublisher is working!"
      '';
    };
  };
}
