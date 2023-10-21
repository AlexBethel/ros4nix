# A service to continually replay a rosbag recording.
{ config, lib, ... }:

with lib;
{
  options.services.ros = {
    bagPlayer = {
      file = mkOption {
        type = types.nullOr types.path;
        default = null;
        description = ''
          Path of a ROS bag file to continually re-play, as spoofed
          sensor data.
        '';
      };

      topics = mkOption {
        type = with types; nullOr (listOf str);
        default = null;
        description = ''
          List of ROS topics to re-play from the bag file. If this
          option is `null`, then every topic contained in the bag file
          will be looped forever.
        '';
      };
    };
  };

  config = mkIf (config.services.ros.bagPlayer.file != null) {
    services.ros = {
      enable = true;
      runServices.bagPlayer = {
        packageName = "rosbag";
        executable = "play";
        rawArgs = [
          # Loop playback forever.
          "-l"

          # Load the file we need.
          "--bags"
          "${config.services.ros.bagPlayer.file}"
        ] ++ (if config.services.ros.bagPlayer.topics != null
        then [ "--topics" ] ++ config.services.ros.bagPlayer.topics
        else [ ]);
      };
    };
  };
}
