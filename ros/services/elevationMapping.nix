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

      enable = mkOption {
        type = types.bool;
        default = false;
      };

      pointClouds = mkOption {
        type = types.listOf types.str;
        description = ''
          Names of point cloud sensors to build the map out of.
        '';
      };

      trackPointFrameId = mkOption {
        type = types.str;
        description = ''
          I currently have no idea what this means.
        '';
      };
    };
  };

  config =
    let
      rosLib = import ./rosLib.nix { inherit lib; };
      pointclouds = config.services.ros.elevationMapping.pointClouds;
      mkPointcloudEntry = sourceName: {
        type = "pointcloud";
        topic = "/${sourceName}/depth/color/points";
        queue_size = 2;
        publish_on_update = true;
        sensor_processor = {
          type = "structured_light";
          ignore_points_above = 0.3;
          ignore_points_below = -0.5;
        };
      };
      inputSources = rosLib.mapIntoAttrs
        (name: {
          inherit name;
          value = mkPointcloudEntry name;
        })
        pointclouds;
      mappingConfig = {
        input_sources = inputSources;

        map_frame_id = "map";
        robot_base_frame_id = "base_link";
        num_callback_threads = 4;

        robot_pose_with_covariance_topic = "/pose_with_covariance";
        track_point_frame_id = config.services.ros.elevationMapping.trackPointFrameId;
        track_point_x = 0.0;
        track_point_y = 0.0;
        track_point_z = 0.0;

        # Map.
        length_in_x = 7.5;
        length_in_y = 7.5;
        position_x = 0.0;
        position_y = 0.0;
        resolution = 0.025;
      };
    in
    mkMerge [
      (mkIf config.services.ros.elevationMapping.build {
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
      })

      (mkIf config.services.ros.elevationMapping.enable {
        services.ros = {
          enable = true;
          elevationMapping.build = true;

          runServices.elevationMapping = {
            packageName = "elevation_mapping";
            executable = "elevation_mapping";
            rosParams = pkgs.writeText "elevation_mapping.yaml" (
              # use JSON because it's a subset of YAML and easier to
              # generate from Nix.
              builtins.toJSON mappingConfig
            );
          };
        };
      })
    ];
}
