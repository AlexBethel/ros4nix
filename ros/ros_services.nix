# Generic ROS service management.
{ config, lib, pkgs, ... }:

with lib;
{
  options.services.ros = {
    enable = mkOption {
      type = types.bool;
      default = false;
      description = ''
        Enable the ROS subsystem, and enable a ROS master systemd
        service if `services.ros.masterUri` is not the default.
      '';
    };

    masterUri = mkOption {
      type = types.str;
      default = "http://localhost:11311";
      description = ''
        ROS master URI.
      '';
    };

    realsense2 = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Intel Realsense camera.
        '';
      };

      options = {
        mode = mkOption {
          type = types.str;
          default = "preset";
          description = ''
            Specify the mode to start camera streams. Mode comprises of
            height, width and fps. Preset mode enables default values whereas
            Manual mode enables the specified parameter values.
          '';
        };

        serial_no = mkOption {
          type = types.str;
          default = "";
          description = ''
            Specify the serial_no to uniquely connect to a camera, especially
            if multiple cameras are detected by the nodelet. You may get the
            serial_no from the info stream by launching the default launch
            file.
          '';
        };

        usb_port_id = mkOption {
          type = types.str;
          default = "";
          description = ''
            Alternatively to serial_no, this can be used to connect to a
            camera by its USB Port ID, which is a Bus Number-Port Number in
            the format "Bus#-Port#". If used with serial_no, both must match
            correctly for camera to be connected.
          '';
        };

        camera_type = mkOption {
          type = types.str;
          default = "";
          description = ''
            Specify the type of the camera - "R200", "F200", "SR300" or
            "ZR300".
          '';
        };

        enable_ir = mkOption {
          type = types.bool;
          default = false;
          description = ''
            Specify if to enable or not the infrared camera(s). Note: On
            cameras with two Infrared streams, both will be enabled or
            disabled together.
          '';
        };

        enable_depth = mkOption {
          type = types.bool;
          default = true;
          description = ''
            Specify if to enable or not the depth camera.
          '';
        };

        depth_width = mkOption {
          type = types.nullOr types.int;
          default = null; # Depends on camera
          description = ''
            Specify the depth camera width resolution.
          '';
        };

        depth_height = mkOption {
          type = types.nullOr types.int;
          default = null; # Depends on camera
          description = ''
            Specify the depth camera height resolution.
          '';
        };

        depth_fps = mkOption {
          type = types.int;
          default = 60;
          description = ''
            Specify the depth camera FPS.
          '';
        };

        enable_color = mkOption {
          type = types.bool;
          default = true;
          description = ''
            Specify if to enable or not the color camera.
          '';
        };

        color_width = mkOption {
          type = types.int;
          default = 640;
          description = ''
            Specify the color camera width resolution.
          '';
        };

        color_height = mkOption {
          type = types.int;
          default = 480;
          description = ''
            Specify the color camera height resolution.
          '';
        };

        color_fps = mkOption {
          type = types.int;
          default = 60;
          description = ''
            Specify the color camera FPS.
          '';
        };

        enable_fisheye = mkOption {
          type = types.bool;
          default = true;
          description = ''
            Available only for ZR300 cameras. Specify if to enable or not the
            fisheye camera.
          '';
        };

        fisheye_width = mkOption {
          type = types.int;
          default = 640;
          description = ''
            Available only for ZR300 cameras. Specify the fisheye camera width
            resolution.
          '';
        };

        fisheye_height = mkOption {
          type = types.int;
          default = 480;
          description = ''
            Available only for ZR300 cameras. Specify the fisheye camera
            height resolution.
          '';
        };

        fisheye_fps = mkOption {
          type = types.int;
          default = 60;
          description = ''
            Available only for ZR300 cameras. Specify the fisheye camera FPS.
          '';
        };

        enable_imu = mkOption {
          type = types.bool;
          default = true;
          description = ''
            Available only for ZR300 cameras. Specify if to enable or not the
            IMU sensor.
          '';
        };

        enable_pointcloud = mkOption {
          type = types.bool;
          default = false;
          description = ''
            Specify if to enable or not the native pointcloud. By default, it
            is set to false due to performance issues. This option is
            depreciated in favor of the rgbd_launch pointcloud and will be
            removed in the near future.
          '';
        };

        enable_tf = mkOption {
          type = types.bool;
          default = true;
          description = ''
            Specify if to enable or not the transform frames publication.
          '';
        };

        enable_tf_dynamic = mkOption {
          type = types.bool;
          default = false;
          description = ''
            Publish transform frames as dynamic; default is false = Static,
            true = Dynamic.
          '';
        };

        base_frame_id = mkOption {
          type = types.str;
          default = "camera_link";
          description = ''
            Specify the base frame id of the camera.
          '';
        };

        depth_frame_id = mkOption {
          type = types.str;
          default = "camera_depth_frame";
          description = ''
            Specify the depth frame id of the camera.
          '';
        };

        depth_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_depth_optical_frame";
          description = ''
            Specify the depth optical frame id of the camera.
          '';
        };

        color_frame_id = mkOption {
          type = types.str;
          default = "camera_rgb_frame";
          description = ''
            Specify the color frame id of the camera.
          '';
        };

        color_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_rgb_optical_frame";
          description = ''
            Specify the color optical frame id of the camera.
          '';
        };

        ir_frame_id = mkOption {
          type = types.str;
          default = "camera_ir_frame";
          description = ''
            Specify the IR frame id of the camera.
          '';
        };

        ir_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_ir_optical_frame";
          description = ''
            Specify the IR optical frame id of the camera.
          '';
        };

        ir2_frame_id = mkOption {
          type = types.str;
          default = "camera_ir2_frame";
          description = ''
            Available only for R200 and ZR300 cameras. Specify the IR2 frame
            id of the camera.
          '';
        };

        ir2_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_ir2_optical_frame";
          description = ''
            Available only for R200 and ZR300 cameras. Specify the IR2 optical
            frame id of the camera.
          '';
        };

        fisheye_frame_id = mkOption {
          type = types.str;
          default = "camera_fisheye_frame";
          description = ''
            Available only for ZR300 camera. Specify the fisheye frame id of
            the camera.
          '';
        };

        fisheye_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_fisheye_optical_frame";
          description = ''
            Available only for ZR300 camera. Specify the fisheye optical frame
            id of the camera.
          '';
        };

        imu_frame_id = mkOption {
          type = types.str;
          default = "camera_imu_frame";
          description = ''
            Available only for ZR300 camera. Specify the IMU frame id of the
            camera.
          '';
        };

        imu_optical_frame_id = mkOption {
          type = types.str;
          default = "camera_imu_optical_frame";
          description = ''
            Available only for ZR300 camera. Specify the IMU optical frame id
            of the camera.
          '';
        };
      };
    };

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

    rosbridge = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Enable rosbridge, a server for communication with ROS from
          outside processes.
        '';
      };
    };

    elevationMapping = {
      build = mkOption {
        type = types.bool;
        default = false;
      };
    };

    libraries = {
      messageLogger = mkOption {
        type = types.bool;
        default = false;
      };

      kindrRos = mkOption {
        type = types.bool;
        default = false;
      };

      kindr = mkOption {
        type = types.bool;
        default = false;
      };
    };

    launchServices = mkOption {
      description = ''
        Attribute set of services to launch with roslaunch. This
        should generally only be used to launch nodes that are
        provided by external libraries; writing Nix modules should be
        preferred over writing launch files for new code.
      '';
      type = with types; attrsOf (submodule {
        options = {
          packageName = mkOption {
            type = str;
            description = ''
              Name of the package containing the launch file.
            '';
          };

          launchFile = mkOption {
            type = str;
            description = ''
              Name of the launch file. This should almost always end
              with the string ".launch".
            '';
          };

          args = mkOption {
            type = attrsOf str;
            default = { };
            description = ''
              Set of arguments to pass to the launch file.
            '';
          };

          # Probably remapping arguments should go here as well? I'm
          # not sure how exactly remapping arguments work with
          # roslaunch.
        };
      });
    };

    runServices = mkOption {
      description = ''
        Attribute set of services to launch with rosrun.
      '';
      type = with types; attrsOf (submodule {
        options = {
          packageName = mkOption {
            type = str;
            description = ''
              Name of the package containing the executable.
            '';
          };

          executable = mkOption {
            type = str;
            description = ''
              Name of the executable to run.
            '';
          };

          remap = mkOption {
            type = attrsOf str;
            description = ''
              Attribute set of paths to remap.
            '';
          };

          namespace = mkOption {
            type = str;
            default = "";
            description = ''
              Value of the ROS_NAMESPACE environment variable. This
              gets appended to the start of most ROS paths that the
              node uses.
            '';
          };
        };
      });
    };
  };

  config = mkMerge [
    (mkIf (config.services.ros.enable) {
      programs.ros.enable = true;

      programs.ros.packages = [ "ros-core" ];

      programs.ros.ubuntuPackages = [
        # ros-core installs catkin_make, which has an undeclared
        # dependency on a C++ compiler.
        "g++"

        # We use catkin-tools for building the global catkin
        # workspace.
        "python3-catkin-tools"
      ];

      systemd.services.rosMaster = {
        wantedBy = [ "multi-user.target" ];
        path = [
          pkgs.coreutils
          pkgs.netcat
          pkgs.procps
        ];
        script = ''
          set -e

          # Start the roscore, and poll it 10 times per second until
          # it starts accepting connections.
          ${config.programs.ros.rootDir}/nixWrappers/roscore &
          while true; do
            ${pkgs.coreutils}/bin/sleep 0.1
            # Report failure if roscore isn't running anymore.
            if ! ps | grep -q roscore; then
              echo 'Failed to start roscore'
              exit 1
            fi

            # Report success if roscore is accepting connections.
            if echo exit | nc localhost 11311 > /dev/null; then
              break
            fi
          done
        '';
        serviceConfig.Type = "forking";
      };
    })

    (mkIf config.services.ros.realsense2.enable {
      services.ros.enable = true;

      programs.ros.packages = [ "realsense2-camera" ];

      systemd.services.rosRealsense = {
        wantedBy = [ "multi-user.target" ];
        after = [ "rosMaster.service" ];

        script =
          let
            opts = builtins.concatStringsSep " " (
              builtins.attrValues (builtins.mapAttrs
                (
                  name: value:
                    if value == true
                    then "${name}:=true"
                    else if value == false
                    then "${name}:=false"
                    else "${name}:=${builtins.toString value}"
                )
                config.services.ros.realsense2.options)
            );
          in
          ''
            /var/ros/nixWrappers/roslaunch realsense2_camera rs_camera.launch ${opts}
      #     '';
      };

      # services.ros.launchServices.realsense = {
      #   packageName = "realsense2_camera";
      #   launchFile = "rs_camera.launch";
      #   args = config.services.ros.realsense2.options;
      # };
    })

    (mkIf config.services.ros.usbCamera.enable {
      services.ros.enable = true;

      programs.ros.packages = [ "usb-cam" ];

      systemd.services.rosUsbCamera = {
        wantedBy = [ "multi-user.target" ];
        after = [ "rosMaster.service" ];

        script = ''
          /var/ros/nixWrappers/rosrun usb_cam usb_cam_node _video_device:=${config.services.ros.usbCamera.device}
        '';
      };
    })

    (mkIf config.services.ros.rosbridge.enable {
      services.ros.enable = true;

      programs.ros.packages = [ "rosbridge-server" ];

      systemd.services.rosBridge = {
        wantedBy = [ "multi-user.target" ];
        after = [ "rosMaster.service" ];

        script = ''
          /var/ros/nixWrappers/roslaunch rosbridge_server rosbridge_tcp.launch
        '';
      };
    })

    (mkIf config.services.ros.elevationMapping.build {
      services.ros = {
        enable = true;

        libraries.kindrRos = true;
        libraries.messageLogger = true;
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

    (mkIf config.services.ros.libraries.messageLogger {
      services.ros = {
        enable = true;
      };

      programs.ros = {
        buildPackages.message_logger =
            pkgs.fetchFromGitHub {
            owner = "ANYbotics";
            repo = "message_logger";
            rev = "0.2.0";
            sha256 = "sha256-X3/ckoLIr9glWT+S28buM2F3ftd3Fy4EMR4Mm4N4+Ws=";
          };
      };
    })

    (mkIf config.services.ros.libraries.kindrRos {
      services.ros = {
        enable = true;

        libraries.kindr = true;
      };

      programs.ros = {
        packages = [
          "pcl-ros"
          "tf-conversions"
        ];

        ubuntuPackages = [
          "libeigen3-dev"
        ];

        buildPackages.kindr_ros =
            pkgs.fetchFromGitHub {
            owner = "ANYbotics";
            repo = "kindr_ros";
            rev = "b5bf6e8";
            sha256 = "sha256-6v3sdXVKy6L8KKWHYmll0Lft6KMp3WQOe1/GV6FPE1M=";
          };
      };
    })

    (mkIf config.services.ros.libraries.kindr {
      services.ros = {
        enable = true;
      };

      programs.ros = {
        ubuntuPackages = [
          "libeigen3-dev"
        ];

        buildPackages.kindr =
            pkgs.fetchFromGitHub {
            owner = "ANYbotics";
            repo = "kindr";
            rev = "e18abfc";
            sha256 = "sha256-HZwCaG5wMoOdABhNnI5ele7VTJ/chCE/o4BVJlvum5o=";
          };
      };
    })
  ];
}
