# Support for Intel Realsense cameras.
{ config, lib, ... }:

with lib;
let rosLib = import ./rosLib.nix; in
{
  options.services.ros = {
    realsense2 = {
      enable = mkOption {
        type = types.bool;
        default = false;
        description = ''
          Intel Realsense camera.
        '';
      };

      # These options are copy-pasted straight from the ROS wiki with
      # few adaptations.
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
  };

  config = mkIf config.services.ros.realsense2.enable {
    programs.ros.packages = [ "realsense2-camera" ];

    services.ros = {
      enable = true;
      launchServices.realsense = {
        packageName = "realsense2_camera";
        launchFile = "rs_camera.launch";
        # options = config.services.ros.realsense2.options;
      };
    };
  };
}
