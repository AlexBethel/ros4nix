{ pkgs ? import <nixpkgs> { } }:

let
  ros = import ./ros.nix { inherit pkgs; };

  unfuckSource = source: pkgs.runCommand "unfucked" { } ''
    cp -r ${source} $out
    chmod -R u+w $out

    for file in $(find $out | grep main_bus.hpp); do
      rm $file
      cp ${./main_bus.hpp} $file
    done
  '';

  actuator_ctrl = ros.buildRosPackage {
    name = "actuator_ctrl";
    src = unfuckSource ./src/actuator_ctrl;
    buildInputs = [ can_raw ];
  };

  can_convert = ros.buildRosPackage {
    name = "can_convert";
    src = unfuckSource ./src/can_convert;
    buildInputs = [ can_raw ];
  };

  can_raw = ros.buildRosPackage {
    name = "can_raw";
    src = ./src/can_raw;
  };

  can_raw_input = ros.buildRosPackage {
    name = "can_raw_input";
    src = ./src/can_raw_input;
    buildInputs = [ can_raw ];
  };

  control = ros.buildRosPackage {
    name = "control";
    src = ./src/control;
  };

  heartbeat = ros.buildRosPackage {
    name = "heartbeat";
    src = ./src/heartbeat;
  };

  hud = ros.buildRosPackage {
    name = "hud";
    src = ./src/hud;
    buildInputs = [ can_convert ];
    extraDebPackages = [
      "ros-noetic-cv-bridge"
      "ros-noetic-image-transport"
      "ros-noetic-theora-image-transport"
    ];
  };

  leds = ros.buildRosPackage {
    name = "leds";
    src = unfuckSource ./src/leds;
    buildInputs = [ can_raw ];
  };

  mapping = ros.buildRosPackage {
    name = "mapping";
    src = ./src/mapping;
    extraDebPackages = [
      "ros-noetic-grid-map"
    ];
  };

  motor_ctrl = ros.buildRosPackage {
    name = "motor_ctrl";
    src = unfuckSource ./src/motor_ctrl;
    buildInputs = [ can_raw ];
  };

  kindr = ros.buildRosPackage rec {
    name = "kindr";
    src = pkgs.fetchFromGitHub {
      owner = "ANYbotics";
      repo = "kindr";
      rev = "e18abfc";
      sha256 = "sha256-HZwCaG5wMoOdABhNnI5ele7VTJ/chCE/o4BVJlvum5o=";
    };
    extraDebPackages = [ "libeigen3-dev" ];
  };

  kindr_ros = ros.buildRosPackage rec {
    name = "kindr_ros";
    src = pkgs.fetchFromGitHub {
      owner = "ANYbotics";
      repo = "kindr_ros";
      rev = "b5bf6e8";
      sha256 = "sha256-6v3sdXVKy6L8KKWHYmll0Lft6KMp3WQOe1/GV6FPE1M=";
    };
    buildInputs = [ kindr ];
    extraDebPackages = [
      "ros-noetic-rviz"
      "ros-noetic-pcl-ros"
      "ros-noetic-tf-conversions"
    ];
  };

  message_logger = ros.buildRosPackage rec {
    name = "message_logger";
    src = pkgs.fetchFromGitHub {
      owner = "ANYbotics";
      repo = "message_logger";
      rev = "0.2.0";
      sha256 = "sha256-X3/ckoLIr9glWT+S28buM2F3ftd3Fy4EMR4Mm4N4+Ws=";
    };
  };


  elevation_mapping = ros.buildRosPackage rec {
    name = "elevation_mapping";
    src =
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

    buildInputs = [ kindr kindr_ros message_logger ];

    extraDebPackages = [
      "libpcl-dev"
      "libeigen3-dev"
      "ros-noetic-grid-map"
      "ros-noetic-eigen-conversions"
    ];
  };

  apriltag-ros = ros.buildRosPackage {
    name = "apriltag-ros";
    extraDebPackages = [ "ros-noetic-apriltag-ros" ];
    src = pkgs.emptyDirectory;
  };

  usb-cam = ros.buildRosPackage {
    name = "usb-cam";
    extraDebPackages = [ "ros-noetic-usb-cam" ];
    src = pkgs.emptyDirectory;
  };
in
ros.withPackages [
  actuator_ctrl
  can_convert
  can_raw
  can_raw_input
  control
  heartbeat
  hud
  leds
  mapping
  motor_ctrl
  elevation_mapping

  apriltag-ros
  usb-cam
]
