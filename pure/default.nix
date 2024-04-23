{ pkgs ? import <nixpkgs> { } }:

let
  ros = import ./ros.nix { inherit pkgs; };

  unfuckSource = source: pkgs.runCommand "unfucked" {} ''
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
in
ros.withPackages [ ] [
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
]
