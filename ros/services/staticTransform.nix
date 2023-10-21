# Services for running static transform publishers.
{ config, lib, ... }:

with lib;
let rosLib = import ./rosLib.nix { inherit lib; }; in
{
  options.services.ros = {
    staticTransforms = mkOption {
      type = types.listOf (types.submodule {
        options = {
          parent = mkOption {
            description = "Parent frame.";
            type = types.str;
          };
          child = mkOption {
            description = "Child frame.";
            type = types.str;
          };

          x = mkOption {
            description = "X offset.";
            type = types.float;
            default = 0.0;
          };
          y = mkOption {
            description = "Y offset.";
            type = types.float;
            default = 0.0;
          };
          z = mkOption {
            description = "Z offset.";
            type = types.float;
            default = 0.0;
          };

          # TODO: replace these with arbitrary sequenced rotations. I
          # have no idea how to implement those, but we should be able
          # to write:
          # rotation = [
          #   (rotateZ 90)
          #   (rotateX 45)
          #   (rotateZ 5)
          # ];
          # and that kind of thing: don't mention roll, pitch, and
          # yaw, because those impose automatic notions of "forward",
          # "right", and "up", that aren't present in the real
          # coordinate system.
          roll = mkOption {
            description = "Roll offset (degrees).";
            type = types.float;
            default = 0.0;
          };
          pitch = mkOption {
            description = "Pitch offset (degrees).";
            type = types.float;
            default = 0.0;
          };
          yaw = mkOption {
            description = "Yaw offset (degrees).";
            type = types.float;
            default = 0.0;
          };
        };
      });
      default = [ ];
    };
  };

  config = mkIf (config.services.ros.staticTransforms != [ ]) {
    services.ros = {
      enable = true;
      runServices = rosLib.mapIntoAttrs
        (
          { parent, child, x, y, z, roll, pitch, yaw }:
          # {
          #   name = "test1";
          #   value = {
          #     packageName = "tf2_ros";
          #     executable = "static_transform_publisher";
          #   };
          # }
          let
            toRad = deg: deg / 180 * 3.14159265;
            rollRad = toRad roll;
            pitchRad = toRad pitch;
            yawRad = toRad yaw;
          in
          {
            name = "tf-${parent}-${child}";
            value = {
              packageName = "tf2_ros";
              executable = "static_transform_publisher";
              rawArgs = with builtins; [
                (toString x)
                (toString y)
                (toString z)
                (toString yawRad)
                (toString pitchRad)
                (toString rollRad)
                (toString parent)
                (toString child)
              ];
            };
          }
        )
        config.services.ros.staticTransforms;
    };
  };
}
