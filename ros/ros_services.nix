# Generic ROS service management.
{ config, lib, pkgs, ... }:

with lib;
let rosLib = import ./services/rosLib.nix; in
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

    launchServices = mkOption {
      description = ''
        Attribute set of services to launch with roslaunch. This
        should generally only be used to launch nodes that are
        provided by external libraries; writing Nix modules should be
        preferred over writing launch files for new code.
      '';

      default = { };

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

      default = { };

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
            default = { };
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
              echo 'Failed to start roscore, retrying.'
              ${config.programs.ros.rootDir}/nixWrappers/roscore &
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

    # roslaunch services
    (mkIf (config.services.ros.enable) {
      systemd.services = rosLib.mapAttrsFull
        (
          name: { packageName, launchFile, args }:
            {
              name = "ros-${name}";
              value = {
                wantedBy = [ "multi-user.target" ];
                after = [ "rosMaster.service" ];

                script =
                  let
                    opts = rosLib.attrsToCmdLine args;
                  in
                  ''
                    /var/ros/nixWrappers/roslaunch ${packageName} ${launchFile} ${opts}
                  '';
              };
            }
        )
        config.services.ros.launchServices;
    })

    # rosrun services
    (mkIf (config.services.ros.enable) {
      systemd.services = rosLib.mapAttrsFull
        (
          name: { packageName, executable, remap, namespace }:
            {
              name = "ros-${name}";
              value = {
                wantedBy = [ "multi-user.target" ];
                after = [ "rosMaster.service" ];

                # TODO: we should use namespace here somewhere.
                script =
                  let
                    opts = rosLib.attrsToCmdLine remap;
                  in
                  ''
                    /var/ros/nixWrappers/rosrun ${packageName} ${executable} ${opts}
                  '';
              };
            }
        )
        config.services.ros.runServices;
    })
  ];
}
