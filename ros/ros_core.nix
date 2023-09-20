# Basic ROS chroot environment management. This module just makes sure
# that the environment is always up to date and builds Nix wrappers
# around tools in the environment, and doesn't handle any runtime
# requirements like services.

{ config, pkgs, lib, ... }:
with lib;
{
  options.programs.ros = {
    enable = mkOption {
      type = types.bool;
      default = false;
    };

    rootDir = mkOption {
      type = types.str;
      default = "/var/ros";
      description = "Directory to install ROS into.";
    };

    baseImage = {
      mirror = mkOption {
        type = types.str;
        default =
          if config.nixpkgs.hostPlatform.isAarch
          then "http://ports.ubuntu.com/ubuntu-ports/"
          else "http://archive.ubuntu.com/ubuntu/";
        description = "URL to pull archives from.";
      };

      securityMirror = mkOption {
        type = types.str;
        default =
          if config.nixpkgs.hostPlatform.isAarch
          then "http://ports.ubuntu.com/ubuntu-ports/"
          else "http://security.ubuntu.com/ubuntu/";
        description = "URL to pull security archives from.";
      };

      suite = mkOption {
        type = types.str;
        default = "focal";
        description = "";
      };

      rosDistro = mkOption {
        type = types.str;
        default = "noetic";
        description = "";
      };
    };

    extraSources = mkOption {
      type = types.listOf types.str;
      default = [
        "deb http://packages.ros.org/ros/ubuntu ${config.programs.ros.baseImage.suite} main"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates main restricted"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite} universe"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates universe"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite} multiverse"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates multiverse"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-backports main restricted universe multiverse"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security main restricted"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security universe"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security multiverse"
      ];
      description = "Extra sources to add to the Ubuntu image.";
    };

    extraKeys = mkOption {
      type = types.listOf types.str;
      default = [
        "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
      ];
      description = "Key URLs to authorize for APT.";
    };

    ubuntuPackages = mkOption {
      type = types.listOf types.str;
      default = [ ];
      description = "Ubuntu packages to make available in ROS.";
    };

    packages = mkOption {
      type = types.listOf types.str;
      default = [ ];
      description = "ROS packages to make available.";
    };

    buildPackages = mkOption {
      type = types.attrsOf types.path;
      default = { };
      description = "Paths to copy into /catkin_ws.";
    };
  };

  config =
    let
      # Program to update the ROS installation.
      updateRos = pkgs.writeScriptBin "update-ros" ''
        #!${pkgs.bash}/bin/bash
        export PATH=/bin:/sbin
        export SHELL=/bin/sh
        PATH=/bin:/sbin ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc /init
        PATH=/bin:/sbin ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc --bind /nix /nix /stage3
      '';

      # Program to rebuild the ROS catkin workspace.
      updateCatkin = pkgs.writeScriptBin "update-catkin" ''
        #!${pkgs.bash}/bin/bash
        export PATH=/bin:/sbin
        export SHELL=/bin/sh
        ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc --bind /nix /nix /stage3
      '';

      # # Program to enter a ROS chroot.
      # chrootRos = pkgs.writeScriptBin "chroot-ros" ''
      #   #!${pkgs.bash}/bin/bash
      #   PATH=/bin:/sbin ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc /bin/bash
      # '';

      # Packages that are unnecessary, and whose installation should
      # be completely skipped under all circumstances.
      skippedPackages = [
        # Completely broken inside bwrap.
        "desktop-file-utils"

        # Testing
        "libc6"
      ];

      # Inner wrapper script for ROS programs.
      wrapperInner = pkgs.writeScript "wrapper-inner" ''
        #!${pkgs.bash}/bin/bash
        IFS=$'\n\t'

        # . /opt/ros/**/setup.sh
        . /catkin_ws/devel/setup.sh

        cat "$ARG_FILE" | xargs -0 "$PROGRAM"
      '';

      # Outer wrapper script for ROS programs.
      wrapperOuter = pkgs.writeScript "wrapper" ''
        #!${pkgs.bash}/bin/bash

        export PROGRAM=$(basename "$0")
        export ARG_FILE=$(mktemp)
        for arg in "$@"; do
            echo -n "$arg" >> $ARG_FILE
            echo -en '\0' >> $ARG_FILE
        done

        export PATH=/bin:/sbin
        exec ${pkgs.bubblewrap}/bin/bwrap \
            --bind /var/ros / \
            --dev-bind /dev /dev \
            --dev-bind /sys /sys \
            --bind /etc/hosts /etc/hosts \
            --bind /home /home \
            --proc /proc \
            --bind /tmp /tmp \
            --bind /nix /nix \
            "${wrapperInner}" # $(${pkgs.coreutils}/bin/basename "$0") "$@"
      '';

      chrootRos = pkgs.writeScriptBin "chroot-ros" ''
        #!${pkgs.bash}/bin/bash
        [ -e /tmp/bash ] || ln -s ${wrapperOuter} /tmp/bash
        exec /tmp/bash
      '';

      # The stage 1 script builds a base Ubuntu installation from
      # scratch, with no ROS-specific software. It runs in the base
      # NixOS system, not inside the container (because the container
      # doesn't exist yet). It runs exactly once, when the rootDir
      # doesn't exist.
      #
      # TODO: It should theoretically also rerun whenever it changes,
      # to accomodate changes to the ROS distro option.
      stage1Install = ''
        ${pkgs.coreutils}/bin/mkdir ${config.programs.ros.rootDir}
        ${pkgs.debootstrap}/bin/debootstrap \
            ${config.programs.ros.baseImage.suite} \
            ${config.programs.ros.rootDir} \
            ${config.programs.ros.baseImage.mirror}
      '';

      # The stage 2 script runs inside the container, performs a
      # system update, configures APT, and downloads all APT packages
      # necessary. It runs once per boot, but the activation snippet
      # records its contents and only runs the stage 2 script if it
      # has changed since its last run.
      stage2Install = builtins.concatStringsSep "\n"
        (
          [
            "#!/bin/sh"

            # Print commands as they're executed.
            "set -xe"

            # Never prompt the user.
            "export DEBIAN_FRONTEND=noninteractive"
          ] ++

          # We need curl and gnupg to be able to add keys.
          [ "apt-get install -y curl gnupg" ] ++

          # Authorize needed keys.
          map
            (key:
              "curl ${key} | apt-key add -"
            )
            config.programs.ros.extraKeys ++

          # Add installation sources to APT.
          [ "echo > /etc/apt/sources.list.d/nix.list" ] ++
          map
            (source:
              "echo \"${source}\" >> /etc/apt/sources.list.d/nix.list"
            )
            config.programs.ros.extraSources ++

          # Update sources.
          [
            "apt-get update"
            "apt-get upgrade -y"
            "apt-get autoremove -y"
          ] ++

          # Install needed packages.
          [
            ("apt-get install -y " +
            builtins.concatStringsSep " " (
              config.programs.ros.ubuntuPackages ++
              map
                (name:
                  "ros-${config.programs.ros.baseImage.rosDistro}-${name}"
                )
                config.programs.ros.packages ++
              map
                (name:
                  "${name}-"
                )
                skippedPackages
            )
            )
          ] ++

          [ ]
        );
      stage2Script = pkgs.writeScript "stage2" stage2Install;

      # The stage 3 script runs inside the chroot and builds user
      # software, and runs every time the user software inputs have
      # changed. It skips doing the system update or any APT
      # configuration, and should be capable of running offline.
      stage3Install = builtins.concatStringsSep "\n"
        (
          [
            ''
              #!/bin/sh
              rm -rf /catkin_ws/src  # Temporary
              mkdir -p /catkin_ws/src
              PACKAGE_REPO=$(mktemp -d)
            ''
          ] ++
          builtins.attrValues (builtins.mapAttrs (name: path:
            ''
              cp -r ${path} $PACKAGE_REPO/${name}
              for f in $(find $PACKAGE_REPO/${name}); do
                touch "$f"
              done
              ${pkgs.rsync}/bin/rsync -av --update $PACKAGE_REPO/${name} /catkin_ws/src/
            ''
          ) config.programs.ros.buildPackages) ++
          [
            ''
              . /opt/ros/**/setup.sh
              cd /catkin_ws
              # catkin_make_isolated
              # catkin_make
              catkin build
            ''
          ]
        );

      stage3Script = pkgs.writeScript "stage3" stage3Install;

      rosActivationSnippet = ''
        if ! [ -e ${config.programs.ros.rootDir} ]; then
          ${stage1Install}
        fi

        if ( ! [ -e ${config.programs.ros.rootDir}/init ] ) || \
           ( ! ${pkgs.diffutils}/bin/diff -q \
                 ${config.programs.ros.rootDir}/init \
                 ${stage2Script} \
           ); then
          cp ${stage2Script} ${config.programs.ros.rootDir}/init
          ${updateRos}/bin/update-ros
        fi

        if ( ! [ -e ${config.programs.ros.rootDir}/stage3 ] ) || \
           ( ! ${pkgs.diffutils}/bin/diff -q \
                 ${config.programs.ros.rootDir}/stage3 \
                 ${stage3Script} \
           ); then
          cp ${stage3Script} ${config.programs.ros.rootDir}/stage3
          ${updateCatkin}/bin/update-catkin
        fi

        PATH=${pkgs.proot}/bin:${pkgs.bash}/bin:${pkgs.gnused}/bin:${pkgs.which}/bin:$PATH ${./ros_generate_wrappers} ${config.programs.ros.rootDir} ${wrapperOuter}
      '';
      rosActivationScript = pkgs.writeScriptBin "activate-ros" ''
        #!/bin/sh
        ${rosActivationSnippet}
      '';
    in
    mkIf config.programs.ros.enable {
      system.activationScripts.ros = rosActivationSnippet;

      environment.extraInit = ''
        export PATH=${config.programs.ros.rootDir}/nixWrappers:$PATH
      '';

      environment.systemPackages = [
        updateRos
        updateCatkin
        chrootRos
      ];

      system.build.ros4nix = pkgs.symlinkJoin {
        name = "ros4nix";
        paths = [
          config.system.build.etc
          rosActivationScript
          updateRos
          updateCatkin
          chrootRos
        ];
      };
    };
}
