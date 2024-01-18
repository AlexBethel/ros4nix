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

    useMainRoot = mkOption {
      type = types.bool;
      default = false;
      description = ''
        Whether to install ROS and all related components into the
        main current system with no sandboxing whatsoever, rather than
        the default of generating our own rootfs and installing into
        that using bwrap for sandboxing.

        This only works if the running system is an Ubuntu or Debian
        installation that is compatible with the ROS distribution
        specified in programs.ros.baseImage.rosDistro, and it also
        applies non-trivially-reversible major changes to the system.
        For this reason it is generally advisable to use this option
        *inside* of a Podman container (no, docker won't do because
        we configure ROS using systemd services, so the environment
        has to run systemd).
      '';
    };

    rootDir = mkOption {
      type = types.str;
      default =
        if config.programs.ros.useMainRoot
        then ""
        else "/var/ros";
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
      ] ++ (if !config.programs.ros.useMainRoot then [
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates main restricted"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite} universe"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates universe"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite} multiverse"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-updates multiverse"
        "deb ${config.programs.ros.baseImage.mirror} ${config.programs.ros.baseImage.suite}-backports main restricted universe multiverse"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security main restricted"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security universe"
        "deb ${config.programs.ros.baseImage.securityMirror} ${config.programs.ros.baseImage.suite}-security multiverse"
      ] else [ ]);
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

    master = mkOption {
      type = types.nullOr types.str;
      default = null;
      description = "Address of the default ROS master.";
    };

    myIP = mkOption {
      type = types.nullOr types.str;
      default = null;
      description = ''
        My IP address; this is the ROS_IP variable. Unless DNS
        resolution exactly matches every hostname on the network, ROS
        requires this variable needs to be set on every machine that
        publishes a topic.

        If it is left unset, ros4nix will make a guess as to the
        correct IP address; unless you have an unusual routing setup,
        the guess should almost always be correct and won't need
        manual adjustment.
      '';
    };
  };

  config =
    let
      # Program to update the ROS installation.
      updateRos = pkgs.writeScriptBin "update-ros" ''
        #!${pkgs.bash}/bin/bash
        set -e
        export PATH=/bin:/sbin
        export SHELL=/bin/sh
        ${if config.programs.ros.useMainRoot
          then ''
            /init
            if [ -e /stage3 ]; then
                /stage3
            fi
          '' else ''
            PATH=/bin:/sbin ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc /init
            if [ -e /stage3 ]; then
                PATH=/bin:/sbin ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc --bind /nix /nix /stage3
            fi
          ''
         }
      '';

      # Program to rebuild the ROS catkin workspace.
      updateCatkin = pkgs.writeScriptBin "update-catkin" ''
        #!${pkgs.bash}/bin/bash
        export PATH=/bin:/sbin
        export SHELL=/bin/sh
        ${if config.programs.ros.useMainRoot
          then ''
            /stage3
          ''
          else ''
            ${pkgs.bubblewrap}/bin/bwrap --dev-bind ${config.programs.ros.rootDir} / --dev /dev --proc /proc --bind /nix /nix /stage3
          ''
         }
      '';

      # Packages that are unnecessary, and whose installation should
      # be completely skipped under all circumstances.
      skippedPackages = [
        # Completely broken inside bwrap.
        "desktop-file-utils"
      ];

      # Inner wrapper script for ROS programs.
      wrapperInner = pkgs.writeScript "wrapper-inner" ''
        #!${pkgs.zsh}/bin/zsh

        . /catkin_ws/devel/setup.zsh
        if grep -qe 'catkin_ws' <<< "$PWD"
        then
            setup_path=$(echo "$PWD" | sed 's:/catkin_ws.*::')/catkin_ws/devel/setup.zsh
            if [ -e "$setup_path" ]
            then
                . $(echo "$PWD" | sed 's:/catkin_ws.*::')/catkin_ws/devel/setup.zsh
            fi
        fi

        xargs -0 -a "$ARG_FILE" "$PROGRAM"
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

        ${
          if config.programs.ros.master != null
            then ''export ROS_MASTER_URI=''${ROS_MASTER_URI:-http://${config.programs.ros.master}:11311}''
            else ''''
        }
        ${
          if config.programs.ros.myIP != null
            then ''ip_guess=${config.programs.ros.myIP}''
          else ''ip_guess="$(${pkgs.iproute2}/bin/ip route get ''
               +
               (if config.programs.ros.master != null
                  then config.programs.ros.master
                  else "1.1.1.1")
               +
            '' | head -n1 | sed 's/.*src //;s/ .*//')"''
        }
        export ROS_IP=''${ROS_IP:-$ip_guess}

        export PATH=/bin:/sbin
        export SHELL=/bin/sh
        ${if config.programs.ros.useMainRoot
          then ''
            exec "${wrapperInner}"
          '' else ''
            exec ${pkgs.bubblewrap}/bin/bwrap \
                --bind /var/ros / \
                --dev-bind /dev /dev \
                --dev-bind /sys /sys \
                --bind /etc/hosts /etc/hosts \
                --bind /home /home \
                --proc /proc \
                --bind /tmp /tmp \
                --bind /nix /nix \
                "${wrapperInner}"
          ''
         }
      '';

      chrootRos = pkgs.writeScriptBin "chroot-ros" ''
        #!${pkgs.bash}/bin/bash

        export PROGRAM=bash
        export ARG_FILE=$(mktemp)

        ${
          if config.programs.ros.master != null
            then ''export ROS_MASTER_URI=''${ROS_MASTER_URI:-http://${config.programs.ros.master}:11311}''
            else ''''
        }
        ${
          if config.programs.ros.myIP != null
            then ''export ROS_IP=''${ROS_IP:-${config.programs.ros.myIP}}''
            else ''''
        }

        export PATH=/bin:/sbin
        ${if config.programs.ros.useMainRoot
          then ''
            exec "${wrapperInner}"
          '' else ''
            exec ${pkgs.bubblewrap}/bin/bwrap \
                --bind /var/ros / \
                --dev-bind /dev /dev \
                --dev-bind /sys /sys \
                --bind /etc/hosts /etc/hosts \
                --bind /home /home \
                --proc /proc \
                --bind /tmp /tmp \
                --bind /nix /nix \
                "${wrapperInner}"
          ''
         }
      '';

      # The stage 1 script builds a base Ubuntu installation from
      # scratch, with no ROS-specific software. It runs in the base
      # NixOS system, not inside the container (because the container
      # doesn't exist yet). It runs exactly once, when the rootDir
      # doesn't exist.
      #
      # TODO: It should theoretically also rerun whenever it changes,
      # to accomodate changes to the ROS distro option.
      stage1Install =
        if config.programs.ros.useMainRoot
        then ''
          : # Do nothing
        ''
        else ''
          ${pkgs.coreutils}/bin/mkdir ${config.programs.ros.rootDir}
          ${pkgs.debootstrap}/bin/debootstrap \
              ${config.programs.ros.baseImage.suite} \
              ${config.programs.ros.rootDir} \
              ${config.programs.ros.baseImage.mirror}

          # Drop a copy of Boost where we can reliably find it, while
          # we're here. This will get used by `ros4nix cflags`.
          ${pkgs.coreutils}/bin/ln -s ${pkgs.boost.dev} \
              ${config.programs.ros.rootDir}/ext_libs
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
            ("apt-get install --no-install-recommends -y " +
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
                skippedPackages ++
              [ "make" ]
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
          builtins.attrValues (builtins.mapAttrs
            (name: path:
              ''
                cp -r ${path} $PACKAGE_REPO/${name}
                for f in $(find $PACKAGE_REPO/${name}); do
                  touch "$f"
                done
                ${pkgs.rsync}/bin/rsync -av --update $PACKAGE_REPO/${name} /catkin_ws/src/
              ''
            )
            config.programs.ros.buildPackages) ++
          [
            ''
              . /opt/ros/**/setup.sh
              cd /catkin_ws
              catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
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

        PATH=${pkgs.proot}/bin:${pkgs.bash}/bin:${pkgs.gnused}/bin:${pkgs.which}/bin:$PATH ${./ros_generate_wrappers} "${config.programs.ros.rootDir}" ${wrapperOuter}
      '';
      rosActivationScript = pkgs.writeScriptBin "activate-ros" ''
        #!/bin/sh
        set -e
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
          rosActivationScript
          updateRos
          updateCatkin
          chrootRos
          config.system.build.etc
        ];
      };
    };
}
