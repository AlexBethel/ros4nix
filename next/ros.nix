{ pkgs ? import <nixpkgs> { } }:
with pkgs;
let
  neededPackages = [
    "base-passwd"
    "dpkg"
    "libc6-dev"
    "perl"
    "bash"
    "dash"
    "gzip"
    "bzip2"
    "tar"
    "grep"
    "mawk"
    "sed"
    "findutils"
    "g++"
    "make"
    "curl"
    "patch"
    "locales"
    "coreutils"
    # Needed by checkinstall:
    "util-linux"
    "file"
    "dpkg-dev"
    "pkg-config"
    # Needed because it provides /etc/login.defs, whose absence causes
    # the "passwd" post-installs script to fail.
    "login"
    "passwd"
    # ----

    "libattr1"
    "python3" # undeclared catkin dependency
    "python3-minimal"
    "python3-catkin-tools" # provides `catkin` binary
    "ros-noetic-ros-core"
  ];

  debFilesAndPrefixes = [
    {
      listing = fetchurl {
        url = "http://packages.ros.org/ros/ubuntu/lists/ros-noetic-focal-amd64_focal_main_amd64_Packages";
        sha256 = "sha256-znQDy5Nd/2Bp4MIJjoyQiLjKyL0vCFHK5d8/nzQvSEk=";
      };
      prefix = "http://packages.ros.org/ros/ubuntu/";
    }

    {
      listing = fetchurl {
        url = "http://archive.ubuntu.com/ubuntu/dists/focal/main/binary-amd64/Packages.xz";
        sha256 = "sha256-d1eSH/j+7Zw5NKDJk21EG6SiOL7j6myMHfXLzUP8mGE=";
      };
      prefix = "http://archive.ubuntu.com/ubuntu/";
    }

    {
      listing = fetchurl {
        url = "http://archive.ubuntu.com/ubuntu/dists/focal/universe/binary-amd64/Packages.xz";
        sha256 = "sha256-RqdG2seJvZU3rKVNsWgLnf9RwkgVMRE1A4IZnX2WudE=";
      };
      prefix = "http://archive.ubuntu.com/ubuntu/";
    }

    {
      listing = fetchurl {
        url = "http://archive.ubuntu.com/ubuntu/dists/focal/multiverse/binary-amd64/Packages.xz";
        sha256 = "sha256-IzdXt99o4WwYzK1hL/E6i7aPw2rG018TQnyv1w2LXUI=";
      };
      prefix = "http://archive.ubuntu.com/ubuntu/";
    }
  ];

  mkDebEnv = extraInputs: import ./buildDebEnv.nix {
    inherit debFilesAndPrefixes;
    debianPackages = neededPackages ++ extraInputs;
  };

  enterScriptWrapper = writeScript "enter-script" ''
    #!${bash}/bin/bash
    if [ -e /build/install/setup.sh ]; then
      . /build/install/setup.sh
    else
      . /opt/ros/noetic/setup.sh
    fi
    exec "$@"
  '';

  # Build ROS wrappers with a set of Catkin packages.
  rosWrappersWithPackages = debPkgs: packages:
    let
      combinedPackages = runCommand "combinedPackages" { } ''
        mkdir -p $out/install
        for f in ${builtins.concatStringsSep " " packages}; do
          cp -r $f/* $out/install 2>/dev/null || true
          chmod -R u+w $out
        done
      '';
      requiredDebPkgs = builtins.concatMap (input: input.extraDebPackages) (
        builtins.filter (pkg: pkg.isROSPackage or false)
          packages
      );
      debEnv = mkDebEnv (debPkgs ++ requiredDebPkgs);
    in
    runCommand "rosWrappers" { } ''
      mkdir -p $out/bin

      cd ${debEnv.root}/opt/ros/noetic/bin
      for file in *; do
        target=$out/bin/$file
        echo "#!/bin/sh" > $target
        echo "${debEnv.enterScript}/bin/deb-env -b ${combinedPackages}:/build ${enterScriptWrapper} /opt/ros/noetic/bin/$file \"\$@\"" >> $target
        chmod 755 $target
      done

      cd ${debEnv.root}/usr/bin
      for file in catkin*; do
        target=$out/bin/$file
        echo "#!/bin/sh" > $target
        echo "${debEnv.enterScript}/bin/deb-env -b ${combinedPackages}:/build ${enterScriptWrapper} /usr/bin/$file \"\$@\"" >> $target
        chmod 755 $target
      done

      target=$out/bin/chroot-ros
      echo "#!/bin/sh" > $target
      echo "${debEnv.enterScript}/bin/deb-env -b ${combinedPackages}:/build ${enterScriptWrapper} /bin/bash \"\$@\"" >> $target
      chmod 755 $target
    '';

  buildRosPackage = package:
    let
      # symlinkJoin all the build inputs that are Catkin packages so
      # we can source this as a workspace later.
      rosInputs = symlinkJoin {
        name = "ros-inputs";
        paths = builtins.filter (pkg: pkg.isROSPackage or false)
          (package.buildInputs or [ ]);
      };

      inputDebPkgs = builtins.concatMap (input: input.extraDebPackages) (
        builtins.filter (pkg: pkg.isROSPackage or false)
          (package.buildInputs or [ ])
      );
      myDebPkgs = package.extraDebPackages or [];
      debPkgs = myDebPkgs ++ inputDebPkgs;

      debEnv = mkDebEnv debPkgs;
    in
    debEnv.runInEnv (runCommand package.name
      ({
        # Mark this package as a Catkin package so it gets included in
        # other packages' `rosInputs`.
        passthru.isROSPackage = true;
        passthru.extraDebPackages = debPkgs;
      } // package)
      ''
        export HOME=$(pwd)  # ROS writes to $HOME when building

        # Make a catkin workspace in the build root.
        mkdir -p src
        cp -r ${package.src} src/${package.name}

        # Catkin doesn't work properly if we move the generated
        # install space around. The current strategy for dealing with
        # this is: we copy the inputs' `install` directory into our
        # own `install` directory, then source *that* as a workspace;
        # that succeeds because it's always in `/build/install`. Then
        # we augment the install space with our own stuff using
        # `catkin config --install`.
        cp -r ${rosInputs} install
        chmod -R u+w install

        export PATH=/bin:/usr/bin
        if [ -e install/setup.sh ]; then
          . install/setup.sh
        else
          . /opt/ros/*/setup.sh
        fi

        catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

        cp -r install $out
      '');
in
{
  inherit buildRosPackage;
  withPackages = rosWrappersWithPackages;
}
