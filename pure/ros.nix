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

  debFilesAndPrefixes =
    let
      listings = {
        x86_64 = [
          {
            listing = fetchurl {
              url = "http://packages.ros.org/ros/ubuntu/lists/ros-noetic-focal-amd64_focal_main_amd64_Packages";
              sha256 = "0ja85ws9ygyzwp55221gpp4cmf48j268w2f2w1ln1zsxjg5h6x6f";
            };
            prefix = "http://packages.ros.org/ros/ubuntu/";
          }

          {
            listing = fetchurl {
              url = "http://archive.ubuntu.com/ubuntu/dists/focal/main/binary-amd64/Packages.xz";
              sha256 = "0qcqzi1wvjzm3n66rsp3pqwa590v8inr7jd06hwrrvgyz0gr4mvp";
            };
            prefix = "http://archive.ubuntu.com/ubuntu/";
          }

          {
            listing = fetchurl {
              url = "http://archive.ubuntu.com/ubuntu/dists/focal/universe/binary-amd64/Packages.xz";
              sha256 = "1ldrjryrs6c20csi2c8m93153zwx1dlb2kd5mhvrbgc9qzd4d9s6";
            };
            prefix = "http://archive.ubuntu.com/ubuntu/";
          }

          {
            listing = fetchurl {
              url = "http://archive.ubuntu.com/ubuntu/dists/focal/multiverse/binary-amd64/Packages.xz";
              sha256 = "0hjxic6xgbvw889mzly6db1qzdlb7bqjyqddrhc6rqb8vyvmfdr3";
            };
            prefix = "http://archive.ubuntu.com/ubuntu/";
          }
        ];

        aarch64 = [
          # N.B. Ubuntu refers to aarch64 as "arm64", and hosts its
          # packages from "ports.ubuntu.com" as opposed to
          # "archive.ubuntu.com"; and "ports.ubuntu.com" has its whole
          # directory tree shifted up by one level.
          {
            listing = fetchurl {
              url = "http://packages.ros.org/ros/ubuntu/lists/ros-noetic-focal-arm64_focal_main_arm64_Packages";
              sha256 = "013na1k7dcjaa0xsi70m3gjiyflscdmjiyzjyxa1a78j4i7a05iy";
            };
            prefix = "http://packages.ros.org/ros/ubuntu/";
          }

          {
            listing = fetchurl {
              url = "http://ports.ubuntu.com/dists/focal/main/binary-arm64/Packages.xz";
              sha256 = "06rc5vi3lpc4fxmz7fk9fx84cxzvrbs48kxrdkqhr30x090v4570";
            };
            prefix = "http://ports.ubuntu.com/";
          }

          {
            listing = fetchurl {
              url = "http://ports.ubuntu.com/dists/focal/universe/binary-arm64/Packages.xz";
              sha256 = "18fzjgl3vw2rixd6xckyn7pixfmcndhj30kk6726mpwjrgisgq42";
            };
            prefix = "http://ports.ubuntu.com/";
          }

          {
            listing = fetchurl {
              url = "http://ports.ubuntu.com/dists/focal/multiverse/binary-arm64/Packages.xz";
              sha256 = "1b8lsxrfx6q6f2q1y05vi7f73mz1dqvn112y09s3yn1k9cs7y06l";
            };
            prefix = "http://ports.ubuntu.com/";
          }
        ];
      };
    in
    listings.${hostPlatform.uname.processor};

  mkDebEnv = extraInputs: import ./buildDebEnv.nix {
    inherit pkgs debFilesAndPrefixes;
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

      # Link in the sysroot and the combined packages for convenience,
      # and provide a compile_flags.txt for clangd.
      ln -s ${debEnv.root} $out/sysroot
      ln -s ${combinedPackages} $out/packages
      echo "-I${debEnv.root}/opt/ros/noetic/include" >> $out/compile_flags.txt
      echo "-I${boost.dev}/include" >> $out/compile_flags.txt
      echo "-I${combinedPackages}/install/include" >> $out/compile_flags.txt
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
      myDebPkgs = package.extraDebPackages or [ ];
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
