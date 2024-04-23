This directory contains a WIP rewrite of the system that's much more
deterministic and avoids abusing activation scripts as was done
previously.

Current issues:
- The generated wrapper scripts aren't capable of running built
  packages for some reason.
- We're currently relying on Nix building into `/build/`, but my
  understanding is that this behavior isn't followed when sandboxing
  is not enabled.
- Support for architectures other than amd64 is not implemented.
  (Trivial to fix by adding autodetection to `debFilesAndPrefixes` in
  `ros.nix`.)
  - We should also ideally have support for cross-compilation somehow.
    Upstream ROS's cross-compilation infrastructure seems to be
    lacking enough that it may be far easier to do the Poor Man's
    Cross Compilation and use qemu to emulate a native compilation
    process on the target host.
- `ros.withPackages` has an unintuitive, non-standard interface: it
  takes an array of Debian packages as strings, then an array of built
  ROS packages. It should just take an array of built ROS packages; if
  you want an extra Debian package, the way to do it will probably be
  to make an _empty_ ROS package that has a Debian dependency on the
  package you want, then include that. It might be a good idea to
  extend this approach to all the normal ROS packages: we would have
  an automatically generated file that lists things like
  ```nix
    {
      # ... bazillions of packages ...

      move-base = runCommand {
        isROSPackage = true;
        extraDebPackages = [ "ros-noetic-move-base" ];
      } "mkdir $out";  # empty directory

      # ... bazillions more packages ...
    }
  ```
  and supply this as a `pkgs` argument to the argument of
  `ros.withPackages`. So then we could write
  ```nix
    ros.withPackages (pkgs: [
      pkgs.move-base
      myPackageThatIBuilt
    ])
  ```
  and it would install the package that was built on the fly, and also
  bring in `ros-noetic-move-base`.
