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
