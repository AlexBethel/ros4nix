# ros4nix

`ros4nix` is a program for managing ROS using Nix and NixOS. It is not
a direct *port*, per se, of ROS to Nix, as no ROS software ever gets
compiled or downloaded into the Nix store itself; rather, it manages
an Ubuntu virtual machine into which ROS is installed in the
traditional manner.

The project has a few major goals:
- Portability. ROS itself is difficult to port to new operating
  systems because of its integration with APT and operating system
  version numbers (e.g., getting ROS Noetic running on Ubuntu 18 is
  not at all trivial); `ros4nix` should work on as many platforms as
  possible.
- Conciseness. ROS requires writing many configuration files, but the
  overall information contained therein is not that large; `ros4nix`
  should allow these files to be combined into one file or split apart
  at the user's preference, and should also have support for macros
  that generate common configurations.
- Safety. ROS nodes know nothing about the internals of other nodes or
  the overall configuration of the system, so designing configurations
  of nodes that are invalid (e.g., with a node subscribing to a
  misspelling of a topic being published by another node) is easy. The
  advantage of specifying the whole ROS configuration in one set of
  files is that we can statically check that it makes sense.
- Reproducibility. ROS normally uses `apt`, a stateful package
  manager, which keeps no record of which precise commands were used
  to get the system into a particular state, so reproducing the same
  state from a development machine to a production machine is not
  guaranteed to be easy. `ros4nix` defines the entire ROS installation
  from one configuration, and this configuration can be copied between
  machines freely.

# System requirements

`ros4nix` is known to work on x86_64 machines and aarch64 machines;
beyond this, the program has support for running under the following
configurations:
* As a `bwrap` chroot-like environment on a NixOS machine, installed
  as a NixOS module. This is the primary target because we can make
  the most assumptions about the kernel and other software running on
  the system.
* As a `bwrap` chroot-like environment on a non-NixOS machine. This
  requires:
  - Support for running Nix on the system. `ros4nix` will try to
    install Nix for you if you haven't already set it up.
  - The ability to bootstrap an Ubuntu installation using
    `debootstrap`. This requires that the kernel not have a minor
    version number higher than 255, which is known not to be the case
    on some pathological NVIDIA SBC's.
  - User namespaces to be enabled on the kernel. This is true of most
    Linux distributions released within the last decade.
* As a manager for a real ROS installation on the bare operating
  system. This requires that the operating system be a supported
  version of either Debian or Ubuntu for the particular targeted
  version of ROS.
* As a manager for a ROS installation inside a Podman container with
  systemd as its init.

We explicitly *do not* support Docker, because `ros4nix` requires
systemd to manage scheduling services at boot time, and Docker's
support for initializing a container with systemd is less than ideal,
to say the least.

# Configuring

TODO
