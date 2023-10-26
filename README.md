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

## System requirements

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

## Basic use

The most important command to remember is `ros4nix switch
<configuration.nix>`: it will initialize the Nix and ROS systems,
build the ROS configuration described, bootstrap an Ubuntu container
into `/var/ros` if configured to do so, install wrappers for all the
ROS system binaries into `/var/ros/nixWrappers/`, and install, enable,
and start systemd services for all enabled ROS nodes: in a nutshell,
it means "make the ROS subsystem resemble the configuration, by
whatever means necessary."

**Important:** `ros4nix` uses `nix-env` to maintain different
generations of the configuration, and advanced users can use this
feature to roll back to old configurations if new ones break the
system. However, this preservation of old generations can cause
a large amount of disk space to be used. Use `sudo nix-collect-garbage
-d` periodically to delete old generations.

To uninstall `ros4nix`, there are 2 commands: `ros4nix reset`, and
`ros4nix purge`, neither of which take other arguments. `ros4nix
reset` stops all currently running ROS nodes, and removes them from
systemd so that they will no longer be started when the system boots.
`ros4nix purge` does the same thing, but *also* deletes all of
`/var/ros`: it leaves the system in the state it was in before
`ros4nix` had ever been installed or run. A safe way to completely
rebuild a ROS installation and discard all its state is to run
`ros4nix purge; ros4nix switch <configuration.nix>`.

## Configuring

See the default configuration in `sample_configuration.nix`.

### Selecting a ROS distribution

The ROS distribution and the Ubuntu distribution it is based on are
both controlled using options under `programs.ros`. The defaults used
if the options are left unspecified are:
```nix
{ config, pkgs, ... }:
{
  programs.ros.baseImage = {
    suite = "focal";       # Ubuntu distribution
    rosDistro = "noetic";  # ROS distribution
  };
}
```

### ROS Master

The ROS master is controlled by the `services.ros.enable` option:
```nix
{ config, pkgs, ... }:
{
  services.ros.enable = true;
}
```
This configuration will install a ROS master, and set it to start up
at boot time. In general this should always be `true`.

### Installing packages

There are two ways of installing packages in `ros4nix`: general Ubuntu
packages, and ROS-specific packages.
```nix
{ config, pkgs, ... }:
{
  programs.ros = {
    packages = [ "tf2" ];               # Installs ros-<rosdistro>-tf2.
    ubuntuPackages = [
      "libpcl-dev"                      # Installs libpcl-dev.
      "libcurses-dev"                   # Multiple packages are allowed.
    ];
  };
}
```

### Remote interfacing

ROS can interface with a remote machine running the ROS core:
```nix
{ config, pkgs, ... }:
{
  programs.ros = {
    master = "1.2.3.4";
  };
}
```
If set, then the value of `services.ros.enable` is ignored; a ROS
master is never run.

This installs an extra line into the wrapper scripts generated that
sets the `ROS_MASTER_URI` to this default machine if it is left unset
by the used.

### Scheduling `roslaunch` programs

TODO

### Scheduling `rosrun` programs

TODO
