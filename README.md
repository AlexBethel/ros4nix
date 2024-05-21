# ros4nix

*This program is no longer actively developed, and its use is not
recommended.* This is the software deployment system developed for NMT
Lunabotics in 2024; however, it suffers from purity problems and
usability issues, and its port from NixOS to other operating systems
was always super hacky. As it stands I'm not using ROS for any major
projects, so for the time being I'm no longer working on this
repository.

----------------------------------------------------------------------

`ros4nix` is a program for managing ROS using Nix and NixOS. It is not
a direct *port*, per se, of ROS to Nix, as no ROS software ever gets
compiled or downloaded into the Nix store itself; rather, it manages a
highly customized Ubuntu container system into which ROS is installed
in the traditional manner.

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

## Quick reference

Here are a few common tasks, illustrating how they are typically done
with standard ROS and `ros4nix` respectively.

### Installing apt packages

**On normal ROS:** `sudo apt install ros-<distro>-<pkg_name>`.

**On `ros4nix`:** add `programs.ros.packages = [ "<pkg_name>" ]` to
`configuration.nix` (or if it has a module written for it, just enable
that module), then use `ros4nix switch configuration.nix` to reload
the configuration.

### Installing non-apt packages

**On normal ROS:** install the APT package dependencies, then clone
all the non-APT Catkin dependencies into a Catkin workspace, then use
`catkin build` to compile them, and always do other development in
that Catkin workspace.

**On `ros4nix`:** add the non-APT dependencies as normal, then write
```nix
{
  programs.ros.buildPackages.<package name> = pkgs.fetchFromGitHub {
    owner = "<owner name>";
    repo = "<package name>";
    rev = "<tag to check out>";
    sha256 = "";
  };
}
```
, then use `ros4nix switch configuration.nix` to attempt to build the
package; when that fails, copy the sha256 hash it prints to the
terminal into the `configuration.nix`, and it should succeed; it is
strongly recommended that you then refactor the work into a proper
module so no one else has to do the build themself. See
`ros/services/kindrRos.nix` for an example of a package that's been
configured to compile and install by anyone just specifying
`services.ros.libraries.kindrRos.enable = true`, and
`ros/services/elevationMapping.nix` for a more intricate example.

### Loading a Catkin workspace

**On normal ROS:** `. <workspace>/devel/setup.bash`.

**On `ros4nix`:** simply `cd` into the workspace, and the tooling will
load the workspace for you automatically; `rosrun`, `roslaunch`,
`catkin build`, etc., should just work immediately.

### Running a ROS master at boot

**On normal ROS:** typically write a systemd service to do it; there
is no official standardized solution.

**On `ros4nix`:** set `services.ros.enable` to `true` in
`configuration.nix`, then use `ros4nix switch configuration.nix`.

### Running ROS nodes at boot

**On normal ROS:** typically write a systemd service to do it; there
is no official standardized solution.

**On `ros4nix`:** add an entry to `services.ros.launchServices` and
`services.ros.runServices`.

### Reinstalling and/or uninstalling ROS

**On normal ROS:** no direct support, but your best bet is to run
`sudo apt remove 'ros-*'`, then reinstall your needed packages if
necessary.

**On `ros4nix`:** use `ros4nix purge` to delete the entire ROS
container, then `ros4nix switch configuration.nix` to rebuild it if
needed.

### Getting a shell with real ROS tooling available

**On normal ROS:** unnecessary, because they're installed by default.

**On `ros4nix`:** use `ros4nix chroot` to get a shell inside the
Ubuntu container that all ROS software is run within; the raw ROS
tools will then be in `/opt/ros`. If you need to do this for your
actual project and not just interest in the design of `ros4nix`,
please add a bug report to the issue tracker for this repo, because if
the software does its job correctly, this operation should never be
necessary.

## Modes of operation

`ros4nix` is known to work on x86_64 machines and aarch64 machines;
beyond this, the program has support for running under a number of
different configurations.

### As a chroot on a NixOS host

Import the `ros` directory into your system `configuration.nix`, and
all the ROS settings can be integrated directly into your overall host
configuration. This is the most ideal way to use `ros4nix` because it
allows us to make the most assumptions about the kernel and other
software running on the system, and obviously allows the most
powerful config.

In this mode, `ros4nix` will use `debootstrap` to generate an Ubuntu
installation inside `/var/ros`, then use `bwrap` to manage it like a
chroot environment.

### As a chroot on a non-NixOS host

Use the provided `ros4nix` program. This requires:
- Support for running Nix on the system. `ros4nix` will try to install
  Nix for you if you haven't already set it up.
- The ability to bootstrap an Ubuntu installation using `debootstrap`.
  This requires that the kernel not have a minor version number higher
  than 255, which is known not to be the case on some goofy NVIDIA
  SBC's.
- User namespaces to be enabled on the kernel. This is true of most
  Linux distributions released within the last decade.

Once again, `ros4nix` will build a `bwrap` container in `/var/ros` in
this mode.

### As a ROS manager for an Ubuntu/Debian host

If your host is running the correct version of Ubuntu or Debian to
install the ROS version you want, you can use `ros4nix` to manage your
actual ROS installation; in this case, `ros4nix` just dispatches `apt`
commands and creates systemd services. To use this mode, set
`programs.ros.useMainRoot` to `true`.

### As a ROS manager for an Ubuntu container

If you use Podman to build an Ubuntu container of the right version,
you can use `ros4nix` to manage its installation of ROS the same way
as you would for a real host, with `program.ros.useMainRoot` set to
`true`. Podman must be configured to run `systemd` as its initial
program.

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

ROS defines two primary ways of launching nodes: `roslaunch` and
`rosrun`. `roslaunch` is typically used to launch ensembles of nodes
rather than individual ones, and is thus preferred by most authors.
```nix
{ config, pkgs, ... }:
{
  # Replace <name> with a name for the service; it can be arbitrary,
  # and is typically the same as the name of the node being launched.
  services.ros.launchServices.<name> = {
    # Name of the ROS package in which the .launch file is found.
    packageName = "foo";

    # Name of the launch file itself.
    launchFile = "bar.launch";

    # Key-value pairs to send to the service.
    args = {
      xyz = true;
      zyx = "something";
    };
  };
}
```
For reference, this example configuration is the same as typing
`roslaunch --wait foo bar.launch xyz:=true zyx:=something` each time
the system boots.

This option creates and enables a systemd service called `ros-<name>`;
it can be managed like any other systemd service, meaning you can pull
up logs for it with `systemctl logs ros-<name>`, or start and stop it
with the `start` and `stop` subcommands of `systemctl`, respectively.
It is generally recommended that you not `disable` the service,
because doing so is unreliable: first, it can cause other `ros4nix`
modules that automatically enabled this node because they depend on it
to not function properly, because the node they depend on is missing;
also, it is not respected by `ros4nix`, as whenever you re-run
`ros4nix switch` the program will automatically re-enable every
service it created.

### Scheduling `rosrun` programs

`rosrun` is used to launch individual nodes rather than groups of
nodes.
```nix
{ config, pkgs, ... }:
{
  services.ros.runServices.<name> = {
    # Name of the ROS package providing the node.
    packageName = "foo";

    # Name of the executable to run.
    executable = "bar";

    # ROS paths to remap.
    remap."/somewhere" = "/somewhere_else";

    # Other raw arguments to pass to the program.
    rawArgs = [ "arg1" "arg2" ];

    # Value of the ROS_NAMESPACE environment variable (support is
    # still WIP).
    namespace = "/my_namespace";

    # Values to load into the ROS parameter server before running the
    # service. This can be the path to a file, but this `writeText`
    # pattern is usually more appropriate.
    rosParams = pkgs.writeText "params.yaml" (builtins.toJSON {
      someKey = "someValue";
      # arbitrary other key-value pairs can go here.
    });
  };
}
```
Only `packageName` and `executable` are required. This example is
roughly equivalent to running these commands at boot time:
```bash
# Assuming `params.yaml` has the parameters we specified.
rosparam load params.yaml /foo
export ROS_NAMESPACE=/my_namespace
rosrun foo bar /somewhere:=/somewhere_else arg1 arg2
```

## Macros for pre-configured packages

A number of packages have special support integrated under the
`ros/services/` directory. These files are actually just NixOS
modules, and new ones can be developed the same way; see
https://nixos.org/manual/nixos/stable/#sec-writing-modules for precise
details, which shan't be documented here.

### rosbag player

The rosbag player plays back recorded ROS content from a rosbag file.
```nix
{ config, pkgs, ... }:
{
  services.ros.bagPlayer = {
    file = <path-to-rosbag-file>;

    # Default if `topics` is unspecified is to play back all topics in
    # the file.
    topics = [
      "/topic1"
      "/topic2"
    ];
  };
}
```
`bagPlayer` always plays back the file in a continuous loop.

### elevation_mapping

elevation_mapping is an ANYbotics library for building a map out of
point clouds. The library is not present in APT, so it must be
compiled from source.
```nix
{ config, pkgs, ... }:
{
  services.ros.elevationMapping = {
    # Use `build` to have `elevationMapping` compile but not actually
    # install.
    # build = true;  # Not usually required

    # Actually install and run `elevationMapping`; implies `build`.
    enable = true;

    # Set of point cloud sensors to build the map out of.
    pointClouds = [
      "sensor1"
      "sensor2"
    ];

    # Reference frame for the point clouds.
    trackPointFrameId = "some_frame";
  };
}
```

### move_base

The ROS1 navigation library. Support is currently very limited and
untested.
```nix
{ config, pkgs, ... }:
{
  services.ros.moveBase.enable = true;
}
```

### Intel Realsense cameras

At present a single Realsense camera is supported (this needs to
change ASAP).
```nix
{ config, pkgs, ... }:
{
  services.ros.realsense2 = {
    enable = true;

    # Standard realsense2 options.
    options = {
      # These are all the defaults and may be omitted.
      mode = "preset";
      serial_no = "";
      usb_port_id = "";
      camera_type = "";
      enable_ir = false;
      enable_depth = true;
      depth_width = null;
      depth_height = null;
      depth_fps = 60;
      enable_color = true;
      color_width = 640;
      color_height = 480;
      color_fps = 60;
      enable_fisheye = true;
      fisheye_width = 640;
      fisheye_height = 480;
      fisheye_fps = 60;
      enable_imu = true;
      enable_pointcloud = false;
      enable_tf = true;
      enable_tf_dynamic = false;
      base_frame_id = "camera_link";
      depth_frame_id = "camera_depth_frame";
      depth_optical_frame_id = "camera_depth_optical_frame";
      color_frame_id = "camera_rgb_frame";
      color_optical_frame_id = "camera_rgb_optical_frame";
      ir_frame_id = "camera_ir_frame";
      ir_optical_frame_id = "camera_ir_optical_frame";
      ir2_frame_id = "camera_ir2_frame";
      ir2_optical_frame_id = "camera_ir2_optical_frame";
      fisheye_frame_id = "camera_fisheye_frame";
      fisheye_optical_frame_id = "camera_fisheye_optical_frame";
      imu_frame_id = "camera_imu_frame";
      imu_optical_frame_id = "camera_optical_frame_id";
    };
  };
}
```

### rosbridge

A simple bridge between ROS and the rest of the world; a TCP server
that listens for commands written by outside programs, and performs
them within the ROS network.

```nix
{ config, pkgs, ... }:
{
  services.ros.rosbridge.enable = true;
}
```

### Static transform publishers

TODO

### Test publisher

TODO

### USB camera

TODO

## To-do

* Make a function called `ros4nix doc` that auto-generates
  documentation for the system.
* Get some means of having auto-complete; the actual ROS tools will
  complete things like `rosrun r<TAB>` into a list of every ROS tool
  that starts with "r", and this is currently not implemented.

# Implementation

`ros4nix` is implemented as a NixOS module, which operates by adding a
large set of custom configuration options to the `ros` namespace of
options in NixOS.

The configuration is very roughly split up into two sections,
`programs.ros` and `services.ros`; some redundancy is present and the
subdivision here is not even, but in general, `program.ros.*` concerns
itself with *static* configuration of the system and making the ROS
command-line options available, whereas `services.ros.*` concerns
itself with making run-time configurations that are enabled at boot
time. The main core of the `programs.ros.*` namespace is in
`ros/ros_core.nix`, and the core of the `services.ros.*` namespace is
in `ros/ros_services.nix`.

## ROS Core

The main ROS core is implemented as part of the system activation
script (which runs every time `nixos-rebuild switch` is executed), and
uses three "stages" to build a ROS subsystem on the current system:
1. Get the main operating system up and running, upon which we will
   build a ROS system. In non-chroot mode (i.e., with a native
   Ubuntu/Debian host that will have a real ROS installation into its
   root directory, or when set up for use in Podman; when
   `programs.ros.useMainRoot` is enabled) this stage does nothing,
   because the host operating system is already capable of using `apt`
   to install ROS packages. On all other systems, we make a directory
   `/var/ros`, and use `debootstrap` to install Ubuntu into that
   directory.
2. Use `apt` to ensure `curl` and `gnupg` are available to be able to
   download the ROS public key, add the ROS public key to the system,
   ensure that the standard Ubuntu APT sources are placed in
   `/etc/apt/sources.list.d`, do a system update, and then install all
   needed ROS packages. In chroot mode, stage 2 is called using
   `bwrap` to chroot into the generated Ubuntu system. Stage 2 always
   leaves a script called `/var/ros/init` (`/init` if non-chroot)
   containing the precise work done.
3. For any packages that need to be built from source, copy their
   source trees into `/var/ros/catkin_ws/src`, then use `catkin build`
   to build `/var/ros/catkin_ws` in release mode. Stage 3 always
   leaves a script called `/var/ros/stage3` containing the precise
   work done.
4. Link the generated ROS executables from the container environment
   into the main system. If the system is running in non-chroot mode
   this step does nothing; otherwise, we create the directory
   `/var/ros/nixWrappers`, and fill it with links to wrappers for ROS
   programs. Every program installed in the container's `/bin`
   starting with the string `catkin` s installed, as is every
   executable in the container's `/opt/ros/<distro>/bin` directory.

   The actual wrapper program that is linked is not as trivial as it
   seems. It performs a few basic tasks:
   - Get all the arguments off of the command line, and put them in a
     temporary file instead, separated by zeroes. This is because we
     source the catkin workspace every time we run a ROS command, and
     sourcing the workspace seems to break horribly whenever the
     script has any arguments. I don't know why, it's an issue with
     ROS; all I know is it causes massive problems that took weeks to
     figure out.
   - Take an educated guess at `ROS_IP` if it's not specified by a
     command-line argument. ROS by default doesn't initialize `ROS_IP`
     and uses hostnames instead; on the majority of networks (at
     educational institutions at least) this doesn't work.
   - Add `/bin` and `/sbin` to the `PATH`, and set the `SHELL`
     variable to `/bin/sh`; this fixes behavior in some ROS packages
     that make assumptions about these variables.
   - Use `bwrap` (from nixpkgs) to enter the container, and call an
     "inner wrapper" program.
   - The inner wrapper, which now is in the container and has no
     arguments, can safely source the container `setup` file. For
     technical reasons relating to the script trying to set up
     autocompletion thinking this is an interactive session, this
     works better if the script is a `zsh` script than a `bash`
     script, so the inner wrapper is actually a `zsh` script.
   - Load the command-line arguments off of the temporary file that
     they were dumped into before, and use them to execute the target
     program.

These four stages are more-or-less idempotent: after they are run
once, they will be relatively quick to execute and not greatly change
the system state. That being said, for efficiency, we only re-execute
stage 2 and 3 when the generated scripts to do so change. Stage 1 is
never re-executed once the system is initialized once; you can force
it to be re-executed by doing `sudo rm -rf /var/ros`. Stage 4 is so
fast we re-execute it every time.

On non-NixOS targets using the `ros4nix` executable, the ROS core
setup is implemented by tearing out the relevant section of the
activation script into a standalone executable and calling it after
building the system.

## ROS Services

The services setup provides the option `services.ros.enable` that
starts up a ROS master (assuming `programs.ros.master` doesn't specify
that a remote node is running the ROS master), a notoriously tricky
service to get right. Beyond this, it provides options `runServices`
and `launchServices` that take ROS-specific options and will use
`rosrun` and `roslaunch`, respectively, to start up nodes after the
ROS master has started. The commands are mapped to systemd services
starting with the string `ros-`.

On non-NixOS targets using the `ros4nix` executable, this setup works
by building the target system's `/etc/systemd/system` directory, then
copying the generated service files that all start with `ros-` out
into the main host's `/etc/systemd/system` directory, then using
`systemctl` to enable and start them.
