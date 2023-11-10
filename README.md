# ros4nix

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
different configurations

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
actual ROS installation; in this case, `ros4nix` basically just
dispatches `apt` commands and creates systemd services. To use this
mode, set `programs.ros.useMainRoot` to `true`.

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
