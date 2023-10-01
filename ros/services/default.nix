# This file just imports all the services available in this directory.
{ ... }:
{
  imports = [
    ./elevationMapping.nix
    ./kindr.nix
    ./kindrRos.nix
    ./messageLogger.nix
    ./motorCtrl.nix
    ./moveBase.nix
    ./realsense.nix
    ./rosbridge.nix
    ./usbCamera.nix
  ];
}
