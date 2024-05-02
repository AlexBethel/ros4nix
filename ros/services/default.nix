# This file just imports all the services available in this directory.
{ ... }:
{
  imports = [
    ./bagPlayer.nix
    ./elevationMapping.nix
    ./kindr.nix
    ./kindrRos.nix
    ./messageLogger.nix
    ./motorCtrl.nix
    ./moveBase.nix
    ./pointcloudConcatenate.nix
    ./realsense.nix
    ./rosbridge.nix
    ./staticTransform.nix
    ./testPublisher.nix
    ./usbCamera.nix
  ];
}
