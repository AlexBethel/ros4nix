{ config, pkgs, ... }:
{
  imports = [
    ./ros
  ];

  system.stateVersion = "23.05";
  boot.isContainer = true;

  nixpkgs.hostPlatform = "x86_64-linux";

  services.ros.enable = true;
  services.sshd.enable = true;
}
