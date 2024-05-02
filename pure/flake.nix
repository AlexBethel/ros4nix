{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs = { self, nixpkgs }: {
    packages.x86_64-linux.ros = import ./default.nix {
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
    };
    packages.x86_64-linux.default = self.packages.x86_64-linux.ros;

    packages.aarch64-linux.ros = import ./default.nix {
      pkgs = nixpkgs.legacyPackages.aarch64-linux;
    };
    packages.aarch64-linux.default = self.packages.aarch64-linux.ros;
  };
}
