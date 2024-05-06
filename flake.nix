{
  outputs = { self, nixpkgs }: {
    nixosModules.default = { config, pkgs, ... }: {
      imports = [ ./ros ];
    };
  };
}
