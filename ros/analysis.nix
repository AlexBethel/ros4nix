{ config, lib, ... }:
let rosLib = import ./services/rosLib.nix;
in
{
  options = with lib; {
    services.ros.analysis = {
      publishedTopics = mkOption {
        description = ''
          Topics published by this machine.
        '';

        default = {};

        type = with types; attrsOf (submodule {
          options = {
            name = mkOption {
              type = str;
            };

            rosType = mkOption {
              type = str;
            };
          };
        });
      };

      # Don't analyze these, assume they're published somewhere else.
      ignoreTopics = { };

      subscribedTopics = mkOption {
        description = ''
          Topics subscribed to by this machine.
        '';

        default = {};

        type = with types; attrsOf (submodule {
          options = {
            # name = mkOption {
            #   type = str;
            # };

            rosType = mkOption {
              type = str;
            };
          };
        });
      };
    };
  };

  config = {
    assertions = rosLib.mapAttrValues
      (name: value: {
        assertion = false;
        message = ''
          Topic ${name} is subscribed to, but not published.
        '';
      })
      config.services.ros.analysis.subscribedTopics;
  };
}
