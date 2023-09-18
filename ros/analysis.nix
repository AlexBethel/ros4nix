{ config, lib, ... }:
{
  options = with lib; {
    ros.analysis.publishedTopics = mkOption {
      description = ''
        Topics published by this machine.
      '';

      type = with types; attrsOf submodule {
        options = {
          name = mkOption {
            type = str;
          };

          rosType = mkOption {
            type = str;
          };
        };
      };
    };

    # Don't analyze these, assume they're published somewhere else.
    ros.analysis.ignoreTopics = {};
  };

  config = {
    # assertions

    #

    ros.services.move_base.enable = true;
    # /map isn't being published to
    #   valid publishers of /map include:
    #   ...
  };
}
