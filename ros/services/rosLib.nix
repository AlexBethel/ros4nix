# Library of functions specifically for ROS.
{ lib, ... }:
{
  # Convert an attrset to the `roslaunch` and `rosrun` command-line
  # format. For example, given { a = "5"; b = true; c = null; },
  # returns the string "a:=5 b:=true c:=".
  attrsToCmdLine = attrs:
    builtins.concatStringsSep " " (
      builtins.attrValues (builtins.mapAttrs
        (
          name: value:
            # builtins.toString tries to convert bools to integers;
            # don't use that.
            if value == true
            then "${name}:=true"
            else if value == false
            then "${name}:=false"
            else "${name}:=${builtins.toString value}"
        )
        attrs)
    );

  # (str -> t -> {name: str, value: u}) -> attrsOf t -> attrsOf u
  mapAttrsFull = with builtins; transform: attrs:
    listToAttrs (attrValues (mapAttrs transform attrs));

  # (str -> t -> u) -> attrsOf t -> [u]
  mapAttrValues = with builtins; transform: attrs:
    attrValues (mapAttrs transform attrs);

  # (t -> {name: str, value: u}) -> [t] -> attrsOf u
  mapIntoAttrs = with builtins; transform: list:
    listToAttrs (map transform list);
}
