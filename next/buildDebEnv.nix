# Build a Debian environment.
{ pkgs ? import <nixpkgs> { }
, # List of Debian packages to install, as strings.
  debianPackages
, # List of { listing, prefix } pairs, where `listing` is a path to a
  # `Packages` or `Packages.xz` file, and `prefix` is the prefix to
  # prepend to all download paths.
  debFilesAndPrefixes
, # Function that takes the root directory and applies any finishing
  # touches to it, returning a new root directory. If not supplied,
  # this does nothing.
  fixupRoot ? (x: x)
,
}:
with pkgs;

let
  fetchdebs = rustPlatform.buildRustPackage rec {
    pname = "fetchdebs";
    version = "0.1.0";

    src = ./fetchdebs;
    cargoLock.lockFile = ./fetchdebs/Cargo.lock;

    buildInputs = [ xz.dev ];
    nativeBuildInputs = [ pkg-config ];
  };

  neededPackagesArg = builtins.concatStringsSep "," debianPackages;
  debFilesArgs = builtins.concatStringsSep " " (map
    ({ listing, prefix }: "${listing}@${prefix}")
    debFilesAndPrefixes
  );

  debListNixFile = runCommand "deb-listing" { } ''
    ${fetchdebs}/bin/fetchdebs ${neededPackagesArg} ${debFilesArgs} > $out
  '';
  debList = import debListNixFile { inherit fetchurl; };
  debListNeedsInstall = builtins.filter
    (deb:
      let
        debName = builtins.substring 44 (-1) "${deb}";
        allowedNames = [
          "libblas" "liblapack"
        ];
      in
      builtins.any
        (allowedName:
          lib.strings.hasPrefix allowedName debName
        )
        allowedNames
    )
    debList;

  # see nixpkgs/pkgs/build-support/vm/deb/default.nix, `fillDiskWithDebs`
  runPostinstHooks = writeScript "runPostinst" ''
    #!${bash}/bin/bash
    set -e

    export PATH=/bin:/usr/bin:/sbin:/usr/sbin:$PATH

    mv "/sbin/start-stop-daemon" "/sbin/start-stop-daemon.REAL"
    echo "#!/bin/true" > "/sbin/start-stop-daemon"
    chmod 755 "/sbin/start-stop-daemon"

    export DEBIAN_FRONTEND=noninteractive
    for deb in ${builtins.concatStringsSep " " debListNeedsInstall}; do
      echo "installing $deb..."
      ${dpkg}/bin/dpkg --install --force-all "$deb" 2>/dev/null >/dev/null || true
    done

    mv "/sbin/start-stop-daemon.REAL" "/sbin/start-stop-daemon"
  '';

  root = runCommand "sysroot" { } ''
    mkdir -p $out/build  # convenience if it needs to be bound by proot
    for deb in ${builtins.concatStringsSep " " debList}; do
      echo "unpacking $deb..."
      ${dpkg}/bin/dpkg-deb --extract "$deb" $out 2>/dev/null || true
    done

    ${proot}/bin/proot -R $out --bind=/nix --pwd=/ \
      -0 ${runPostinstHooks}
  '';

  enterScript = writeScriptBin "deb-env" ''
    #!${bash}/bin/bash
    CMD=
    if [ "$#" = 0 ]; then
      CMD=/bin/bash
    fi
    PATH=/bin:/usr/bin:$PATH \
      ${proot}/bin/proot -R ${root} \
      -b /nix \
      -b /home \
      -b /persistent \
      --kill-on-exit \
      $CMD "$@"
  '';

  runInEnv = drv: lib.overrideDerivation drv ({...}: {
    builder = writeScript "builder.sh" ''
      #!${bash}/bin/bash
      set -e

      PATH=/bin:/usr/bin:$PATH \
        ${proot}/bin/proot -R ${root} \
        -b /nix \
        -b /build \
        --kill-on-exit \
        ${drv.builder} ${builtins.concatStringsSep " " drv.args}
    '';
    args = [];
  });
in
{
  inherit root enterScript runInEnv;
}
