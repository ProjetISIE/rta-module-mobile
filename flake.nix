{
  description = "Nix flake C/C++ development environment";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  inputs.esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
  outputs =
    {
      self,
      nixpkgs,
      esp-dev,
    }:
    let
      systems =
        f:
        nixpkgs.lib.genAttrs
          [
            "x86_64-linux" # "aarch64-linux"
            "aarch64-darwin"
          ]
          (
            system:
            f {
              pkgs = import nixpkgs {
                inherit system;
                overlays = [ esp-dev.overlays.default ];
                config.permittedInsecurePackages = [
                  "python3.13-ecdsa-0.19.1"
                ];
              };
            }
          );
    in
    {
      # devShells = esp-dev.devShells;
      devShells = systems (
        { pkgs }:
        {
          default =
            pkgs.mkShell.override
              {
                stdenv = pkgs.clangStdenv; # Clang instead of GCC
              }
              {
                packages = with pkgs; [
                  clang-tools # Clang CLIs
                  cmake # Automation tool
                  cmake-format # CMake formatter
                  cmake-language-server # LSP
                  cppcheck # Static analysis
                  clang-uml # Generate UML from c(++)
                  doctest # Testing framework
                  doxygen # Documentation generator
                  espflash # ESP32
                  esp-generate # ESP32
                  esphome # ESP32
                  esptool # ESP32
                  esp-idf-full # ESP32
                  gdb # Debugger
                  lcov # Code coverage
                  lldb # Debug adapter
                  mkspiffs-presets.esp-idf
                  ninja # Build
                  # pkg-config # Find libraries
                  tio # Serial
                  valgrind # Debugging and profiling
                  (pkgs.writeShellScriptBin "setup" "cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -GNinja -B build")
                ];
              };
        }
      );
    };
}
