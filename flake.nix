{
  description = "Rust workspace for embedded BLDC FOC drivers";

  inputs = {
    nixpkgs.url = "nixpkgs";
  };

  outputs = { self, nixpkgs }:
    let
      systems = [
        "aarch64-darwin"
        "x86_64-darwin"
        "aarch64-linux"
        "x86_64-linux"
      ];

      forAllSystems = f:
        nixpkgs.lib.genAttrs systems (system:
          f (import nixpkgs { inherit system; }) system);
    in
    {
      formatter = forAllSystems (pkgs: _: pkgs.nixfmt-rfc-style);

      devShells = forAllSystems (pkgs: system:
        let
          rustToolchain = with pkgs; [
            rustc
            cargo
            clippy
            rustfmt
            rust-analyzer
          ];
        in
        {
          default = pkgs.mkShell {
            packages = rustToolchain ++ (with pkgs; [
              bacon
              cargo-llvm-cov
              cargo-nextest
              git
              just
              llvm
              pkg-config
            ]);

            shellHook = ''
              export CARGO_TERM_COLOR=always
              export LLVM_COV="${pkgs.llvm}/bin/llvm-cov"
              export LLVM_PROFDATA="${pkgs.llvm}/bin/llvm-profdata"
              export RUST_BACKTRACE=1
              echo "driverdriver dev shell (${system})"
            '';
          };
        });
    };
}
