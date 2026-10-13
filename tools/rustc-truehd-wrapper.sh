#!/bin/sh
# RUSTC_WRAPPER for low-memory machines: compiles only the `truehd` crate with reduced MIR
# inlining (fully optimised otherwise needs > 6 GB RAM); everything else is untouched.
#   RUSTC_BOOTSTRAP=1 RUSTC_WRAPPER=tools/rustc-truehd-wrapper.sh cargo build --release
rustc="$1"; shift
case " $* " in
    *" --crate-name truehd "*) exec "$rustc" "$@" -Zmir-opt-level=1 ;;
    *) exec "$rustc" "$@" ;;
esac
