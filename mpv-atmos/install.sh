#!/usr/bin/env bash
# mpv-atmos: real-time Dolby Atmos (heights) in mpv on Linux. Needs linux-atmos installed first
# (the "Atmos Headphones" PipeWire sink). Installs:
#   ~/.local/bin/atmos-render        native Atmos decoder (E-AC-3 + JOC and TrueHD) - no .NET,
#                                    no runtime; atmos-decode and thd-atmos become links to it
#   ~/.local/bin/atmos-stream        per-seek real-time pipeline used by the mpv script
#   ~/.config/mpv/scripts/atmos-live.lua + an mpv.conf block
#
#   ./install.sh                     install (prebuilt binary, x86_64, glibc 2.34+)
#   ./install.sh --build SRC_DIR     compile atmos-render from its source repo with cargo instead
#   ./install.sh --uninstall
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BIN="$HOME/.local/bin"
DATA="${XDG_DATA_HOME:-$HOME/.local/share}/linux-atmos"
MPV="${XDG_CONFIG_HOME:-$HOME/.config}/mpv"
note() { echo "==> $*"; }
warn() { echo "WARNING: $*" >&2; }

if [[ ${1:-} == --uninstall ]]; then
    rm -f "$BIN/atmos-stream" "$BIN/atmos-render" "$MPV/scripts/atmos-live.lua"
    for l in atmos-decode thd-atmos; do [[ -L $BIN/$l && $(readlink "$BIN/$l") == *atmos-render ]] && rm -f "$BIN/$l"; done
    [[ -f $MPV/mpv.conf ]] && sed -i '/# >>> linux-atmos >>>/,/# <<< linux-atmos <<</d' "$MPV/mpv.conf"
    note "Removed."
    exit 0
fi

for t in ffmpeg ffprobe mpv mkfifo setsid; do command -v $t >/dev/null || { echo "missing: $t (pacman -S ffmpeg mpv util-linux)"; exit 1; }; done
mkdir -p "$BIN" "$MPV/scripts"

if [[ ${1:-} == --build ]]; then
    SRC="${2:?usage: ./install.sh --build path/to/atmos-render}"
    command -v cargo >/dev/null || { echo "cargo missing: sudo pacman -S rust"; exit 1; }
    note "Building atmos-render (the truehd dependency needs ~4 GB RAM to compile)"
    (cd "$SRC" && cargo build --release)
    install -m755 "$SRC/target/release/atmos-render" "$BIN/atmos-render"
else
    install -m755 "$HERE/bin/atmos-render" "$BIN/atmos-render"
fi
"$BIN/atmos-render" --positions >/dev/null || { echo "atmos-render doesn't run here; try ./install.sh --build"; exit 1; }

# Older decoders are replaced: the .NET atmos-decode and thd-atmos become links to atmos-render
# (same command-line options, so linux-atmos's atmos-play keeps working).
for l in atmos-decode thd-atmos; do ln -sfn "$BIN/atmos-render" "$BIN/$l"; done
rm -rf "$DATA/decoder-live"
note "atmos-render ready (native; the .NET runtime is no longer needed)"

install -m755 "$HERE/atmos-stream" "$BIN/atmos-stream"
install -m644 "$HERE/mpv/scripts/atmos-live.lua" "$MPV/scripts/atmos-live.lua"
rm -f "$MPV/scripts/atmos-auto.lua"   # older background-conversion script, replaced by this one

touch "$MPV/mpv.conf"
if ! grep -q "# >>> linux-atmos >>>" "$MPV/mpv.conf"; then
    cat >> "$MPV/mpv.conf" <<'CONF'
# >>> linux-atmos >>>
# mpv -> 12-ch "Atmos Headphones" sink; the file's own layout is passed through
# (7.1.4 live Atmos, 7.1 / 5.1 films, stereo music).
audio-device=pipewire/effect_input.atmos_headphones
audio-channels=auto
# <<< linux-atmos <<<
CONF
fi

case ":$PATH:" in *":$BIN:"*) ;; *) warn "$BIN is not in your PATH (the mpv script finds it anyway)";; esac
note "Done. Play an Atmos file in mpv: the OSD says 'Dolby Atmos: height channels ON'."
note "Ctrl+a toggles Atmos heights / core audio."
