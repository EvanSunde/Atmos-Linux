# atmos-render

A native, real-time Dolby Atmos decoder for Linux. It reads a raw **E-AC-3 + JOC** stream
(streaming services, Atmos music) or a **TrueHD Atmos** stream (Blu-ray) and writes 7.1.4 PCM
(12-channel interleaved float32: `FL FR FC LFE RL RR SL SR TFL TFR TRL TRR`). That output plugs
straight into the "Atmos Headphones" PipeWire sink of linux-atmos. It needs no .NET and no
runtime: it's one ~2 MB static-ish binary.

```sh
ffmpeg -i song.m4a -map 0:a:0 -c copy -f eac3 - | atmos-render - |
  pw-play --raw --target effect_input.atmos_headphones --format f32 --rate 48000 \
          --channels 12 --channel-map "$(atmos-render --positions)" -
```

It's a drop-in for the former C# `atmos-decode` (`--layout 7.1.4`, `--gain dB`,
`--positions`, `--info`) and for `thd-atmos`. Install it under those names as symlinks and
existing scripts keep working.

## How it works

| Stage | Module | Origin |
|---|---|---|
| E-AC-3 / AC-3 core decoding (headers, exponents, bit allocation, coupling, SPX, IMDCT) | `src/eac3/` | port of Cavern `Transcoders/EnhancedAC3*` |
| EMDF, object metadata (OAMD), joint object coding (JOC), QMF filterbank | `src/objects/` | port of Cavern `Decoders/EnhancedAC3/*` + `EnhancedAC3Renderer` glue |
| Objects → 7.1.4 speakers (balance-based panner, limiter) | `src/cavern_render/` | port of Cavern `Listener`/`Source` rendering, set up exactly like the former `atmos-decode` |
| TrueHD Atmos decoding | `src/truehd.rs` | the [`truehd`](https://crates.io/crates/truehd) crate + an OAMD-driven object panner (`src/render.rs`) |
| Streaming CLI, format detection | `src/main.rs`, `src/eac3_render.rs` | |

Input is processed as it arrives, with bounded memory (a few MB). There is no seeking:
players restart the decoder at the new position (see mpv-atmos's `atmos-stream`).

## Verification against Cavern

Every stage was compared with Cavern's own output on a real Atmos track (E-AC-3 JOC,
16 objects, 215 s):

| Check | Result |
|---|---|
| Core PCM, 1200 frames, 6 channels (`examples/verify_core`) | **bit-identical** (max error 0); aux bytes identical |
| Object samples + positions, 28,800 timeslots (`examples/verify_objects`, `verify_pipeline`) | samples ≤ −127.7 dB re peak, positions **exact** |
| Final 7.1.4 output, whole track (`examples/verify_render`, and the binary vs the C# `atmos-decode`) | max difference 1.5e-7 (−127.9 dB): float rounding |
| Decoding started mid-stream (as after a seek) | same as C# from the same point (−128 dB) |
| Plain 5.1 E-AC-3 (no Atmos) | identical to C# |
| TrueHD | byte-identical to the former `thd-atmos` |

Speed: about 20× real time on one core for E-AC-3 Atmos (the core decoder alone runs at
about 200×). Peak memory is about 4 MB, against about 145 MB for the .NET version.

To reproduce, `tools/cavern-reference/` has the C# harness that dumps Cavern's intermediate
data (core PCM, aux bytes, per-timeslot objects). Apply `cavern-hooks.patch` to a Cavern
checkout, build it with the .NET 8 SDK, then run the `verify_*` examples with the dump paths
as arguments.

Cavern quirks are kept on purpose so the output matches Cavern, and each is commented in the
code. Examples: `ReadSigned` always returns 0, the JOC timeslot counter wraps at 7, the
sparse-JOC workaround, and the first coupling coordinate offset. Where Cavern would throw on
broken input, this port resyncs or skips instead.

## Build

```sh
cargo build --release      # target/release/atmos-render
```

The `truehd` dependency needs more than 6 GB of RAM to compile with optimisation, so
`Cargo.toml` builds just that crate at `opt-level = 0`. TrueHD still decodes at about 3× real
time. On a machine with more RAM, delete that profile override for a faster TrueHD path.

## Licence

The E-AC-3, JOC and rendering parts are ported from **Cavern** by VoidX
(http://en.sbence.hu, https://github.com/VoidXH/Cavern). They fall under the Cavern licence
(`LICENSE.md`):
- **Allowed:** free use and free redistribution, with a link to the Cavern repository.
- **Not allowed:** selling it or showing advertisements.
- **Needs the author's permission:** public or commercial use.

The `truehd` crate is Apache-2.0.
