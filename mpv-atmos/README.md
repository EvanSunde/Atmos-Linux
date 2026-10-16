# mpv-atmos — real-time Dolby Atmos heights in mpv

Atmos films and music play in mpv with their height channels decoded live, from the moment
you press play. There's no pre-decoding and no waiting; pause, seek and speed all work.

```
mpv opens film.mkv → selected audio is Atmos? (ffprobe profile)
   ├─ yes: atmos-stream <file> <track> <time>      restarted on every seek
   │        ffmpeg (copy, cut at exact frame) → decoder → 12-ch 7.1.4 → mpv audio track
   │          E-AC-3 + JOC  → atmos-decode (Cavern, now reads the pipe live)
   │          TrueHD Atmos  → thd-atmos (truehd crate + object panner, this repo)
   └─ no:  normal mpv playback
mpv → "Atmos Headphones" PipeWire sink → KU100 HRTF → headphones
```

Install (after linux-atmos):

```sh
sudo pacman -S --needed mpv ffmpeg dotnet-runtime   # runtime only, no SDK needed
./install.sh
```

Both decoders are prebuilt (`bin/`). `./install.sh --build` compiles them from source instead,
which needs `rust` (about 8 GB of RAM for an optimised build, which also makes the TrueHD
decoder faster) and `dotnet-sdk`.

* **Seeking:** after a seek, the normal 5.1/7.1 track plays for about half a second while the
  decoder restarts, then the Atmos track takes over. The live track is cut at the exact frame
  and stays within ±0.5 ms of the video.
* **Ctrl+a:** switches between Atmos heights and the core audio.
* **Options:** set these in `~/.config/mpv/script-opts/atmos_live.conf`.
  * `enabled=yes|no`
  * `force=yes`: treats every E-AC-3/TrueHD track as Atmos.
  * `preroll=0.3`
* **Failure:** if anything fails, mpv keeps playing the core audio and logs
  `atmos-stream: ...` (run `mpv -v` to see it).

## Limits

* The truehd crate is experimental. Object positions follow the Atmos metadata with ramps,
  rendered onto 7.1.4. Object size, zones and screen-relative positioning aren't applied yet.
* Streams that only use Atmos "ISF" (intermediate spatial format) objects aren't rendered
  (rare).

## Tested

* **E-AC-3 Atmos** ("On The Run", 2023 Atmos mix: 16 objects, 5.1 core):
  * Live pipe decode is byte-identical to a full-file decode over all 215.264 s.
  * Seek points land with 0.00 ms error.
  * After a restart, the per-speaker energy matches a continuous decode to within 0.1 dB.
  * In mpv: seek, pause, seek back, quit, no leftover processes, and files that don't
    start at 0:00 all behave correctly.
* **TrueHD:** FFmpeg-encoded test streams (no real Atmos TrueHD available):
  * Decode is bit-exact; seek alignment is within ±0.33 ms (Matroska millisecond
    timestamps).
  * The object panner was checked against the truehd crate's real Atmos OAMD sample.
