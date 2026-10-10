using System;
using System.Collections.Generic;
using System.IO;
using Cavern.Format.Consts;
using Cavern.Format.Decoders;
using Cavern.Format.Renderers;
using Cavern.Format.Utilities;

namespace Cavern.Format {
    /// <summary>Reference dumps for verifying a port (local harness, not part of Cavern).</summary>
    public static class RefDump {
        /// core.bin per frame: int32 nch, int32 frameSize, int32 ch ids[nch], float32 planar pcm[nch*frameSize]
        /// aux.bin per frame: int32 nsub, then per substream: int32 substreamId, int32 nbytes, bytes
        public static void Core(string eac3Path, string outDir, int maxFrames) {
            using FileStream fs = File.OpenRead(eac3Path);
            BlockBuffer<byte> buffer = BlockBuffer<byte>.CreateForConstantPacketSize(fs, FormatConsts.blockSize);
            EnhancedAC3Decoder dec = new EnhancedAC3Decoder(buffer);
            using BinaryWriter core = new BinaryWriter(File.Create(Path.Combine(outDir, "core.bin")));
            using BinaryWriter aux = new BinaryWriter(File.Create(Path.Combine(outDir, "aux.bin")));
            List<(int, byte[])> pending = new List<(int, byte[])>();
            EnhancedAC3Decoder.RefAuxTap = (id, data) => pending.Add((id, data));
            for (int f = 0; f < maxFrames; f++) {
                pending.Clear();
                float[] inter = dec.RefDecodeFrame();
                if (dec.Finished || inter.Length == 0) break;
                var chs = dec.RefSortedChannels();
                int n = chs.Length, fsz = inter.Length / n;
                core.Write(n); core.Write(fsz);
                foreach (var c in chs) core.Write((int)c);
                for (int c = 0; c < n; c++) for (int i = 0; i < fsz; i++) core.Write(inter[i * n + c]);
                aux.Write(pending.Count);
                foreach (var (id, data) in pending) { aux.Write(id); aux.Write(data.Length); aux.Write(data); }
            }
        }

        /// objects.bin header: int32 nobj; per timeslot (64 samples): float32 pos[nobj*3], float32 vol[nobj], float32 samples[nobj*64]
        public static void Objects(string eac3Path, string outDir, int maxTimeslots) {
            using FileStream fs = File.OpenRead(eac3Path);
            AudioReader reader = AudioReader.Open(fs);
            Renderer r = reader.GetRenderer();
            using BinaryWriter w = new BinaryWriter(File.Create(Path.Combine(outDir, "objects.bin")));
            int n = r.Objects.Count;
            w.Write(n);
            for (int t = 0; t < maxTimeslots; t++) {
                float[][] s = r.GetNextObjectSamples(64);
                for (int o = 0; o < n; o++) { var p = r.Objects[o].Position; w.Write(p.X); w.Write(p.Y); w.Write(p.Z); }
                for (int o = 0; o < n; o++) w.Write(r.Objects[o].Volume);
                for (int o = 0; o < n; o++) for (int i = 0; i < 64; i++) w.Write(s[o][i]);
            }
        }
    }
}
