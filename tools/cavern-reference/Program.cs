// refharness <eac3> <outdir> <frames>
Directory.CreateDirectory(args[1]);
int frames = int.Parse(args[2]);
Cavern.Format.RefDump.Core(args[0], args[1], frames);
Cavern.Format.RefDump.Objects(args[0], args[1], frames * 24);
Console.WriteLine("done");
