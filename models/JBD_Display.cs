//
// JBD_Display — JBD microdisplay controller behind SPI (mode 3).
//
// Attached by default to the SPIM box at 0x4000A000/0x5000A000 ("spim_a";
// firmware calls it "SPIM2"); attachable to the 0x4000C000 box ("spim_b")
// by moving the registration line in platforms/nrf5340.repl. Pins and
// bitrate are runtime configuration of the SPI master only - this device
// has no fixed pin assignment (per the board map, deliberately).
//
// Protocol per the project spec (authoritative). The device is framed by
// the SPI transaction: a command starts at the first byte after chip
// select / transaction start and ends at FinishTransmission().
//
//   - 0x9F: identification. The two ID bytes 0x40, 0x10 (ID 0x4010) are
//     clocked out on the bytes following the opcode.
//   - 0x02: pixel window write:
//       byte0   = 0x02
//       byte1-3 = 19-bit coordinate word coord = (x & 0x3FF) | ((y << 10) & 0x7FC00),
//                 serialized in 3 bytes
//       byte4   = 0xFF marker
//       byte5.. = packed 4-bpp pixel bytes (two pixels per byte) written into
//                 the 640x200 framebuffer (320 bytes/row) starting at (x, y),
//                 advancing linearly with row wrap
//       last    = trailing dummy byte (not written to the framebuffer)
//   - 0x01, 0x06, 0x31, 0x36, 0x46, 0x66, 0x71, 0x73, 0x97, 0x99, 0xA3,
//     0xA9, 0xB9, 0xC0: known opcodes whose panel meaning is unknown.
//     Recorded in the command journal (opcode + payload) and logged at
//     Info; NO speculative side effects are applied.
//   - anything else: UNSUPPORTED log (conventions format) + journaled.
//
// DOCUMENTED ASSUMPTIONS (spec gaps - each is a configurable property, not
// a silent guess):
//   * Coordinate byte order: LITTLE-ENDIAN (byte1 = coord[7:0], byte2 =
//     coord[15:8], byte3 = coord[18:16]) by default; set CoordBigEndian
//     true for the reverse.
//   * ID byte order: MSB first (0x40 then 0x10) by default; set IdLsbFirst
//     true for 0x10, 0x40.
//   * 4-bpp packing: the HIGH nibble of each byte is the LEFT (first)
//     pixel by default; set FirstPixelInLowNibble true for the reverse.
//     Raw framebuffer bytes are stored exactly as received - this option
//     only affects GetPixel()/PPM expansion.
//   * Pixel writes are byte-granular: the start byte offset is
//     y * 320 + x / 2; an odd x is aligned down to its byte (logged).
//     The linear offset wraps modulo the 64000-byte framebuffer.
//   * MISO fill: bytes clocked out when the device has nothing to say are
//     FillByte (default 0x00).
//
// Monitor/debug surface (what the web UI polls):
//   FrameCounter            - completed pixel-window writes
//   DumpFrame "path"        - dump the framebuffer as a binary P6 PPM
//                             (grayscale, nibble * 17 in R=G=B)
//   GetFramebufferBase64    - raw 64000-byte framebuffer, base64
//   GetFramebufferRowHex n  - one 320-byte row as hex
//   GetPixel x y            - single pixel value 0..15
//   GetJournal / JournalCount / ClearJournal - command journal
//   RecordFrames "path"     - from now on, append one line to the file after
//                             every completed pixel-window write (see below)
//   StopRecording           - close that file
//
// Frame recording. After every completed pixel-window write (each time
// FrameCounter goes up) RecordFrames appends one line:
//
//   <virtual time s> <frame#> <x> <y> <w> <h> <sha1> <framebuffer>
//
// x y w h is the bounding box of the pixels the write covered (a write that
// runs past the end of a row covers whole rows). <framebuffer> is the full
// 640x480 4-bpp framebuffer (153600 bytes, two pixels per byte, the left
// pixel in the high nibble unless FirstPixelInLowNibble) compressed with raw
// deflate (RFC 1951, no zlib header) and base64-encoded; <sha1> is the SHA-1
// of the uncompressed framebuffer in hex. If the framebuffer is the same as in
// the previous line, the line is only "<virtual time s> <frame#> same". The
// first line after RecordFrames is always a full one. When the lens reboots
// FrameCounter starts again at 0 (the next line is frame 1); the recording
// goes on.
//
// Fault injection (default off):
//   FaultWrongId - the 0x9F response bytes are bit-inverted.
//
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class JBD_Display : ISPIPeripheral, IDisposable
    {
        public JBD_Display(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            Array.Clear(framebuffer, 0, framebuffer.Length);
            journal.Clear();
            journalSequence = 0;
            journalDropped = 0;
            FrameCounter = 0;
            FaultWrongId = false;
            // The framebuffer was just cleared: the next recorded line has to
            // say so, whatever was written before.
            framebufferChanged = true;
            ResetFrameState();
        }

        public void Dispose()
        {
            StopRecording();
        }

        public byte Transmit(byte data)
        {
            byte response = FillByte;
            if(responseQueue.Count > 0)
            {
                response = responseQueue.Dequeue();
            }

            switch(state)
            {
            case State.Idle:
                BeginCommand(data);
                break;

            case State.CommandPayload:
                if(payloadCaptured < PayloadCaptureLimit)
                {
                    currentPayload.Add(data);
                }
                payloadCaptured++;
                break;

            case State.PixelCoord:
                coordBytes[coordByteCount++] = data;
                if(coordByteCount == 3)
                {
                    DecodeCoordinate();
                    state = State.PixelMarker;
                }
                break;

            case State.PixelMarker:
                if(data != 0xFF)
                {
                    this.Log(LogLevel.Warning,
                        "UNSUPPORTED periph={0} what=pixel-marker detail=expected 0xFF after coordinate, got 0x{1:X2}; discarding window at x={2} y={3}",
                        Label, data, pixelX, pixelY);
                    state = State.DiscardFrame;
                }
                else
                {
                    pixelBytesWritten = 0;
                    hasPendingPixelByte = false;
                    state = State.PixelData;
                }
                break;

            case State.PixelData:
                // One byte of lookahead: the panel needs four more clocks
                // after a pixel to take it in ("Final 4 dummy clocks are
                // needed", datasheet 7.3.15), so the last byte of the
                // transaction is handled in FinishTransmission.
                if(hasPendingPixelByte)
                {
                    CommitPixelByte(pendingPixelByte);
                }
                pendingPixelByte = data;
                hasPendingPixelByte = true;
                break;

            case State.DiscardFrame:
                break;
            }

            return response;
        }

        public void FinishTransmission()
        {
            switch(state)
            {
            case State.PixelData:
                // The last four clocks of the transaction are the dummy
                // clocks: of the held-back byte the first pixel (high nibble)
                // is taken in and the second is not. The firmware's row
                // flushes end on pixel data with no dummy byte, so the last
                // pixel of each such row is lost, on the panel as here; its
                // window writes with a trailing dummy byte put that byte's
                // first nibble one pixel past the window.
                var lastNibble = hasPendingPixelByte;
                if(hasPendingPixelByte)
                {
                    var merged = (byte)((pendingPixelByte & 0xF0) | (framebuffer[writeOffset] & 0x0F));
                    framebufferChanged |= merged != framebuffer[writeOffset];
                    framebuffer[writeOffset] = merged;
                    hasPendingPixelByte = false;
                }
                FrameCounter++;
                AddJournalEntry(0x02, true,
                    string.Format("pixel-window x={0} y={1} bytes={2} (+1 nibble)",
                        pixelX, pixelY, pixelBytesWritten));
                this.Log(LogLevel.Info, "pixel window done: x={0} y={1} {2} pixel bytes, frame #{3}",
                    pixelX, pixelY, pixelBytesWritten, FrameCounter);
                RecordFrame(lastNibble);
                break;

            case State.PixelCoord:
            case State.PixelMarker:
                this.Log(LogLevel.Warning,
                    "UNSUPPORTED periph={0} what=pixel-truncated detail=transaction ended inside the 0x02 header",
                    Label);
                AddJournalEntry(0x02, true, "truncated pixel-window header");
                break;

            case State.CommandPayload:
                AddJournalEntry(currentOpcode, currentOpcodeKnown, FormatPayload());
                break;

            case State.DiscardFrame:
                AddJournalEntry(0x02, true,
                    string.Format("discarded pixel-window (bad header) x={0} y={1}", pixelX, pixelY));
                break;
            }
            ResetFrameState();
        }

        // ------------------------- configuration surface -------------------------

        // Assumption knobs - see the header comment.
        public bool CoordBigEndian { get; set; }
        public bool IdLsbFirst { get; set; }
        public bool FirstPixelInLowNibble { get; set; }
        public byte FillByte { get; set; }

        // Dummy bytes emitted after the 0x9F opcode, before the ID bytes.
        // The real G1 firmware reads the ID on received bytes 2 and 3, so it
        // needs one dummy byte of read latency (set to 1 in g1.resc). Default
        // 0 keeps the ID on the byte immediately after the opcode.
        public uint IdReadLatency { get; set; }

        // Fault injection.
        public bool FaultWrongId { get; set; }

        // ------------------------- debug / monitor surface -------------------------

        public uint FrameCounter { get; private set; }

        public int JournalCount => journal.Count;

        public string GetJournal()
        {
            if(journal.Count == 0)
            {
                return "(journal empty)";
            }
            var sb = new StringBuilder();
            if(journalDropped > 0)
            {
                sb.AppendFormat("({0} oldest entries dropped)\n", journalDropped);
            }
            foreach(var entry in journal)
            {
                sb.AppendLine(entry);
            }
            return sb.ToString().TrimEnd('\n');
        }

        public void ClearJournal()
        {
            journal.Clear();
            journalDropped = 0;
        }

        public string DumpFrame(string path)
        {
            var header = Encoding.ASCII.GetBytes(string.Format("P6\n{0} {1}\n255\n", Width, Height));
            var body = new byte[Width * Height * 3];
            var idx = 0;
            for(var y = 0; y < Height; y++)
            {
                for(var x = 0; x < Width; x++)
                {
                    var v = (byte)(GetPixel(x, y) * 17);
                    body[idx++] = v;
                    body[idx++] = v;
                    body[idx++] = v;
                }
            }
            using(var stream = new System.IO.FileStream(path, System.IO.FileMode.Create, System.IO.FileAccess.Write))
            {
                stream.Write(header, 0, header.Length);
                stream.Write(body, 0, body.Length);
            }
            this.Log(LogLevel.Info, "framebuffer (frame #{0}) dumped to {1}", FrameCounter, path);
            return string.Format("frame #{0} -> {1}", FrameCounter, path);
        }

        // Start appending a line per completed pixel-window write to the file
        // at path (created or truncated); see the header comment.
        public void RecordFrames(string path)
        {
            StopRecording();
            recorder = new StreamWriter(path, false, new UTF8Encoding(false));
            recorder.AutoFlush = true;
            lastRecordedSha1 = null;
            this.Log(LogLevel.Info, "recording frames to {0}", path);
        }

        public void StopRecording()
        {
            if(recorder != null)
            {
                recorder.Flush();
                recorder.Dispose();
                recorder = null;
                this.Log(LogLevel.Info, "frame recording stopped");
            }
        }

        public string GetFramebufferBase64()
        {
            return Convert.ToBase64String(framebuffer);
        }

        public string GetFramebufferRowHex(int y)
        {
            if(y < 0 || y >= Height)
            {
                throw new ArgumentOutOfRangeException("y");
            }
            return string.Concat(framebuffer.Skip(y * BytesPerRow).Take(BytesPerRow).Select(b => b.ToString("X2")));
        }

        public int GetPixel(int x, int y)
        {
            if(x < 0 || x >= Width || y < 0 || y >= Height)
            {
                throw new ArgumentOutOfRangeException("x/y");
            }
            var b = framebuffer[y * BytesPerRow + x / 2];
            var leftPixel = (x & 1) == 0;
            var highNibble = leftPixel != FirstPixelInLowNibble; // left pixel in high nibble by default
            return highNibble ? (b >> 4) & 0xF : b & 0xF;
        }

        // ------------------------- internals -------------------------

        private void BeginCommand(byte opcode)
        {
            currentOpcode = opcode;
            currentPayload.Clear();
            payloadCaptured = 0;

            if(opcode == CommandId)
            {
                var first = IdLsbFirst ? (byte)(IdValue & 0xFF) : (byte)(IdValue >> 8);
                var second = IdLsbFirst ? (byte)(IdValue >> 8) : (byte)(IdValue & 0xFF);
                if(FaultWrongId)
                {
                    first = (byte)~first;
                    second = (byte)~second;
                    this.Log(LogLevel.Warning, "FaultWrongId active: responding with corrupted ID");
                }
                // Some hosts (the real G1 firmware's spi_read_id @0x47724)
                // clock one or more dummy bytes after the 0x9F opcode before
                // reading the ID: that firmware sends {9F,00,00,00} and checks
                // ((rx[2]<<8)|rx[3]) == 0x4010, i.e. it expects the ID on the
                // THIRD and FOURTH received bytes. IdReadLatency inserts that
                // many FillByte dummies ahead of the ID so it lands where the
                // host reads. Default 0 = ID on the byte right after the opcode.
                for(var i = 0; i < IdReadLatency; i++)
                {
                    responseQueue.Enqueue(FillByte);
                }
                responseQueue.Enqueue(first);
                responseQueue.Enqueue(second);
                currentOpcodeKnown = true;
                state = State.CommandPayload;
                this.Log(LogLevel.Info, "command 0x9F (identification), responding with ID 0x{0:X4}", IdValue);
            }
            else if(opcode == CommandPixelWrite)
            {
                coordByteCount = 0;
                state = State.PixelCoord;
            }
            else if(KnownOpcodes.Contains(opcode))
            {
                currentOpcodeKnown = true;
                state = State.CommandPayload;
                this.Log(LogLevel.Info, "known command 0x{0:X2} (panel meaning unknown), journaling without side effects", opcode);
            }
            else
            {
                currentOpcodeKnown = false;
                state = State.CommandPayload;
                this.Log(LogLevel.Warning,
                    "UNSUPPORTED periph={0} what=command detail=unknown opcode 0x{1:X2}; journaled, no side effects",
                    Label, opcode);
            }
        }

        private void DecodeCoordinate()
        {
            uint coord;
            if(CoordBigEndian)
            {
                coord = (uint)((coordBytes[0] << 16) | (coordBytes[1] << 8) | coordBytes[2]);
            }
            else
            {
                coord = (uint)(coordBytes[0] | (coordBytes[1] << 8) | (coordBytes[2] << 16));
            }
            if((coord & ~0x7FFFFu) != 0)
            {
                this.Log(LogLevel.Debug, "coordinate word 0x{0:X6} has bits above bit 18 set, masking to 19 bits", coord);
                coord &= 0x7FFFF;
            }
            pixelX = (int)(coord & 0x3FF);
            pixelY = (int)((coord >> 10) & 0x1FF);

            if(pixelX >= Width || pixelY >= Height)
            {
                this.Log(LogLevel.Warning,
                    "UNSUPPORTED periph={0} what=pixel-coord detail=out-of-range x={1} y={2} (max {3}x{4}); discarding window",
                    Label, pixelX, pixelY, Width, Height);
                state = State.DiscardFrame;
                return;
            }
            if((pixelX & 1) != 0)
            {
                this.Log(LogLevel.Debug, "odd x={0}: byte-granular writes align down to x={1} (documented assumption)",
                    pixelX, pixelX & ~1);
            }
            writeOffset = pixelY * BytesPerRow + pixelX / 2;
        }

        private void CommitPixelByte(byte value)
        {
            framebufferChanged |= framebuffer[writeOffset] != value;
            framebuffer[writeOffset] = value;
            writeOffset = (writeOffset + 1) % framebuffer.Length;
            pixelBytesWritten++;
        }

        // Appends the line for the pixel window that just completed to the
        // recording, if one is open. lastNibble: the window ended with the
        // high nibble of one more byte (see FinishTransmission).
        private void RecordFrame(bool lastNibble)
        {
            if(recorder == null)
            {
                return;
            }
            var time = machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds.ToString("F6", CultureInfo.InvariantCulture);
            string sha1 = null;
            if(framebufferChanged || lastRecordedSha1 == null)
            {
                using(var hash = SHA1.Create())
                {
                    sha1 = string.Concat(hash.ComputeHash(framebuffer).Select(b => b.ToString("x2")));
                }
            }
            framebufferChanged = false;
            if(sha1 == null || sha1 == lastRecordedSha1)
            {
                recorder.WriteLine("{0} {1} same", time, FrameCounter);
                return;
            }
            lastRecordedSha1 = sha1;

            // Bounding box of the pixels written, from the linear pixel range.
            var firstPixel = (pixelY * BytesPerRow + pixelX / 2) * 2;
            var endPixel = firstPixel + pixelBytesWritten * 2 + (lastNibble ? 1 : 0);
            endPixel = Math.Min(Math.Max(endPixel, firstPixel + 1), Width * Height);
            var firstRow = firstPixel / Width;
            var lastRow = (endPixel - 1) / Width;
            int x, w;
            if(firstRow == lastRow)
            {
                x = firstPixel % Width;
                w = endPixel - firstPixel;
            }
            else
            {
                x = 0;
                w = Width;
            }

            string packed;
            using(var compressed = new MemoryStream())
            {
                using(var deflate = new DeflateStream(compressed, CompressionLevel.Optimal, true))
                {
                    deflate.Write(framebuffer, 0, framebuffer.Length);
                }
                packed = Convert.ToBase64String(compressed.GetBuffer(), 0, (int)compressed.Length);
            }
            recorder.WriteLine("{0} {1} {2} {3} {4} {5} {6} {7}",
                time, FrameCounter, x, firstRow, w, lastRow - firstRow + 1, sha1, packed);
        }

        private string FormatPayload()
        {
            if(payloadCaptured == 0)
            {
                return "no payload";
            }
            var hex = string.Concat(currentPayload.Select(b => b.ToString("X2")));
            return payloadCaptured > currentPayload.Count
                ? string.Format("payload[{0}]={1}... (first {2} bytes)", payloadCaptured, hex, currentPayload.Count)
                : string.Format("payload[{0}]={1}", payloadCaptured, hex);
        }

        private void AddJournalEntry(byte opcode, bool known, string detail)
        {
            var timestamp = machine.ElapsedVirtualTime.TimeElapsed;
            journal.Enqueue(string.Format("#{0} t={1} op=0x{2:X2} {3} {4}",
                journalSequence++, timestamp, opcode, known ? "known" : "UNKNOWN", detail));
            while(journal.Count > JournalCapacity)
            {
                journal.Dequeue();
                journalDropped++;
            }
        }

        private void ResetFrameState()
        {
            state = State.Idle;
            responseQueue.Clear();
            currentPayload.Clear();
            payloadCaptured = 0;
            coordByteCount = 0;
            hasPendingPixelByte = false;
            pixelBytesWritten = 0;
        }

        private string Label
        {
            get
            {
                try
                {
                    return machine.GetLocalName(this);
                }
                catch
                {
                    return "JBD_Display";
                }
            }
        }

        private State state;
        private byte currentOpcode;
        private bool currentOpcodeKnown;
        private int payloadCaptured;
        private int coordByteCount;
        private int pixelX;
        private int pixelY;
        private int writeOffset;
        private int pixelBytesWritten;
        private byte pendingPixelByte;
        private bool hasPendingPixelByte;
        private ulong journalSequence;
        private ulong journalDropped;
        private StreamWriter recorder;
        private string lastRecordedSha1;
        private bool framebufferChanged;

        private readonly byte[] coordBytes = new byte[3];
        private readonly byte[] framebuffer = new byte[BytesPerRow * Height];
        private readonly List<byte> currentPayload = new List<byte>();
        private readonly Queue<byte> responseQueue = new Queue<byte>();
        private readonly Queue<string> journal = new Queue<string>();
        private readonly IMachine machine;

        private const int Width = 640;
        // Geometry corrected 200 -> 480: the real JBD panel is 640x480. The
        // boot clear writes 640x480x4bpp = 153600 bytes, and the dashboard
        // renderer emits pixel windows with y up to ~199 (dashboard is 640x199)
        // but the panel address space is 480 rows tall; a 200-row framebuffer
        // wrapped/lost any window landing at y>=200. 320 bytes/row (2px/byte).
        private const int Height = 480;
        private const int BytesPerRow = 320;
        private const ushort IdValue = 0x4010;
        private const byte CommandId = 0x9F;
        private const byte CommandPixelWrite = 0x02;
        private const int JournalCapacity = 1024;
        private const int PayloadCaptureLimit = 64;

        private static readonly HashSet<byte> KnownOpcodes = new HashSet<byte>
        {
            0x01, 0x06, 0x31, 0x36, 0x46, 0x66, 0x71, 0x73, 0x97, 0x99, 0xA3, 0xA9, 0xB9, 0xC0,
        };

        private enum State
        {
            Idle,
            CommandPayload,
            PixelCoord,
            PixelMarker,
            PixelData,
            DiscardFrame,
        }
    }
}
