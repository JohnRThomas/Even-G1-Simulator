//
// NRF5340_TrapStub — register-presence stub with mandatory access logging.
//
// Replaces silent read-zero stubs: EVERY access is logged in a stable,
// machine-parseable format including the accessing CPU's PC, direction,
// width, offset, value and the peripheral identity:
//
//   UNIMPL periph=<label> dir=R width=4 offset=0x504 value=0x00000000 pc=0x000012A4
//
// scripts/coverage_report.py aggregates these lines into the coverage
// report required by the acceptance criteria. Behaviour: writes are stored
// and read back (register-file semantics, so drivers that write-then-verify
// make progress); `presets` forces fixed read values for specific offsets
// (e.g. NVMC READY=1). Reads of untouched offsets return 0 — loudly.
//
// Also used for the reserved/unknown block at 0x41007000, per spec:
// trap and log, no speculative semantics.
//
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_TrapStub : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public NRF5340_TrapStub(IMachine machine, string label = "?", bool storeWrites = true, string presets = "", long size = 0x1000)
        {
            this.machine = machine;
            this.label = label;
            this.storeWrites = storeWrites;
            this.size = size;
            foreach(var pair in presets.Split(new[] { ';' }, System.StringSplitOptions.RemoveEmptyEntries))
            {
                var kv = pair.Split('=');
                presetValues[System.Convert.ToInt64(kv[0].Trim(), 16)] = System.Convert.ToUInt32(kv[1].Trim(), 16);
            }
        }

        public void Reset()
        {
            stored.Clear();
        }

        public uint ReadDoubleWord(long offset) => Read(offset, 4);
        public ushort ReadWord(long offset) => (ushort)Read(offset, 2);
        public byte ReadByte(long offset) => (byte)Read(offset, 1);

        public void WriteDoubleWord(long offset, uint value) => Write(offset, value, 4);
        public void WriteWord(long offset, ushort value) => Write(offset, value, 2);
        public void WriteByte(long offset, byte value) => Write(offset, value, 1);

        public long Size => size;

        private uint Read(long offset, int width)
        {
            uint value;
            if(presetValues.TryGetValue(offset, out var preset))
            {
                value = preset;
            }
            else if(!stored.TryGetValue(offset, out value))
            {
                value = 0;
            }
            LogAccess("R", offset, value, width);
            return value;
        }

        private void Write(long offset, uint value, int width)
        {
            LogAccess("W", offset, value, width);
            if(storeWrites)
            {
                stored[offset] = value;
            }
        }

        private void LogAccess(string dir, long offset, uint value, int width)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph={0} dir={1} width={2} offset=0x{3:X} value=0x{4:X8} pc=0x{5:X8}",
                label, dir, width, offset, value, pc);
        }

        private readonly IMachine machine;
        private readonly string label;
        private readonly bool storeWrites;
        private readonly long size;
        private readonly Dictionary<long, uint> stored = new Dictionary<long, uint>();
        private readonly Dictionary<long, uint> presetValues = new Dictionary<long, uint>();
    }
}
