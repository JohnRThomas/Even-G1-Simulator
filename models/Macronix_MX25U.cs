//
// Macronix_MX25U — MX25U25645G-family serial (quad) NOR flash.
//
// Source of truth: Macronix MX25U25645G datasheet rev. 1.4 (P/N PM2428,
// 2020-07-23) plus the pinned NCS v2.5.1 driver stack
// (third_party/zephyr/drivers/flash/nrf_qspi_nor.c,
//  third_party/hal_nordic/nrfx/drivers/src/nrfx_qspi.c).
//
// Identity (datasheet Table 11): RDID(9F) = C2 25 39, RES(AB) = 39,
// REMS(90) = C2/39. Capacity 32 MiB (256 Mbit), configurable.
//
// Implemented commands (3-byte/4-byte address per the 4BYTE state, except
// the dedicated 4-byte command set which always takes 4 address bytes):
//   READ 03/13, FAST READ 0B/0C, DREAD 3B/3C, 2READ BB/BC, QREAD 6B/6C,
//   4READ EB/EC, PP 02/12, 4PP 38/3E, SE 20/21, BE32K 52/5C, BE D8/DC,
//   CE 60/C7, RDSR 05, RDCR 15, WRSR 01, WREN 06, WRDI 04, RDID 9F,
//   RES/RDP AB, REMS 90, DP B9, EN4B B7, EX4B E9, RDEAR C8, WREAR C5,
//   RSTEN 66, RST 99, NOP 00.
// Unknown/unmodelled opcodes (RDSFDP 5A, QPI 35/F5, DTR ED/EE, suspend/
// resume, OTP/security/lock commands, SBL C0, fast-boot 16/17/18) log
// UNSUPPORTED and return 0xFF (inert), per docs/conventions.md.
//
// NOR semantics: program only clears bits (mem &= data), erase sets 0xFF.
// Page program wraps inside the 256-byte page (datasheet 10-28); when more
// than 256 bytes are clocked in, only the last 256 are kept. Erase
// commands erase the sector/block CONTAINING the given address (low bits
// ignored, as on hardware); a diagnostic is logged if the address was not
// aligned. Program/erase to a busy device is ignored (WIP set); WEL is
// required and auto-clears when the operation completes.
//
// Block protection (BP3..0), SRWD/WP# hardware protection and the OTP/
// security areas are NOT enforced/modelled: BP/SRWD bits are stored and
// read back; if BP != 0 when a program/erase arrives, an UNSUPPORTED log
// is emitted and the operation proceeds as if unprotected.
//
// Timing (virtual time, lazily evaluated against the machine's virtual
// clock — fully deterministic): tPP/tSE/tBE32/tBE/tCE/tW busy periods use
// datasheet typical values by default, all configurable in microseconds.
// Deep power-down: tDP (entry) and tRES1 (exit) default to 10 us / 35 us
// per the board configuration (datasheet rev 1.4 gives tDP max 10 us,
// tRES1 max 30 us; the board reserves 35 us). While in DPD only RDP/RES
// (AB) and software reset are accepted; everything else is ignored.
// Commands arriving inside the tDP entry window are treated as already in
// DPD, and commands arriving inside the tRES1 exit window are ignored
// (datasheet leaves both windows unspecified; both cases log).
//
// Byte-transport contract (shared with NRF5340_QSPI): dummy/performance-
// enhance cycles of the fast/dual/quad read opcodes are line-level timing
// artifacts and are NOT represented at the byte level — data is returned
// starting with the first byte clocked after the address. The RES(AB)
// electronic-ID read keeps its 3 documented dummy bytes because the
// pinned driver actually clocks them.
//
// 24-bit addressing on a 32 MiB part: 3-byte commands address the bank
// selected by the extended address register (WREAR C5 / RDEAR C8,
// default 0), exactly as on hardware. Addresses that exceed the capacity
// (only possible via 4-byte commands or a non-zero EAR on a smaller
// configured capacity) follow WrapAddresses: true (default) wraps modulo
// capacity with a diagnostic log; false ignores the access (reads 0xFF).
//
// Fault injection (docs/conventions.md):
//   FaultWrongId   — RDID returns C2 25 00 (corrupted density byte).
//   FaultStuckBusy — WIP reads as 1 forever; program/erase are ignored
//                    (busy) and BusyUntil reports "never".
//
// Optional persistence: PersistTo (from the monitor), or persistencePath
// at creation, keeps the array in a file. An existing file replaces the
// array's contents (short file: remainder erased); a missing or empty one
// is created from the array as it is. From then on every change to the
// array (program, erase, LoadContents, LoadSegment) is written through to
// the file at once, so the contents survive however the emulation ends.
// The file is a raw image of the whole chip and is held open and locked
// (flock), so a second emulation cannot use it at the same time.
// SaveContents/LoadContents are callable from the monitor at any time.
//
using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.SPI
{
    // Timing back-channel used by NRF5340_QSPI to emulate the controller's
    // hardware WIP polling (which cannot be expressed with Transmit() alone,
    // because virtual time does not advance inside a single bus access).
    public interface INorFlashBusyTiming : ISPIPeripheral
    {
        // Virtual time at which the current program/erase/WRSR busy period
        // ends; TimeInterval.Maximal if it never will (stuck-busy fault).
        TimeInterval BusyUntil { get; }

        // Emulates a controller that polled RDSR until WIP cleared: commands
        // sent afterwards (in the same virtual instant) are evaluated as if
        // issued at BusyUntil.
        void NotifyControllerWipWait();
    }

    public class Macronix_MX25U : ISPIPeripheral, INorFlashBusyTiming, IDisposable
    {
        public Macronix_MX25U(IMachine machine, int capacity = 0x2000000, string persistencePath = null,
            byte manufacturerId = 0xC2, byte memoryType = 0x25, byte memoryDensity = 0x39)
        {
            if(capacity < (int)Block64Size || (capacity & (capacity - 1)) != 0)
            {
                throw new Exceptions.ConstructionException("capacity must be a power of two, at least 64 KiB");
            }
            this.machine = machine;
            this.capacity = capacity;
            this.manufacturerId = manufacturerId;
            this.memoryType = memoryType;
            this.memoryDensity = memoryDensity;
            memory = new byte[capacity];
            for(var i = 0; i < capacity; i++)
            {
                memory[i] = EmptyByte;
            }
            if(persistencePath != null)
            {
                PersistTo(persistencePath);
            }
            ResetVolatileState();
        }

        public void Reset()
        {
            // Power-on/reset clears volatile state only; the array and the
            // non-volatile status/configuration bits are retained.
            ResetVolatileState();
        }

        public void Dispose()
        {
            backing?.Dispose();
            backing = null;
        }

        public byte Transmit(byte data)
        {
            if(phase == Phase.Idle)
            {
                DecodeOpcode(data);
                return EmptyByte;
            }
            return HandleDataByte(data);
        }

        public void FinishTransmission()
        {
            FinalizeCommand();
            phase = Phase.Idle;
            currentOpcode = 0;
            bytesCollected = 0;
            dataBytesSeen = 0;
        }

        // ---------------- monitor-facing API ----------------

        public void PersistTo(string path)
        {
            // Keeps the array in a file from now on (see header).
            FileStream stream;
            try
            {
                stream = new FileStream(path, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
            }
            catch(IOException e)
            {
                throw new Exceptions.RecoverableException($"cannot open the flash file {path}: {e.Message}");
            }
            if(stream.Length > capacity)
            {
                var length = stream.Length;
                stream.Dispose();
                throw new Exceptions.RecoverableException(
                    $"the flash file {path} is {length} bytes, more than the chip's {capacity}");
            }
            var existing = (int)stream.Length;
            if(existing > 0)
            {
                var read = 0;
                while(read < existing)
                {
                    var got = stream.Read(memory, read, existing - read);
                    if(got <= 0)
                    {
                        break;
                    }
                    read += got;
                }
                for(var i = read; i < capacity; i++)
                {
                    memory[i] = EmptyByte;
                }
            }
            backing?.Dispose();
            backing = stream;
            if(existing != capacity)
            {
                Persist(0, capacity);
            }
            this.Log(LogLevel.Info, "Flash contents kept in {0} ({1})", path,
                existing == 0 ? "new file" : existing == capacity ? "existing file" : "existing file, extended");
        }

        public void SaveContents(string path)
        {
            File.WriteAllBytes(path, memory);
            this.Log(LogLevel.Info, "Flash contents ({0} bytes) saved to {1}", memory.Length, path);
        }

        public void LoadContents(string path)
        {
            var data = File.ReadAllBytes(path);
            if(data.Length != capacity)
            {
                this.Log(LogLevel.Warning, "Persistence file {0} is {1} bytes, capacity is {2}; {3}",
                    path, data.Length, capacity, data.Length < capacity ? "remainder stays erased" : "excess ignored");
            }
            var count = Math.Min(data.Length, capacity);
            Array.Copy(data, memory, count);
            for(var i = count; i < capacity; i++)
            {
                memory[i] = EmptyByte;
            }
            Persist(0, capacity);
            this.Log(LogLevel.Info, "Flash contents loaded from {0}", path);
        }

        public void LoadSegment(long offset, string path)
        {
            // Backdoor seeding of a sub-range of the array (e.g. the G1
            // external-flash settings store at 0x130000..). Reads a small
            // file and copies it verbatim at `offset`; the rest of the array
            // is untouched. Runs before the machine starts, so it models
            // pre-provisioned NOR contents. Unlike LoadContents this does not
            // require a full-capacity image.
            var data = File.ReadAllBytes(path);
            if(offset < 0 || offset + data.Length > capacity)
            {
                throw new Exceptions.RecoverableException(
                    $"segment [0x{offset:X}, 0x{offset + data.Length:X}) does not fit in capacity 0x{capacity:X}");
            }
            Array.Copy(data, 0, memory, offset, data.Length);
            Persist(offset, data.Length);
            this.Log(LogLevel.Info, "Seeded {0} bytes at 0x{1:X} from {2}", data.Length, offset, path);
        }

        public uint ReadMemory(uint address)
        {
            // Backdoor inspection helper (little-endian double word).
            var a = (int)(address % (uint)capacity);
            return (uint)(memory[a]
                | memory[(a + 1) % capacity] << 8
                | memory[(a + 2) % capacity] << 16
                | memory[(a + 3) % capacity] << 24);
        }

        // Fault injection (see header).
        public bool FaultWrongId { get; set; }
        public bool FaultStuckBusy { get; set; }

        // High-address wrap/alias policy (see header).
        public bool WrapAddresses { get; set; } = true;

        // Busy durations, virtual-time microseconds (datasheet typical).
        public ulong PageProgramTimeUs { get; set; } = 150;         // tPP typ 0.15 ms
        public ulong SectorEraseTimeUs { get; set; } = 25000;       // tSE typ 25 ms
        public ulong Block32EraseTimeUs { get; set; } = 150000;     // tBE32 typ 150 ms
        public ulong Block64EraseTimeUs { get; set; } = 220000;     // tBE typ 220 ms
        public ulong ChipEraseTimeUs { get; set; } = 75000000;      // tCE typ 75 s
        public ulong StatusWriteTimeUs { get; set; } = 40000;       // tW 40 ms
        public ulong DpdEnterTimeUs { get; set; } = 10;             // tDP (board: 10 us)
        public ulong DpdExitTimeUs { get; set; } = 35;              // tRES1 (board: 35 us)

        // Observability for tests.
        public bool DeepPowerDown => deepPowerDown;
        public bool QuadEnabled => quadEnable;
        public bool WriteEnabled => writeEnableLatch;
        public bool FourByteMode => fourByteMode;
        public bool Busy => IsBusy(EffectiveNow);

        public TimeInterval BusyUntil => FaultStuckBusy ? TimeInterval.Maximal : busyUntil;

        public void NotifyControllerWipWait()
        {
            var target = BusyUntil;
            if(target > wipWaitBaseline)
            {
                wipWaitBaseline = target;
            }
        }

        // ---------------- command decode ----------------

        private void DecodeOpcode(byte opcode)
        {
            var now = EffectiveNow;
            currentOpcode = opcode;
            bytesCollected = 0;
            dataBytesSeen = 0;
            phase = Phase.Ignore;

            if(deepPowerDown && opcode != (byte)Commands.ReleaseDeepPowerDown
                && opcode != (byte)Commands.ResetEnable && opcode != (byte)Commands.ResetMemory)
            {
                this.Log(LogLevel.Debug, "Command 0x{0:X2} ignored: device is in deep power-down", opcode);
                return;
            }
            if(!deepPowerDown && now < standbyReadyAt && opcode != (byte)Commands.ReleaseDeepPowerDown)
            {
                // Datasheet: CS# must stay high for tRES1 after RDP; behaviour of
                // earlier commands is unspecified -> ignore, loudly.
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=MX25U what=cmd-during-dpd-exit detail=opcode 0x{0:X2} issued before tRES elapsed, ignored", opcode);
                return;
            }
            if(IsBusy(now) && !CommandAllowedWhileBusy(opcode))
            {
                this.Log(LogLevel.Debug, "Command 0x{0:X2} ignored: write in progress", opcode);
                return;
            }

            switch((Commands)opcode)
            {
            case Commands.ReadStatusRegister:
                phase = Phase.ReadStatus;
                break;
            case Commands.ReadConfigurationRegister:
                phase = Phase.ReadConfiguration;
                break;
            case Commands.WriteStatusRegister:
                phase = writeEnableLatch ? Phase.WriteStatus : IgnoredForMissingWel();
                break;
            case Commands.WriteEnable:
                writeEnableLatch = true;
                break;
            case Commands.WriteDisable:
                writeEnableLatch = false;
                break;
            case Commands.ReadIdentification:
                phase = Phase.ReadId;
                break;
            case Commands.ReleaseDeepPowerDown:
                // RDP/RES: DPD exit is applied at CS# high (FinalizeCommand);
                // if clocked further it outputs the electronic ID after three
                // dummy bytes (datasheet 10-4).
                phase = Phase.ReadElectronicId;
                break;
            case Commands.ReadElectronicManufacturerDeviceId:
                phase = Phase.RemsAddress;
                break;
            case Commands.DeepPowerDown:
                phase = Phase.DeepPowerDownPending;
                break;
            case Commands.Enter4ByteMode:
                fourByteMode = true;
                break;
            case Commands.Exit4ByteMode:
                fourByteMode = false;
                break;
            case Commands.ReadExtendedAddressRegister:
                phase = Phase.ReadExtendedAddress;
                break;
            case Commands.WriteExtendedAddressRegister:
                phase = writeEnableLatch ? Phase.WriteExtendedAddress : IgnoredForMissingWel();
                break;
            case Commands.ResetEnable:
                resetEnabled = true;
                return; // do not clear resetEnabled below
            case Commands.ResetMemory:
                if(resetEnabled)
                {
                    SoftwareReset();
                }
                else
                {
                    this.Log(LogLevel.Debug, "RST without preceding RSTEN, ignored");
                }
                break;
            case Commands.Nop:
                break;
            case Commands.Read:
            case Commands.FastRead:
            case Commands.DualOutputRead:
            case Commands.DualIoRead:
                StartRead(false);
                break;
            case Commands.QuadOutputRead:
            case Commands.QuadIoRead:
                StartRead(true);
                break;
            case Commands.Read4B:
            case Commands.FastRead4B:
            case Commands.DualOutputRead4B:
            case Commands.DualIoRead4B:
                StartRead(false, forced4Byte: true);
                break;
            case Commands.QuadOutputRead4B:
            case Commands.QuadIoRead4B:
                StartRead(true, forced4Byte: true);
                break;
            case Commands.PageProgram:
                StartProgram(false);
                break;
            case Commands.QuadPageProgram:
                StartProgram(true);
                break;
            case Commands.PageProgram4B:
                StartProgram(false, forced4Byte: true);
                break;
            case Commands.QuadPageProgram4B:
                StartProgram(true, forced4Byte: true);
                break;
            case Commands.SectorErase:
                StartErase(EraseKind.Sector4K);
                break;
            case Commands.SectorErase4B:
                StartErase(EraseKind.Sector4K, forced4Byte: true);
                break;
            case Commands.Block32Erase:
                StartErase(EraseKind.Block32K);
                break;
            case Commands.Block32Erase4B:
                StartErase(EraseKind.Block32K, forced4Byte: true);
                break;
            case Commands.Block64Erase:
                StartErase(EraseKind.Block64K);
                break;
            case Commands.Block64Erase4B:
                StartErase(EraseKind.Block64K, forced4Byte: true);
                break;
            case Commands.ChipErase60:
            case Commands.ChipEraseC7:
                StartChipErase();
                break;
            default:
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=MX25U what=opcode detail=0x{0:X2} not modelled, ignored", opcode);
                break;
            }
            resetEnabled = false;
        }

        private Phase IgnoredForMissingWel()
        {
            this.Log(LogLevel.Debug, "Command 0x{0:X2} ignored: WEL not set", currentOpcode);
            return Phase.Ignore;
        }

        private void StartRead(bool quad, bool forced4Byte = false)
        {
            if(quad && !quadEnable)
            {
                this.Log(LogLevel.Warning, "Quad read 0x{0:X2} ignored: QE bit not set", currentOpcode);
                return;
            }
            addressBytesNeeded = forced4Byte || fourByteMode ? 4 : 3;
            addressAccumulator = 0;
            phase = Phase.ReadAddress;
        }

        private void StartProgram(bool quad, bool forced4Byte = false)
        {
            if(!writeEnableLatch)
            {
                phase = IgnoredForMissingWel();
                return;
            }
            if(quad && !quadEnable)
            {
                this.Log(LogLevel.Warning, "Quad page program 0x{0:X2} ignored: QE bit not set", currentOpcode);
                return;
            }
            WarnIfBlockProtected();
            addressBytesNeeded = forced4Byte || fourByteMode ? 4 : 3;
            addressAccumulator = 0;
            Array.Clear(pageBufferValid, 0, PageSize);
            phase = Phase.ProgramAddress;
        }

        private void StartErase(EraseKind kind, bool forced4Byte = false)
        {
            if(!writeEnableLatch)
            {
                phase = IgnoredForMissingWel();
                return;
            }
            WarnIfBlockProtected();
            pendingEraseKind = kind;
            addressBytesNeeded = forced4Byte || fourByteMode ? 4 : 3;
            addressAccumulator = 0;
            phase = Phase.EraseAddress;
        }

        private void StartChipErase()
        {
            if(!writeEnableLatch)
            {
                phase = IgnoredForMissingWel();
                return;
            }
            if(blockProtectBits != 0)
            {
                // Datasheet: CE executes only when BP3..0 == 0.
                this.Log(LogLevel.Warning, "Chip erase ignored: BP bits are 0x{0:X}", blockProtectBits);
                writeEnableLatch = false;
                return;
            }
            pendingEraseKind = EraseKind.Chip;
            phase = Phase.ChipErasePending;
        }

        private void WarnIfBlockProtected()
        {
            if(blockProtectBits != 0)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=MX25U what=block-protection detail=BP=0x{0:X} set but protected ranges are not modelled; operation proceeds", blockProtectBits);
            }
        }

        // ---------------- data phase ----------------

        private byte HandleDataByte(byte data)
        {
            switch(phase)
            {
            case Phase.ReadStatus:
                return ComputeStatusRegister();
            case Phase.ReadConfiguration:
                return ComputeConfigurationRegister();
            case Phase.ReadId:
            {
                var id = FaultWrongId
                    ? new byte[] { manufacturerId, memoryType, 0x00 }
                    : new byte[] { manufacturerId, memoryType, memoryDensity };
                return id[dataBytesSeen++ % 3];
            }
            case Phase.ReadElectronicId:
                // 3 dummy bytes, then the electronic ID repeatedly.
                dataBytesSeen++;
                return dataBytesSeen <= 3 ? EmptyByte : memoryDensity;
            case Phase.RemsAddress:
                bytesCollected++;
                if(bytesCollected == 3)
                {
                    remsDeviceIdFirst = (data & 1) != 0;
                    phase = Phase.RemsData;
                }
                return EmptyByte;
            case Phase.RemsData:
            {
                var first = remsDeviceIdFirst ? memoryDensity : manufacturerId;
                var second = remsDeviceIdFirst ? manufacturerId : memoryDensity;
                return dataBytesSeen++ % 2 == 0 ? first : second;
            }
            case Phase.ReadExtendedAddress:
                return extendedAddress;
            case Phase.WriteExtendedAddress:
                if(dataBytesSeen++ == 0)
                {
                    extendedAddress = data;
                    writeEnableLatch = false; // tWREAW is nanoseconds; treated as instant
                }
                return EmptyByte;
            case Phase.WriteStatus:
                if(dataBytesSeen == 0)
                {
                    pendingStatusByte = data;
                }
                else if(dataBytesSeen == 1)
                {
                    pendingConfigurationByte = data;
                    hasPendingConfigurationByte = true;
                }
                else
                {
                    this.Log(LogLevel.Debug, "WRSR: extra data byte ignored");
                }
                dataBytesSeen++;
                return EmptyByte;
            case Phase.ReadAddress:
                addressAccumulator = (addressAccumulator << 8) | data;
                bytesCollected++;
                if(bytesCollected == addressBytesNeeded)
                {
                    if(!ResolveArrayAddress(out readPointer))
                    {
                        phase = Phase.Ignore;
                        return EmptyByte;
                    }
                    phase = Phase.ReadData;
                }
                return EmptyByte;
            case Phase.ReadData:
            {
                var value = memory[readPointer];
                readPointer = (readPointer + 1) % (ulong)capacity; // address counter rolls over
                return value;
            }
            case Phase.ProgramAddress:
                addressAccumulator = (addressAccumulator << 8) | data;
                bytesCollected++;
                if(bytesCollected == addressBytesNeeded)
                {
                    if(!ResolveArrayAddress(out programAddress))
                    {
                        phase = Phase.Ignore;
                        return EmptyByte;
                    }
                    phase = Phase.ProgramData;
                }
                return EmptyByte;
            case Phase.ProgramData:
            {
                // Wraps within the 256-byte page; >256 bytes: last 256 win.
                var slot = (int)((programAddress + (ulong)dataBytesSeen) & (PageSize - 1));
                pageBuffer[slot] = data;
                pageBufferValid[slot] = true;
                dataBytesSeen++;
                return EmptyByte;
            }
            case Phase.EraseAddress:
                addressAccumulator = (addressAccumulator << 8) | data;
                bytesCollected++;
                return EmptyByte;
            case Phase.DeepPowerDownPending:
            case Phase.ChipErasePending:
                // Datasheet: CS# must rise exactly at the byte boundary;
                // extra bytes invalidate the command.
                this.Log(LogLevel.Debug, "Unexpected data byte after 0x{0:X2}; command cancelled", currentOpcode);
                phase = Phase.Ignore;
                return EmptyByte;
            case Phase.Ignore:
            default:
                return EmptyByte;
            }
        }

        // ---------------- CS#-high actions ----------------

        private void FinalizeCommand()
        {
            var now = EffectiveNow;
            switch(phase)
            {
            case Phase.ProgramData:
                if(dataBytesSeen == 0)
                {
                    this.Log(LogLevel.Debug, "Page program with no data bytes, rejected");
                    break;
                }
                ApplyPageProgram(now);
                break;
            case Phase.ProgramAddress:
                this.Log(LogLevel.Debug, "Page program with incomplete address, rejected");
                break;
            case Phase.EraseAddress:
                if(bytesCollected != addressBytesNeeded)
                {
                    this.Log(LogLevel.Debug, "Erase with incomplete address, rejected");
                    break;
                }
                ApplyErase(now);
                break;
            case Phase.ChipErasePending:
                ApplyChipErase(now);
                break;
            case Phase.WriteStatus:
                if(dataBytesSeen == 0)
                {
                    this.Log(LogLevel.Debug, "WRSR with no data byte, rejected");
                    break;
                }
                ApplyWriteStatus(now);
                break;
            case Phase.DeepPowerDownPending:
                deepPowerDown = true;
                dpdEnteredAt = now + TimeInterval.FromMicroseconds(DpdEnterTimeUs);
                this.Log(LogLevel.Info, "Entering deep power-down (complete at {0})", dpdEnteredAt);
                break;
            case Phase.ReadElectronicId:
                if(deepPowerDown)
                {
                    deepPowerDown = false;
                    standbyReadyAt = now + TimeInterval.FromMicroseconds(DpdExitTimeUs);
                    this.Log(LogLevel.Info, "Released from deep power-down (standby at {0})", standbyReadyAt);
                }
                break;
            }
        }

        private void Persist(long start, int count)
        {
            // Writes a changed part of the array through to its file.
            if(backing == null)
            {
                return;
            }
            backing.Seek(start, SeekOrigin.Begin);
            backing.Write(memory, (int)start, count);
            backing.Flush();
        }

        private void ApplyPageProgram(TimeInterval now)
        {
            var pageBase = programAddress & ~(ulong)(PageSize - 1);
            for(var i = 0; i < PageSize; i++)
            {
                if(pageBufferValid[i])
                {
                    memory[pageBase + (ulong)i] &= pageBuffer[i]; // NOR: 1 -> 0 only
                }
            }
            Persist((long)pageBase, PageSize);
            writeEnableLatch = false;
            SetBusy(now, PageProgramTimeUs);
            this.Log(LogLevel.Debug, "Page programmed at 0x{0:X}, busy until {1}", pageBase, busyUntil);
        }

        private void ApplyErase(TimeInterval now)
        {
            if(!ResolveArrayAddress(out var address))
            {
                return;
            }
            ulong size;
            ulong durationUs;
            switch(pendingEraseKind)
            {
            case EraseKind.Sector4K:
                size = Sector4KSize;
                durationUs = SectorEraseTimeUs;
                break;
            case EraseKind.Block32K:
                size = Block32Size;
                durationUs = Block32EraseTimeUs;
                break;
            default:
                size = Block64Size;
                durationUs = Block64EraseTimeUs;
                break;
            }
            var start = address & ~(size - 1);
            if(start != address)
            {
                this.Log(LogLevel.Warning, "Erase address 0x{0:X} not aligned to 0x{1:X}; erasing containing unit at 0x{2:X}",
                    address, size, start);
            }
            for(var i = 0UL; i < size; i++)
            {
                memory[start + i] = EmptyByte;
            }
            Persist((long)start, (int)size);
            writeEnableLatch = false;
            SetBusy(now, durationUs);
            this.Log(LogLevel.Debug, "Erased 0x{0:X} bytes at 0x{1:X}, busy until {2}", size, start, busyUntil);
        }

        private void ApplyChipErase(TimeInterval now)
        {
            for(var i = 0; i < capacity; i++)
            {
                memory[i] = EmptyByte;
            }
            Persist(0, capacity);
            writeEnableLatch = false;
            SetBusy(now, ChipEraseTimeUs);
            this.Log(LogLevel.Debug, "Chip erased, busy until {0}", busyUntil);
        }

        private void ApplyWriteStatus(TimeInterval now)
        {
            // Writable SR bits: BP3..0 (5:2), QE (6), SRWD (7). WIP/WEL are not.
            blockProtectBits = (byte)((pendingStatusByte >> 2) & 0xF);
            quadEnable = (pendingStatusByte & (1 << 6)) != 0;
            statusRegisterWriteDisable = (pendingStatusByte & (1 << 7)) != 0;
            if(hasPendingConfigurationByte)
            {
                // CR: DC1/DC0 (7:6), PBE (4), TB (3, OTP); 4BYTE (5) is read-only.
                dummyCycleBits = (byte)((pendingConfigurationByte >> 6) & 0x3);
                preambleBitEnable = (pendingConfigurationByte & (1 << 4)) != 0;
                topBottom = (pendingConfigurationByte & (1 << 3)) != 0;
                hasPendingConfigurationByte = false;
            }
            writeEnableLatch = false;
            SetBusy(now, StatusWriteTimeUs);
            this.Log(LogLevel.Debug, "WRSR applied: BP=0x{0:X} QE={1} SRWD={2}, busy until {3}",
                blockProtectBits, quadEnable, statusRegisterWriteDisable, busyUntil);
        }

        private void SoftwareReset()
        {
            this.Log(LogLevel.Info, "Software reset (66h/99h)");
            if(IsBusy(EffectiveNow))
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=MX25U what=reset-during-write detail=data of the interrupted operation is undefined on hardware; model keeps the already-applied result");
            }
            ResetVolatileState();
        }

        private void ResetVolatileState()
        {
            phase = Phase.Idle;
            writeEnableLatch = false;
            fourByteMode = false;
            extendedAddress = 0;
            deepPowerDown = false;
            resetEnabled = false;
            busyUntil = TimeInterval.Empty;
            wipWaitBaseline = TimeInterval.Empty;
            standbyReadyAt = TimeInterval.Empty;
            dpdEnteredAt = TimeInterval.Empty;
            hasPendingConfigurationByte = false;
        }

        // ---------------- helpers ----------------

        private byte ComputeStatusRegister()
        {
            var now = EffectiveNow;
            var busy = now < busyUntil;
            var wip = busy || FaultStuckBusy;
            var wel = busy || writeEnableLatch; // WEL stays set while the operation runs
            return (byte)((wip ? 1 : 0)
                | (wel ? 1 << 1 : 0)
                | blockProtectBits << 2
                | (quadEnable ? 1 << 6 : 0)
                | (statusRegisterWriteDisable ? 1 << 7 : 0));
        }

        private byte ComputeConfigurationRegister()
        {
            return (byte)(dummyCycleBits << 6
                | (fourByteMode ? 1 << 5 : 0)
                | (preambleBitEnable ? 1 << 4 : 0)
                | (topBottom ? 1 << 3 : 0));
        }

        private bool CommandAllowedWhileBusy(byte opcode)
        {
            // Datasheet: RDSR "can be read at any time"; RDP/RES are accepted
            // during a write cycle without affecting it (10-4).
            return opcode == (byte)Commands.ReadStatusRegister
                || opcode == (byte)Commands.ReleaseDeepPowerDown;
        }

        private bool ResolveArrayAddress(out ulong resolved)
        {
            var full = addressAccumulator;
            if(addressBytesNeeded == 3)
            {
                full |= (ulong)extendedAddress << 24; // EAR selects the 16 MiB bank
            }
            if(full >= (ulong)capacity)
            {
                if(WrapAddresses)
                {
                    resolved = full % (ulong)capacity;
                    this.Log(LogLevel.Warning, "Address 0x{0:X} exceeds capacity 0x{1:X}; wrapping to 0x{2:X}",
                        full, capacity, resolved);
                    return true;
                }
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=MX25U what=address-overflow detail=0x{0:X} exceeds capacity and WrapAddresses is false; access ignored", full);
                resolved = 0;
                return false;
            }
            resolved = full;
            return true;
        }

        private void SetBusy(TimeInterval from, ulong durationUs)
        {
            busyUntil = from + TimeInterval.FromMicroseconds(durationUs);
        }

        private bool IsBusy(TimeInterval now)
        {
            return FaultStuckBusy || now < busyUntil;
        }

        private TimeInterval EffectiveNow
        {
            get
            {
                var now = machine.LocalTimeSource.ElapsedVirtualTime;
                if(wipWaitBaseline != TimeInterval.Maximal && wipWaitBaseline > now)
                {
                    return wipWaitBaseline;
                }
                return now;
            }
        }

        private readonly IMachine machine;
        private readonly int capacity;
        private FileStream backing;
        private readonly byte manufacturerId;
        private readonly byte memoryType;
        private readonly byte memoryDensity;
        private readonly byte[] memory;
        private readonly byte[] pageBuffer = new byte[PageSize];
        private readonly bool[] pageBufferValid = new bool[PageSize];

        private Phase phase;
        private byte currentOpcode;
        private int bytesCollected;
        private int dataBytesSeen;
        private int addressBytesNeeded;
        private ulong addressAccumulator;
        private ulong readPointer;
        private ulong programAddress;
        private EraseKind pendingEraseKind;
        private byte pendingStatusByte;
        private byte pendingConfigurationByte;
        private bool hasPendingConfigurationByte;
        private bool remsDeviceIdFirst;

        // volatile device state
        private bool writeEnableLatch;
        private bool fourByteMode;
        private byte extendedAddress;
        private bool deepPowerDown;
        private bool resetEnabled;
        private TimeInterval busyUntil;
        private TimeInterval wipWaitBaseline;
        private TimeInterval standbyReadyAt;
        private TimeInterval dpdEnteredAt;

        // non-volatile status/configuration bits
        private byte blockProtectBits;
        private bool quadEnable;
        private bool statusRegisterWriteDisable;
        private byte dummyCycleBits;
        private bool preambleBitEnable;
        private bool topBottom;

        private const int PageSize = 256;
        private const ulong Sector4KSize = 0x1000;
        private const ulong Block32Size = 0x8000;
        private const ulong Block64Size = 0x10000;
        private const byte EmptyByte = 0xFF;

        private enum Phase
        {
            Idle,
            Ignore,
            ReadStatus,
            ReadConfiguration,
            ReadId,
            ReadElectronicId,
            RemsAddress,
            RemsData,
            ReadExtendedAddress,
            WriteExtendedAddress,
            WriteStatus,
            ReadAddress,
            ReadData,
            ProgramAddress,
            ProgramData,
            EraseAddress,
            ChipErasePending,
            DeepPowerDownPending,
        }

        private enum EraseKind
        {
            Sector4K,
            Block32K,
            Block64K,
            Chip,
        }

        private enum Commands : byte
        {
            Nop = 0x00,
            WriteStatusRegister = 0x01,
            PageProgram = 0x02,
            Read = 0x03,
            WriteDisable = 0x04,
            ReadStatusRegister = 0x05,
            WriteEnable = 0x06,
            FastRead = 0x0B,
            FastRead4B = 0x0C,
            PageProgram4B = 0x12,
            Read4B = 0x13,
            ReadConfigurationRegister = 0x15,
            SectorErase = 0x20,
            SectorErase4B = 0x21,
            QuadPageProgram = 0x38,
            QuadIoRead = 0xEB,
            QuadIoRead4B = 0xEC,
            DualOutputRead = 0x3B,
            DualOutputRead4B = 0x3C,
            QuadPageProgram4B = 0x3E,
            Block32Erase = 0x52,
            Block32Erase4B = 0x5C,
            ChipErase60 = 0x60,
            ResetEnable = 0x66,
            QuadOutputRead = 0x6B,
            QuadOutputRead4B = 0x6C,
            ReadElectronicManufacturerDeviceId = 0x90,
            ResetMemory = 0x99,
            ReadIdentification = 0x9F,
            ReleaseDeepPowerDown = 0xAB,
            Enter4ByteMode = 0xB7,
            DeepPowerDown = 0xB9,
            DualIoRead = 0xBB,
            DualIoRead4B = 0xBC,
            WriteExtendedAddressRegister = 0xC5,
            ChipEraseC7 = 0xC7,
            ReadExtendedAddressRegister = 0xC8,
            Block64Erase = 0xD8,
            Block64Erase4B = 0xDC,
            Exit4ByteMode = 0xE9,
        }
    }
}
