//
// NRF5340_NVMC — nRF5340 Non-Volatile Memory Controller + an internal-flash
// region that implements real NOR erase/program semantics.
//
// Why this exists: Zephyr's settings subsystem is backed by NVS on the
// internal flash `settings_storage` partition (flash area id 9 ->
// 0x000FC000, size 0x2000, sector 0x1000). NVS assumes flash reads erased
// (unwritten) cells as 0xFF, that a page erase clears a whole 0x1000 page
// to 0xFF, and that a program only clears bits (AND). A plain
// Memory.MappedMemory reads back 0x00 after reset and never erases, so
// nvs_mount / settings_subsys_init fails (the firmware's sett_init() prints
// "-45" and bt_settings reports settings_subsys_init failed).
//
// On the nRF53/nRF91 the flash-write primitives (see hal_nordic
// nrfx/hal/nrf_nvmc.h) are:
//   * word program  : NVMC->CONFIG = Wen(1); *(u32*)addr = value;      (AND)
//   * page erase     : NVMC->CONFIG = Een(2); *(u32*)pageAddr = 0xFFFFFFFF;
// i.e. the actual program/erase are ordinary stores to the mapped flash;
// the CONFIG register only selects the mode. This file therefore splits the
// behaviour into two cooperating peripherals:
//   * NRF5340_NVMC  — the register block at 0x40039000/0x50039000. Tracks
//     CONFIG.WEN, always reports READY, and handles ERASEALL.
//   * NRF5340_InternalFlash — a byte-backed region mapped over the storage
//     partition. Stores initialise to 0xFF (erased). A store while the
//     controller is in Erase mode erases the whole containing page to 0xFF;
//     any other store programs with AND semantics (1 -> 0 only). Reads
//     return the stored bytes. Contents survive a soft Reset (flash is
//     non-volatile); they are seeded in the ctor to 0xFF.
//
// The region is registered only over the small settings partition, so the
// executable code flash (0x0..0xFC000) stays a fast MappedMemory.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.MTD
{
    // CONFIG.WEN encodings (nrf_nvmc_mode_t): 0 read-only, 1 write, 2 erase,
    // 4 partial-erase. Only bits [2:0] are meaningful.
    public enum NrfNvmcMode
    {
        ReadOnly = 0,
        Write = 1,
        Erase = 2,
        PartialErase = 4,
    }

    public class NRF5340_NVMC : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public NRF5340_NVMC(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            Mode = NrfNvmcMode.ReadOnly;
        }

        public long Size => 0x1000;

        // Read by the InternalFlash regions to decide erase vs. program.
        public NrfNvmcMode Mode { get; private set; }

        // InternalFlash regions register themselves here so ERASEALL can wipe
        // them (the firmware does not use it at boot, but model it faithfully).
        public void RegisterRegion(NRF5340_InternalFlash region)
        {
            regions.Add(region);
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case Ready:
                case ReadyNext:
                    return 1u; // erase/write engine always idle in the model
                case Config:
                case ConfigNs:
                    return (uint)Mode;
                default:
                    return 0u;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case Config:
                case ConfigNs:
                    Mode = (NrfNvmcMode)(value & 0x7);
                    break;
                case EraseAll:
                    if((value & 1) != 0)
                    {
                        foreach(var region in regions)
                        {
                            region.EraseWhole();
                        }
                        this.Log(LogLevel.Info, "ERASEALL: wiped {0} managed flash region(s)", regions.Count);
                    }
                    break;
                default:
                    // Other NVMC registers (ERASEPAGEPARTIALCFG, WRITEUICRNS,
                    // ICACHECNF, ...) are inert in this model.
                    break;
            }
        }

        public ushort ReadWord(long offset) => (ushort)ReadDoubleWord(offset);
        public byte ReadByte(long offset) => (byte)ReadDoubleWord(offset);
        public void WriteWord(long offset, ushort value) => WriteDoubleWord(offset, value);
        public void WriteByte(long offset, byte value) => WriteDoubleWord(offset, value);

        private readonly IMachine machine;
        private readonly List<NRF5340_InternalFlash> regions = new List<NRF5340_InternalFlash>();

        // Register offsets (svd/nrf5340_application.svd, NVMC).
        private const long Ready = 0x400;
        private const long ReadyNext = 0x408;
        private const long Config = 0x504;
        private const long EraseAll = 0x50C;
        private const long ConfigNs = 0x584;
    }

    public class NRF5340_InternalFlash : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public NRF5340_InternalFlash(IMachine machine, NRF5340_NVMC nvmc, long size, int pageSize = 0x1000)
        {
            if(size <= 0 || (size % pageSize) != 0)
            {
                throw new Exceptions.ConstructionException("size must be a positive multiple of pageSize");
            }
            this.machine = machine;
            this.nvmc = nvmc;
            this.size = size;
            this.pageSize = pageSize;
            data = new byte[size];
            // Flash is delivered erased; seeding (LoadBinary) happens after
            // construction and programs with AND against this 0xFF baseline.
            for(var i = 0L; i < size; i++)
            {
                data[i] = ErasedByte;
            }
            nvmc?.RegisterRegion(this);
        }

        // Flash is non-volatile: a soft Reset must not wipe the array. Erased
        // state is established once, in the constructor.
        public void Reset()
        {
        }

        public long Size => size;

        public byte ReadByte(long offset) => data[offset];

        public ushort ReadWord(long offset)
        {
            return (ushort)(data[offset] | (data[offset + 1] << 8));
        }

        public uint ReadDoubleWord(long offset)
        {
            return (uint)(data[offset]
                | (data[offset + 1] << 8)
                | (data[offset + 2] << 16)
                | (data[offset + 3] << 24));
        }

        public void WriteByte(long offset, byte value)
        {
            if(IsErasing())
            {
                ErasePage(offset);
                return;
            }
            Program(offset, value);
        }

        public void WriteWord(long offset, ushort value)
        {
            if(IsErasing())
            {
                ErasePage(offset);
                return;
            }
            Program(offset, (byte)value);
            Program(offset + 1, (byte)(value >> 8));
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(IsErasing())
            {
                // nRF53 page erase: a store to any address in the page while
                // CONFIG.WEN == Erase clears the whole page to 0xFF.
                ErasePage(offset);
                return;
            }
            Program(offset, (byte)value);
            Program(offset + 1, (byte)(value >> 8));
            Program(offset + 2, (byte)(value >> 16));
            Program(offset + 3, (byte)(value >> 24));
        }

        // Used by NRF5340_NVMC.ERASEALL.
        public void EraseWhole()
        {
            for(var i = 0L; i < size; i++)
            {
                data[i] = ErasedByte;
            }
        }

        private bool IsErasing()
        {
            return nvmc != null && nvmc.Mode == NrfNvmcMode.Erase;
        }

        private void ErasePage(long offset)
        {
            var pageBase = offset & ~((long)pageSize - 1);
            for(var i = 0; i < pageSize; i++)
            {
                data[pageBase + i] = ErasedByte;
            }
        }

        private void Program(long offset, byte value)
        {
            // NOR program can only clear bits (1 -> 0). Programming an already
            // 0xFF (erased) cell yields exactly `value`, so seeding via
            // LoadBinary against the erased baseline stores bytes verbatim.
            data[offset] &= value;
        }

        private readonly IMachine machine;
        private readonly NRF5340_NVMC nvmc;
        private readonly long size;
        private readonly int pageSize;
        private readonly byte[] data;

        private const byte ErasedByte = 0xFF;
    }
}
