//
// NRF5340_SPIM — nRF5340 SPI master with EasyDMA.
//
// Register offsets, field positions and reset values verbatim from the
// pinned nRF5340 SVD (hal_nordic 9784731461018d3e983604698fbbed6af2bea801,
// svd/nrf5340_application.svd). All SPIM instances in that SVD (SERIAL
// boxes and the dedicated SPIM4 block at 0x4000A000/0x5000A000) derive
// from the same SPIM0_NS register map — including the SPIM4-class extras
// STALLSTAT, PSEL.CSN, IFTIMING, CSNPOL, PSELDCX and DCXCNT — so one
// model class covers both instances used by this platform.
//
// Implemented behaviour (what the pinned NCS v2.5.1 drivers
// zephyr/drivers/spi/spi_nrfx_spim.c + nrfx_spim.c exercise):
//   - TASKS_START: full-duplex EasyDMA transfer against the attached
//     ISPIPeripheral, executed synchronously in virtual time (the model is
//     idealized: a transfer takes zero virtual time, matching the project's
//     instant-clock convention). max(TXD.MAXCNT, RXD.MAXCNT) bytes are
//     clocked; TX shorter than RX is padded with ORC; RX bytes beyond
//     RXD.MAXCNT are discarded. TXD/RXD.AMOUNT report exactly MAXCNT.
//   - EVENTS_STARTED/ENDTX/ENDRX/END/STOPPED as RW latches; INTENSET/
//     INTENCLR (SET/CLR read back the enable mask); level IRQ recomputed
//     on every event/enable change (so drivers that enable interrupts
//     after the synchronous transfer completes still get the IRQ).
//   - TASKS_STOP: raises EVENTS_STOPPED immediately (nrfx spim_abort
//     busy-waits on it). TASKS_SUSPEND/RESUME are no-ops for zero-time
//     transfers (logged at Debug).
//   - SHORTS.END_START: re-triggers the next transfer in the nearest
//     synced state (not synchronously, so firmware can run in between and
//     a forgotten STOP cannot hang the emulator inside one bus access).
//   - RXD/TXD.LIST = 1 (ArrayList): PTR is advanced by MAXCNT after the
//     transfer, per the EasyDMA array-list semantics.
//   - ENABLE: 0 = disabled, 7 = SPIM. Other values (this box muxed as
//     UARTE/TWIM/SPIS) log UNSUPPORTED and leave the model disabled.
//   - CONFIG.ORDER/CPHA/CPOL stored and honoured trivially: byte-level
//     exchange is mode-agnostic in emulation, so SPI mode 3 (CPOL=1,
//     CPHA=1) works; the configured mode is reported at Noisy on transfer.
//   - PSEL.SCK/MOSI/MISO/CSN, PSELDCX (reset 0xFFFFFFFF), FREQUENCY
//     (reset 0x04000000), IFTIMING.RXDELAY/CSNDUR (reset 2), CSNPOL,
//     DCXCNT, ORC, STALLSTAT: configuration/status storage only — no
//     electrical or timing effect in the emulation (documented deviation:
//     pins are not routed to GPIO, bitrate does not scale virtual time).
//   - SUBSCRIBE_*/PUBLISH_* (DPPI endpoints): stored; enabling one logs
//     UNSUPPORTED (TODO: wire to NRF5340_DPPIC when it lands).
//
// Anything not in the SVD map (e.g. there is *no* INTEN at 0x300 on the
// nRF5340 SPIM) logs UNIMPL with direction/width/value/PC, in the
// scripts/coverage_report.py format.
//
// Fault injection (monitor-settable, default off):
//   FaultSpiFailure — next TASKS_START "fails on the wire": STARTED is
//     raised, no data is moved, AMOUNTs read 0 and no ENDTX/ENDRX/END is
//     generated (drivers observe a transfer that never completes).
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class NRF5340_SPIM : NullRegistrationPointPeripheralContainer<ISPIPeripheral>,
        IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public NRF5340_SPIM(IMachine machine) : base(machine)
        {
            this.machine = machine;
            sysbus = machine.GetSystemBus(this);
            IRQ = new GPIO();
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            RegistersCollection.Reset();
            enabled = false;
            stopRequested = false;
            FaultSpiFailure = false;
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(RegistersCollection.TryRead(offset, out var value))
            {
                return value;
            }
            LogUnimplemented("R", offset, 0);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(!RegistersCollection.TryWrite(offset, value))
            {
                LogUnimplemented("W", offset, value);
            }
        }

        public GPIO IRQ { get; }
        public DoubleWordRegisterCollection RegistersCollection { get; }
        public long Size => 0x1000;

        // Fault injection: transfer error — no data moved, no END event.
        public bool FaultSpiFailure { get; set; }

        private void DefineRegisters()
        {
            Registers.TasksStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) =>
                {
                    if(!value)
                    {
                        return;
                    }
                    if(!enabled)
                    {
                        this.Log(LogLevel.Warning, "TASKS_START while not enabled as SPIM, ignoring");
                        return;
                    }
                    stopRequested = false;
                    StartTransfer();
                })
                .WithReservedBits(1, 31);

            Registers.TasksStop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) =>
                {
                    if(!value)
                    {
                        return;
                    }
                    if(!enabled)
                    {
                        this.Log(LogLevel.Debug, "TASKS_STOP while not enabled, ignoring");
                        return;
                    }
                    stopRequested = true;
                    stoppedPending.Value = true;
                    UpdateInterrupts();
                })
                .WithReservedBits(1, 31);

            Registers.TasksSuspend.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_SUSPEND", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        // Transfers are instantaneous in this model - nothing to suspend.
                        this.Log(LogLevel.Debug, "TASKS_SUSPEND: transfers are instantaneous, no-op");
                    }
                })
                .WithReservedBits(1, 31);

            Registers.TasksResume.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_RESUME", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        this.Log(LogLevel.Debug, "TASKS_RESUME: transfers are instantaneous, no-op");
                    }
                })
                .WithReservedBits(1, 31);

            DefineDppiEndpoint(Registers.SubscribeStart, "SUBSCRIBE_START");
            DefineDppiEndpoint(Registers.SubscribeStop, "SUBSCRIBE_STOP");
            DefineDppiEndpoint(Registers.SubscribeSuspend, "SUBSCRIBE_SUSPEND");
            DefineDppiEndpoint(Registers.SubscribeResume, "SUBSCRIBE_RESUME");
            DefineDppiEndpoint(Registers.PublishStopped, "PUBLISH_STOPPED");
            DefineDppiEndpoint(Registers.PublishEndRx, "PUBLISH_ENDRX");
            DefineDppiEndpoint(Registers.PublishEnd, "PUBLISH_END");
            DefineDppiEndpoint(Registers.PublishEndTx, "PUBLISH_ENDTX");
            DefineDppiEndpoint(Registers.PublishStarted, "PUBLISH_STARTED");

            Registers.EventsStopped.Define(this)
                .WithFlag(0, out stoppedPending, name: "EVENTS_STOPPED")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.EventsEndRx.Define(this)
                .WithFlag(0, out endRxPending, name: "EVENTS_ENDRX")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.EventsEnd.Define(this)
                .WithFlag(0, out endPending, name: "EVENTS_END")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.EventsEndTx.Define(this)
                .WithFlag(0, out endTxPending, name: "EVENTS_ENDTX")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.EventsStarted.Define(this)
                .WithFlag(0, out startedPending, name: "EVENTS_STARTED")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.Shorts.Define(this)
                .WithReservedBits(0, 17)
                .WithFlag(17, out shortEndStart, name: "END_START")
                .WithReservedBits(18, 14);

            // INTENSET/INTENCLR both read back the current enable mask (SVD).
            Registers.IntenSet.Define(this)
                .WithReservedBits(0, 1)
                .WithFlag(1, out stoppedEnabled, FieldMode.Read | FieldMode.Set, name: "STOPPED")
                .WithReservedBits(2, 2)
                .WithFlag(4, out endRxEnabled, FieldMode.Read | FieldMode.Set, name: "ENDRX")
                .WithReservedBits(5, 1)
                .WithFlag(6, out endEnabled, FieldMode.Read | FieldMode.Set, name: "END")
                .WithReservedBits(7, 1)
                .WithFlag(8, out endTxEnabled, FieldMode.Read | FieldMode.Set, name: "ENDTX")
                .WithReservedBits(9, 10)
                .WithFlag(19, out startedEnabled, FieldMode.Read | FieldMode.Set, name: "STARTED")
                .WithReservedBits(20, 12)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.IntenClr.Define(this)
                .WithReservedBits(0, 1)
                .WithFlag(1, name: "STOPPED",
                    valueProviderCallback: _ => stoppedEnabled.Value,
                    writeCallback: (_, value) => { if(value) stoppedEnabled.Value = false; })
                .WithReservedBits(2, 2)
                .WithFlag(4, name: "ENDRX",
                    valueProviderCallback: _ => endRxEnabled.Value,
                    writeCallback: (_, value) => { if(value) endRxEnabled.Value = false; })
                .WithReservedBits(5, 1)
                .WithFlag(6, name: "END",
                    valueProviderCallback: _ => endEnabled.Value,
                    writeCallback: (_, value) => { if(value) endEnabled.Value = false; })
                .WithReservedBits(7, 1)
                .WithFlag(8, name: "ENDTX",
                    valueProviderCallback: _ => endTxEnabled.Value,
                    writeCallback: (_, value) => { if(value) endTxEnabled.Value = false; })
                .WithReservedBits(9, 10)
                .WithFlag(19, name: "STARTED",
                    valueProviderCallback: _ => startedEnabled.Value,
                    writeCallback: (_, value) => { if(value) startedEnabled.Value = false; })
                .WithReservedBits(20, 12)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            // Never set by this model (no stalls with instantaneous DMA); storage only.
            Registers.StallStat.Define(this)
                .WithFlag(0, name: "TX")
                .WithFlag(1, name: "RX")
                .WithReservedBits(2, 30);

            Registers.Enable.Define(this)
                .WithValueField(0, 4, name: "ENABLE",
                    valueProviderCallback: _ => enabled ? EnableValueSpim : 0u,
                    writeCallback: (_, value) =>
                    {
                        switch(value)
                        {
                        case 0:
                            enabled = false;
                            break;
                        case EnableValueSpim:
                            enabled = true;
                            break;
                        default:
                            // This serial box muxed to another function (UARTE/TWIM/SPIS)
                            // is not modelled by this class.
                            enabled = false;
                            this.Log(LogLevel.Warning,
                                "UNSUPPORTED periph={0} what=enable-mode detail=ENABLE=0x{1:X} is not SPIM(7); box stays disabled",
                                Label, value);
                            break;
                        }
                        UpdateInterrupts();
                    })
                .WithReservedBits(4, 28);

            Registers.PselSck.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, out pselSck, name: "PSEL_SCK");
            Registers.PselMosi.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, out pselMosi, name: "PSEL_MOSI");
            Registers.PselMiso.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, out pselMiso, name: "PSEL_MISO");
            Registers.PselCsn.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, out pselCsn, name: "PSEL_CSN");

            Registers.Frequency.Define(this, 0x04000000)
                .WithValueField(0, 32, out frequency, name: "FREQUENCY",
                    writeCallback: (_, value) =>
                    {
                        if(!KnownFrequencies.Contains((uint)value))
                        {
                            this.Log(LogLevel.Warning,
                                "UNSUPPORTED periph={0} what=frequency detail=non-enumerated FREQUENCY value 0x{1:X8}",
                                Label, value);
                        }
                    });

            Registers.RxdPtr.Define(this)
                .WithValueField(0, 32, out rxdPtr, name: "RXD_PTR");
            Registers.RxdMaxCnt.Define(this)
                .WithValueField(0, 16, out rxdMaxCnt, name: "RXD_MAXCNT")
                .WithReservedBits(16, 16);
            Registers.RxdAmount.Define(this)
                .WithValueField(0, 16, out rxdAmount, FieldMode.Read, name: "RXD_AMOUNT")
                .WithReservedBits(16, 16);
            Registers.RxdList.Define(this)
                .WithValueField(0, 2, out rxdList, name: "RXD_LIST")
                .WithReservedBits(2, 30);

            Registers.TxdPtr.Define(this)
                .WithValueField(0, 32, out txdPtr, name: "TXD_PTR");
            Registers.TxdMaxCnt.Define(this)
                .WithValueField(0, 16, out txdMaxCnt, name: "TXD_MAXCNT")
                .WithReservedBits(16, 16);
            Registers.TxdAmount.Define(this)
                .WithValueField(0, 16, out txdAmount, FieldMode.Read, name: "TXD_AMOUNT")
                .WithReservedBits(16, 16);
            Registers.TxdList.Define(this)
                .WithValueField(0, 2, out txdList, name: "TXD_LIST")
                .WithReservedBits(2, 30);

            Registers.Config.Define(this)
                .WithFlag(0, out configOrder, name: "ORDER")
                .WithFlag(1, out configCpha, name: "CPHA")
                .WithFlag(2, out configCpol, name: "CPOL")
                .WithReservedBits(3, 29);

            Registers.IfTimingRxDelay.Define(this, 0x00000002)
                .WithValueField(0, 3, name: "RXDELAY")
                .WithReservedBits(3, 29);
            Registers.IfTimingCsnDur.Define(this, 0x00000002)
                .WithValueField(0, 8, name: "CSNDUR")
                .WithReservedBits(8, 24);
            Registers.CsnPol.Define(this)
                .WithFlag(0, name: "CSNPOL")
                .WithReservedBits(1, 31);
            Registers.PselDcx.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, name: "PSELDCX");
            Registers.DcxCnt.Define(this)
                .WithValueField(0, 4, name: "DCXCNT")
                .WithReservedBits(4, 28);

            Registers.Orc.Define(this)
                .WithValueField(0, 8, out orc, name: "ORC")
                .WithReservedBits(8, 24);
        }

        private void DefineDppiEndpoint(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 8, name: "CHIDX")
                .WithReservedBits(8, 23)
                .WithFlag(31, name: "EN", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        // TODO: wire to the shared DPPI hub once NRF5340_DPPIC lands.
                        this.Log(LogLevel.Warning,
                            "UNSUPPORTED periph={0} what=dppi detail={1} enabled but DPPI is not modelled",
                            Label, name);
                    }
                });
        }

        private void StartTransfer()
        {
            startedPending.Value = true;

            if(FaultSpiFailure)
            {
                this.Log(LogLevel.Warning,
                    "FaultSpiFailure active: transfer started but fails on the wire - no data moved, no END event");
                txdAmount.Value = 0;
                rxdAmount.Value = 0;
                UpdateInterrupts();
                return;
            }

            var txCount = (int)txdMaxCnt.Value;
            var rxCount = (int)rxdMaxCnt.Value;
            var total = Math.Max(txCount, rxCount);

            this.Log(LogLevel.Noisy,
                "SPI transfer: tx={0}B@0x{1:X8} rx={2}B@0x{3:X8} mode={4} order={5} freq=0x{6:X8}",
                txCount, txdPtr.Value, rxCount, rxdPtr.Value,
                (configCpol.Value ? 2 : 0) | (configCpha.Value ? 1 : 0),
                configOrder.Value ? "LSB" : "MSB", frequency.Value);

            var rxBuffer = new byte[rxCount];
            if(RegisteredPeripheral == null)
            {
                this.Log(LogLevel.Warning, "SPI transfer with no attached device - RX filled with 0x00");
            }
            else
            {
                var txBuffer = txCount > 0 ? sysbus.ReadBytes(txdPtr.Value, txCount) : new byte[0];
                for(var i = 0; i < total; i++)
                {
                    var outByte = i < txCount ? txBuffer[i] : (byte)orc.Value;
                    var inByte = RegisteredPeripheral.Transmit(outByte);
                    if(i < rxCount)
                    {
                        rxBuffer[i] = inByte;
                    }
                }
                RegisteredPeripheral.FinishTransmission();
                // --- ADDITIVE OPT-IN TRANSACTION TRACE (default off) --------
                // Pure observation of buffers the model already built; no
                // behavioural or timing effect. See TraceFile below.
                if(traceWriter != null)
                {
                    traceWriter.WriteLine("{0} seq={1} tick={2} txn={3} rxn={4} tx={5} rx={6}",
                        Label, traceSeq++, machine.ElapsedVirtualTime.TimeElapsed.Ticks,
                        txCount, rxCount, TraceHex(txBuffer, txCount), TraceHex(rxBuffer, rxCount));
                }
            }
            if(rxCount > 0)
            {
                sysbus.WriteBytes(rxBuffer, rxdPtr.Value);
            }

            txdAmount.Value = (ulong)txCount;
            rxdAmount.Value = (ulong)rxCount;

            // EasyDMA array list: advance pointers by MAXCNT after the transfer.
            if(txdList.Value == 1)
            {
                txdPtr.Value = (uint)(txdPtr.Value + txdMaxCnt.Value);
            }
            if(rxdList.Value == 1)
            {
                rxdPtr.Value = (uint)(rxdPtr.Value + rxdMaxCnt.Value);
            }

            endTxPending.Value = true;
            endRxPending.Value = true;
            endPending.Value = true;
            UpdateInterrupts();

            if(shortEndStart.Value)
            {
                // Re-trigger in the nearest synced state so firmware can run
                // between back-to-back transfers (and STOP can break the chain).
                machine.LocalTimeSource.ExecuteInNearestSyncedState(_ =>
                {
                    if(enabled && shortEndStart.Value && !stopRequested)
                    {
                        StartTransfer();
                    }
                });
            }
        }

        private void UpdateInterrupts()
        {
            var state = false;
            state |= stoppedPending.Value && stoppedEnabled.Value;
            state |= endRxPending.Value && endRxEnabled.Value;
            state |= endPending.Value && endEnabled.Value;
            state |= endTxPending.Value && endTxEnabled.Value;
            state |= startedPending.Value && startedEnabled.Value;
            IRQ.Set(state);
        }

        private void LogUnimplemented(string dir, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph={0} dir={1} width=4 offset=0x{2:X} value=0x{3:X8} pc=0x{4:X8}",
                Label, dir, offset, value, pc);
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
                    return "SPIM";
                }
            }
        }

        // ===== ADDITIVE OPT-IN TRANSACTION TRACE (G1 parity oracle) =========
        // Disabled unless TraceFile is assigned from the monitor. Setting it
        // opens an append-free text log of completed SPI transactions; setting
        // it to "" (or calling TraceClose) flushes and closes. No register,
        // event, timing or data-path behaviour is affected in either state.
        public string TraceFile
        {
            set
            {
                TraceClose();
                if(!string.IsNullOrEmpty(value))
                {
                    traceWriter = new System.IO.StreamWriter(value, false);
                    traceWriter.AutoFlush = true;
                    traceSeq = 0;
                }
            }
        }

        public void TraceClose()
        {
            if(traceWriter != null)
            {
                traceWriter.Flush();
                traceWriter.Dispose();
                traceWriter = null;
            }
        }

        private static string TraceHex(byte[] buffer, int count)
        {
            if(buffer == null || count <= 0)
            {
                return "-";
            }
            var sb = new System.Text.StringBuilder(count * 2);
            for(var i = 0; i < count && i < buffer.Length; i++)
            {
                sb.Append(buffer[i].ToString("X2"));
            }
            return sb.ToString();
        }

        private System.IO.StreamWriter traceWriter;
        private ulong traceSeq;
        // ===== end additive trace ==========================================

        private bool enabled;
        private bool stopRequested;

        private IFlagRegisterField stoppedPending;
        private IFlagRegisterField endRxPending;
        private IFlagRegisterField endPending;
        private IFlagRegisterField endTxPending;
        private IFlagRegisterField startedPending;
        private IFlagRegisterField stoppedEnabled;
        private IFlagRegisterField endRxEnabled;
        private IFlagRegisterField endEnabled;
        private IFlagRegisterField endTxEnabled;
        private IFlagRegisterField startedEnabled;
        private IFlagRegisterField shortEndStart;
        private IFlagRegisterField configOrder;
        private IFlagRegisterField configCpha;
        private IFlagRegisterField configCpol;
        private IValueRegisterField pselSck;
        private IValueRegisterField pselMosi;
        private IValueRegisterField pselMiso;
        private IValueRegisterField pselCsn;
        private IValueRegisterField frequency;
        private IValueRegisterField rxdPtr;
        private IValueRegisterField rxdMaxCnt;
        private IValueRegisterField rxdAmount;
        private IValueRegisterField rxdList;
        private IValueRegisterField txdPtr;
        private IValueRegisterField txdMaxCnt;
        private IValueRegisterField txdAmount;
        private IValueRegisterField txdList;
        private IValueRegisterField orc;

        private readonly IMachine machine;
        private readonly IBusController sysbus;

        private const uint EnableValueSpim = 7;

        private static readonly HashSet<uint> KnownFrequencies = new HashSet<uint>
        {
            0x02000000, // K125
            0x04000000, // K250
            0x08000000, // K500
            0x10000000, // M1
            0x20000000, // M2
            0x40000000, // M4
            0x80000000, // M8
            0x0A000000, // M16 (SPIM4-class)
            0x14000000, // M32 (SPIM4-class)
        };

        private enum Registers : long
        {
            TasksStart = 0x010,
            TasksStop = 0x014,
            TasksSuspend = 0x01C,
            TasksResume = 0x020,
            SubscribeStart = 0x090,
            SubscribeStop = 0x094,
            SubscribeSuspend = 0x09C,
            SubscribeResume = 0x0A0,
            EventsStopped = 0x104,
            EventsEndRx = 0x110,
            EventsEnd = 0x118,
            EventsEndTx = 0x120,
            EventsStarted = 0x14C,
            PublishStopped = 0x184,
            PublishEndRx = 0x190,
            PublishEnd = 0x198,
            PublishEndTx = 0x1A0,
            PublishStarted = 0x1CC,
            Shorts = 0x200,
            IntenSet = 0x304,
            IntenClr = 0x308,
            StallStat = 0x400,
            Enable = 0x500,
            PselSck = 0x508,
            PselMosi = 0x50C,
            PselMiso = 0x510,
            PselCsn = 0x514,
            Frequency = 0x524,
            RxdPtr = 0x534,
            RxdMaxCnt = 0x538,
            RxdAmount = 0x53C,
            RxdList = 0x540,
            TxdPtr = 0x544,
            TxdMaxCnt = 0x548,
            TxdAmount = 0x54C,
            TxdList = 0x550,
            Config = 0x554,
            IfTimingRxDelay = 0x560,
            IfTimingCsnDur = 0x564,
            CsnPol = 0x568,
            PselDcx = 0x56C,
            DcxCnt = 0x570,
            Orc = 0x5C0,
        }
    }
}
