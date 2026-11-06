//
// NRF5340_TWIM — nRF5340 I2C master with EasyDMA (serial-box TWIM function).
//
// Register offsets verbatim from the pinned nRF5340 MDK/SVD
// (hal_nordic 9784731461018d3e983604698fbbed6af2bea801),
// svd/nrf5340_application.svd peripheral TWIM0_NS (see also
// reference/nrf5340_regs.rs `mod twim`).
//
// Behaviour is modelled against the pinned drivers that must run on top:
//   third_party/hal_nordic/nrfx/drivers/src/nrfx_twim.c
//   third_party/zephyr/drivers/i2c/i2c_nrfx_twim.c
// In particular:
//   - TXRX: SHORTS.LASTTX_STARTRX | SHORTS.LASTRX_STOP, TASKS_RESUME,
//     TASKS_STARTTX  (nrfx_twim.c:482-489)
//   - TX no-stop:   SHORTS.LASTTX_SUSPEND, wait EVENTS_SUSPENDED
//     (nrfx_twim.c:490-503)
//   - TX with stop: SHORTS.LASTTX_STOP    (nrfx_twim.c:499)
//   - RX:           SHORTS.LASTRX_STOP    (nrfx_twim.c:504-509)
//   - error path: on EVENTS_ERROR without STOPPED the driver triggers
//     TASKS_RESUME + TASKS_STOP and waits for EVENTS_STOPPED
//     (nrfx_twim.c:669-691), then reads+clears ERRORSRC (write-1-clear).
//   - completeness check compares TXD.AMOUNT/RXD.AMOUNT with the requested
//     lengths (nrfx_twim.c:134-187) — AMOUNT reports bytes actually
//     transferred; 0 after an address NACK.
//
// Transfers execute instantaneously in virtual time at task-trigger time
// (Renode convention for EasyDMA peripherals). TASKS_STARTTX/STARTRX
// triggered while suspended are latched and executed on TASKS_RESUME,
// which makes both driver orderings (RESUME;STARTTX and STARTTX;RESUME)
// behave like hardware.
//
// DPPI (SUBSCRIBE_*/PUBLISH_*) endpoints are stored but not wired to a
// DPPIC hub yet.
// TODO: route SUBSCRIBE_*/PUBLISH_* through NRF5340_DPPIC once it lands;
// until then non-zero writes log UNSUPPORTED.
//
// Fault injection:
//   FaultNak  — bool; while true every transaction gets an address NACK
//               (ERRORSRC.ANACK + EVENTS_ERROR), as if no device answered.
// Devices attached to this bus may implement II2CDeviceFaultNak to request
// the same per-device (e.g. ST_LSM6DSO.FaultNak).
//
using System;
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.I2C
{
    // Implemented by I2C device models that can be told to NACK their address
    // (fault injection compatible with the TWIM ANACK path).
    public interface II2CDeviceFaultNak
    {
        bool FaultNak { get; }
    }

    public class NRF5340_TWIM : SimpleContainer<II2CPeripheral>, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_TWIM(IMachine machine, string label = "TWIM") : base(machine)
        {
            this.label = label;
            IRQ = new GPIO();
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
        }

        public override void Reset()
        {
            RegistersCollection.Reset();
            enabled = false;
            suspended = false;
            pendingStartTx = false;
            pendingStartRx = false;
            transactionDevice = null;
            transactionOpen = false;
            txdAmount = 0;
            rxdAmount = 0;
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

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public GPIO IRQ { get; }

        public long Size => 0x1000;

        // Fault injection: force an address NACK on every transaction while set.
        public bool FaultNak { get; set; }

        private void DefineRegisters()
        {
            Registers.TasksStartRx.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STARTRX", writeCallback: (_, value) => { if(value) TriggerStartRx(); })
                .WithReservedBits(1, 31);

            Registers.TasksStartTx.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STARTTX", writeCallback: (_, value) => { if(value) TriggerStartTx(); })
                .WithReservedBits(1, 31);

            Registers.TasksStop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) => { if(value) DoStop(); })
                .WithReservedBits(1, 31);

            Registers.TasksSuspend.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_SUSPEND", writeCallback: (_, value) =>
                {
                    if(!value)
                    {
                        return;
                    }
                    suspended = true;
                    eventSuspended.Value = true;
                    UpdateInterrupts();
                })
                .WithReservedBits(1, 31);

            Registers.TasksResume.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_RESUME", writeCallback: (_, value) => { if(value) DoResume(); })
                .WithReservedBits(1, 31);

            DefineDppiStub(Registers.SubscribeStartRx, "SUBSCRIBE_STARTRX");
            DefineDppiStub(Registers.SubscribeStartTx, "SUBSCRIBE_STARTTX");
            DefineDppiStub(Registers.SubscribeStop, "SUBSCRIBE_STOP");
            DefineDppiStub(Registers.SubscribeSuspend, "SUBSCRIBE_SUSPEND");
            DefineDppiStub(Registers.SubscribeResume, "SUBSCRIBE_RESUME");

            DefineEvent(Registers.EventsStopped, "EVENTS_STOPPED", out eventStopped);
            DefineEvent(Registers.EventsError, "EVENTS_ERROR", out eventError);
            DefineEvent(Registers.EventsSuspended, "EVENTS_SUSPENDED", out eventSuspended);
            DefineEvent(Registers.EventsRxStarted, "EVENTS_RXSTARTED", out eventRxStarted);
            DefineEvent(Registers.EventsTxStarted, "EVENTS_TXSTARTED", out eventTxStarted);
            DefineEvent(Registers.EventsLastRx, "EVENTS_LASTRX", out eventLastRx);
            DefineEvent(Registers.EventsLastTx, "EVENTS_LASTTX", out eventLastTx);

            DefineDppiStub(Registers.PublishStopped, "PUBLISH_STOPPED");
            DefineDppiStub(Registers.PublishError, "PUBLISH_ERROR");
            DefineDppiStub(Registers.PublishSuspended, "PUBLISH_SUSPENDED");
            DefineDppiStub(Registers.PublishRxStarted, "PUBLISH_RXSTARTED");
            DefineDppiStub(Registers.PublishTxStarted, "PUBLISH_TXSTARTED");
            DefineDppiStub(Registers.PublishLastRx, "PUBLISH_LASTRX");
            DefineDppiStub(Registers.PublishLastTx, "PUBLISH_LASTTX");

            Registers.Shorts.Define(this)
                .WithReservedBits(0, 7)
                .WithFlag(7, out shortLastTxStartRx, name: "LASTTX_STARTRX")
                .WithFlag(8, out shortLastTxSuspend, name: "LASTTX_SUSPEND")
                .WithFlag(9, out shortLastTxStop, name: "LASTTX_STOP")
                .WithFlag(10, out shortLastRxStartTx, name: "LASTRX_STARTTX")
                .WithReservedBits(11, 1)
                .WithFlag(12, out shortLastRxStop, name: "LASTRX_STOP")
                .WithReservedBits(13, 19);

            Registers.InterruptEnable.Define(this)
                .WithValueField(0, 32, name: "INTEN",
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable = (uint)value & InterruptMask; UpdateInterrupts(); });

            Registers.InterruptEnableSet.Define(this)
                .WithValueField(0, 32, name: "INTENSET",
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable |= (uint)value & InterruptMask; UpdateInterrupts(); });

            Registers.InterruptEnableClear.Define(this)
                .WithValueField(0, 32, name: "INTENCLR",
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable &= ~((uint)value & InterruptMask); UpdateInterrupts(); });

            Registers.ErrorSource.Define(this)
                .WithFlag(0, out errorOverrun, FieldMode.Read | FieldMode.WriteOneToClear, name: "OVERRUN")
                .WithFlag(1, out errorAddressNack, FieldMode.Read | FieldMode.WriteOneToClear, name: "ANACK")
                .WithFlag(2, out errorDataNack, FieldMode.Read | FieldMode.WriteOneToClear, name: "DNACK")
                .WithReservedBits(3, 29);

            Registers.Enable.Define(this)
                .WithValueField(0, 4, name: "ENABLE",
                    valueProviderCallback: _ => enabled ? EnableValueTwim : 0u,
                    writeCallback: (_, value) =>
                    {
                        switch(value)
                        {
                        case 0:
                            enabled = false;
                            // Disabling resets the internal transfer state machine
                            // (nrfx re-enables after a premature stop to reinitialize it).
                            suspended = false;
                            pendingStartTx = false;
                            pendingStartRx = false;
                            transactionOpen = false;
                            transactionDevice = null;
                            break;
                        case EnableValueTwim:
                            enabled = true;
                            break;
                        default:
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=ENABLE detail=value 0x{1:X} selects a non-TWIM serial-box function; ignoring", label, value);
                            break;
                        }
                    })
                .WithReservedBits(4, 28);

            Registers.PselScl.Define(this, PselResetValue)
                .WithValueField(0, 32, name: "PSEL_SCL");

            Registers.PselSda.Define(this, PselResetValue)
                .WithValueField(0, 32, name: "PSEL_SDA");

            Registers.Frequency.Define(this, FrequencyResetValue)
                .WithValueField(0, 32, out frequency, name: "FREQUENCY");

            Registers.RxdPtr.Define(this)
                .WithValueField(0, 32, out rxdPointer, name: "RXD_PTR");

            Registers.RxdMaxCnt.Define(this)
                .WithValueField(0, 16, out rxdMaximumCount, name: "RXD_MAXCNT")
                .WithReservedBits(16, 16);

            Registers.RxdAmount.Define(this)
                .WithValueField(0, 16, FieldMode.Read, name: "RXD_AMOUNT", valueProviderCallback: _ => rxdAmount)
                .WithReservedBits(16, 16);

            Registers.RxdList.Define(this)
                .WithValueField(0, 3, out rxdList, name: "RXD_LIST")
                .WithReservedBits(3, 29);

            Registers.TxdPtr.Define(this)
                .WithValueField(0, 32, out txdPointer, name: "TXD_PTR");

            Registers.TxdMaxCnt.Define(this)
                .WithValueField(0, 16, out txdMaximumCount, name: "TXD_MAXCNT")
                .WithReservedBits(16, 16);

            Registers.TxdAmount.Define(this)
                .WithValueField(0, 16, FieldMode.Read, name: "TXD_AMOUNT", valueProviderCallback: _ => txdAmount)
                .WithReservedBits(16, 16);

            Registers.TxdList.Define(this)
                .WithValueField(0, 3, out txdList, name: "TXD_LIST")
                .WithReservedBits(3, 29);

            Registers.Address.Define(this)
                .WithValueField(0, 7, out address, name: "ADDRESS")
                .WithReservedBits(7, 25);
        }

        private void DefineEvent(Registers register, string name, out IFlagRegisterField field)
        {
            register.Define(this)
                .WithFlag(0, out field, name: name)
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());
        }

        private void DefineDppiStub(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 32, name: name,
                    valueProviderCallback: _ => dppiStubs.TryGetValue((long)register, out var stored) ? stored : 0,
                    writeCallback: (_, value) =>
                    {
                        dppiStubs[(long)register] = (uint)value;
                        if(value != 0)
                        {
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what={1} detail=DPPI endpoints are not connected to a DPPIC hub; stored 0x{2:X8}", label, name, value);
                        }
                    });
        }

        private void TriggerStartTx()
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "TASKS_STARTTX triggered but the peripheral is not enabled as TWIM");
                return;
            }
            if(suspended)
            {
                pendingStartTx = true;
                return;
            }
            DoStartTx();
        }

        private void TriggerStartRx()
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "TASKS_STARTRX triggered but the peripheral is not enabled as TWIM");
                return;
            }
            if(suspended)
            {
                pendingStartRx = true;
                return;
            }
            DoStartRx();
        }

        private void DoResume()
        {
            suspended = false;
            if(pendingStartTx)
            {
                pendingStartTx = false;
                DoStartTx();
            }
            else if(pendingStartRx)
            {
                pendingStartRx = false;
                DoStartRx();
            }
        }

        private void DoStartTx()
        {
            eventTxStarted.Value = true;
            txdAmount = 0;
            UpdateInterrupts();

            if(!TrySelectDevice())
            {
                return;
            }

            var count = (int)txdMaximumCount.Value;
            if(count > 0)
            {
                var data = machine.GetSystemBus(this).ReadBytes(txdPointer.Value, count);
                this.Log(LogLevel.Noisy, "TX {0} byte(s) to device 0x{1:X2} from 0x{2:X8}", count, address.Value, txdPointer.Value);
                TraceWrite("W", (int)address.Value, data, count);
                transactionDevice.Write(data);
                txdAmount = (uint)count;
                if(txdList.Value == ListArrayList)
                {
                    txdPointer.Value += (uint)count;
                }
                eventLastTx.Value = true;
                UpdateInterrupts();
                if(shortLastTxStartRx.Value)
                {
                    DoStartRx();
                }
                else if(shortLastTxSuspend.Value)
                {
                    suspended = true;
                    eventSuspended.Value = true;
                    UpdateInterrupts();
                }
                else if(shortLastTxStop.Value)
                {
                    DoStop();
                }
            }
            // A zero-length transfer performs only the (acknowledged) address
            // phase; no LASTTX is generated and the driver triggers TASKS_STOP
            // itself (nrfx_twim.c:525-532).
        }

        private void DoStartRx()
        {
            eventRxStarted.Value = true;
            rxdAmount = 0;
            UpdateInterrupts();

            if(!TrySelectDevice())
            {
                return;
            }

            var count = (int)rxdMaximumCount.Value;
            if(count > 0)
            {
                var data = transactionDevice.Read(count) ?? new byte[0];
                if(data.Length != count)
                {
                    var padded = new byte[count];
                    Array.Copy(data, padded, Math.Min(data.Length, count));
                    for(var i = data.Length; i < count; i++)
                    {
                        padded[i] = 0xFF; // released SDA reads as ones
                    }
                    data = padded;
                }
                machine.GetSystemBus(this).WriteBytes(data, rxdPointer.Value);
                this.Log(LogLevel.Noisy, "RX {0} byte(s) from device 0x{1:X2} into 0x{2:X8}", count, address.Value, rxdPointer.Value);
                TraceWrite("R", (int)address.Value, data, count);
                rxdAmount = (uint)count;
                if(rxdList.Value == ListArrayList)
                {
                    rxdPointer.Value += (uint)count;
                }
                eventLastRx.Value = true;
                UpdateInterrupts();
                if(shortLastRxStartTx.Value)
                {
                    DoStartTx();
                }
                else if(shortLastRxStop.Value)
                {
                    DoStop();
                }
            }
        }

        private bool TrySelectDevice()
        {
            if(FaultNak)
            {
                this.Log(LogLevel.Warning, "FaultNak active: injecting address NACK for address 0x{0:X2}", address.Value);
                SignalAddressNack();
                return false;
            }
            if(transactionOpen && transactionDevice != null)
            {
                // repeated start: keep talking to the already-addressed device
                // unless firmware changed ADDRESS in between
                if(TryGetByAddress((int)address.Value, out var current) && current == transactionDevice)
                {
                    return true;
                }
            }
            if(!TryGetByAddress((int)address.Value, out var device))
            {
                this.Log(LogLevel.Warning, "No device at address 0x{0:X2} — address NACK", address.Value);
                SignalAddressNack();
                return false;
            }
            if(device is II2CDeviceFaultNak nakDevice && nakDevice.FaultNak)
            {
                this.Log(LogLevel.Warning, "Device at address 0x{0:X2} has FaultNak set — address NACK", address.Value);
                SignalAddressNack();
                return false;
            }
            transactionDevice = device;
            transactionOpen = true;
            return true;
        }

        private void SignalAddressNack()
        {
            errorAddressNack.Value = true;
            eventError.Value = true;
            UpdateInterrupts();
        }

        private void DoStop()
        {
            suspended = false;
            pendingStartTx = false;
            pendingStartRx = false;
            if(transactionOpen)
            {
                transactionDevice?.FinishTransmission();
            }
            transactionOpen = false;
            transactionDevice = null;
            eventStopped.Value = true;
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var pending = 0u;
            if(eventStopped.Value)
            {
                pending |= 1u << 1;
            }
            if(eventError.Value)
            {
                pending |= 1u << 9;
            }
            if(eventSuspended.Value)
            {
                pending |= 1u << 18;
            }
            if(eventRxStarted.Value)
            {
                pending |= 1u << 19;
            }
            if(eventTxStarted.Value)
            {
                pending |= 1u << 20;
            }
            if(eventLastRx.Value)
            {
                pending |= 1u << 23;
            }
            if(eventLastTx.Value)
            {
                pending |= 1u << 24;
            }
            IRQ.Set((pending & interruptEnable) != 0);
        }

        private void LogUnimplemented(string direction, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph={0} dir={1} width=4 offset=0x{2:X} value=0x{3:X8} pc=0x{4:X8}",
                label, direction, offset, value, pc);
        }

        private IFlagRegisterField eventStopped;
        private IFlagRegisterField eventError;
        private IFlagRegisterField eventSuspended;
        private IFlagRegisterField eventRxStarted;
        private IFlagRegisterField eventTxStarted;
        private IFlagRegisterField eventLastRx;
        private IFlagRegisterField eventLastTx;

        private IFlagRegisterField shortLastTxStartRx;
        private IFlagRegisterField shortLastTxSuspend;
        private IFlagRegisterField shortLastTxStop;
        private IFlagRegisterField shortLastRxStartTx;
        private IFlagRegisterField shortLastRxStop;

        private IFlagRegisterField errorOverrun;
        private IFlagRegisterField errorAddressNack;
        private IFlagRegisterField errorDataNack;

        private IValueRegisterField frequency;
        private IValueRegisterField address;
        private IValueRegisterField rxdPointer;
        private IValueRegisterField rxdMaximumCount;
        private IValueRegisterField rxdList;
        private IValueRegisterField txdPointer;
        private IValueRegisterField txdMaximumCount;
        private IValueRegisterField txdList;

        private uint interruptEnable;
        private uint txdAmount;
        private uint rxdAmount;
        private bool enabled;
        private bool suspended;
        private bool pendingStartTx;
        private bool pendingStartRx;
        private bool transactionOpen;
        private II2CPeripheral transactionDevice;

        // ===== ADDITIVE OPT-IN TRANSACTION TRACE (G1 parity oracle) =========
        // Disabled unless TraceFile is assigned from the monitor. Setting it
        // opens a text log of completed I2C transfers (direction, 7-bit device
        // address, payload); setting it to "" (or TraceClose) flushes+closes.
        // Pure observation of buffers the model already built — no register,
        // event, timing or data-path behaviour changes in either state.
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

        private void TraceWrite(string direction, int deviceAddress, byte[] data, int count)
        {
            if(traceWriter == null)
            {
                return;
            }
            var sb = new System.Text.StringBuilder(count * 2);
            for(var i = 0; i < count && i < data.Length; i++)
            {
                sb.Append(data[i].ToString("X2"));
            }
            traceWriter.WriteLine("{0} seq={1} tick={2} dev=0x{3:X2} dir={4} n={5} data={6}",
                label, traceSeq++, machine.ElapsedVirtualTime.TimeElapsed.Ticks,
                deviceAddress, direction, count, sb.ToString());
        }

        private System.IO.StreamWriter traceWriter;
        private ulong traceSeq;
        // ===== end additive trace ==========================================

        private readonly string label;
        private readonly Dictionary<long, uint> dppiStubs = new Dictionary<long, uint>();

        private const uint EnableValueTwim = 6;
        private const uint PselResetValue = 0xFFFFFFFF;
        private const uint FrequencyResetValue = 0x04000000; // 250 kbps
        private const uint ListArrayList = 1;
        // STOPPED, ERROR, SUSPENDED, RXSTARTED, TXSTARTED, LASTRX, LASTTX
        private const uint InterruptMask = (1u << 1) | (1u << 9) | (1u << 18) | (1u << 19) | (1u << 20) | (1u << 23) | (1u << 24);

        private enum Registers : long
        {
            TasksStartRx = 0x000,
            TasksStartTx = 0x008,
            TasksStop = 0x014,
            TasksSuspend = 0x01C,
            TasksResume = 0x020,
            SubscribeStartRx = 0x080,
            SubscribeStartTx = 0x088,
            SubscribeStop = 0x094,
            SubscribeSuspend = 0x09C,
            SubscribeResume = 0x0A0,
            EventsStopped = 0x104,
            EventsError = 0x124,
            EventsSuspended = 0x148,
            EventsRxStarted = 0x14C,
            EventsTxStarted = 0x150,
            EventsLastRx = 0x15C,
            EventsLastTx = 0x160,
            PublishStopped = 0x184,
            PublishError = 0x1A4,
            PublishSuspended = 0x1C8,
            PublishRxStarted = 0x1CC,
            PublishTxStarted = 0x1D0,
            PublishLastRx = 0x1DC,
            PublishLastTx = 0x1E0,
            Shorts = 0x200,
            InterruptEnable = 0x300,
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            ErrorSource = 0x4C4,
            Enable = 0x500,
            PselScl = 0x508,
            PselSda = 0x50C,
            Frequency = 0x524,
            RxdPtr = 0x534,
            RxdMaxCnt = 0x538,
            RxdAmount = 0x53C,
            RxdList = 0x540,
            TxdPtr = 0x544,
            TxdMaxCnt = 0x548,
            TxdAmount = 0x54C,
            TxdList = 0x550,
            Address = 0x588,
        }
    }
}
