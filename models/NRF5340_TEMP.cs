//
// NRF5340_TEMP — nRF5340 network-core die temperature sensor
// (0x41010000, IRQ 16).
//
// Register layout verbatim from svd/nrf5340_network.svd (hal_nordic 9784731…,
// see reference/nrf5340_regs.rs net::temp). Behaviour per the nRF5340 PS TEMP
// chapter (https://docs.nordicsemi.com/bundle/ps_nrf5340/page/temp.html):
// one-shot measurement started by TASKS_START; EVENTS_DATARDY after the
// conversion time; TEMP holds the result in 0.25 °C steps, two's complement.
//
// Deterministic conversion latency: 36 us of virtual time (PS: tTEMP ≈ 36 us),
// plus FaultConversionDelay microseconds when set. The measured value is the
// injectable Temperature property (°C, default 25.0). The A0..A5/B0..B5/T0..T4
// piecewise-linearisation coefficients are storage-only with their SVD reset
// values — the model reports Temperature directly (the linearisation applies
// to the internal ADC reading, which does not exist here).
//
// DPPI: SUBSCRIBE_START/STOP and PUBLISH_DATARDY wired to the network-core
// hub (models/DPPI_README.md).
//
// Fault injection (monitor-settable, default off):
//   FaultConversionDelay — extra conversion latency in microseconds
//   FaultDropInterrupt   — swallow the next rising IRQ edge
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_TEMP : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_TEMP(IMachine machine, NRF5340_DPPIC dppic = null) : base(machine)
        {
            IRQ = new GPIO();
            subscribeStart = new DPPISubscribeEndpoint(dppic, this, "START", TaskStart);
            subscribeStop = new DPPISubscribeEndpoint(dppic, this, "STOP", TaskStop);
            publishDataRdy = new DPPIPublishEndpoint(dppic, this, "DATARDY");

            conversionTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "tempConversion",
                limit: 1, direction: Direction.Ascending, enabled: false,
                workMode: WorkMode.OneShot, eventEnabled: true);
            conversionTimer.LimitReached += FinishConversion;

            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            dataReady = false;
            interruptEnabled = false;
            temperatureValue = 0;
            conversionTimer.Enabled = false;
            subscribeStart.Reset();
            subscribeStop.Reset();
            publishDataRdy.Reset();
            IRQ.Unset();
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; }

        // Injectable die temperature in °C (0.25 °C resolution is applied on read).
        public double Temperature { get; set; } = 25.0;
        // Extra conversion latency in microseconds.
        public uint FaultConversionDelay { get; set; }
        // Swallow the next rising IRQ edge.
        public bool FaultDropInterrupt { get; set; }

        public override uint ReadDoubleWord(long offset)
        {
            if(RegistersCollection.TryRead(offset, out var result))
            {
                return result;
            }
            NRF5340_DPPIC.LogUnimplementedAccess(this, machine, "R", offset, 0);
            return 0;
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            if(!RegistersCollection.TryWrite(offset, value))
            {
                NRF5340_DPPIC.LogUnimplementedAccess(this, machine, "W", offset, value);
            }
        }

        private void DefineRegisters()
        {
            Registers.TasksStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) => { if(value) TaskStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksStop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) => { if(value) TaskStop(); })
                .WithReservedBits(1, 31);

            Registers.SubscribeStart.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_START",
                    valueProviderCallback: _ => subscribeStart.RegisterValue,
                    writeCallback: (_, value) => subscribeStart.RegisterValue = (uint)value);

            Registers.SubscribeStop.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_STOP",
                    valueProviderCallback: _ => subscribeStop.RegisterValue,
                    writeCallback: (_, value) => subscribeStop.RegisterValue = (uint)value);

            Registers.EventsDataRdy.Define(this)
                .WithFlag(0, name: "EVENTS_DATARDY",
                    valueProviderCallback: _ => dataReady,
                    writeCallback: (_, value) => { dataReady = value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.PublishDataRdy.Define(this)
                .WithValueField(0, 32, name: "PUBLISH_DATARDY",
                    valueProviderCallback: _ => publishDataRdy.RegisterValue,
                    writeCallback: (_, value) => publishDataRdy.RegisterValue = (uint)value);

            Registers.IntenSet.Define(this)
                .WithFlag(0, name: "DATARDY",
                    valueProviderCallback: _ => interruptEnabled,
                    writeCallback: (_, value) => { interruptEnabled |= value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.IntenClr.Define(this)
                .WithFlag(0, name: "DATARDY",
                    valueProviderCallback: _ => interruptEnabled,
                    writeCallback: (_, value) => { interruptEnabled &= !value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.Temp.Define(this)
                .WithValueField(0, 32, FieldMode.Read, name: "TEMP", valueProviderCallback: _ => temperatureValue);

            // Linearisation coefficients: storage-only, SVD reset values
            DefineCoefficient(Registers.A0, "A0", 0x2D9, 12);
            DefineCoefficient(Registers.A1, "A1", 0x322, 12);
            DefineCoefficient(Registers.A2, "A2", 0x355, 12);
            DefineCoefficient(Registers.A3, "A3", 0x3DF, 12);
            DefineCoefficient(Registers.A4, "A4", 0x44E, 12);
            DefineCoefficient(Registers.A5, "A5", 0x4B7, 12);
            DefineCoefficient(Registers.B0, "B0", 0xFC7, 12);
            DefineCoefficient(Registers.B1, "B1", 0xF71, 12);
            DefineCoefficient(Registers.B2, "B2", 0xF6C, 12);
            DefineCoefficient(Registers.B3, "B3", 0xFCB, 12);
            DefineCoefficient(Registers.B4, "B4", 0x04B, 12);
            DefineCoefficient(Registers.B5, "B5", 0x0F6, 12);
            DefineCoefficient(Registers.T0, "T0", 0xE1, 8);
            DefineCoefficient(Registers.T1, "T1", 0xF9, 8);
            DefineCoefficient(Registers.T2, "T2", 0x10, 8);
            DefineCoefficient(Registers.T3, "T3", 0x26, 8);
            DefineCoefficient(Registers.T4, "T4", 0x3F, 8);
        }

        private void DefineCoefficient(Registers register, string name, uint resetValue, int width)
        {
            register.Define(this, resetValue)
                .WithValueField(0, width, name: name)
                .WithReservedBits(width, 32 - width);
        }

        private void TaskStart()
        {
            conversionTimer.Limit = (ulong)Math.Max(1, (ConversionTimeUs + FaultConversionDelay) * TicksPerMicrosecond);
            conversionTimer.Value = 0;
            conversionTimer.Enabled = true;
        }

        private void TaskStop()
        {
            conversionTimer.Enabled = false;
        }

        private void FinishConversion()
        {
            // 0.25 °C steps, two's complement, full 32-bit register width
            temperatureValue = unchecked((uint)(int)Math.Round(Temperature * 4));
            dataReady = true;
            publishDataRdy.Publish();
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var asserted = dataReady && interruptEnabled;
            if(asserted && !IRQ.IsSet && FaultDropInterrupt)
            {
                FaultDropInterrupt = false;
                this.Log(LogLevel.Warning, "FaultDropInterrupt active: swallowing IRQ assertion");
                return;
            }
            IRQ.Set(asserted);
        }

        private bool dataReady;
        private bool interruptEnabled;
        private uint temperatureValue;

        private readonly DPPISubscribeEndpoint subscribeStart;
        private readonly DPPISubscribeEndpoint subscribeStop;
        private readonly DPPIPublishEndpoint publishDataRdy;
        private readonly LimitTimer conversionTimer;

        private const uint ConversionTimeUs = 36; // PS: tTEMP ≈ 36 us
        private const long TimerFrequency = 16000000;
        private const uint TicksPerMicrosecond = 16;

        private enum Registers : long
        {
            TasksStart = 0x000,
            TasksStop = 0x004,
            SubscribeStart = 0x080,
            SubscribeStop = 0x084,
            EventsDataRdy = 0x100,
            PublishDataRdy = 0x180,
            IntenSet = 0x304,
            IntenClr = 0x308,
            Temp = 0x508,
            A0 = 0x520,
            A1 = 0x524,
            A2 = 0x528,
            A3 = 0x52C,
            A4 = 0x530,
            A5 = 0x534,
            B0 = 0x540,
            B1 = 0x544,
            B2 = 0x548,
            B3 = 0x54C,
            B4 = 0x550,
            B5 = 0x554,
            T0 = 0x560,
            T1 = 0x564,
            T2 = 0x568,
            T3 = 0x56C,
            T4 = 0x570,
        }
    }
}
