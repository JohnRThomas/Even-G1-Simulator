//
// NRF5340_RTC — nRF5340 real-time counter.
//
// Fork of Renode's NRF52840_RTC (reference/renode-models/NRF52840_RTC.cs,
// Antmicro, MIT) extended for the nRF5340, offsets verbatim from the pinned
// SVDs (hal_nordic 9784731…), RTC0 map (RTC1 derives from it in both SVDs):
//   TASKS_CAPTURE[n]      0x040  (new on nRF5340: copies COUNTER to CC[n])
//   SUBSCRIBE_START/STOP/CLEAR/TRIGOVRFLW 0x080..0x08C
//   SUBSCRIBE_CAPTURE[n]  0x0C0
//   PUBLISH_TICK          0x180
//   PUBLISH_OVRFLW        0x184
//   PUBLISH_COMPARE[n]    0x1C0
// plus TASKS_TRIGOVRFLW (0x00C, sets COUNTER to 0xFFFFF0 as on hardware).
//
// Pass `dppic:` in the platform description to attach the instance to its
// DPPI domain hub (see models/DPPI_README.md).
//
// Inherited model limitations: EVENTS_OVRFLW is not modelled (the counter
// never wraps an OVRFLW event), so PUBLISH_OVRFLW is stored but never
// fires; PUBLISH_TICK fires only while the TICK interrupt is unmasked
// (the base model only runs its tick timer in that case); PUBLISH_COMPARE
// fires when the compare event is routed per EVTEN, like the base model's
// EventTriggered.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class NRF5340_RTC : IDoubleWordPeripheral, IKnownSize, INRFEventProvider
    {
        public NRF5340_RTC(IMachine machine, int numberOfEvents, NRF5340_DPPIC dppic = null)
        {
            this.machine = machine;
            IRQ = new GPIO();

            if(numberOfEvents > MaxNumberOfEvents)
            {
                throw new ConstructionException($"Cannot create {nameof(NRF5340_RTC)} with {numberOfEvents} events (must be at most {MaxNumberOfEvents})");
            }
            this.numberOfEvents = numberOfEvents;
            compareEventEnabled = new IFlagRegisterField[numberOfEvents];
            compareReached = new IFlagRegisterField[numberOfEvents];
            compareInterruptEnabled = new IFlagRegisterField[numberOfEvents];
            publishCompare = new DPPIPublishEndpoint[numberOfEvents];
            subscribeCapture = new DPPISubscribeEndpoint[numberOfEvents];

            innerTimers = new ComparingTimer[numberOfEvents];
            for(var i = 0u; i < innerTimers.Length; i++)
            {
                var j = i;
                // counters are 24-bits
                innerTimers[i] = new ComparingTimer(machine.ClockSource, InitialFrequency, this, $"compare{i}", eventEnabled: true, limit: 0xFFFFFF, compare: 0xFFFFFF);
                innerTimers[i].CompareReached += () =>
                {
                    this.Log(LogLevel.Noisy, "IRQ #{0} triggered", j);
                    compareReached[j].Value = true;
                    if(compareEventEnabled[j].Value)
                    {
                        EventTriggered?.Invoke((uint)Register.Compare0EventPending + j * 4);
                        publishCompare[j].Publish();
                    }
                    UpdateInterrupts();
                };
                publishCompare[j] = new DPPIPublishEndpoint(dppic, this, $"COMPARE[{j}]");
                subscribeCapture[j] = new DPPISubscribeEndpoint(dppic, this, $"CAPTURE[{j}]", () => Capture((int)j));
            }

            tickTimer = new LimitTimer(machine.ClockSource, InitialFrequency, this, "tick", eventEnabled: true, limit: 0x1);
            tickTimer.LimitReached += () =>
            {
                tickEvent.Value = true;
                publishTick.Publish();
                UpdateInterrupts();
            };

            subscribeStart = new DPPISubscribeEndpoint(dppic, this, "START", () => UpdateTimersEnable(global: true));
            subscribeStop = new DPPISubscribeEndpoint(dppic, this, "STOP", () => UpdateTimersEnable(global: false));
            subscribeClear = new DPPISubscribeEndpoint(dppic, this, "CLEAR", ClearCounter);
            subscribeTrigOvrflw = new DPPISubscribeEndpoint(dppic, this, "TRIGOVRFLW", TriggerOverflow);
            publishTick = new DPPIPublishEndpoint(dppic, this, "TICK");
            publishOvrflw = new DPPIPublishEndpoint(dppic, this, "OVRFLW");

            DefineRegisters();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(registers.TryRead(offset, out var result))
            {
                return result;
            }
            NRF5340_DPPIC.LogUnimplementedAccess(this, machine, "R", offset, 0);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(!registers.TryWrite(offset, value))
            {
                NRF5340_DPPIC.LogUnimplementedAccess(this, machine, "W", offset, value);
            }
        }

        public void Reset()
        {
            registers.Reset();
            foreach(var timer in innerTimers)
            {
                timer.Reset();
            }
            tickTimer.Reset();
            for(var i = 0; i < numberOfEvents; i++)
            {
                publishCompare[i].Reset();
                subscribeCapture[i].Reset();
            }
            subscribeStart.Reset();
            subscribeStop.Reset();
            subscribeClear.Reset();
            subscribeTrigOvrflw.Reset();
            publishTick.Reset();
            publishOvrflw.Reset();
            IRQ.Unset();
        }

        public GPIO IRQ { get; }

        public long Size => 0x1000;

        public event Action<uint> EventTriggered;

        private void UpdateTimersEnable(bool? global = null, bool? tick = null)
        {
            if(global.HasValue)
            {
                foreach(var timer in innerTimers)
                {
                    timer.Enabled = global.Value;
                }
            }

            // due to optimization reasons we try to keep
            // the tick timer disabled as long as possible
            // - we enable it only when the global timer is
            // enabled and the tick event is unmasked
            if(tick.HasValue || (global.HasValue && !global.Value))
            {
                tickTimer.Enabled = innerTimers[0].Enabled && tick.Value;
            }
        }

        private void ClearCounter()
        {
            foreach(var timer in innerTimers)
            {
                timer.Value = 0;
            }
            tickTimer.Value = 0;
        }

        private void TriggerOverflow()
        {
            // hardware: TRIGOVRFLW sets COUNTER to 0xFFFFF0
            foreach(var timer in innerTimers)
            {
                timer.Value = 0xFFFFF0;
            }
        }

        private void Capture(int j)
        {
            // nRF5340 TASKS_CAPTURE[n]: copy COUNTER to CC[n]; follows the
            // CC-write path (clears the pending compare event, like a CC write)
            var counter = innerTimers[0].Value;
            compareReached[j].Value = false;
            UpdateInterrupts();
            innerTimers[j].Compare = counter;
        }

        private void DefineRegisters()
        {
            var registersMap = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Register.Start, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            UpdateTimersEnable(global: true);
                        }
                    })
                    .WithReservedBits(1, 31)
                },
                {(long)Register.Stop, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            UpdateTimersEnable(global: false);
                        }
                    })
                    .WithReservedBits(1, 31)
                },
                {(long)Register.Clear, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_CLEAR", writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            ClearCounter();
                        }
                    })
                    .WithReservedBits(1, 31)
                },
                {(long)Register.TriggerOverflow, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: "TASKS_TRIGOVRFLW", writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            TriggerOverflow();
                        }
                    })
                    .WithReservedBits(1, 31)
                },
                {(long)Register.SubscribeStart, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "SUBSCRIBE_START",
                        valueProviderCallback: _ => subscribeStart.RegisterValue,
                        writeCallback: (_, value) => subscribeStart.RegisterValue = (uint)value)
                },
                {(long)Register.SubscribeStop, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "SUBSCRIBE_STOP",
                        valueProviderCallback: _ => subscribeStop.RegisterValue,
                        writeCallback: (_, value) => subscribeStop.RegisterValue = (uint)value)
                },
                {(long)Register.SubscribeClear, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "SUBSCRIBE_CLEAR",
                        valueProviderCallback: _ => subscribeClear.RegisterValue,
                        writeCallback: (_, value) => subscribeClear.RegisterValue = (uint)value)
                },
                {(long)Register.SubscribeTrigOvrflw, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "SUBSCRIBE_TRIGOVRFLW",
                        valueProviderCallback: _ => subscribeTrigOvrflw.RegisterValue,
                        writeCallback: (_, value) => subscribeTrigOvrflw.RegisterValue = (uint)value)
                },
                {(long)Register.PublishTick, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "PUBLISH_TICK",
                        valueProviderCallback: _ => publishTick.RegisterValue,
                        writeCallback: (_, value) => publishTick.RegisterValue = (uint)value)
                },
                {(long)Register.PublishOvrflw, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "PUBLISH_OVRFLW",
                        valueProviderCallback: _ => publishOvrflw.RegisterValue,
                        writeCallback: (_, value) => publishOvrflw.RegisterValue = (uint)value)
                },
                {(long)Register.InterruptEnableSet, new DoubleWordRegister(this)
                    .WithFlag(0, out tickInterruptEnabled, FieldMode.Set | FieldMode.Read, name: "TICK")
                    .WithTaggedFlag("OVRFLW", 1)
                    .WithReservedBits(2, 14)
                    .WithFlags(16, numberOfEvents, out compareInterruptEnabled, FieldMode.Set | FieldMode.Read, name: "COMPARE")
                    .WithChangeCallback((_, __) =>
                    {
                        UpdateTimersEnable(tick: tickInterruptEnabled.Value);
                        UpdateInterrupts();
                    })
                },
                {(long)Register.InterruptEnableClear, new DoubleWordRegister(this)
                    .WithFlag(0, name: "TICK",
                          writeCallback: (_, value) => tickInterruptEnabled.Value &= !value,
                          valueProviderCallback: _ => tickInterruptEnabled.Value)
                    .WithTaggedFlag("OVRFLW", 1)
                    .WithReservedBits(2, 14)
                    .WithFlags(16, numberOfEvents, name: "COMPARE",
                          writeCallback: (j, _, value) => compareInterruptEnabled[j].Value &= !value,
                          valueProviderCallback: (j, value) => compareInterruptEnabled[j].Value)
                    //missing register fields defined below
                    .WithChangeCallback((_, __) =>
                    {
                        UpdateTimersEnable(tick: tickInterruptEnabled.Value);
                        UpdateInterrupts();
                    })
                },
                {(long)Register.Counter, new DoubleWordRegister(this)
                    .WithValueField(0, 24, FieldMode.Read, name: "COUNTER", valueProviderCallback: _ =>
                    {
                        // all timers have the same value, so let's just pick the first one
                        return (uint)innerTimers[0].Value;
                    })
                    .WithReservedBits(24, 8)
                },
                {(long)Register.Prescaler, new DoubleWordRegister(this)
                    .WithValueField(0, 12, out prescaler, name: "PRESCALER", writeCallback: (_, value) =>
                    {
                        foreach(var timer in innerTimers)
                        {
                            timer.Divider = value + 1;
                        }
                        tickTimer.Divider = value + 1;
                    })
                    .WithReservedBits(12, 20)
                },
                {(long)Register.EventEnable, new DoubleWordRegister(this)
                   .WithFlags(16, numberOfEvents, out compareEventEnabled, name: "COMPARE")
                },
                {(long)Register.EventSet, new DoubleWordRegister(this)
                   .WithTaggedFlag("TICK", 0)
                   .WithTaggedFlag("OVRFLW", 1)
                   .WithReservedBits(2, 14)
                   .WithFlags(16, numberOfEvents,
                         writeCallback: (i, _, val) => compareEventEnabled[i].Value |= val,
                         valueProviderCallback: (i, _) => compareEventEnabled[i].Value)
                },
                {(long)Register.EventClear, new DoubleWordRegister(this)
                   .WithFlags(16, numberOfEvents,
                         writeCallback: (i, _, val) => compareEventEnabled[i].Value &= !val,
                         valueProviderCallback: (i, _) => compareEventEnabled[i].Value)
                },
                {(long)Register.Tick, new DoubleWordRegister(this)
                   .WithFlag(0, out tickEvent, name: "EVENTS_TICK")
                   .WithReservedBits(1, 31)
                   .WithWriteCallback((_, __) => UpdateInterrupts())
                }
            };

            for(var i = 0; i < numberOfEvents; i++)
            {
                var j = i;
                registersMap.Add((long)Register.Capture0 + j * 4, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Write, name: $"TASKS_CAPTURE[{j}]", writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            Capture(j);
                        }
                    })
                    .WithReservedBits(1, 31)
                );

                registersMap.Add((long)Register.SubscribeCapture0 + j * 4, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"SUBSCRIBE_CAPTURE[{j}]",
                        valueProviderCallback: _ => subscribeCapture[j].RegisterValue,
                        writeCallback: (_, value) => subscribeCapture[j].RegisterValue = (uint)value)
                );

                registersMap.Add((long)Register.Compare0 + j * 4, new DoubleWordRegister(this)
                    .WithValueField(0, 24, name: $"COMPARE[{j}]", writeCallback: (_, value) =>
                    {
                        compareReached[j].Value = false;
                        UpdateInterrupts();
                        innerTimers[j].Compare = value;
                    },
                    valueProviderCallback: _ =>
                    {
                        return (uint)innerTimers[j].Compare;
                    })
                    .WithReservedBits(24, 8)
                );

                registersMap.Add((long)Register.Compare0EventPending + j * 4, new DoubleWordRegister(this)
                    .WithFlag(0, out compareReached[j], name: $"EVENTS_COMPARE[{j}]", writeCallback: (_, __) =>
                    {
                        UpdateInterrupts();
                    })
                    .WithReservedBits(1, 31)
                );

                registersMap.Add((long)Register.PublishCompare0 + j * 4, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"PUBLISH_COMPARE[{j}]",
                        valueProviderCallback: _ => publishCompare[j].RegisterValue,
                        writeCallback: (_, value) => publishCompare[j].RegisterValue = (uint)value)
                );
            }

            registers = new DoubleWordRegisterCollection(this, registersMap);
        }

        private void UpdateInterrupts()
        {
            var flag = false;

            for(var i = 0; i < numberOfEvents; i++)
            {
                var thisEventSet = compareInterruptEnabled[i].Value && compareReached[i].Value;
                if(thisEventSet)
                {
                    this.Log(LogLevel.Noisy, "Interrupt set by CC{0} interruptEnable={1} compareSet={2} compareEventEnable={3}",
                          i, compareInterruptEnabled[i].Value, compareReached[i].Value, compareEventEnabled[i].Value);
                }
                flag |= thisEventSet;
            }

            flag |= tickEvent.Value && tickInterruptEnabled.Value;
            IRQ.Set(flag);
        }

        private IFlagRegisterField[] compareInterruptEnabled;
        private IValueRegisterField prescaler;
        private IFlagRegisterField tickInterruptEnabled;
        private IFlagRegisterField tickEvent;

        private DoubleWordRegisterCollection registers;
        private IFlagRegisterField[] compareEventEnabled;
        private readonly IFlagRegisterField[] compareReached;
        private readonly DPPIPublishEndpoint[] publishCompare;
        private readonly DPPISubscribeEndpoint[] subscribeCapture;
        private readonly DPPISubscribeEndpoint subscribeStart;
        private readonly DPPISubscribeEndpoint subscribeStop;
        private readonly DPPISubscribeEndpoint subscribeClear;
        private readonly DPPISubscribeEndpoint subscribeTrigOvrflw;
        private readonly DPPIPublishEndpoint publishTick;
        private readonly DPPIPublishEndpoint publishOvrflw;

        private readonly LimitTimer tickTimer;
        private readonly ComparingTimer[] innerTimers;
        private readonly IMachine machine;

        private readonly int numberOfEvents;
        private const ulong InitialFrequency = 32768;
        private const int MaxNumberOfEvents = 4;

        private enum Register : long
        {
            Start = 0x000,
            Stop = 0x004,
            Clear = 0x008,
            TriggerOverflow = 0x00C,
            Capture0 = 0x040,
            // ..Capture3 = 0x04C, stride 4
            SubscribeStart = 0x080,
            SubscribeStop = 0x084,
            SubscribeClear = 0x088,
            SubscribeTrigOvrflw = 0x08C,
            SubscribeCapture0 = 0x0C0,
            // ..SubscribeCapture3 = 0x0CC, stride 4
            Tick = 0x100,
            Overflow = 0x104,
            Compare0EventPending = 0x140,
            // ..Compare3EventPending = 0x14C, stride 4
            PublishTick = 0x180,
            PublishOvrflw = 0x184,
            PublishCompare0 = 0x1C0,
            // ..PublishCompare3 = 0x1CC, stride 4
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            EventEnable = 0x340,
            EventSet = 0x344,
            EventClear = 0x348,
            Counter = 0x504,
            Prescaler = 0x508,
            Compare0 = 0x540,
            // ..Compare3 = 0x54C, stride 4
        }
    }
}
