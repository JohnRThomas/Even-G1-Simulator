//
// NRF5340_Timer — nRF5340 TIMER peripheral.
//
// Fork of Renode's NRF52840_Timer (reference/renode-models/NRF52840_Timer.cs,
// Antmicro, MIT) extended for the nRF5340:
//   - DPPI endpoint registers, offsets verbatim from the pinned SVDs
//     (hal_nordic 9784731…), TIMER0 map:
//       SUBSCRIBE_START/STOP/COUNT/CLEAR/SHUTDOWN 0x080..0x090
//       SUBSCRIBE_CAPTURE[n]                      0x0C0
//       PUBLISH_COMPARE[n]                        0x1C0
//   - up to 8 CC channels (network-core TIMER0..2 have 8 per the network
//     SVD; application-core TIMER0..2 have 6)
//   - TASKS_SHUTDOWN implemented as stop (task is deprecated; stops the
//     timer without clearing the counter)
//
// Pass `dppic:` in the platform description to attach the instance to its
// DPPI domain hub (see models/DPPI_README.md).
//
//   - SHORTS (0x200): COMPARE[n]_CLEAR (bits 0-7) and COMPARE[n]_STOP (bits
//     8-15). MPSL depends on COMPARE0_CLEAR: it starts TIMER0 from an RTC
//     tick, lets it run to CC[0] and has the short clear it there, which
//     defines time zero of every radio event and timeslot. Without the short
//     all later timestamps are CC[0] too large, the SoftDevice Controller
//     finds its transmit deadlines already passed and asserts (module 45,
//     line 3429), which stops the net core.
//
// Not implemented (accesses log UNIMPL): ONESHOTEN[n] (0x580), INTEN (0x300).
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class NRF5340_Timer : BasicDoubleWordPeripheral, IKnownSize, INRFEventProvider
    {
        public NRF5340_Timer(IMachine machine, int numberOfEvents, NRF5340_DPPIC dppic = null) : base(machine)
        {
            IRQ = new GPIO();

            if(numberOfEvents > MaxNumberOfEvents)
            {
                throw new ConstructionException($"Cannot create {nameof(NRF5340_Timer)} with {numberOfEvents} events (must be at most {MaxNumberOfEvents})");
            }
            this.numberOfEvents = numberOfEvents;

            this.eventCompareEnabled = new IFlagRegisterField[numberOfEvents];
            publishCompare = new DPPIPublishEndpoint[numberOfEvents];
            subscribeCapture = new DPPISubscribeEndpoint[numberOfEvents];
            innerTimers = new ComparingTimer[numberOfEvents];
            for(var i = 0u; i < innerTimers.Length; i++)
            {
                var j = i;
                innerTimers[j] = new ComparingTimer(machine.ClockSource, InitialFrequency, this, $"compare{j}", eventEnabled: true);
                innerTimers[j].CompareReached += () =>
                {
                    this.Log(LogLevel.Noisy, "Compare Reached on CC{0} is {1}", j, innerTimers[j].Compare);
                    eventCompareEnabled[j].Value = true;
                    EventTriggered?.Invoke((uint)Register.Compare0EventPending + 0x4u * j);
                    publishCompare[j].Publish();
                    UpdateInterrupts();
                    RunShortcuts((int)j);
                };
                publishCompare[j] = new DPPIPublishEndpoint(dppic, this, $"COMPARE[{j}]");
                subscribeCapture[j] = new DPPISubscribeEndpoint(dppic, this, $"CAPTURE[{j}]", () => Capture((int)j));
            }

            subscribeStart = new DPPISubscribeEndpoint(dppic, this, "START", StartTimer);
            subscribeStop = new DPPISubscribeEndpoint(dppic, this, "STOP", StopTimer);
            subscribeCount = new DPPISubscribeEndpoint(dppic, this, "COUNT", CountTick);
            subscribeClear = new DPPISubscribeEndpoint(dppic, this, "CLEAR", ClearTimer);
            subscribeShutdown = new DPPISubscribeEndpoint(dppic, this, "SHUTDOWN", StopTimer);

            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();

            foreach(var timer in innerTimers)
            {
                timer.Reset();
            }
            for(var i = 0; i < numberOfEvents; i++)
            {
                publishCompare[i].Reset();
                subscribeCapture[i].Reset();
            }
            subscribeStart.Reset();
            subscribeStop.Reset();
            subscribeCount.Reset();
            subscribeClear.Reset();
            subscribeShutdown.Reset();

            IRQ.Unset();
        }

        public GPIO IRQ { get; }

        public long Size => 0x1000;

        public event Action<uint> EventTriggered;

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
            Register.Start.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        StartTimer();
                    }
                })
                .WithReservedBits(1, 31)
            ;

            Register.Stop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        StopTimer();
                    }
                })
                .WithReservedBits(1, 31)
            ;

            Register.Count.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASK_COUNT", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        CountTick();
                    }
                })
                .WithReservedBits(1, 31)
            ;

            Register.Clear.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASK_CLEAR", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        ClearTimer();
                    }
                })
                .WithReservedBits(1, 31)
            ;

            Register.Shutdown.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_SHUTDOWN", writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        // deprecated task: stops the timer, counter is not cleared
                        StopTimer();
                    }
                })
                .WithReservedBits(1, 31)
            ;

            Register.Capture0.DefineMany(this, (uint)numberOfEvents, setup: (register, idx) =>
            {
                register
                    .WithFlag(0, FieldMode.Write, name: "TASKS_CAPTURE", writeCallback: (_, __) =>
                    {
                        Capture(idx);
                    })
                    .WithReservedBits(1, 31);
            });

            Register.SubscribeStart.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_START",
                    valueProviderCallback: _ => subscribeStart.RegisterValue,
                    writeCallback: (_, value) => subscribeStart.RegisterValue = (uint)value);
            Register.SubscribeStop.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_STOP",
                    valueProviderCallback: _ => subscribeStop.RegisterValue,
                    writeCallback: (_, value) => subscribeStop.RegisterValue = (uint)value);
            Register.SubscribeCount.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_COUNT",
                    valueProviderCallback: _ => subscribeCount.RegisterValue,
                    writeCallback: (_, value) => subscribeCount.RegisterValue = (uint)value);
            Register.SubscribeClear.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_CLEAR",
                    valueProviderCallback: _ => subscribeClear.RegisterValue,
                    writeCallback: (_, value) => subscribeClear.RegisterValue = (uint)value);
            Register.SubscribeShutdown.Define(this)
                .WithValueField(0, 32, name: "SUBSCRIBE_SHUTDOWN",
                    valueProviderCallback: _ => subscribeShutdown.RegisterValue,
                    writeCallback: (_, value) => subscribeShutdown.RegisterValue = (uint)value);

            Register.SubscribeCapture0.DefineMany(this, (uint)numberOfEvents, setup: (register, idx) =>
            {
                register.WithValueField(0, 32, name: "SUBSCRIBE_CAPTURE",
                    valueProviderCallback: _ => subscribeCapture[idx].RegisterValue,
                    writeCallback: (_, value) => subscribeCapture[idx].RegisterValue = (uint)value);
            });

            Register.Compare0EventPending.DefineMany(this, (uint)numberOfEvents, setup: (register, idx) =>
            {
                register
                    .WithFlag(0, out eventCompareEnabled[idx], name: $"EVENTS_COMPARE[{idx}]", writeCallback: (_, __) =>
                    {
                        UpdateInterrupts();
                    })
                    .WithReservedBits(1, 31);
            });

            Register.Shortcuts.Define(this)
                .WithValueField(0, 8, out shortCompareClear, name: "COMPAREn_CLEAR")
                .WithValueField(8, 8, out shortCompareStop, name: "COMPAREn_STOP")
                .WithReservedBits(16, 16)
            ;

            Register.PublishCompare0.DefineMany(this, (uint)numberOfEvents, setup: (register, idx) =>
            {
                register.WithValueField(0, 32, name: "PUBLISH_COMPARE",
                    valueProviderCallback: _ => publishCompare[idx].RegisterValue,
                    writeCallback: (_, value) => publishCompare[idx].RegisterValue = (uint)value);
            });

            Register.InterruptEnableSet.Define(this)
                .WithReservedBits(0, 16)
                .WithFlags(16, numberOfEvents, out eventCompareInterruptEnabled, FieldMode.Set | FieldMode.Read, name: "COMPARE")
                .WithReservedBits(16 + numberOfEvents, 16 - numberOfEvents)
                .WithChangeCallback((_, __) =>
                {
                    UpdateInterrupts();
                })
            ;

            Register.InterruptEnableClear.Define(this)
                .WithReservedBits(0, 16)
                .WithFlags(16, numberOfEvents, name: "COMPARE", writeCallback: (i, _, value) => { if(value) eventCompareInterruptEnabled[i].Value = false; })
                .WithReservedBits(16 + numberOfEvents, 16 - numberOfEvents)
                .WithChangeCallback((_, __) =>
                {
                    UpdateInterrupts();
                })
            ;

            Register.Mode.Define(this)
                .WithEnumField(0, 2, out mode, name: "MODE", changeCallback: (_, value) =>
                {
                    if(value != Mode.Timer && innerTimers[0].Enabled)
                    {
                        this.Log(LogLevel.Error, "Switching timer to COUNTER mode while the timer is running");
                    }
                })
            ;

            Register.Prescaler.Define(this)
                .WithValueField(0, 4, out prescaler, name: "PRESCALER", writeCallback: (_, value) =>
                {
                    foreach(var timer in innerTimers)
                    {
                        timer.Divider = (uint)(1 << (int)value);
                    }
                })
                .WithReservedBits(12, 20)
            ;

            // BITMODE (0=16, 1=8, 2=24, 3=32 bit). Stored so the SDC/MPSL timeslot
            // setup readback verify passes (net TIMER0 is polled with BITMODE=32).
            Register.BitMode.Define(this)
                .WithValueField(0, 2, out bitMode, name: "BITMODE")
                .WithReservedBits(2, 30)
            ;

            Register.Compare0.DefineMany(this, (uint)numberOfEvents, setup: (register, idx) =>
            {
                register
                    .WithValueField(0, 32, name: "CAPTURE_COMPARE", writeCallback: (_, value) =>
                    {
                        SetCompare(idx, value);
                    },
                    valueProviderCallback: _ =>
                    {
                        return (uint)innerTimers[idx].Compare;
                    });
            });
        }

        private void StartTimer()
        {
            timerRunning = true;
            if(mode.Value == Mode.Timer)
            {
                foreach(var timer in innerTimers)
                {
                    timer.Enabled = true;
                }
            }
        }

        private void StopTimer()
        {
            timerRunning = false;
            foreach(var timer in innerTimers)
            {
                timer.Enabled = false;
            }
        }

        private void CountTick()
        {
            if(!timerRunning)
            {
                this.Log(LogLevel.Warning, "Triggered TASK_COUNT before issuing TASK_START, ignoring...");
                return;
            }
            if(mode.Value == Mode.Timer)
            {
                this.Log(LogLevel.Warning, "Triggered TASK_COUNT in TIMER mode, ignoring...");
                return;
            }
            var i = 0;
            var fired = 0;
            foreach(var timer in innerTimers)
            {
                timer.Value++;
                if(timer.Compare == timer.Value)
                {
                    eventCompareEnabled[i].Value = true;
                    EventTriggered?.Invoke((uint)Register.Compare0EventPending + 0x4u * (uint)i);
                    publishCompare[i].Publish();
                    UpdateInterrupts();
                    fired |= 1 << i;
                }
                i++;
            }
            for(i = 0; i < numberOfEvents; i++)
            {
                if((fired & (1 << i)) != 0)
                {
                    RunShortcuts(i);
                }
            }
        }

        // SHORTS: a COMPARE[idx] event clears and/or stops the timer.
        private void RunShortcuts(int idx)
        {
            if(((shortCompareClear.Value >> idx) & 1) != 0)
            {
                ClearTimer();
            }
            if(((shortCompareStop.Value >> idx) & 1) != 0)
            {
                StopTimer();
            }
        }

        private void ClearTimer()
        {
            foreach(var timer in innerTimers)
            {
                timer.Value = 0;
            }
        }

        private void Capture(int idx)
        {
            SetCompare(idx, innerTimers[idx].Value);
        }

        private void SetCompare(int idx, ulong value)
        {
            eventCompareEnabled[idx].Value = false;
            UpdateInterrupts();
            innerTimers[idx].Compare = value;
        }

        private void UpdateInterrupts()
        {
            var flag = false;

            for(var i = 0; i < numberOfEvents; i++)
            {
                flag |= eventCompareInterruptEnabled[i].Value && eventCompareEnabled[i].Value;
            }

            IRQ.Set(flag);
        }

        private IFlagRegisterField[] eventCompareInterruptEnabled;
        private IValueRegisterField prescaler;
        private IEnumRegisterField<Mode> mode;
        private IValueRegisterField bitMode;
        private IValueRegisterField shortCompareClear;
        private IValueRegisterField shortCompareStop;
        private bool timerRunning;

        private readonly IFlagRegisterField[] eventCompareEnabled;
        private readonly DPPIPublishEndpoint[] publishCompare;
        private readonly DPPISubscribeEndpoint[] subscribeCapture;
        private readonly DPPISubscribeEndpoint subscribeStart;
        private readonly DPPISubscribeEndpoint subscribeStop;
        private readonly DPPISubscribeEndpoint subscribeCount;
        private readonly DPPISubscribeEndpoint subscribeClear;
        private readonly DPPISubscribeEndpoint subscribeShutdown;

        private readonly ComparingTimer[] innerTimers;

        private readonly int numberOfEvents;
        private const int InitialFrequency = 16000000;
        private const int MaxNumberOfEvents = 8;

        private enum Mode
        {
            Timer,
            Counter,
            LowPowerCounter
        }

        private enum Register : long
        {
            Start = 0x000,
            Stop = 0x004,
            Count = 0x008,
            Clear = 0x00C,
            Shutdown = 0x010,

            Capture0 = 0x040,
            // ..Capture7 = 0x05C, stride 4

            SubscribeStart = 0x080,
            SubscribeStop = 0x084,
            SubscribeCount = 0x088,
            SubscribeClear = 0x08C,
            SubscribeShutdown = 0x090,

            SubscribeCapture0 = 0x0C0,
            // ..SubscribeCapture7 = 0x0DC, stride 4

            Compare0EventPending = 0x140,
            // ..Compare7EventPending = 0x15C, stride 4

            PublishCompare0 = 0x1C0,
            // ..PublishCompare7 = 0x1DC, stride 4

            Shortcuts = 0x200,

            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,

            Mode = 0x504,
            BitMode = 0x508,
            Prescaler = 0x510,

            Compare0 = 0x540,
            // ..Compare7 = 0x55C, stride 4
        }
    }
}
