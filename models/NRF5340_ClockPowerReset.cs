//
// NRF5340_ClockPowerReset — combined CLOCK + POWER + RESET page of the
// nRF5340 (application: 0x50005000/0x40005000, network: 0x41005000).
//
// Register offsets verbatim from the pinned nRF5340 MDK/SVD
// (hal_nordic 9784731461018d3e983604698fbbed6af2bea801).
//
// Clock requests complete instantly: TASKS_*START immediately raises the
// corresponding EVENTS_*STARTED and updates *STAT/RUN, which satisfies the
// nrfx_clock / Zephyr clock_control drivers.
//
// The application-core variant (hasNetworkForceOff: true) implements
// RESET.NETWORK.FORCEOFF (0x614): clearing it releases the network core
// (un-halts the given CPU), setting it halts the core again.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_ClockPowerReset : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_ClockPowerReset(IMachine machine, ICPU networkCore = null, bool hasNetworkForceOff = false) : base(machine)
        {
            this.networkCore = networkCore;
            this.hasNetworkForceOff = hasNetworkForceOff;
            hfclkCtrl = HfclkCtrlReset;
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            hfclkStarted = false;
            lfclkStarted = false;
            hfclk192mStarted = false;
            hfclkAudioStarted = false;
            calDone = false;
            networkForceOff = true;
            hfclkRunning = false;
            lfclkRunning = false;
            hfclkAudioRunning = false;
            hfclk192mRunning = false;
            interruptEnable = 0;
            // SVD reset values (nRF5340 CLOCK):
            hfclkSource = 1;          // HFCLKSRC = HFXO
            lfclkSource = 1;          // LFCLKSRC = LFXO (SVD reset, both cores)
            hfclkCtrl = HfclkCtrlReset;
            hfclkAudioFrequency = 0x9BAE;
            hfclkAlwaysRun = 0;
            lfclkAlwaysRun = 0;
            hfclkAudioAlwaysRun = 0;
            hfclk192mSource = 1;      // HFCLK192MSRC = HFXO
            hfclk192mAlwaysRun = 0;
            hfclk192mCtrl = 2;        // HFCLK192MCTRL = DIV4
            resetReason = 0;
            gpRegRet0 = 0;
            gpRegRet1 = 0;
            UpdateInterrupt();
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; } = new GPIO();

        private void DefineRegisters()
        {
            Registers.TasksHfclkStart.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(!value) return;
                    hfclkRunning = true;
                    hfclkStarted = true;
                    UpdateInterrupt();
                })
                .WithReservedBits(1, 31);

            Registers.TasksHfclkStop.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) => { if(value) hfclkRunning = false; })
                .WithReservedBits(1, 31);

            Registers.TasksLfclkStart.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(!value) return;
                    lfclkRunning = true;
                    lfclkStarted = true;
                    UpdateInterrupt();
                })
                .WithReservedBits(1, 31);

            Registers.TasksLfclkStop.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) => { if(value) lfclkRunning = false; })
                .WithReservedBits(1, 31);

            Registers.TasksCal.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(!value) return;
                    calDone = true;
                    UpdateInterrupt();
                })
                .WithReservedBits(1, 31);

            Registers.TasksHfclkAudioStart.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(!value) return;
                    hfclkAudioRunning = true;
                    hfclkAudioStarted = true;
                    UpdateInterrupt();
                })
                .WithReservedBits(1, 31);

            Registers.TasksHfclkAudioStop.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) => { if(value) hfclkAudioRunning = false; })
                .WithReservedBits(1, 31);

            Registers.TasksHfclk192MStart.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(!value) return;
                    hfclk192mRunning = true;
                    hfclk192mStarted = true;
                    UpdateInterrupt();
                })
                .WithReservedBits(1, 31);

            Registers.TasksHfclk192MStop.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, value) => { if(value) hfclk192mRunning = false; })
                .WithReservedBits(1, 31);

            Registers.TasksConstLat.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, __) => { /* latency model: no-op */ })
                .WithReservedBits(1, 31);

            Registers.TasksLowPwr.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, __) => { /* latency model: no-op */ })
                .WithReservedBits(1, 31);

            Registers.EventsHfclkStarted.Define(this)
                .WithFlag(0,
                    valueProviderCallback: _ => hfclkStarted,
                    writeCallback: (_, value) => { hfclkStarted = value; UpdateInterrupt(); })
                .WithReservedBits(1, 31);

            Registers.EventsLfclkStarted.Define(this)
                .WithFlag(0,
                    valueProviderCallback: _ => lfclkStarted,
                    writeCallback: (_, value) => { lfclkStarted = value; UpdateInterrupt(); })
                .WithReservedBits(1, 31);

            Registers.EventsDone.Define(this)
                .WithFlag(0,
                    valueProviderCallback: _ => calDone,
                    writeCallback: (_, value) => { calDone = value; UpdateInterrupt(); })
                .WithReservedBits(1, 31);

            Registers.EventsHfclkAudioStarted.Define(this)
                .WithFlag(0,
                    valueProviderCallback: _ => hfclkAudioStarted,
                    writeCallback: (_, value) => { hfclkAudioStarted = value; UpdateInterrupt(); })
                .WithReservedBits(1, 31);

            Registers.EventsHfclk192MStarted.Define(this)
                .WithFlag(0,
                    valueProviderCallback: _ => hfclk192mStarted,
                    writeCallback: (_, value) => { hfclk192mStarted = value; UpdateInterrupt(); })
                .WithReservedBits(1, 31);

            // POWER events (POFWARN / SLEEPENTER / SLEEPEXIT): modelled as
            // never-firing, but the registers exist so drivers can clear them.
            Registers.EventsPofWarn.Define(this)
                .WithFlag(0, valueProviderCallback: _ => false, writeCallback: (_, __) => { })
                .WithReservedBits(1, 31);
            Registers.EventsSleepEnter.Define(this)
                .WithFlag(0, valueProviderCallback: _ => false, writeCallback: (_, __) => { })
                .WithReservedBits(1, 31);
            Registers.EventsSleepExit.Define(this)
                .WithFlag(0, valueProviderCallback: _ => false, writeCallback: (_, __) => { })
                .WithReservedBits(1, 31);

            // INTEN / INTENSET / INTENCLR — shared by CLOCK and POWER views.
            // CLOCK bits: HFCLKSTARTED=0, LFCLKSTARTED=1, DONE=7,
            //             HFCLKAUDIOSTARTED=8, HFCLK192MSTARTED=9.
            // POWER bits: POFWARN=2, SLEEPENTER=5, SLEEPEXIT=6.
            Registers.InterruptEnable.Define(this)
                .WithValueField(0, 32,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable = (uint)value; UpdateInterrupt(); });

            Registers.InterruptEnableSet.Define(this)
                .WithValueField(0, 32,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable |= (uint)value; UpdateInterrupt(); });

            Registers.InterruptEnableClear.Define(this)
                .WithValueField(0, 32,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable &= ~(uint)value; UpdateInterrupt(); });

            Registers.InterruptPending.Define(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => PendingInterrupts());

            Registers.HfclkRun.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hfclkRunning)
                .WithReservedBits(1, 31);

            Registers.HfclkStat.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hfclkRunning) // SRC: HFXO when started
                .WithReservedBits(1, 3)
                .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => false)        // ALWAYSRUNNING
                .WithReservedBits(5, 11)
                .WithFlag(16, FieldMode.Read, valueProviderCallback: _ => hfclkRunning) // STATE
                .WithReservedBits(17, 15);

            Registers.LfclkRun.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => lfclkRunning)
                .WithReservedBits(1, 31);

            Registers.LfclkStat.Define(this)
                // SRC reflects the source of the running LFCLK; 0 until started (SVD reset 0)
                .WithValueField(0, 2, FieldMode.Read, valueProviderCallback: _ => lfclkRunning ? lfclkSource : 0)
                .WithReservedBits(2, 2)
                .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => false)
                .WithReservedBits(5, 11)
                .WithFlag(16, FieldMode.Read, valueProviderCallback: _ => lfclkRunning)
                .WithReservedBits(17, 15);

            Registers.LfclkSrcCopy.Define(this)
                .WithValueField(0, 2, FieldMode.Read, valueProviderCallback: _ => lfclkSource)
                .WithReservedBits(2, 30);

            Registers.HfclkAudioRun.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hfclkAudioRunning)
                .WithReservedBits(1, 31);
            Registers.HfclkAudioStat.Define(this)
                .WithReservedBits(0, 16)
                .WithFlag(16, FieldMode.Read, valueProviderCallback: _ => hfclkAudioRunning)
                .WithReservedBits(17, 15);
            Registers.Hfclk192MRun.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hfclk192mRunning)
                .WithReservedBits(1, 31);
            Registers.Hfclk192MStat.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hfclk192mRunning)
                .WithReservedBits(1, 15)
                .WithFlag(16, FieldMode.Read, valueProviderCallback: _ => hfclk192mRunning)
                .WithReservedBits(17, 15);

            Registers.HfclkSrc.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclkSource, writeCallback: (_, value) => hfclkSource = (uint)value);
            Registers.LfclkSrc.Define(this)
                .WithValueField(0, 2, valueProviderCallback: _ => lfclkSource, writeCallback: (_, value) => lfclkSource = (uint)value)
                .WithReservedBits(2, 30);
            Registers.HfclkCtrl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclkCtrl, writeCallback: (_, value) => hfclkCtrl = (uint)value);
            Registers.HfclkAudioFrequency.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclkAudioFrequency, writeCallback: (_, value) => hfclkAudioFrequency = (uint)value);
            Registers.HfclkAlwaysRun.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclkAlwaysRun, writeCallback: (_, value) => hfclkAlwaysRun = (uint)value);
            Registers.LfclkAlwaysRun.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => lfclkAlwaysRun, writeCallback: (_, value) => lfclkAlwaysRun = (uint)value);
            Registers.HfclkAudioAlwaysRun.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclkAudioAlwaysRun, writeCallback: (_, value) => hfclkAudioAlwaysRun = (uint)value);
            Registers.Hfclk192MSrc.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclk192mSource, writeCallback: (_, value) => hfclk192mSource = (uint)value);
            Registers.Hfclk192MAlwaysRun.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclk192mAlwaysRun, writeCallback: (_, value) => hfclk192mAlwaysRun = (uint)value);
            Registers.Hfclk192MCtrl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => hfclk192mCtrl, writeCallback: (_, value) => hfclk192mCtrl = (uint)value);

            Registers.ResetReas.Define(this)
                .WithValueField(0, 32,
                    valueProviderCallback: _ => resetReason,
                    writeCallback: (_, value) => resetReason &= ~(uint)value); // write-1-to-clear

            Registers.GpRegRet0.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => gpRegRet0, writeCallback: (_, value) => gpRegRet0 = (uint)value);
            Registers.GpRegRet1.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => gpRegRet1, writeCallback: (_, value) => gpRegRet1 = (uint)value);

            if(hasNetworkForceOff)
            {
                Registers.NetworkForceOff.Define(this, 0x1)
                    .WithFlag(0,
                        valueProviderCallback: _ => networkForceOff,
                        writeCallback: (_, value) =>
                        {
                            networkForceOff = value;
                            if(networkCore == null)
                            {
                                this.Log(LogLevel.Warning, "NETWORK.FORCEOFF written but no network core attached");
                                return;
                            }
                            networkCore.IsHalted = value;
                            this.Log(LogLevel.Info, "Network core {0}", value ? "forced off" : "released");
                        })
                    .WithReservedBits(1, 31);
            }
        }

        // The application-core CLOCK resets HFCLKCTRL to 1 (DIV2, 64 MHz); the
        // network-core CLOCK's HFCLKCTRL resets to 0 (128 MHz domain absent).
        // The app instance is identified by hasNetworkForceOff.
        private uint HfclkCtrlReset => hasNetworkForceOff ? 1u : 0u;

        private uint PendingInterrupts()
        {
            uint pending = 0;
            if(hfclkStarted) pending |= 1u << 0;
            if(lfclkStarted) pending |= 1u << 1;
            if(calDone) pending |= 1u << 7;
            if(hfclkAudioStarted) pending |= 1u << 8;
            if(hfclk192mStarted) pending |= 1u << 9;
            return pending & interruptEnable;
        }

        private void UpdateInterrupt()
        {
            IRQ.Set(PendingInterrupts() != 0);
        }

        private readonly ICPU networkCore;
        private readonly bool hasNetworkForceOff;

        private bool hfclkRunning, lfclkRunning, hfclkAudioRunning, hfclk192mRunning;
        private bool hfclkStarted, lfclkStarted, hfclkAudioStarted, hfclk192mStarted, calDone;
        private bool networkForceOff = true;
        private uint interruptEnable;
        // initializers = SVD reset values (see Reset())
        private uint hfclkSource = 1, lfclkSource = 1, hfclkAudioFrequency = 0x9BAE;
        private uint hfclkCtrl;  // set in the constructor (per-core reset value)
        private uint hfclkAlwaysRun, lfclkAlwaysRun, hfclkAudioAlwaysRun;
        private uint hfclk192mSource = 1, hfclk192mAlwaysRun, hfclk192mCtrl = 2;
        private uint resetReason;
        private uint gpRegRet0, gpRegRet1;

        private enum Registers : long
        {
            TasksHfclkStart = 0x000,
            TasksHfclkStop = 0x004,
            TasksLfclkStart = 0x008,
            TasksLfclkStop = 0x00C,
            TasksCal = 0x010,
            TasksHfclkAudioStart = 0x018,
            TasksHfclkAudioStop = 0x01C,
            TasksHfclk192MStart = 0x020,
            TasksHfclk192MStop = 0x024,
            TasksConstLat = 0x078,
            TasksLowPwr = 0x07C,
            EventsHfclkStarted = 0x100,
            EventsLfclkStarted = 0x104,
            EventsPofWarn = 0x108,
            EventsSleepEnter = 0x114,
            EventsSleepExit = 0x118,
            EventsDone = 0x11C,
            EventsHfclkAudioStarted = 0x120,
            EventsHfclk192MStarted = 0x124,
            InterruptEnable = 0x300,
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            InterruptPending = 0x30C,
            ResetReas = 0x400,
            HfclkRun = 0x408,
            HfclkStat = 0x40C,
            LfclkRun = 0x414,
            LfclkStat = 0x418,
            LfclkSrcCopy = 0x41C,
            HfclkAudioRun = 0x450,
            HfclkAudioStat = 0x454,
            Hfclk192MRun = 0x458,
            Hfclk192MStat = 0x45C,
            HfclkSrc = 0x514,
            LfclkSrc = 0x518,
            GpRegRet0 = 0x51C,
            GpRegRet1 = 0x520,
            HfclkCtrl = 0x558,
            HfclkAudioFrequency = 0x55C,
            HfclkAlwaysRun = 0x570,
            LfclkAlwaysRun = 0x574,
            HfclkAudioAlwaysRun = 0x57C,
            Hfclk192MSrc = 0x580,
            Hfclk192MAlwaysRun = 0x584,
            Hfclk192MCtrl = 0x5B8,
            NetworkForceOff = 0x614,
        }
    }
}
