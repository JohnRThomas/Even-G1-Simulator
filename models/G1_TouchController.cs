//
// G1_TouchController — the Even Realities G1 "TouchBar" capacitive touch
// controller, as it actually appears on the glasses: an I2C device on
// TWIM2 (0x5000B000) at 7-bit address 0x12, 16-bit BIG-ENDIAN register
// addresses and 32-bit BIG-ENDIAN values.
//
// Everything here is taken from docs/g1-button-semantics.md, which decoded the
// stock app-core image and then MEASURED the results on the real left leg
// (payloads T15/T16/T17/T18). Nothing is invented:
//
//   * transport (§2): write_reg32 sends 6 bytes [reg>>8, reg, v>>24, v>>16,
//     v>>8, v] with STOP; read_reg32 writes 2 bytes [reg>>8, reg] with NO stop,
//     repeated START, then reads 4 bytes MSB-first.
//   * 0xFF00 chip ID — hardware read 0x20223203 (accepted set also
//     0x20223205 / 0x20223207).
//   * 0xFF20 unlock/key register — firmware writes magic 0x3C00F091.
//   * 0x4700 reset/mode — 0x110 then 0x102 in the power-on sequence.
//   * 0x4408 config commit — firmware writes 1 after loading the 44-entry table.
//   * 0x4414 event-enable mask — firmware writes 0x0000040F.
//   * 0x4410 event/status register — READ-TO-CLEAR (T15 proved this for bit0:
//     the first read returned 0x00000001 and the next read 0 with nothing
//     written in between). bit0 = "controller was reset, re-apply config",
//     bit1 = TOUCH-DOWN, bit2 = TOUCH-UP, bit5 = periodic measurement-cycle
//     status that the stock firmware never reads.
//
// The two behavioural gates below are the substance of the model, and both are
// hardware findings rather than guesses:
//
//   1. bit0 READY HANDSHAKE. T17 measured bit0 set within 1 ms of the wake
//      sequence ("after reset my configuration is gone, load it"). The model
//      therefore raises bit0 out of reset and after any 0x4700 wake write.
//   2. EVENTS REQUIRE THE FULL CONFIGURATION. T16 (minimal enable only) saw
//      0 non-zero samples in 18 957 polls WITH A HUMAN TAPPING; T17 (full
//      44-register table + 0x4408 commit + 0x4414 = 0x40F) saw 972 non-zero
//      samples in 1264 polls with nobody touching it. So an image that has not
//      committed the table and enabled events gets NO touch bits, no matter how
//      hard the bar is pressed. A payload that skips the table cannot use the
//      TouchBar as a reset abort, and this model reproduces that failure rather
//      than papering over it.
//
// INTERRUPT LINE (added for the simulator, from the v1.66 firmware source).
// The controller's INT output goes to P1.09, active low. The firmware does
// not poll: its P1.09 edge interrupt wakes the touch thread, which then reads
// 0x4410 once. So an event is only noticed if the line falls for it, and a
// press and its release must arrive as two separate edges, each read on its
// own (a read showing both bits is taken as a press only). Events are
// therefore presented one at a time: the line goes low with the event's bit
// in 0x4410, returns high when 0x4410 is read, and the next queued event
// follows after its gap. While the bar is touched, the channel-0 signal
// register 0x1C04 reads PressedSignal; the firmware re-reads it after a press
// and discards presses whose signal (>> 10) is below 20000.
//
// Injection (monitor-settable / callable):
//   touch Press           — queue one TOUCH-DOWN (bit1) event
//   touch Release         — queue one TOUCH-UP   (bit2) event
//   touch Tap [ms]        — Press, then Release ms later (default 120)
//   touch MeasurementBit5 true   — emit the idle bit5 chatter T17 measured
// Observability (read-only):
//   ConfigWriteCount, CommitWritten, EnableMask, MagicWritten,
//   PressesDelivered, EventReadCount
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class G1_TouchController : II2CPeripheral, II2CDeviceFaultNak
    {
        public G1_TouchController(IMachine machine)
        {
            IRQ = new GPIO();
            eventTimer = new LimitTimer(machine.ClockSource, 1000000, this, "nextEvent",
                1, Direction.Ascending, enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            eventTimer.LimitReached += PresentNextEvent;
            PowerCycle();
        }

        // INT output, active low: wire it to the SoC pin (P1.09 on the G1).
        public GPIO IRQ { get; }

        // Renode calls Reset() on every peripheral when the MACHINE resets. That
        // is wrong for this device and the difference is load-bearing: the touch
        // controller is a SEPARATE CHIP on its own rail, fed from P0.30 — a
        // SYSRESETREQ or a WDT0 timeout does not power-cycle it, and it does not
        // sit in the SoC's reset domain. Only an nPM1300 power-cycle takes its
        // rail down. So a SoC reset clears the transport state and NOTHING else;
        // the register contents, the loaded configuration and the observability
        // counters survive, exactly as they would on the bench.
        //
        // (This is also what makes the mechanism-4 test meaningful: T18's exit IS
        // a reset, so if Reset() wiped the counters, every press-path measurement
        // would read zero afterwards and the test would report the payload did
        // nothing when in fact it did everything.)
        public void Reset()
        {
            pointerReg = 0;
            pointerValid = false;
            readIndex = 0;
            lineSynced = false;     // the SoC's pin state was just reset
        }

        // Makes the SoC pin see the line's level. The level set before the
        // pin was connected, or before a reset of the SoC's GPIO port, never
        // reached it, and a later change to low would then not be an edge.
        private void SyncLine()
        {
            if(lineSynced)
            {
                return;
            }
            lineSynced = true;
            IRQ.Set(presentedEvent != 0);
            IRQ.Set(presentedEvent == 0);
        }

        // Losing the rail: this is what an nPM1300 MODE_RESET power-cycle does to
        // the controller. Not wired to anything today — the PMIC model resets the
        // machine, and nothing tells this device its rail dropped — but it is the
        // one operation that legitimately clears the configuration, and it leaves
        // bit0 set so the next reader gets the "re-apply config" handshake.
        public void PowerCycle()
        {
            registers.Clear();
            registers[RegChipId] = ChipId;
            registers[RegStatus1BA8] = HealthyFastPath;
            pendingEvents = EventControllerReset; // bit0: "config is gone, load it"
            pointerReg = 0;
            pointerValid = false;
            readIndex = 0;
            configWrites = 0;
            CommitWritten = false;
            MagicWritten = false;
            EnableMask = 0;
            PressesDelivered = 0;
            EventReadCount = 0;
            queuedEvents.Clear();
            eventTimer.Enabled = false;
            presentedEvent = 0;
            touched = false;
            IRQ.Set(true);
        }

        // -------------------- I2C transport --------------------
        // A write of exactly 2 bytes is the address phase of a read (§2
        // read_reg32); 6 bytes is a register write. Anything else is logged and
        // dropped rather than silently half-applied.

        // NRF5340_TWIM hands the whole EasyDMA TXD buffer over in ONE Write call
        // (transactionDevice.Write(data) after reading TXD.MAXCNT bytes), so a
        // call is a complete frame: 2 bytes = the address phase of a read,
        // 6 bytes = a register write.
        public void Write(byte[] data)
        {
            if(data.Length < 2)
            {
                this.Log(LogLevel.Warning, "short {0}-byte frame; dropping", data.Length);
                return;
            }
            SyncLine();
            pointerReg = (ushort)((data[0] << 8) | data[1]);
            pointerValid = true;
            readIndex = 0;
            if(data.Length == 2)
            {
                return; // address phase; the repeated-start read follows
            }
            if(data.Length != 6)
            {
                this.Log(LogLevel.Warning, "unexpected {0}-byte frame for reg 0x{1:X4}; dropping",
                    data.Length, pointerReg);
                return;
            }
            var value = ((uint)data[2] << 24) | ((uint)data[3] << 16)
                | ((uint)data[4] << 8) | data[5];
            WriteRegister(pointerReg, value);
        }

        public byte[] Read(int count = 1)
        {
            if(!pointerValid)
            {
                this.Log(LogLevel.Warning, "read with no register address phase");
                return new byte[count];
            }
            var value = ReadRegister(pointerReg);
            var bytes = new byte[count];
            for(var i = 0; i < count; i++)
            {
                // 32-bit register, MSB first; no auto-increment.
                bytes[i] = (byte)(value >> (8 * (3 - (readIndex & 3))));
                readIndex++;
            }
            return bytes;
        }

        public void FinishTransmission()
        {
            // The register pointer must survive: a write-then-restart-read is one
            // logical access and the address phase carries no STOP.
            readIndex = 0;
        }

        // -------------------- injection --------------------

        public void Press()
        {
            Enqueue(EventTouchDown, MinimumGapMicroseconds);
            this.Log(LogLevel.Info, "G1TOUCH press queued");
        }

        public void Release()
        {
            Enqueue(EventTouchUp, MinimumGapMicroseconds);
            this.Log(LogLevel.Info, "G1TOUCH release queued");
        }

        public void Tap(uint milliseconds = 120)
        {
            Enqueue(EventTouchDown, MinimumGapMicroseconds);
            Enqueue(EventTouchUp, Math.Max(MinimumGapMicroseconds, (ulong)milliseconds * 1000));
            this.Log(LogLevel.Info, "G1TOUCH {0} ms tap queued", milliseconds);
        }

        // What the channel-0 signal register reads while the bar is touched.
        public uint PressedSignal { get; set; } = 0x02000000;

        public bool MeasurementBit5 { get; set; }
        public bool FaultNak { get; set; }

        // -------------------- observability --------------------

        public int ConfigWriteCount => configWrites;
        public bool CommitWritten { get; private set; }
        public bool MagicWritten { get; private set; }
        public uint EnableMask { get; private set; }
        public int PressesDelivered { get; private set; }
        public int EventReadCount { get; private set; }

        // Events are only visible once the image has done what the stock
        // firmware does: unlock, load the table, commit, enable (T16 vs T17).
        public bool EventsArmed => CommitWritten && (EnableMask & 0x6) == 0x6
            && configWrites >= MinimumConfigWrites;

        // -------------------- register model --------------------

        private uint ReadRegister(ushort register)
        {
            if(register == RegSignalChannel0)
            {
                return touched ? PressedSignal : 0;
            }
            if(register == RegEvents)
            {
                EventReadCount++;
                var value = pendingEvents;
                if(presentedEvent != 0)
                {
                    value |= presentedEvent;
                    if((presentedEvent & EventTouchDown) != 0)
                    {
                        PressesDelivered++;
                    }
                    presentedEvent = 0;
                    IRQ.Set(true);      // read-to-clear releases the line
                    ScheduleNextEvent();
                }
                if(MeasurementBit5)
                {
                    value |= EventMeasurementCycle;
                }
                pendingEvents = 0; // read-to-clear (§7 Q1, measured by T15)
                return value;
            }
            uint stored;
            return registers.TryGetValue(register, out stored) ? stored : 0;
        }

        private void WriteRegister(ushort register, uint value)
        {
            registers[register] = value;
            switch(register)
            {
            case RegMagic:
                MagicWritten = value == UnlockMagic;
                break;
            case RegWake:
                // 0x110 then 0x102 power-on sequence: the controller comes up
                // announcing "my configuration is gone" (T17: bit0 within 1 ms).
                pendingEvents |= EventControllerReset;
                break;
            case RegCommit:
                CommitWritten = (value & 1) != 0;
                break;
            case RegEnable:
                EnableMask = value;
                break;
            default:
                // Everything that is not one of the named control registers is a
                // configuration-table entry. The measured table is 44 entries:
                // 40 in 0x1Bxx..0x1Exx, entry 34 = 0x4414 (deferred until after
                // the commit) and entries 35..37 = 0xFF84/88/8C.
                configWrites++;
                break;
            }
        }

        private void Enqueue(uint eventBit, ulong gapMicroseconds)
        {
            queuedEvents.Add(eventBit);
            queuedGaps.Add(gapMicroseconds);
            ScheduleNextEvent();
        }

        // Arms the timer for the head of the queue unless an event is still
        // waiting to be read or the timer is already running.
        private void ScheduleNextEvent()
        {
            if(presentedEvent != 0 || eventTimer.Enabled || queuedEvents.Count == 0)
            {
                return;
            }
            eventTimer.Limit = queuedGaps[0];
            eventTimer.Value = 0;
            eventTimer.Enabled = true;
        }

        private void PresentNextEvent()
        {
            eventTimer.Enabled = false;
            if(queuedEvents.Count == 0)
            {
                return;
            }
            var eventBit = queuedEvents[0];
            queuedEvents.RemoveAt(0);
            queuedGaps.RemoveAt(0);
            touched = (eventBit & EventTouchDown) != 0;
            if(!EventsArmed)
            {
                // No configuration, no events (T16): the touch goes unnoticed.
                this.Log(LogLevel.Warning, "G1TOUCH event 0x{0:X} dropped: the controller is not configured", eventBit);
                ScheduleNextEvent();
                return;
            }
            SyncLine();
            presentedEvent = eventBit;
            IRQ.Set(false);
        }

        private uint pendingEvents;
        private uint presentedEvent;
        private bool touched;
        private bool lineSynced;
        private readonly LimitTimer eventTimer;
        private readonly List<ulong> queuedGaps = new List<ulong>();
        private ushort pointerReg;
        private bool pointerValid;
        private int readIndex;
        private int configWrites;

        private readonly List<uint> queuedEvents = new List<uint>();
        private readonly Dictionary<ushort, uint> registers = new Dictionary<ushort, uint>();

        private const ushort RegCommit = 0x4408;
        private const ushort RegEvents = 0x4410;
        private const ushort RegEnable = 0x4414;
        private const ushort RegWake = 0x4700;
        private const ushort RegChipId = 0xFF00;
        private const ushort RegMagic = 0xFF20;
        private const ushort RegStatus1BA8 = 0x1BA8;
        private const ushort RegSignalChannel0 = 0x1C04;

        // Shortest time between two interrupts: the firmware's touch thread
        // must have finished with one event before the line falls again.
        private const ulong MinimumGapMicroseconds = 20000;

        private const uint ChipId = 0x20223203;         // measured on the left leg
        private const uint UnlockMagic = 0x3C00F091;
        private const uint HealthyFastPath = 0x06000007; // T17 read this back

        private const uint EventControllerReset = 0x1;
        private const uint EventTouchDown = 0x2;
        private const uint EventTouchUp = 0x4;
        private const uint EventMeasurementCycle = 0x20;

        // T17 wrote 43 of the 44 table entries in the loop plus entry 34 after
        // the commit. Require most of them so a minimal-enable image (T16) is
        // rejected exactly as the silicon rejected it.
        private const int MinimumConfigWrites = 40;
    }
}
