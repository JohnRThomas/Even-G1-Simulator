//
// NRF5340_SAADC — successive-approximation ADC with EasyDMA.
// Application core: 0x4000E000 (NS) / 0x5000E000 (S), IRQ 14.
//
// Register offsets verbatim from the pinned nRF5340 SVD
// (hal_nordic 9784731461018d3e983604698fbbed6af2bea801),
// svd/nrf5340_application.svd, peripheral SAADC_NS/S.
//
// Analog inputs are injected from the monitor:
//   sysbus.saadc SetAnalogInput 3 900000     # AIN3 = 0.9 V (microvolts)
//   sysbus.saadc VddMicrovolts 3300000       # VDD source (PSELP=9)
//   sysbus.saadc VddhMicrovolts 3300000      # VDDHDIV5 source (PSELP=0xD)
//
// Conversion model (ideal transfer function, checked against the pinned
// Zephyr driver third_party/zephyr/drivers/adc/adc_nrfx_saadc.c and the
// nrf_saadc HAL):
//   single-ended: RESULT = round(V(P)        * GAIN / REF * 2^RESOLUTION)
//   differential: RESULT = round((V(P)-V(N)) * GAIN / REF * 2^(RESOLUTION-1))
// REF(internal) = 600000 uV (the driver's `ref_internal = 600` mV),
// REF(VDD1_4)   = VddMicrovolts / 4.
// Single-ended results are NOT clamped at zero: negative injected voltages
// produce negative codes, matching real hardware which returns small
// negative codes near 0 V (offset error); the Zephyr driver passes the raw
// signed value to the application. Saturation bounds (the internal
// differential full-scale range expressed in the SE/diff code domain):
//   single-ended: [-2^RESOLUTION,     2^RESOLUTION - 1]
//   differential: [-2^(RESOLUTION-1), 2^(RESOLUTION-1) - 1]
// With ideal injection, 0 V single-ended converts to exactly 0.
//
// Behavioural notes vs hardware:
//  - Conversions are instantaneous by default (TACQ stored, not modelled);
//    the driver only waits for EVENTS_END, which it receives.
//  - EVENTS_END stops the internal SAMPLERATE timer (the driver STOPs and
//    disables the ADC after END; sampling past MAXCNT is not modelled and
//    logs UNSUPPORTED).
//  - In SAMPLERATE.MODE=Timers, TASKS_SAMPLE starts the internal timer and
//    the first conversion happens one full period (CC ticks of 16 MHz)
//    later — deterministic virtual time.
//  - TASKS_STOP generates EVENTS_END (with AMOUNT latched) only if an
//    unfinished buffer was in progress, then EVENTS_STOPPED. The Zephyr
//    driver triggers STOP from its END handler; re-raising END there would
//    loop its IRQ handler, so END-at-STOP is suppressed once already fired.
//  - Oversampling with BURST=1 (the only mode the pinned driver uses)
//    yields one result per TASKS_SAMPLE; with BURST=0 a result is produced
//    every 2^OVERSAMPLE TASKS_SAMPLE triggers (accumulate-and-average).
//  - Limit events: LIMITH when result >= CH[n].LIMIT.HIGH, LIMITL when
//    result <= CH[n].LIMIT.LOW, checked on every result written to RAM.
//  - SUBSCRIBE_*/PUBLISH_* (DPPI) are not implemented: accesses log UNIMPL
//    and writes are stored (register-file semantics), no event routing.
//
// Faults (monitor-settable, default off):
//   FaultConversionDelay — microseconds of virtual-time latency inserted
//       before a scan / calibration completes (STATUS reads Busy meanwhile).
//   FaultDmaError — EasyDMA result writes are dropped (logged); events and
//       AMOUNT still advance.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class NRF5340_SAADC : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_SAADC(IMachine machine) : base(machine)
        {
            channels = new Channel[ChannelCount];
            for(var i = 0; i < ChannelCount; i++)
            {
                channels[i] = new Channel();
            }

            sampleTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "sampleTimer",
                limit: 2047, direction: Direction.Descending, enabled: false,
                workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: true);
            sampleTimer.LimitReached += OnSampleTimerTick;

            delayTimer = new LimitTimer(machine.ClockSource, 1000000, this, "faultDelayTimer",
                limit: 1, direction: Direction.Descending, enabled: false,
                workMode: WorkMode.OneShot, eventEnabled: true, autoUpdate: true);
            delayTimer.LimitReached += OnDelayedCompletion;

            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            foreach(var ch in channels)
            {
                ch.Reset();
            }
            enabled = false;
            acquisitionActive = false;
            scanPending = false;
            calibratePending = false;
            endFired = false;
            evStarted = evEnd = evDone = evResultDone = evCalibrateDone = evStopped = false;
            for(var i = 0; i < ChannelCount; i++)
            {
                evLimitH[i] = false;
                evLimitL[i] = false;
            }
            inten = 0;
            resolution = 1;         // SVD reset: 10bit
            oversample = 0;
            samplerateCc = 0;
            samplerateTimerMode = false;
            resultPtr = 0;
            resultMaxcnt = 0;
            resultAmount = 0;
            latchedPtr = 0;
            bufferPos = 0;
            overAccumulator = 0;
            overCount = 0;
            sampleTimer.Enabled = false;
            delayTimer.Enabled = false;
            unimplStore.Clear();
            UpdateInterrupts();
        }

        public override uint ReadDoubleWord(long offset)
        {
            if(RegistersCollection.TryRead(offset, out var value))
            {
                return value;
            }
            if(!unimplStore.TryGetValue(offset, out value))
            {
                value = 0;
            }
            LogUnimplemented("R", offset, value);
            return value;
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            if(RegistersCollection.TryWrite(offset, value))
            {
                return;
            }
            LogUnimplemented("W", offset, value);
            unimplStore[offset] = value;
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; } = new GPIO();

        // ------------- monitor-facing injection / fault API -------------

        // AIN0..AIN7 injected pin voltage, in microvolts (board map: AIN3 = P0.07).
        public void SetAnalogInput(int ainIndex, long microvolts)
        {
            if(ainIndex < 0 || ainIndex >= AinCount)
            {
                this.Log(LogLevel.Error, "SetAnalogInput: AIN index {0} out of range 0..7", ainIndex);
                return;
            }
            ainMicrovolts[ainIndex] = microvolts;
            this.Log(LogLevel.Info, "AIN{0} set to {1} uV", ainIndex, microvolts);
        }

        public long GetAnalogInput(int ainIndex)
        {
            if(ainIndex < 0 || ainIndex >= AinCount)
            {
                this.Log(LogLevel.Error, "GetAnalogInput: AIN index {0} out of range 0..7", ainIndex);
                return 0;
            }
            return ainMicrovolts[ainIndex];
        }

        // VDD source voltage (PSELP/PSELN = 9) and the VDD1_4 reference.
        public long VddMicrovolts { get; set; } = 3300000;

        // VDDH source (PSELP/PSELN = 0xD selects VDDH/5).
        public long VddhMicrovolts { get; set; } = 3300000;

        // Fault: virtual-time microseconds inserted before scan/calibration completion.
        public ulong FaultConversionDelay { get; set; }

        // Fault: drop EasyDMA result writes (events/AMOUNT still advance).
        public bool FaultDmaError { get; set; }

        // ------------------------- registers -------------------------

        private void DefineRegisters()
        {
            Registers.TasksStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) => { if(value) DoStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksSample.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_SAMPLE", writeCallback: (_, value) => { if(value) DoSampleTask(); })
                .WithReservedBits(1, 31);

            Registers.TasksStop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) => { if(value) DoStop(); })
                .WithReservedBits(1, 31);

            Registers.TasksCalibrateOffset.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_CALIBRATEOFFSET", writeCallback: (_, value) => { if(value) DoCalibrate(); })
                .WithReservedBits(1, 31);

            DefineEvent(Registers.EventsStarted, "EVENTS_STARTED", () => evStarted, v => evStarted = v);
            DefineEvent(Registers.EventsEnd, "EVENTS_END", () => evEnd, v => evEnd = v);
            DefineEvent(Registers.EventsDone, "EVENTS_DONE", () => evDone, v => evDone = v);
            DefineEvent(Registers.EventsResultDone, "EVENTS_RESULTDONE", () => evResultDone, v => evResultDone = v);
            DefineEvent(Registers.EventsCalibrateDone, "EVENTS_CALIBRATEDONE", () => evCalibrateDone, v => evCalibrateDone = v);
            DefineEvent(Registers.EventsStopped, "EVENTS_STOPPED", () => evStopped, v => evStopped = v);

            Registers.EventsChLimitH.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithFlag(0, name: "LIMITH",
                        valueProviderCallback: _ => evLimitH[idx],
                        writeCallback: (_, value) => { evLimitH[idx] = value; UpdateInterrupts(); })
                    .WithReservedBits(1, 31);
            }, stepInBytes: 8);

            Registers.EventsChLimitL.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithFlag(0, name: "LIMITL",
                        valueProviderCallback: _ => evLimitL[idx],
                        writeCallback: (_, value) => { evLimitL[idx] = value; UpdateInterrupts(); })
                    .WithReservedBits(1, 31);
            }, stepInBytes: 8);

            Registers.InterruptEnable.Define(this)
                .WithValueField(0, 22, name: "INTEN",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten = (uint)value; UpdateInterrupts(); })
                .WithReservedBits(22, 10);

            Registers.InterruptEnableSet.Define(this)
                .WithValueField(0, 22, name: "INTENSET",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten |= (uint)value; UpdateInterrupts(); })
                .WithReservedBits(22, 10);

            Registers.InterruptEnableClear.Define(this)
                .WithValueField(0, 22, name: "INTENCLR",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten &= ~(uint)value; UpdateInterrupts(); })
                .WithReservedBits(22, 10);

            Registers.Status.Define(this)
                .WithFlag(0, FieldMode.Read, name: "STATUS",
                    valueProviderCallback: _ => scanPending || calibratePending)
                .WithReservedBits(1, 31);

            Registers.Enable.Define(this)
                .WithFlag(0, name: "ENABLE",
                    valueProviderCallback: _ => enabled,
                    writeCallback: (_, value) => enabled = value)
                .WithReservedBits(1, 31);

            Registers.ChPselP.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithValueField(0, 5, name: "PSELP",
                        valueProviderCallback: _ => channels[idx].PselP,
                        writeCallback: (_, value) => channels[idx].PselP = (uint)value)
                    .WithReservedBits(5, 27);
            }, stepInBytes: ChannelStep);

            Registers.ChPselN.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithValueField(0, 5, name: "PSELN",
                        valueProviderCallback: _ => channels[idx].PselN,
                        writeCallback: (_, value) => channels[idx].PselN = (uint)value)
                    .WithReservedBits(5, 27);
            }, stepInBytes: ChannelStep);

            Registers.ChConfig.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithValueField(0, 2, name: "RESP",
                        valueProviderCallback: _ => channels[idx].Resp,
                        writeCallback: (_, value) => channels[idx].Resp = (uint)value)
                    .WithReservedBits(2, 2)
                    .WithValueField(4, 2, name: "RESN",
                        valueProviderCallback: _ => channels[idx].Resn,
                        writeCallback: (_, value) => channels[idx].Resn = (uint)value)
                    .WithReservedBits(6, 2)
                    .WithValueField(8, 3, name: "GAIN",
                        valueProviderCallback: _ => channels[idx].Gain,
                        writeCallback: (_, value) => channels[idx].Gain = (uint)value)
                    .WithReservedBits(11, 1)
                    .WithFlag(12, name: "REFSEL",
                        valueProviderCallback: _ => channels[idx].RefVdd4,
                        writeCallback: (_, value) => channels[idx].RefVdd4 = value)
                    .WithReservedBits(13, 3)
                    .WithValueField(16, 3, name: "TACQ",
                        valueProviderCallback: _ => channels[idx].Tacq,
                        writeCallback: (_, value) => channels[idx].Tacq = (uint)value)
                    .WithReservedBits(19, 1)
                    .WithFlag(20, name: "MODE",
                        valueProviderCallback: _ => channels[idx].Differential,
                        writeCallback: (_, value) => channels[idx].Differential = value)
                    .WithReservedBits(21, 3)
                    .WithFlag(24, name: "BURST",
                        valueProviderCallback: _ => channels[idx].Burst,
                        writeCallback: (_, value) => channels[idx].Burst = value)
                    .WithReservedBits(25, 7);
            }, stepInBytes: ChannelStep, resetValue: 0x00020000);

            Registers.ChLimit.DefineMany(this, ChannelCount, (register, idx) =>
            {
                register
                    .WithValueField(0, 16, name: "LOW",
                        valueProviderCallback: _ => (ushort)channels[idx].LimitLow,
                        writeCallback: (_, value) => channels[idx].LimitLow = (short)(ushort)value)
                    .WithValueField(16, 16, name: "HIGH",
                        valueProviderCallback: _ => (ushort)channels[idx].LimitHigh,
                        writeCallback: (_, value) => channels[idx].LimitHigh = (short)(ushort)value);
            }, stepInBytes: ChannelStep, resetValue: 0x7FFF8000);

            Registers.Resolution.Define(this, 0x1)
                .WithValueField(0, 3, name: "VAL",
                    valueProviderCallback: _ => resolution,
                    writeCallback: (_, value) => resolution = (uint)value)
                .WithReservedBits(3, 29);

            Registers.Oversample.Define(this)
                .WithValueField(0, 4, name: "OVERSAMPLE",
                    valueProviderCallback: _ => oversample,
                    writeCallback: (_, value) => oversample = (uint)value)
                .WithReservedBits(4, 28);

            Registers.SampleRate.Define(this)
                .WithValueField(0, 11, name: "CC",
                    valueProviderCallback: _ => samplerateCc,
                    writeCallback: (_, value) => samplerateCc = (uint)value)
                .WithReservedBits(11, 1)
                .WithFlag(12, name: "MODE",
                    valueProviderCallback: _ => samplerateTimerMode,
                    writeCallback: (_, value) => samplerateTimerMode = value)
                .WithReservedBits(13, 19);

            Registers.ResultPtr.Define(this)
                .WithValueField(0, 32, name: "PTR",
                    valueProviderCallback: _ => resultPtr,
                    writeCallback: (_, value) => resultPtr = (uint)value);

            Registers.ResultMaxCnt.Define(this)
                .WithValueField(0, 15, name: "MAXCNT",
                    valueProviderCallback: _ => resultMaxcnt,
                    writeCallback: (_, value) => resultMaxcnt = (uint)value)
                .WithReservedBits(15, 17);

            Registers.ResultAmount.Define(this)
                .WithValueField(0, 15, FieldMode.Read, name: "AMOUNT",
                    valueProviderCallback: _ => resultAmount)
                .WithReservedBits(15, 17);
        }

        private void DefineEvent(Registers register, string name, Func<bool> getter, Action<bool> setter)
        {
            register.Define(this)
                .WithFlag(0, name: name,
                    valueProviderCallback: _ => getter(),
                    writeCallback: (_, value) => { setter(value); UpdateInterrupts(); })
                .WithReservedBits(1, 31);
        }

        // ------------------------- behaviour -------------------------

        private void DoStart()
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=TASKS_START detail=triggered while ENABLE=0, ignored");
                return;
            }
            latchedPtr = resultPtr;
            bufferPos = 0;
            resultAmount = 0;
            endFired = false;
            overAccumulator = 0;
            overCount = 0;
            acquisitionActive = true;
            evStarted = true;
            UpdateInterrupts();
        }

        private void DoSampleTask()
        {
            if(!enabled || !acquisitionActive)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=TASKS_SAMPLE detail=triggered while not started (ENABLE={0}, started={1}), ignored",
                    enabled, acquisitionActive);
                return;
            }
            if(samplerateTimerMode)
            {
                if(sampleTimer.Enabled)
                {
                    return; // timer already running; hardware needs a single SAMPLE trigger
                }
                if(samplerateCc < 80 || samplerateCc > 2047)
                {
                    this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=SAMPLERATE.CC detail=value {0} outside 80..2047, timer not started", samplerateCc);
                    return;
                }
                sampleTimer.Enabled = false;
                // autoUpdate: the Limit setter reloads the countdown.
                sampleTimer.Limit = samplerateCc;
                sampleTimer.Enabled = true;
                return;
            }
            ScheduleScan();
        }

        private void OnSampleTimerTick()
        {
            ScheduleScan();
        }

        private void ScheduleScan()
        {
            if(FaultConversionDelay > 0)
            {
                if(scanPending || calibratePending)
                {
                    this.Log(LogLevel.Warning, "SAMPLE while a delayed operation is pending, ignored");
                    return;
                }
                scanPending = true;
                ArmDelayTimer();
                return;
            }
            DoScan();
        }

        private void DoCalibrate()
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=TASKS_CALIBRATEOFFSET detail=triggered while ENABLE=0, ignored");
                return;
            }
            if(FaultConversionDelay > 0)
            {
                calibratePending = true;
                ArmDelayTimer();
                return;
            }
            evCalibrateDone = true;
            UpdateInterrupts();
        }

        private void ArmDelayTimer()
        {
            delayTimer.Enabled = false;
            // autoUpdate: the Limit setter reloads the countdown.
            delayTimer.Limit = FaultConversionDelay;
            delayTimer.Enabled = true;
        }

        private void OnDelayedCompletion()
        {
            if(calibratePending)
            {
                calibratePending = false;
                evCalibrateDone = true;
                UpdateInterrupts();
            }
            if(scanPending)
            {
                scanPending = false;
                DoScan();
            }
        }

        private void DoStop()
        {
            sampleTimer.Enabled = false;
            delayTimer.Enabled = false;
            scanPending = false;
            calibratePending = false;
            if(acquisitionActive && bufferPos > 0 && !endFired)
            {
                // STOP terminates an in-progress buffer: END with AMOUNT latched.
                resultAmount = bufferPos;
                endFired = true;
                evEnd = true;
            }
            acquisitionActive = false;
            evStopped = true;
            UpdateInterrupts();
        }

        private void DoScan()
        {
            var activeChannels = 0;
            for(var i = 0; i < ChannelCount; i++)
            {
                if(channels[i].PselP != PselNotConnected)
                {
                    activeChannels++;
                }
            }
            if(activeChannels == 0)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=TASKS_SAMPLE detail=no channel has PSELP connected, no conversion");
                return;
            }
            if(oversample > 0 && activeChannels > 1)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=OVERSAMPLE detail=oversampling with {0} channels enabled (hardware supports single channel only)", activeChannels);
            }

            for(var i = 0; i < ChannelCount; i++)
            {
                if(channels[i].PselP == PselNotConnected)
                {
                    continue;
                }
                var result = ConvertChannel(i);
                evDone = true;

                if(oversample > 0 && !channels[i].Burst)
                {
                    // Without burst, each TASKS_SAMPLE contributes one conversion
                    // to the oversampling accumulator.
                    overAccumulator += result;
                    overCount++;
                    if(overCount < (1 << (int)oversample))
                    {
                        continue;
                    }
                    result = (short)Math.Round(overAccumulator / (double)overCount, MidpointRounding.AwayFromZero);
                    overAccumulator = 0;
                    overCount = 0;
                }
                // With BURST=1 the 2^OVERSAMPLE conversions happen back-to-back on
                // the same injected (constant) voltage: the average equals a single
                // conversion, so the result is used directly.

                WriteResult(i, result);
            }
            UpdateInterrupts();
        }

        private void WriteResult(int channelIdx, short result)
        {
            if(bufferPos >= resultMaxcnt)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=RESULT_OVERRUN detail=conversion after MAXCNT={0} results, dropped", resultMaxcnt);
                return;
            }
            var address = latchedPtr + 2UL * bufferPos;
            if(FaultDmaError)
            {
                this.Log(LogLevel.Warning, "FaultDmaError active: dropping result 0x{0:X4} for CH{1} (would go to 0x{2:X8})",
                    (ushort)result, channelIdx, address);
            }
            else
            {
                sysbus.WriteWord(address, (ushort)result);
            }
            bufferPos++;
            evResultDone = true;

            if(result >= channels[channelIdx].LimitHigh)
            {
                evLimitH[channelIdx] = true;
            }
            if(result <= channels[channelIdx].LimitLow)
            {
                evLimitL[channelIdx] = true;
            }

            if(bufferPos >= resultMaxcnt && !endFired)
            {
                resultAmount = bufferPos;
                endFired = true;
                evEnd = true;
                // The driver STOPs after END; continuous sampling past MAXCNT is
                // not modelled, so the internal sample timer stops here.
                sampleTimer.Enabled = false;
            }
        }

        private short ConvertChannel(int idx)
        {
            var ch = channels[idx];
            var vp = SourceMicrovolts(ch.PselP);
            var vn = ch.Differential ? SourceMicrovolts(ch.PselN) : 0;
            var refUv = ch.RefVdd4 ? VddMicrovolts / 4 : InternalReferenceMicrovolts;
            if(refUv <= 0)
            {
                this.Log(LogLevel.Error, "Non-positive reference voltage {0} uV, using internal 600000 uV", refUv);
                refUv = InternalReferenceMicrovolts;
            }
            var bits = ResolutionBits;
            var scaleBits = ch.Differential ? bits - 1 : bits;
            var code = (vp - vn) * GainNumerator[ch.Gain] * (double)(1L << scaleBits)
                       / (GainDenominator[ch.Gain] * (double)refUv);
            var quantized = (long)Math.Round(code, MidpointRounding.AwayFromZero);
            long min, max;
            if(ch.Differential)
            {
                min = -(1L << (bits - 1));
                max = (1L << (bits - 1)) - 1;
            }
            else
            {
                min = -(1L << bits);
                max = (1L << bits) - 1;
            }
            if(quantized > max)
            {
                quantized = max;
            }
            if(quantized < min)
            {
                quantized = min;
            }
            return (short)quantized;
        }

        private long SourceMicrovolts(uint psel)
        {
            if(psel >= 1 && psel <= 8)
            {
                return ainMicrovolts[psel - 1];
            }
            if(psel == 9)
            {
                return VddMicrovolts;
            }
            if(psel == 0xD)
            {
                return VddhMicrovolts / 5;
            }
            this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=PSEL detail=input selector 0x{0:X} not a valid source, treated as 0 V", psel);
            return 0;
        }

        private int ResolutionBits
        {
            get
            {
                switch(resolution)
                {
                case 0: return 8;
                case 1: return 10;
                case 2: return 12;
                case 3: return 14;
                default:
                    this.Log(LogLevel.Warning, "UNSUPPORTED periph=SAADC what=RESOLUTION detail=value {0} reserved, using 12bit", resolution);
                    return 12;
                }
            }
        }

        private uint PendingMask()
        {
            uint pending = 0;
            if(evStarted) pending |= 1u << 0;
            if(evEnd) pending |= 1u << 1;
            if(evDone) pending |= 1u << 2;
            if(evResultDone) pending |= 1u << 3;
            if(evCalibrateDone) pending |= 1u << 4;
            if(evStopped) pending |= 1u << 5;
            for(var i = 0; i < ChannelCount; i++)
            {
                if(evLimitH[i]) pending |= 1u << (6 + 2 * i);
                if(evLimitL[i]) pending |= 1u << (7 + 2 * i);
            }
            return pending;
        }

        private void UpdateInterrupts()
        {
            IRQ.Set((PendingMask() & inten) != 0);
        }

        private void LogUnimplemented(string dir, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph=SAADC dir={0} width=4 offset=0x{1:X} value=0x{2:X8} pc=0x{3:X8}",
                dir, offset, value, pc);
        }

        private readonly Channel[] channels;
        private readonly LimitTimer sampleTimer;
        private readonly LimitTimer delayTimer;
        private readonly long[] ainMicrovolts = new long[AinCount];
        private readonly Dictionary<long, uint> unimplStore = new Dictionary<long, uint>();

        private bool enabled;
        private bool acquisitionActive;
        private bool scanPending;
        private bool calibratePending;
        private bool endFired;

        private bool evStarted, evEnd, evDone, evResultDone, evCalibrateDone, evStopped;
        private readonly bool[] evLimitH = new bool[ChannelCount];
        private readonly bool[] evLimitL = new bool[ChannelCount];
        private uint inten;

        private uint resolution;
        private uint oversample;
        private uint samplerateCc;
        private bool samplerateTimerMode;
        private uint resultPtr;
        private uint resultMaxcnt;
        private uint resultAmount;
        private uint latchedPtr;
        private uint bufferPos;
        private long overAccumulator;
        private int overCount;

        private const int ChannelCount = 8;
        private const int AinCount = 8;
        private const uint ChannelStep = 0x10;
        private const uint PselNotConnected = 0;
        private const long InternalReferenceMicrovolts = 600000;
        private const long TimerFrequency = 16000000;

        private static readonly long[] GainNumerator = { 1, 1, 1, 1, 1, 1, 2, 4 };
        private static readonly long[] GainDenominator = { 6, 5, 4, 3, 2, 1, 1, 1 };

        private class Channel
        {
            public void Reset()
            {
                PselP = 0;
                PselN = 0;
                Resp = 0;
                Resn = 0;
                Gain = 0;
                RefVdd4 = false;
                Tacq = 2;          // SVD reset CONFIG=0x00020000
                Differential = false;
                Burst = false;
                LimitLow = short.MinValue;   // SVD reset LIMIT=0x7FFF8000
                LimitHigh = short.MaxValue;
            }

            public uint PselP;
            public uint PselN;
            public uint Resp;
            public uint Resn;
            public uint Gain;
            public bool RefVdd4;
            public uint Tacq;
            public bool Differential;
            public bool Burst;
            public short LimitLow;
            public short LimitHigh;
        }

        private enum Registers : long
        {
            TasksStart = 0x000,
            TasksSample = 0x004,
            TasksStop = 0x008,
            TasksCalibrateOffset = 0x00C,
            // SUBSCRIBE_* 0x080..0x08C, PUBLISH_* 0x180..0x19C: DPPI endpoints,
            // not implemented — accesses fall through to the UNIMPL logger.
            EventsStarted = 0x100,
            EventsEnd = 0x104,
            EventsDone = 0x108,
            EventsResultDone = 0x10C,
            EventsCalibrateDone = 0x110,
            EventsStopped = 0x114,
            EventsChLimitH = 0x118,     // + n*8
            EventsChLimitL = 0x11C,     // + n*8
            InterruptEnable = 0x300,
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            Status = 0x400,
            Enable = 0x500,
            ChPselP = 0x510,            // + n*0x10
            ChPselN = 0x514,            // + n*0x10
            ChConfig = 0x518,           // + n*0x10
            ChLimit = 0x51C,            // + n*0x10
            Resolution = 0x5F0,
            Oversample = 0x5F4,
            SampleRate = 0x5F8,
            ResultPtr = 0x62C,
            ResultMaxCnt = 0x630,
            ResultAmount = 0x634,
        }
    }
}
