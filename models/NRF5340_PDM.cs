//
// NRF5340_PDM — PDM microphone interface with EasyDMA.
// Application core: 0x40026000 (NS) / 0x50026000 (S), IRQ 38.
//
// Register offsets verbatim from the pinned nRF5340 SVD
// (hal_nordic 9784731461018d3e983604698fbbed6af2bea801),
// svd/nrf5340_application.svd, peripheral PDM0_NS/S.
//
// The microphone itself is deliberately generic (board map: manufacturer
// unspecified; CLK P1.13, DIN P1.14). PCM data is injected from the
// monitor via deterministic sources:
//   sysbus.pdm0 ConfigureSilence                     # all-zero samples (default)
//   sysbus.pdm0 ConfigureConstant -1234              # every sample = value
//   sysbus.pdm0 ConfigureSine 8000 1000              # amplitude, frequency [Hz]
//   sysbus.pdm0 SetWavFile @path/to/file.wav false   # 16-bit PCM WAV, loop flag
//
// WAV constraints: RIFF/WAVE, PCM (format 1), 16-bit, mono or stereo. The
// WAV header sample rate is IGNORED — no resampling is performed; samples
// are consumed one WAV frame per PDM output frame at the rate configured
// by PDMCLKCTRL/RATIO. Mono WAV feeds both channels; stereo WAV maps
// L->left, R->right. When the file is exhausted the source emits silence
// unless loop=true.
//
// Timing (virtual time, deterministic): the PDM output sample rate is
// fs = PDM_CLK / RATIO, with PDM_CLK = 32 MHz / divisor (divisor from
// PDMCLKCTRL: 32/31/30/26/25/24). A buffer of SAMPLE.MAXCNT samples
// (per-frame count = MAXCNT in mono, MAXCNT/2 in stereo) completes after
// exactly divisor * RATIO * frames ticks of the 32 MHz base clock.
//
// Buffer sequence matches the pinned nrfx_pdm driver's expectations
// (third_party/hal_nordic/nrfx/drivers/src/nrfx_pdm.c): TASKS_START
// latches SAMPLE.PTR/MAXCNT and raises EVENTS_STARTED (software then sets
// the next buffer's PTR from the STARTED handler); when the buffer
// completes, samples are written to RAM, EVENTS_END is raised, the next
// buffer is latched and EVENTS_STARTED raised again. TASKS_STOP stops the
// stream and raises EVENTS_STOPPED (a partially-elapsed buffer is
// discarded — the pinned driver releases buffers without reading a
// partial-fill count; the PDM block has no AMOUNT register).
//
// GAINL/GAINR per spec: 0x00..0x50 in 0.5 dB steps, 0x28 = 0 dB; applied
// to injected samples with int16 saturation. MODE.EDGE=LeftRising swaps
// the L/R slot mapping in stereo (which edge is "left"); no effect on the
// injected data in mono.
//
// Not modelled: ACLK master clock source (MCLKCONFIG.SRC=ACLK logs
// UNSUPPORTED and behaves as PCLK32M), SUBSCRIBE_*/PUBLISH_* DPPI
// endpoints (UNIMPL-logged register file), the PDM bitstream itself.
//
// Faults (monitor-settable, default off):
//   FaultDroppedInterrupt — swallow the next rising edge of the IRQ line
//       (event registers still set; the line asserts again on the next
//       event recomputation).
//   FaultDmaError — EasyDMA sample writes are dropped (logged); events
//       still fire.
//
using System;
using System.IO;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sound
{
    public class NRF5340_PDM : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_PDM(IMachine machine) : base(machine)
        {
            bufferTimer = new LimitTimer(machine.ClockSource, BaseClockFrequency, this, "bufferTimer",
                limit: 1, direction: Direction.Descending, enabled: false,
                workMode: WorkMode.OneShot, eventEnabled: true, autoUpdate: true);
            bufferTimer.LimitReached += OnBufferComplete;
            DefineRegisters();
            // Reset() only runs when the machine starts; initialise the same
            // power-on state here so pre-start register reads (e.g. the
            // SVD-generated reset-value tests) see SVD values.
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            enabled = false;
            running = false;
            clockCtrl = 0x08400000;     // SVD reset: 32 MHz / 31
            clockDivisor = 31;
            ratio = 64;
            mono = false;               // SVD reset MODE=0: stereo
            edgeLeftRising = false;
            gainL = 0x28;
            gainR = 0x28;
            samplePtr = 0;
            sampleMaxCnt = 0;
            latchedPtr = 0;
            latchedMaxCnt = 0;
            frameIndex = 0;
            evStarted = evStopped = evEnd = false;
            inten = 0;
            pselClk = 0xFFFFFFFF;
            pselDin = 0xFFFFFFFF;
            mclkFromAclk = false;
            bufferTimer.Enabled = false;
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

        public void ConfigureSilence()
        {
            source = Source.Silence;
            this.Log(LogLevel.Info, "PCM source: silence");
        }

        public void ConfigureConstant(int value)
        {
            if(value < short.MinValue || value > short.MaxValue)
            {
                this.Log(LogLevel.Error, "Constant sample {0} outside int16 range", value);
                return;
            }
            source = Source.Constant;
            constantValue = (short)value;
            this.Log(LogLevel.Info, "PCM source: constant {0}", value);
        }

        public void ConfigureSine(uint amplitude, uint frequencyHz)
        {
            if(amplitude > (uint)short.MaxValue)
            {
                this.Log(LogLevel.Error, "Sine amplitude {0} outside int16 range", amplitude);
                return;
            }
            source = Source.Sine;
            sineAmplitude = amplitude;
            sineFrequency = frequencyHz;
            this.Log(LogLevel.Info, "PCM source: sine amplitude={0} frequency={1} Hz", amplitude, frequencyHz);
        }

        public void SetWavFile(string path, bool loop = false)
        {
            try
            {
                LoadWav(path);
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Error, "Failed to load WAV file '{0}': {1}", path, e.Message);
                return;
            }
            source = Source.Wav;
            wavLoop = loop;
            this.Log(LogLevel.Info, "PCM source: WAV '{0}' ({1} frames, {2}, loop={3})",
                path, wavLeft.Length, wavRight != null ? "stereo" : "mono", loop);
        }

        // Fault: swallow the next rising edge of the IRQ line.
        public bool FaultDroppedInterrupt { get; set; }

        // Fault: drop EasyDMA sample writes (events still fire).
        public bool FaultDmaError { get; set; }

        // ------------------------- registers -------------------------

        private void DefineRegisters()
        {
            Registers.TasksStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_START", writeCallback: (_, value) => { if(value) DoStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksStop.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_STOP", writeCallback: (_, value) => { if(value) DoStop(); })
                .WithReservedBits(1, 31);

            Registers.EventsStarted.Define(this)
                .WithFlag(0, name: "EVENTS_STARTED",
                    valueProviderCallback: _ => evStarted,
                    writeCallback: (_, value) => { evStarted = value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.EventsStopped.Define(this)
                .WithFlag(0, name: "EVENTS_STOPPED",
                    valueProviderCallback: _ => evStopped,
                    writeCallback: (_, value) => { evStopped = value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.EventsEnd.Define(this)
                .WithFlag(0, name: "EVENTS_END",
                    valueProviderCallback: _ => evEnd,
                    writeCallback: (_, value) => { evEnd = value; UpdateInterrupts(); })
                .WithReservedBits(1, 31);

            Registers.InterruptEnable.Define(this)
                .WithValueField(0, 3, name: "INTEN",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten = (uint)value; UpdateInterrupts(); })
                .WithReservedBits(3, 29);

            Registers.InterruptEnableSet.Define(this)
                .WithValueField(0, 3, name: "INTENSET",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten |= (uint)value; UpdateInterrupts(); })
                .WithReservedBits(3, 29);

            Registers.InterruptEnableClear.Define(this)
                .WithValueField(0, 3, name: "INTENCLR",
                    valueProviderCallback: _ => inten,
                    writeCallback: (_, value) => { inten &= ~(uint)value; UpdateInterrupts(); })
                .WithReservedBits(3, 29);

            Registers.Enable.Define(this)
                .WithFlag(0, name: "ENABLE",
                    valueProviderCallback: _ => enabled,
                    writeCallback: (_, value) => enabled = value)
                .WithReservedBits(1, 31);

            Registers.PdmClkCtrl.Define(this, 0x08400000)
                .WithValueField(0, 32, name: "FREQ",
                    valueProviderCallback: _ => clockCtrl,
                    writeCallback: (_, value) => SetClockControl((uint)value));

            Registers.Mode.Define(this)
                .WithFlag(0, name: "OPERATION",
                    valueProviderCallback: _ => mono,
                    writeCallback: (_, value) => mono = value)
                .WithFlag(1, name: "EDGE",
                    valueProviderCallback: _ => edgeLeftRising,
                    writeCallback: (_, value) => edgeLeftRising = value)
                .WithReservedBits(2, 30);

            Registers.GainL.Define(this, 0x28)
                .WithValueField(0, 7, name: "GAINL",
                    valueProviderCallback: _ => gainL,
                    writeCallback: (_, value) => gainL = ClampGain((uint)value, "GAINL"))
                .WithReservedBits(7, 25);

            Registers.GainR.Define(this, 0x28)
                .WithValueField(0, 7, name: "GAINR",
                    valueProviderCallback: _ => gainR,
                    writeCallback: (_, value) => gainR = ClampGain((uint)value, "GAINR"))
                .WithReservedBits(7, 25);

            Registers.Ratio.Define(this)
                .WithFlag(0, name: "RATIO",
                    valueProviderCallback: _ => ratio == 80,
                    writeCallback: (_, value) => ratio = value ? 80u : 64u)
                .WithReservedBits(1, 31);

            Registers.PselClk.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, name: "CLK",
                    valueProviderCallback: _ => pselClk,
                    writeCallback: (_, value) => pselClk = (uint)value);

            Registers.PselDin.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, name: "DIN",
                    valueProviderCallback: _ => pselDin,
                    writeCallback: (_, value) => pselDin = (uint)value);

            Registers.MclkConfig.Define(this)
                .WithFlag(0, name: "SRC",
                    valueProviderCallback: _ => mclkFromAclk,
                    writeCallback: (_, value) =>
                    {
                        mclkFromAclk = value;
                        if(value)
                        {
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what=MCLKCONFIG.SRC detail=ACLK not modelled, timing stays on PCLK32M");
                        }
                    })
                .WithReservedBits(1, 31);

            Registers.SamplePtr.Define(this)
                .WithValueField(0, 32, name: "SAMPLEPTR",
                    valueProviderCallback: _ => samplePtr,
                    writeCallback: (_, value) => samplePtr = (uint)value);

            Registers.SampleMaxCnt.Define(this)
                .WithValueField(0, 15, name: "BUFFSIZE",
                    valueProviderCallback: _ => sampleMaxCnt,
                    writeCallback: (_, value) => sampleMaxCnt = (uint)value)
                .WithReservedBits(15, 17);
        }

        // ------------------------- behaviour -------------------------

        private void DoStart()
        {
            if(!enabled)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what=TASKS_START detail=triggered while ENABLE=0, ignored");
                return;
            }
            if(running)
            {
                this.Log(LogLevel.Warning, "TASKS_START while already running, ignored");
                return;
            }
            running = true;
            frameIndex = 0;
            LatchBuffer();
        }

        private void DoStop()
        {
            if(!running)
            {
                // The pinned driver stops an idle peripheral during error
                // recovery; hardware just reports STOPPED.
                evStopped = true;
                UpdateInterrupts();
                return;
            }
            running = false;
            bufferTimer.Enabled = false;
            evStopped = true;
            UpdateInterrupts();
        }

        private void LatchBuffer()
        {
            latchedPtr = samplePtr;
            latchedMaxCnt = sampleMaxCnt;
            evStarted = true;
            UpdateInterrupts();

            if(latchedMaxCnt == 0)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what=SAMPLE.MAXCNT detail=buffer size 0, no transfer scheduled");
                return;
            }
            var frames = FramesPerBuffer(latchedMaxCnt);
            var ticks = (ulong)clockDivisor * ratio * frames;
            bufferTimer.Enabled = false;
            // autoUpdate: the Limit setter reloads the countdown, so re-arming
            // with an unchanged limit still waits a full buffer period.
            bufferTimer.Limit = ticks;
            bufferTimer.Enabled = true;
        }

        private void OnBufferComplete()
        {
            FillBuffer();
            evEnd = true;
            UpdateInterrupts();
            if(running)
            {
                LatchBuffer();
            }
        }

        private void FillBuffer()
        {
            var frames = FramesPerBuffer(latchedMaxCnt);
            var multiplierL = GainMultiplier(gainL);
            var multiplierR = GainMultiplier(gainR);

            for(var s = 0u; s < latchedMaxCnt; s++)
            {
                long frame = frameIndex + (mono ? s : s / 2);
                bool left;
                if(mono)
                {
                    left = true;
                }
                else
                {
                    left = (s % 2) == 0;
                    if(edgeLeftRising)
                    {
                        left = !left;
                    }
                }
                var raw = GenerateSample(left, frame);
                var scaled = ApplyGain(raw, left ? multiplierL : multiplierR);
                var address = latchedPtr + 2UL * s;
                if(FaultDmaError)
                {
                    if(s == 0)
                    {
                        this.Log(LogLevel.Warning, "FaultDmaError active: dropping {0}-sample DMA transfer to 0x{1:X8}", latchedMaxCnt, latchedPtr);
                    }
                }
                else
                {
                    sysbus.WriteWord(address, (ushort)scaled);
                }
            }
            frameIndex += frames;
        }

        private uint FramesPerBuffer(uint maxcnt)
        {
            if(mono)
            {
                return maxcnt;
            }
            if((maxcnt % 2) != 0)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what=SAMPLE.MAXCNT detail=odd buffer size {0} in stereo mode", maxcnt);
            }
            return (maxcnt + 1) / 2;
        }

        private short GenerateSample(bool left, long frame)
        {
            switch(source)
            {
            case Source.Silence:
                return 0;
            case Source.Constant:
                return constantValue;
            case Source.Sine:
            {
                var fs = BaseClockFrequency / (double)(clockDivisor * ratio);
                var value = sineAmplitude * Math.Sin(2.0 * Math.PI * sineFrequency * frame / fs);
                return (short)Math.Round(value, MidpointRounding.AwayFromZero);
            }
            case Source.Wav:
            {
                var samples = left || wavRight == null ? wavLeft : wavRight;
                if(samples.Length == 0)
                {
                    return 0;
                }
                var index = frame;
                if(wavLoop)
                {
                    index %= samples.Length;
                }
                else if(index >= samples.Length)
                {
                    return 0;
                }
                return samples[index];
            }
            default:
                return 0;
            }
        }

        private static short ApplyGain(short raw, double multiplier)
        {
            var scaled = (long)Math.Round(raw * multiplier, MidpointRounding.AwayFromZero);
            if(scaled > short.MaxValue)
            {
                scaled = short.MaxValue;
            }
            if(scaled < short.MinValue)
            {
                scaled = short.MinValue;
            }
            return (short)scaled;
        }

        private static double GainMultiplier(uint gain)
        {
            // 0x00..0x50 in 0.5 dB steps; 0x28 = 0 dB.
            var db = ((int)gain - 40) / 2.0;
            return Math.Pow(10.0, db / 20.0);
        }

        private uint ClampGain(uint value, string name)
        {
            if(value > 0x50)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what={0} detail=value 0x{1:X} above MaxGain, clamped to 0x50", name, value);
                return 0x50;
            }
            return value;
        }

        private void SetClockControl(uint value)
        {
            uint divisor;
            switch(value)
            {
            case 0x08000000: divisor = 32; break;
            case 0x08400000: divisor = 31; break;
            case 0x08800000: divisor = 30; break;
            case 0x09800000: divisor = 26; break;
            case 0x0A000000: divisor = 25; break;
            case 0x0A800000: divisor = 24; break;
            default:
                // Not one of the documented presets. The register is the PDM
                // clock as a fraction of the 32 MHz source, f = 32 MHz *
                // value / 2^32 (the presets above follow from it), and the
                // Zephyr driver computes its own value: the G1 firmware
                // writes 0x0A0A0000 for 1.28 MHz, 16 kHz at a ratio of 80.
                if(value == 0)
                {
                    this.Log(LogLevel.Warning, "UNSUPPORTED periph=PDM0 what=PDMCLKCTRL detail=value 0, keeping previous divisor {0}", clockDivisor);
                    clockCtrl = value;
                    return;
                }
                divisor = (uint)Math.Max(1UL, (1UL << 32) / value);
                break;
            }
            clockCtrl = value;
            clockDivisor = divisor;
        }

        private void LoadWav(string path)
        {
            var bytes = File.ReadAllBytes(path);
            if(bytes.Length < 44 ||
               bytes[0] != 'R' || bytes[1] != 'I' || bytes[2] != 'F' || bytes[3] != 'F' ||
               bytes[8] != 'W' || bytes[9] != 'A' || bytes[10] != 'V' || bytes[11] != 'E')
            {
                throw new ArgumentException("not a RIFF/WAVE file");
            }
            int channelCount = 0;
            int bitsPerSample = 0;
            int dataOffset = -1;
            int dataLength = 0;
            var pos = 12;
            while(pos + 8 <= bytes.Length)
            {
                var chunkId = System.Text.Encoding.ASCII.GetString(bytes, pos, 4);
                var chunkSize = BitConverter.ToInt32(bytes, pos + 4);
                if(chunkId == "fmt ")
                {
                    var format = BitConverter.ToInt16(bytes, pos + 8);
                    channelCount = BitConverter.ToInt16(bytes, pos + 10);
                    bitsPerSample = BitConverter.ToInt16(bytes, pos + 22);
                    if(format != 1)
                    {
                        throw new ArgumentException($"unsupported WAV format {format} (only PCM=1)");
                    }
                }
                else if(chunkId == "data")
                {
                    dataOffset = pos + 8;
                    dataLength = chunkSize;
                }
                pos += 8 + chunkSize + (chunkSize & 1);
            }
            if(dataOffset < 0 || channelCount == 0)
            {
                throw new ArgumentException("missing fmt/data chunk");
            }
            if(bitsPerSample != 16)
            {
                throw new ArgumentException($"unsupported bits per sample {bitsPerSample} (only 16)");
            }
            if(channelCount != 1 && channelCount != 2)
            {
                throw new ArgumentException($"unsupported channel count {channelCount} (only mono/stereo)");
            }
            if(dataOffset + dataLength > bytes.Length)
            {
                dataLength = bytes.Length - dataOffset;
            }
            var totalSamples = dataLength / 2;
            var frames = totalSamples / channelCount;
            wavLeft = new short[frames];
            wavRight = channelCount == 2 ? new short[frames] : null;
            for(var i = 0; i < frames; i++)
            {
                wavLeft[i] = BitConverter.ToInt16(bytes, dataOffset + 2 * channelCount * i);
                if(channelCount == 2)
                {
                    wavRight[i] = BitConverter.ToInt16(bytes, dataOffset + 2 * channelCount * i + 2);
                }
            }
        }

        private void UpdateInterrupts()
        {
            var pending = ((evStarted ? 1u : 0) & inten) != 0
                || ((evStopped ? 2u : 0) & inten) != 0
                || ((evEnd ? 4u : 0) & inten) != 0;
            if(pending && !IRQ.IsSet && FaultDroppedInterrupt)
            {
                FaultDroppedInterrupt = false;
                this.Log(LogLevel.Warning, "FaultDroppedInterrupt active: swallowing IRQ assertion");
                return;
            }
            IRQ.Set(pending);
        }

        private void LogUnimplemented(string dir, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph=PDM0 dir={0} width=4 offset=0x{1:X} value=0x{2:X8} pc=0x{3:X8}",
                dir, offset, value, pc);
        }

        private readonly LimitTimer bufferTimer;
        private readonly Dictionary<long, uint> unimplStore = new Dictionary<long, uint>();

        private bool enabled;
        private bool running;
        private uint clockCtrl;
        private uint clockDivisor = 31;
        private uint ratio = 64;
        private bool mono;
        private bool edgeLeftRising;
        private uint gainL = 0x28;
        private uint gainR = 0x28;
        private uint samplePtr;
        private uint sampleMaxCnt;
        private uint latchedPtr;
        private uint latchedMaxCnt;
        private long frameIndex;
        private bool evStarted, evStopped, evEnd;
        private uint inten;
        private uint pselClk = 0xFFFFFFFF;
        private uint pselDin = 0xFFFFFFFF;
        private bool mclkFromAclk;

        private Source source = Source.Silence;
        private short constantValue;
        private uint sineAmplitude;
        private uint sineFrequency;
        private short[] wavLeft = new short[0];
        private short[] wavRight;
        private bool wavLoop;

        private const long BaseClockFrequency = 32000000;

        private enum Source
        {
            Silence,
            Constant,
            Sine,
            Wav,
        }

        private enum Registers : long
        {
            TasksStart = 0x000,
            TasksStop = 0x004,
            // SUBSCRIBE_* 0x080/0x084, PUBLISH_* 0x180..0x188: DPPI endpoints,
            // not implemented — accesses fall through to the UNIMPL logger.
            EventsStarted = 0x100,
            EventsStopped = 0x104,
            EventsEnd = 0x108,
            InterruptEnable = 0x300,
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            Enable = 0x500,
            PdmClkCtrl = 0x504,
            Mode = 0x508,
            GainL = 0x518,
            GainR = 0x51C,
            Ratio = 0x520,
            PselClk = 0x540,
            PselDin = 0x544,
            MclkConfig = 0x54C,
            SamplePtr = 0x560,
            SampleMaxCnt = 0x564,
        }
    }
}
