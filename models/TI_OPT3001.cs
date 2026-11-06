//
// TI_OPT3001 — TI OPT3001 ambient light sensor, I2C.
//
// Modelled from:
//   - TI OPT3001 datasheet SBOS681C (register map §7.6: RESULT 0x00,
//     CONFIG 0x01 [reset 0xC810], LOW_LIMIT 0x02 [reset 0x0000],
//     HIGH_LIMIT 0x03 [reset 0xBFFF], MANUFACTURER_ID 0x7E = 0x5449,
//     DEVICE_ID 0x7F = 0x3001; lux = 0.01 * 2^E[3:0] * R[11:0])
//   - pinned Zephyr driver third_party/zephyr/drivers/sensor/opt3001/
//     (big-endian 16-bit register transactions: reg pointer byte, then
//     MSB, LSB — opt3001.c opt3001_reg_read()/opt3001_reg_write();
//     init checks both IDs then RMWs CONFIG mode to continuous).
//
// Conversion runs on virtual time (LimitTimer): CT=0 -> 100 ms,
// CT=1 -> 800 ms (datasheet CONFIG.CT). Continuous (M=10/11) reconverts
// periodically; single-shot (M=01) auto-returns to shutdown, M reads 01
// while converting (datasheet CONFIG.M). CRF sets at end of conversion,
// clears on CONFIG read or on CONFIG write whose M field is not 00
// (datasheet CONFIG.CRF). FH/FL compare against the limit registers with
// FC fault counting; in latched mode (L=1) they clear on CONFIG read.
//
// Injection (monitor-settable):
//   Lux (decimal) — e.g.  opt3001 Lux 1250.5   (encoding computed by the
//   model at the next conversion; auto full-scale RN=0xC picks the
//   smallest exponent that fits, manual RN forces E=RN and sets OVF on
//   overflow, ME masks the exponent field)
// Faults:
//   FaultWrongId          — MANUFACTURER_ID/DEVICE_ID read corrupted (0xFFFF)
//   FaultConversionDelay  — extra conversion latency in milliseconds
//                           (delayed-conversion fault; 0 = off)
//   FaultNak              — address NACK via the TWIM master
//                           (II2CDeviceFaultNak, see NRF5340_TWIM.cs)
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class TI_OPT3001 : II2CPeripheral, II2CDeviceFaultNak
    {
        public TI_OPT3001(IMachine machine)
        {
            conversionTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "conversion",
                1, Antmicro.Renode.Time.Direction.Ascending,
                enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            conversionTimer.LimitReached += OnConversionComplete;
            Reset();
        }

        public void Reset()
        {
            result = 0x0000;
            rangeNumber = 0xC;
            conversionTime800ms = true;
            mode = ModeShutdown;
            overflow = false;
            conversionReady = false;
            flagHigh = false;
            flagLow = false;
            latch = true;
            polarity = false;
            maskExponent = false;
            faultCount = 0;
            lowLimit = 0x0000;
            highLimit = 0xBFFF;
            highFaultEvents = 0;
            lowFaultEvents = 0;
            registerPointer = 0;
            writeBuffer0 = null;
            readByteIndex = 0;
            conversionTimer.Enabled = false;
        }

        // -------------------- I2C transport --------------------
        // The pinned driver frames every access as: write [reg] then read
        // MSB,LSB (repeated start), or write [reg, MSB, LSB]
        // (opt3001.c:19-46).

        public void Write(byte[] data)
        {
            foreach(var item in data)
            {
                WriteByte(item);
            }
        }

        public byte[] Read(int count = 1)
        {
            var value = ReadRegister(registerPointer);
            var bytes = new byte[count];
            for(var i = 0; i < count; i++)
            {
                // 16-bit registers stream MSB first; the pointer does not
                // auto-increment (datasheet §7.5.3) — longer reads wrap on
                // the same register.
                bytes[i] = readByteIndex == 0 ? (byte)(value >> 8) : (byte)value;
                readByteIndex = (readByteIndex + 1) % 2;
            }
            return bytes;
        }

        public void FinishTransmission()
        {
            pointerReceived = false;
            writeBuffer0 = null;
            readByteIndex = 0;
        }

        // -------------------- injection & faults --------------------

        public decimal Lux { get; set; }

        public bool FaultWrongId { get; set; }
        public bool FaultNak { get; set; }
        // Extra conversion latency in milliseconds, applied to every started
        // conversion while non-zero.
        public uint FaultConversionDelay { get; set; }

        // observability helpers for tests
        public ushort ResultRegister => result;
        public bool ConversionReady => conversionReady;

        // -------------------- register model --------------------

        private void WriteByte(byte value)
        {
            if(!pointerReceived)
            {
                registerPointer = value;
                pointerReceived = true;
                writeBuffer0 = null;
                readByteIndex = 0;
                return;
            }
            if(writeBuffer0 == null)
            {
                writeBuffer0 = value;
                return;
            }
            WriteRegister(registerPointer, (ushort)((writeBuffer0.Value << 8) | value));
            writeBuffer0 = null;
        }

        private ushort ReadRegister(byte register)
        {
            switch(register)
            {
            case RegisterResult:
                return result;
            case RegisterConfig:
                var value = ConfigValue;
                // CRF (and latched FH/FL) clear when CONFIG is read
                // (datasheet Table 10, CRF/FH/FL fields).
                conversionReady = false;
                if(latch)
                {
                    flagHigh = false;
                    flagLow = false;
                }
                return value;
            case RegisterLowLimit:
                return lowLimit;
            case RegisterHighLimit:
                return highLimit;
            case RegisterManufacturerId:
                return FaultWrongId ? CorruptedId : ManufacturerId;
            case RegisterDeviceId:
                return FaultWrongId ? CorruptedId : DeviceId;
            default:
                this.Log(LogLevel.Warning, "UNIMPL periph=OPT3001 dir=R width=2 offset=0x{0:X2} value=0x0000 pc=0x0", register);
                return 0;
            }
        }

        private void WriteRegister(byte register, ushort value)
        {
            switch(register)
            {
            case RegisterConfig:
                var newMode = (byte)((value >> 9) & 0x3);
                rangeNumber = (byte)((value >> 12) & 0xF);
                conversionTime800ms = ((value >> 11) & 1) != 0;
                latch = ((value >> 4) & 1) != 0;
                polarity = ((value >> 3) & 1) != 0;
                maskExponent = ((value >> 2) & 1) != 0;
                faultCount = (byte)(value & 0x3);
                // Writing the configuration register aborts an in-progress
                // conversion; a write selecting shutdown does not clear CRF
                // (datasheet §7.6.1.1.2).
                conversionTimer.Enabled = false;
                if(newMode != ModeShutdown)
                {
                    conversionReady = false;
                    if(latch)
                    {
                        flagHigh = false;
                        flagLow = false;
                    }
                }
                mode = newMode;
                if(mode != ModeShutdown)
                {
                    StartConversion();
                }
                break;
            case RegisterLowLimit:
                lowLimit = value;
                break;
            case RegisterHighLimit:
                highLimit = value;
                break;
            case RegisterResult:
            case RegisterManufacturerId:
            case RegisterDeviceId:
                this.Log(LogLevel.Warning, "Write to read-only register 0x{0:X2}, ignoring", register);
                break;
            default:
                this.Log(LogLevel.Warning, "UNIMPL periph=OPT3001 dir=W width=2 offset=0x{0:X2} value=0x{1:X4} pc=0x0", register, value);
                break;
            }
        }

        private ushort ConfigValue =>
            (ushort)((rangeNumber << 12)
                | ((conversionTime800ms ? 1 : 0) << 11)
                | (mode << 9)
                | ((overflow ? 1 : 0) << 8)
                | ((conversionReady ? 1 : 0) << 7)
                | ((flagHigh ? 1 : 0) << 6)
                | ((flagLow ? 1 : 0) << 5)
                | ((latch ? 1 : 0) << 4)
                | ((polarity ? 1 : 0) << 3)
                | ((maskExponent ? 1 : 0) << 2)
                | faultCount);

        // -------------------- conversion engine --------------------

        private void StartConversion()
        {
            var milliseconds = (conversionTime800ms ? 800UL : 100UL) + FaultConversionDelay;
            conversionTimer.Enabled = false;
            conversionTimer.Limit = milliseconds;
            conversionTimer.Value = 0;
            conversionTimer.Enabled = true;
        }

        private void OnConversionComplete()
        {
            conversionTimer.Enabled = false;
            result = EncodeLux(Lux, out overflow);
            conversionReady = true;
            EvaluateLimits();
            if(mode == ModeSingleShot)
            {
                // M auto-clears to shutdown after a single-shot conversion
                // (datasheet §7.6.1.1.2 M field).
                mode = ModeShutdown;
            }
            else if(mode == ModeContinuousA || mode == ModeContinuousB)
            {
                StartConversion();
            }
        }

        private ushort EncodeLux(decimal lux, out bool overflowed)
        {
            overflowed = false;
            if(lux < 0)
            {
                lux = 0;
            }
            int exponent;
            if(rangeNumber == RangeAutomatic)
            {
                // automatic full-scale: smallest exponent whose range fits
                // (datasheet §7.4.1.2)
                exponent = 0;
                while(exponent < MaximumExponent && lux > FullScale(exponent))
                {
                    exponent++;
                }
            }
            else
            {
                exponent = Math.Min((int)rangeNumber, MaximumExponent);
            }
            var lsbWeight = 0.01m * (1 << exponent);
            var mantissa = (long)Math.Round(lux / lsbWeight);
            if(mantissa > 0xFFF)
            {
                mantissa = 0xFFF;
                overflowed = true;
            }
            var reportedExponent = (rangeNumber != RangeAutomatic && maskExponent) ? 0 : exponent;
            return (ushort)((reportedExponent << 12) | (int)mantissa);
        }

        private static decimal FullScale(int exponent) => 40.95m * (1 << exponent);

        private void EvaluateLimits()
        {
            var requiredFaults = 1 << faultCount; // 1, 2, 4, 8
            var resultLux = DecodeLux(result);
            if(resultLux > DecodeLux(highLimit))
            {
                highFaultEvents++;
            }
            else
            {
                highFaultEvents = 0;
                if(!latch)
                {
                    flagHigh = false;
                }
            }
            if(resultLux < DecodeLux(lowLimit))
            {
                lowFaultEvents++;
            }
            else
            {
                lowFaultEvents = 0;
                if(!latch)
                {
                    flagLow = false;
                }
            }
            if(highFaultEvents >= requiredFaults)
            {
                flagHigh = true;
            }
            if(lowFaultEvents >= requiredFaults)
            {
                flagLow = true;
            }
        }

        private static decimal DecodeLux(ushort encoded) => 0.01m * (1 << (encoded >> 12)) * (encoded & 0xFFF);

        private ushort result;
        private ushort lowLimit;
        private ushort highLimit;
        private byte rangeNumber;
        private byte mode;
        private byte faultCount;
        private bool conversionTime800ms;
        private bool overflow;
        private bool conversionReady;
        private bool flagHigh;
        private bool flagLow;
        private bool latch;
        private bool polarity;
        private bool maskExponent;
        private int highFaultEvents;
        private int lowFaultEvents;

        private byte registerPointer;
        private bool pointerReceived;
        private byte? writeBuffer0;
        private int readByteIndex;

        private readonly LimitTimer conversionTimer;

        private const byte RegisterResult = 0x00;
        private const byte RegisterConfig = 0x01;
        private const byte RegisterLowLimit = 0x02;
        private const byte RegisterHighLimit = 0x03;
        private const byte RegisterManufacturerId = 0x7E;
        private const byte RegisterDeviceId = 0x7F;

        private const ushort ManufacturerId = 0x5449;
        private const ushort DeviceId = 0x3001;
        private const ushort CorruptedId = 0xFFFF;

        private const byte ModeShutdown = 0;
        private const byte ModeSingleShot = 1;
        private const byte ModeContinuousA = 2;
        private const byte ModeContinuousB = 3;

        private const byte RangeAutomatic = 0xC;
        private const int MaximumExponent = 0xB;
        private const long TimerFrequency = 1000; // 1 kHz -> limits in ms
    }
}
