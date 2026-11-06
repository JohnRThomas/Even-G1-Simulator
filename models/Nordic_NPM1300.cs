//
// Nordic_NPM1300 — Nordic nPM1300 PMIC (charger/regulators/ADC/GPIO/
// watchdog/ship-mode), I2C.
//
// Register map from the nPM1300 product specification, cross-checked
// against Nordic's npmx ADK header (github.com/nordicsemi/npmx,
// adk/npm1300.h — offsets and reset values quoted in comments) and the
// pinned NCS v2.5.1 Zephyr drivers, which define the exact I2C framing
// and the register subset exercised:
//   third_party/zephyr/drivers/mfd/mfd_npm1300.c        (framing: write =
//     [page, offset, data...]; read = write [page, offset] + repeated-
//     start read, offset auto-increments — mfd_npm1300_reg_read_burst())
//   third_party/zephyr/drivers/sensor/npm1300_charger/npm1300_charger.c
//   third_party/zephyr/drivers/regulator/regulator_npm1300.c
//   third_party/zephyr/drivers/watchdog/wdt_npm1300.c
//   third_party/zephyr/drivers/gpio/gpio_npm1300.c
//
// Pages implemented: MAIN 0x00 (events/inten + TASKSWRESET), VBUSIN 0x02,
// BCHARGER 0x03, BUCK 0x04, ADC 0x05, GPIOS 0x06, TIMER 0x07 (watchdog),
// LDSW 0x08, SHPHLD/ship 0x0B. Unlisted pages/offsets log UNIMPL and act
// as a stored register file (write-read-back) so RMW sequences progress.
//
// Charger state machine (BCHGCHARGESTATUS, npmx BCHARGER_BCHGCHARGESTATUS
// bits): derived from injected battery state on every relevant change:
//   off -> trickle (VBAT < Vtrickle) -> constant current (VBAT < Vterm)
//   -> constant voltage (VBAT >= Vterm, |IBAT| > Iterm) -> completed;
//   error states via InjectChargerError. Completion and error raise the
//   MAIN EVENTSBCHARGER1 events the pinned mfd driver dispatches
//   (mfd_npm1300.c event_reg[]: offset 0x0A masks 0x10/0x20).
//
// Injection (monitor-settable):
//   BatteryVoltageMilliVolts, BatteryCurrentMilliAmps (positive =
//   discharging, negative = charging), BatteryTemperatureCelsius (NTC),
//   VsysMilliVolts, VbusVoltageMilliVolts, VbusConnected,
//   BatteryDetected, InjectChargerError(reason, sensor).
// Thermistor defaults per docs/board-map.md: 100 kOhm / beta 4250
//   (ThermistorOhms / ThermistorBeta properties).
// Board defaults (4.4 V term, 4.1 V warm, 40 mA charge, 1 A discharge,
//   500 mA VBUS) are programmed by the pinned drivers at init; register
//   reset values follow the hardware (npmx ADK ResetValue).
//
// Observability: BuckEnabled(n)/LdswEnabled(n), ChargingEnabled,
// ChargerStatus, WatchdogExpired, ShipModeActive, HibernateRequested.
// Ship mode: after TASKENTERSHIPMODE/TASKENTERHIBERNATE the device stops
// ACKing I2C (as on hardware, where the ship controller powers the
// interface down) until VbusConnected is asserted.
//
// IRQ: optional GPIO output (unconnected by default — board routing is
// unspecified); asserted while any MAIN event with its interrupt enable
// bit set is pending.
//
// Faults: FaultNak (address NACK via II2CDeviceFaultNak).
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
    public class Nordic_NPM1300 : II2CPeripheral, II2CDeviceFaultNak
    {
        public Nordic_NPM1300(IMachine machine)
        {
            IRQ = new GPIO();
            this.machine = machine;
            watchdogTimer = new LimitTimer(machine.ClockSource, WatchdogTimerFrequency, this, "watchdog",
                1, Antmicro.Renode.Time.Direction.Ascending,
                enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            watchdogTimer.LimitReached += OnTimerExpired;
            Reset();
        }

        public void Reset()
        {
            storedRegisters.Clear();
            eventsPending = new byte[EventGroupCount];
            eventsEnabled = new byte[EventGroupCount];
            transferState = TransferState.Idle;
            chargingEnabled = false;
            rechargeDisabled = false;
            chargerErrorReason = 0;
            chargerErrorSensor = 0;
            chargerCompletedLatch = false;
            vbusCurrentLimit = 0x00;      // npmx VBUSIN_VBUSINILIM0 reset 0x00 (500 mA)
            vbusSuspended = false;
            chargeCurrentMsb = 0x08;      // npmx BCHARGER_BCHGISETMSB reset 0x08
            chargeCurrentLsb = 0x00;
            dischargeCurrentMsb = 0xCF;   // npmx BCHARGER_BCHGISETDISCHARGEMSB reset 0xCF
            dischargeCurrentLsb = 0x00;
            terminationVoltage = 0x02;    // npmx BCHARGER_BCHGVTERM reset 0x02 (3.60 V)
            terminationVoltageWarm = 0x02;
            trickleSelect = 0x00;         // 2.9 V
            terminationCurrentSelect = 0x00; // 10 %
            buckEnabled = new[] { true, false }; // BUCK1 typically powers the host; BUCK2 off
            buckPwm = new bool[2];
            ldswEnabled = new bool[2];
            timerRunning = false;
            timerMode = 0;
            timerPrescalerFast = false;
            timerTargetLatched = 0;
            timerTargetShadow = 0;
            watchdogTimer.Enabled = false;
            WatchdogExpired = false;
            ShipModeActive = false;
            HibernateRequested = false;
            gpioInputs = new bool[GpioPinCount];
            UpdateInterrupt();
        }

        // -------------------- I2C transport --------------------
        // mfd_npm1300.c: write = {base, offset, data...};
        // read = i2c_write_read(dev, {base, offset}, 2, data, len)

        public void Write(byte[] data)
        {
            var index = 0;
            if(transferState == TransferState.Idle && index < data.Length)
            {
                currentPage = data[index++];
                transferState = TransferState.PageReceived;
            }
            if(transferState == TransferState.PageReceived && index < data.Length)
            {
                currentOffset = data[index++];
                transferState = TransferState.OffsetReceived;
            }
            for(; index < data.Length; index++)
            {
                transferState = TransferState.Data;
                WriteRegister(currentPage, currentOffset, data[index]);
                currentOffset++;
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadRegister(currentPage, currentOffset);
                currentOffset++;
            }
            return result;
        }

        public void FinishTransmission()
        {
            transferState = TransferState.Idle;
        }

        // -------------------- injection & observability --------------------

        public GPIO IRQ { get; }

        public uint BatteryVoltageMilliVolts { get; set; } = 3800;
        // positive = discharging, negative = charging (sign convention of
        // the reported IBAT full-scale in npm1300_charger.c calc_current())
        public int BatteryCurrentMilliAmps { get; set; }
        public decimal BatteryTemperatureCelsius { get; set; } = 25m;
        public uint VsysMilliVolts { get; set; } = 3800;
        public uint VbusVoltageMilliVolts { get; set; } = 5000;
        public bool BatteryDetected
        {
            get => batteryDetected;
            set
            {
                if(batteryDetected == value)
                {
                    return;
                }
                batteryDetected = value;
                // EVENTSBCHARGER2: bit0 detected, bit1 removed (mfd_npm1300.c event_reg[])
                RaiseEvent(EventGroupCharger2, value ? (byte)0x01 : (byte)0x02);
            }
        }
        public bool VbusConnected
        {
            get => vbusConnected;
            set
            {
                if(vbusConnected == value)
                {
                    return;
                }
                vbusConnected = value;
                if(value)
                {
                    // VBUS wakes the device from ship/hibernate mode
                    ShipModeActive = false;
                }
                // EVENTSVBUSIN0: bit0 detected, bit1 removed (mfd_npm1300.c event_reg[])
                RaiseEvent(EventGroupVbus0, value ? (byte)0x01 : (byte)0x02);
                UpdateChargerEvents();
            }
        }

        public uint ThermistorOhms { get; set; } = 100000; // board map: 100 kOhm
        public uint ThermistorBeta { get; set; } = 4250;   // board map: B4250

        // Fault injection plus ship-mode behaviour: while in ship/hibernate
        // mode the device does not ACK its address (interface unpowered).
        public bool FaultNak
        {
            get => faultNakInjected || ShipModeActive;
            set => faultNakInjected = value;
        }

        public bool ChargingEnabled => chargingEnabled;
        public byte ChargerStatus => ComputeChargeStatus();
        public bool WatchdogExpired { get; private set; }

        // ---- host power-cycle on watchdog expiry (nPM1300 TIMER mode RESET) ----
        // On real silicon TIMERCONFIG mode 0x02 (RESET) makes an expired watchdog
        // POWER-CYCLE every rail, which resets the SoC. The model previously only
        // logged that fact, which made the single reset mechanism a wedged payload
        // cannot defeat UNTESTABLE in emulation (see docs/g1-reset-parity.md).
        //
        // Opt-in, default OFF, deliberately: g1.resc boots stock firmware that arms
        // this watchdog for 30 s in RESET mode, so switching it on by default would
        // start power-cycling every existing long-running suite. Tests that want the
        // hardware behaviour set it explicitly:
        //     sysbus.twim1.npm1300 ResetOnWatchdogExpiry true
        // WatchdogResetCount is real state, incremented only by an actual expiry in
        // RESET mode, so a test asserting on it cannot pass vacuously.
        public bool ResetOnWatchdogExpiry { get; set; }
        public int WatchdogResetCount { get; private set; }
        public bool ShipModeActive { get; private set; }
        public bool HibernateRequested { get; private set; }

        public bool BuckEnabled(int index) => buckEnabled[CheckRailIndex(index)];
        public bool LdswEnabled(int index) => ldswEnabled[CheckRailIndex(index)];

        public void InjectChargerError(byte reason, byte sensor)
        {
            chargerErrorReason = reason;
            chargerErrorSensor = sensor;
            if(reason != 0)
            {
                // EVENTSBCHARGER1 bit5: error (mfd_npm1300.c NPM1300_EVENT_CHG_ERROR)
                RaiseEvent(EventGroupCharger1, 0x20);
            }
        }

        // -------------------- register dispatch --------------------

        private byte ReadRegister(byte page, byte offset)
        {
            switch(page)
            {
            case PageMain:
                return ReadMain(offset);
            case PageVbus:
                return ReadVbus(offset);
            case PageCharger:
                return ReadCharger(offset);
            case PageBuck:
                return ReadBuck(offset);
            case PageAdc:
                return ReadAdc(offset);
            case PageGpio:
                return ReadGpio(offset);
            case PageTimer:
                return ReadTimer(offset);
            case PageLdsw:
                return ReadLdsw(offset);
            case PageShip:
                return ReadShip(offset);
            default:
                return ReadStored(page, offset);
            }
        }

        private void WriteRegister(byte page, byte offset, byte value)
        {
            switch(page)
            {
            case PageMain:
                WriteMain(offset, value);
                break;
            case PageVbus:
                WriteVbus(offset, value);
                break;
            case PageCharger:
                WriteCharger(offset, value);
                break;
            case PageBuck:
                WriteBuck(offset, value);
                break;
            case PageAdc:
                WriteAdc(offset, value);
                break;
            case PageGpio:
                WriteStored(page, offset, value); // config only; STATUS is read-only
                break;
            case PageTimer:
                WriteTimer(offset, value);
                break;
            case PageLdsw:
                WriteLdsw(offset, value);
                break;
            case PageShip:
                WriteShip(offset, value);
                break;
            default:
                WriteStored(page, offset, value);
                break;
            }
        }

        // -------------------- MAIN (0x00) --------------------
        // npmx MAIN: TASKSWRESET 0x01; event groups of four registers
        // (EVENTS*SET/CLR, INTEN*SET/CLR) at 0x02..0x25 with GPIO at 0x22.

        private int EventGroupIndex(byte offset, out int registerInGroup)
        {
            registerInGroup = -1;
            if(offset >= 0x02 && offset <= 0x1D)
            {
                registerInGroup = (offset - 0x02) % 4;
                return (offset - 0x02) / 4;
            }
            if(offset >= 0x22 && offset <= 0x25)
            {
                registerInGroup = (offset - 0x22) % 4;
                return EventGroupGpio;
            }
            return -1;
        }

        private byte ReadMain(byte offset)
        {
            var group = EventGroupIndex(offset, out var register);
            if(group >= 0)
            {
                switch(register)
                {
                case 0:
                case 1:
                    return eventsPending[group];
                default:
                    return eventsEnabled[group];
                }
            }
            return ReadStored(PageMain, offset);
        }

        private void WriteMain(byte offset, byte value)
        {
            if(offset == MainTaskSwReset)
            {
                if(value != 0)
                {
                    this.Log(LogLevel.Info, "TASKSWRESET: full register reset");
                    Reset();
                }
                return;
            }
            var group = EventGroupIndex(offset, out var register);
            if(group >= 0)
            {
                switch(register)
                {
                case 0: // EVENTS*SET — allows software/test injection
                    eventsPending[group] |= value;
                    break;
                case 1: // EVENTS*CLR — write 1 to clear
                    eventsPending[group] &= (byte)~value;
                    break;
                case 2: // INTEN*SET
                    eventsEnabled[group] |= value;
                    break;
                case 3: // INTEN*CLR
                    eventsEnabled[group] &= (byte)~value;
                    break;
                }
                UpdateInterrupt();
                return;
            }
            WriteStored(PageMain, offset, value);
        }

        private void RaiseEvent(int group, byte mask)
        {
            eventsPending[group] |= mask;
            UpdateInterrupt();
        }

        private void UpdateInterrupt()
        {
            var pending = false;
            for(var i = 0; i < EventGroupCount; i++)
            {
                pending |= (eventsPending[i] & eventsEnabled[i]) != 0;
            }
            IRQ.Set(pending);
        }

        // -------------------- VBUSIN (0x02) --------------------

        private byte ReadVbus(byte offset)
        {
            switch(offset)
            {
            case VbusIlim0:
                return vbusCurrentLimit;
            case VbusSuspendReg:
                return (byte)(vbusSuspended ? 1 : 0);
            case VbusUsbcDetectStatus:
                return 0; // no CC resistors modelled
            case VbusStatus:
                // npmx VBUSIN_VBUSINSTATUS: bit0 VBUSINPRESENT,
                // bit5 VBUSINVBUSOUTACTIVE
                return (byte)((vbusConnected ? 0x01 : 0x00) | (vbusConnected && !vbusSuspended ? 0x20 : 0x00));
            default:
                return ReadStored(PageVbus, offset);
            }
        }

        private void WriteVbus(byte offset, byte value)
        {
            switch(offset)
            {
            case VbusTaskUpdateIlimSw:
                // applies VBUSINILIM0 to the current limiter — no separate
                // shadow modelled; accept the task
                break;
            case VbusIlim0:
                vbusCurrentLimit = (byte)(value & 0x0F);
                break;
            case VbusSuspendReg:
                vbusSuspended = (value & 1) != 0;
                break;
            default:
                WriteStored(PageVbus, offset, value);
                break;
            }
        }

        // -------------------- BCHARGER (0x03) --------------------

        private byte ReadCharger(byte offset)
        {
            switch(offset)
            {
            case ChargerEnableSet:
            case ChargerEnableClr:
                // set/clr pairs read back the current enable state
                // (npm1300_charger.c attr_get reads CHGR_OFFSET_EN_SET)
                return (byte)(chargingEnabled ? 1 : 0);
            case ChargerDisableSet:
            case ChargerDisableClr:
                return (byte)(rechargeDisabled ? 1 : 0);
            case ChargerIsetMsb:
                return chargeCurrentMsb;
            case ChargerIsetLsb:
                return chargeCurrentLsb;
            case ChargerIsetDischargeMsb:
                return dischargeCurrentMsb;
            case ChargerIsetDischargeLsb:
                return dischargeCurrentLsb;
            case ChargerVterm:
                return terminationVoltage;
            case ChargerVtermR:
                return terminationVoltageWarm;
            case ChargerTrickleSel:
                return trickleSelect;
            case ChargerItermSel:
                return terminationCurrentSelect;
            case ChargerIlimStatus:
                return 0;
            case ChargerNtcStatus:
                return ComputeNtcStatus();
            case ChargerDieTempStatus:
                return 0; // die temperature below thresholds
            case ChargerChargeStatus:
                return ComputeChargeStatus();
            case ChargerErrReason:
                return chargerErrorReason;
            case ChargerErrSensor:
                return chargerErrorSensor;
            default:
                return ReadStored(PageCharger, offset);
            }
        }

        private void WriteCharger(byte offset, byte value)
        {
            switch(offset)
            {
            case ChargerTaskReleaseErr:
                // releases the charger FSM from its error state
                chargerErrorReason = 0;
                chargerErrorSensor = 0;
                break;
            case ChargerTaskClearChgErr:
                chargerErrorReason = 0;
                chargerErrorSensor = 0;
                break;
            case ChargerTaskClearSafetyTimer:
                break;
            case ChargerEnableSet:
                if((value & 1) != 0)
                {
                    chargingEnabled = true;
                    chargerCompletedLatch = false;
                    UpdateChargerEvents();
                }
                break;
            case ChargerEnableClr:
                if((value & 1) != 0)
                {
                    chargingEnabled = false;
                }
                break;
            case ChargerDisableSet:
                if((value & 1) != 0)
                {
                    rechargeDisabled = true;
                }
                break;
            case ChargerDisableClr:
                if((value & 1) != 0)
                {
                    rechargeDisabled = false;
                }
                break;
            case ChargerIsetMsb:
                chargeCurrentMsb = value;
                break;
            case ChargerIsetLsb:
                chargeCurrentLsb = (byte)(value & 1);
                break;
            case ChargerIsetDischargeMsb:
                dischargeCurrentMsb = value;
                break;
            case ChargerIsetDischargeLsb:
                dischargeCurrentLsb = (byte)(value & 1);
                break;
            case ChargerVterm:
                terminationVoltage = (byte)(value & 0x0F);
                break;
            case ChargerVtermR:
                terminationVoltageWarm = (byte)(value & 0x0F);
                break;
            case ChargerTrickleSel:
                trickleSelect = (byte)(value & 1);
                break;
            case ChargerItermSel:
                terminationCurrentSelect = (byte)(value & 1);
                break;
            default:
                WriteStored(PageCharger, offset, value); // NTC/die-temp thresholds, config
                break;
            }
        }

        // charger configuration decoding, inverses of the linear ranges in
        // npm1300_charger.c
        private uint TerminationMicroVolts(byte code) =>
            code <= 3 ? 3500000u + code * 50000u : 4000000u + (uint)(code - 4) * 50000u;

        private uint ChargeCurrentMicroAmps => 32000u + (uint)(((chargeCurrentMsb << 1) | chargeCurrentLsb) - 16) * 2000u;
        private uint DischargeLimitMicroAmps => 268090u + (uint)(((dischargeCurrentMsb << 1) | dischargeCurrentLsb) - 83) * 3230u;
        private uint TrickleThresholdMicroVolts => trickleSelect == 0 ? 2900000u : 2500000u;
        private uint TerminationCurrentMicroAmps =>
            ChargeCurrentMicroAmps * (terminationCurrentSelect == 0 ? 10u : 20u) / 100u;

        private byte ComputeChargeStatus()
        {
            // npmx BCHARGER_BCHGCHARGESTATUS bit assignment:
            // 0 BATTERYDETECTED, 1 COMPLETED, 2 TRICKLECHARGE,
            // 3 CONSTANTCURRENT, 4 CONSTANTVOLTAGE, 5 RECHARGE,
            // 6 DIETEMPHIGHCHGPAUSED, 7 SUPPLEMENTACTIVE
            byte status = 0;
            if(batteryDetected)
            {
                status |= 0x01;
            }
            if(chargerErrorReason != 0)
            {
                return status; // FSM parked in error: no charge phase bits
            }
            if(!chargingEnabled || !vbusConnected || !batteryDetected)
            {
                return status;
            }
            var vbatMicroVolts = BatteryVoltageMilliVolts * 1000u;
            var vterm = TerminationMicroVolts(terminationVoltage);
            if(chargerCompletedLatch)
            {
                status |= 0x02; // COMPLETED
            }
            else if(vbatMicroVolts < TrickleThresholdMicroVolts)
            {
                status |= 0x04; // TRICKLECHARGE
            }
            else if(vbatMicroVolts < vterm)
            {
                status |= 0x08; // CONSTANTCURRENT
            }
            else
            {
                var chargeMicroAmps = (uint)Math.Abs(BatteryCurrentMilliAmps) * 1000u;
                if(BatteryCurrentMilliAmps < 0 && chargeMicroAmps > TerminationCurrentMicroAmps)
                {
                    status |= 0x10; // CONSTANTVOLTAGE
                }
                else
                {
                    chargerCompletedLatch = true;
                    status |= 0x02; // COMPLETED
                    if(!chargerCompletedEventSent)
                    {
                        chargerCompletedEventSent = true;
                        // EVENTSBCHARGER1 bit4: charge completed (mfd_npm1300.c)
                        RaiseEvent(EventGroupCharger1, 0x10);
                    }
                }
            }
            return status;
        }

        private void UpdateChargerEvents()
        {
            // recompute; completion raises its event inside
            // ComputeChargeStatus when first latched
            var status = ComputeChargeStatus();
            if((status & 0x02) == 0)
            {
                chargerCompletedEventSent = false;
            }
        }

        private byte ComputeNtcStatus()
        {
            // bit0 COLD, bit1 COOL, bit2 WARM, bit3 HOT; thresholds are in
            // the stored NTCCOLD..NTCHOT registers, codes as written by
            // npm1300_charger.c set_ntc_thresholds()
            var code = NtcCode();
            var cold = StoredThreshold(ChargerNtcColdMsb);
            var cool = StoredThreshold(ChargerNtcCoolMsb);
            var warm = StoredThreshold(ChargerNtcWarmMsb);
            var hot = StoredThreshold(ChargerNtcHotMsb);
            // NTC code decreases with increasing temperature
            if(cold != 0 && code >= cold)
            {
                return 0x01;
            }
            if(hot != 0 && code <= hot)
            {
                return 0x08;
            }
            if(warm != 0 && code <= warm)
            {
                return 0x04;
            }
            if(cool != 0 && code >= cool)
            {
                return 0x02;
            }
            return 0x00;
        }

        private ushort StoredThreshold(byte msbOffset)
        {
            var msb = ReadStored(PageCharger, msbOffset, quiet: true);
            var lsb = ReadStored(PageCharger, (byte)(msbOffset + 1), quiet: true);
            return (ushort)((msb << 2) | (lsb & 0x3));
        }

        // -------------------- BUCK (0x04) --------------------

        private byte ReadBuck(byte offset)
        {
            switch(offset)
            {
            case BuckStatusReg:
                // npmx BUCK_BUCKSTATUS: per-buck MODE[1:0] + PWRGOOD + PWMOK
                byte status = 0;
                if(buckEnabled[0])
                {
                    status |= (byte)((buckPwm[0] ? 2 : 1) | 0x04);
                }
                if(buckEnabled[1])
                {
                    status |= (byte)(((buckPwm[1] ? 2 : 1) << 4) | 0x40);
                }
                return status;
            case Buck1VoutStatus:
            case Buck2VoutStatus:
                // VSET-pin-derived voltage; report the programmed normal-mode
                // voltage if set, else a 3.3 V code (23 = 1.0 + 23*0.1)
                var normal = ReadStored(PageBuck, offset == Buck1VoutStatus ? Buck1NormVout : Buck2NormVout, quiet: true);
                return normal != 0 ? normal : (byte)23;
            default:
                return ReadStored(PageBuck, offset);
            }
        }

        private void WriteBuck(byte offset, byte value)
        {
            switch(offset)
            {
            case Buck1EnaSet:
                if((value & 1) != 0) { buckEnabled[0] = true; }
                break;
            case Buck1EnaClr:
                if((value & 1) != 0) { buckEnabled[0] = false; }
                break;
            case Buck2EnaSet:
                if((value & 1) != 0) { buckEnabled[1] = true; }
                break;
            case Buck2EnaClr:
                if((value & 1) != 0) { buckEnabled[1] = false; }
                break;
            case Buck1PwmSet:
                if((value & 1) != 0) { buckPwm[0] = true; }
                break;
            case Buck1PwmClr:
                if((value & 1) != 0) { buckPwm[0] = false; }
                break;
            case Buck2PwmSet:
                if((value & 1) != 0) { buckPwm[1] = true; }
                break;
            case Buck2PwmClr:
                if((value & 1) != 0) { buckPwm[1] = false; }
                break;
            default:
                WriteStored(PageBuck, offset, value); // VOUT/CTRL/SWCTRL config
                break;
            }
        }

        // -------------------- ADC (0x05) --------------------
        // Results are computed from the injected battery state; measurement
        // tasks are accepted (results are always "fresh", deterministic in
        // virtual time).

        private byte ReadAdc(byte offset)
        {
            switch(offset)
            {
            case AdcIbatMeasStatus:
                return ComputeIbatStatus();
            case AdcVbatResultMsb:
                return (byte)(VbatCode >> 2);
            case AdcNtcResultMsb:
                return (byte)(NtcCode() >> 2);
            case AdcTempResultMsb:
                return (byte)(DieTempCode >> 2);
            case AdcVsysResultMsb:
                return (byte)(VsysCode >> 2);
            case AdcGp0ResultLsbs:
                // bits[1:0] VBAT, [3:2] NTC, [5:4] DIE, [7:6] VSYS
                return (byte)((VbatCode & 0x3) | ((NtcCode() & 0x3) << 2) | ((DieTempCode & 0x3) << 4) | ((VsysCode & 0x3) << 6));
            case AdcIbatResultMsb:
                return (byte)(IbatCode >> 2);
            case AdcVbusResultMsb:
                return (byte)(VbusCode >> 2);
            case AdcGp1ResultLsbs:
                // bits[5:4] IBAT, [7:6] VBUS (burst VBAT0/1 unused -> 0)
                return (byte)(((IbatCode & 0x3) << 4) | ((VbusCode & 0x3) << 6));
            default:
                return ReadStored(PageAdc, offset);
            }
        }

        private void WriteAdc(byte offset, byte value)
        {
            switch(offset)
            {
            case AdcTaskVbatMeasure:
            case AdcTaskNtcMeasure:
            case AdcTaskTempMeasure:
            case AdcTaskVsysMeasure:
            case AdcTaskIbatMeasure:
            case AdcTaskVbusMeasure:
            case AdcTaskDelayedVbatMeasure:
            case AdcTaskAutoTimUpdate:
                // measurement completes instantly in virtual time
                break;
            default:
                WriteStored(PageAdc, offset, value); // ADCCONFIG, NTCR_SEL, IBAT_EN, ...
                break;
            }
        }

        private uint VbatCode => ClampCode(BatteryVoltageMilliVolts * 1024 / 5000);      // 5 V full scale
        private uint VsysCode => ClampCode(VsysMilliVolts * 1024 / 6375);                // 6.375 V full scale
        private uint VbusCode => ClampCode((vbusConnected ? VbusVoltageMilliVolts : 0) * 1024 / 7500); // 7.5 V full scale
        private uint DieTempCode => 0;

        private uint NtcCode()
        {
            // inverse of npm1300_charger.c calc_temp(): code such that
            // log(1024/code - 1) = -beta * (1/T - 1/T0)
            var temperatureKelvin = (double)BatteryTemperatureCelsius + 273.15;
            var r = ThermistorOhms * Math.Exp(ThermistorBeta * (1.0 / temperatureKelvin - 1.0 / 298.15));
            var code = 1024.0 * r / (r + ThermistorOhms);
            return ClampCode((uint)Math.Round(code));
        }

        private uint IbatCode
        {
            get
            {
                var magnitudeMicroAmps = (ulong)Math.Abs(BatteryCurrentMilliAmps) * 1000u;
                ulong fullScale = BatteryCurrentMilliAmps >= 0 ? DischargeLimitMicroAmps : ChargeCurrentMicroAmps;
                if(fullScale == 0)
                {
                    return 0;
                }
                return ClampCode((uint)(magnitudeMicroAmps * 1024 / fullScale));
            }
        }

        private byte ComputeIbatStatus()
        {
            // npmx ADC_ADCIBATMEASSTATUS: BCHARGERMODE[3:2], BCHARGERICHARGE[1:0]
            // values consumed by npm1300_charger.c: 0x04 discharge,
            // 0x0C trickle, 0x0D cool, 0x0F normal charge
            if(BatteryCurrentMilliAmps > 0)
            {
                return 0x04;
            }
            if(BatteryCurrentMilliAmps < 0)
            {
                var status = ComputeChargeStatus();
                if((status & 0x04) != 0)
                {
                    return 0x0C; // trickle
                }
                return 0x0F; // normal charge
            }
            return 0x00;
        }

        private static uint ClampCode(uint code) => Math.Min(code, 1023u);

        // -------------------- GPIOS (0x06) --------------------

        public void SetGpioInput(int pin, bool state)
        {
            gpioInputs[CheckGpioIndex(pin)] = state;
        }

        private byte ReadGpio(byte offset)
        {
            if(offset == GpioStatusReg)
            {
                byte status = 0;
                for(var pin = 0; pin < GpioPinCount; pin++)
                {
                    var mode = ReadStored(PageGpio, (byte)(GpioModeBase + pin), quiet: true);
                    bool state;
                    switch(mode)
                    {
                    case GpioModeInput:
                    case GpioModeInputRisingEdge:
                    case GpioModeInputFallingEdge:
                        state = gpioInputs[pin];
                        break;
                    case GpioModeInputLogic1:
                        state = true;
                        break;
                    case GpioModeInputLogic0:
                        state = false;
                        break;
                    case GpioModeOutputIrq:
                        state = IRQ.IsSet;
                        break;
                    case GpioModeOutputLogic1:
                        state = true;
                        break;
                    default: // GPORESET, GPOPWRLOSSWARN, GPOLOGIC0
                        state = false;
                        break;
                    }
                    if(state)
                    {
                        status |= (byte)(1 << pin);
                    }
                }
                return status;
            }
            return ReadStored(PageGpio, offset);
        }

        // -------------------- TIMER / watchdog (0x07) --------------------
        // wdt_npm1300.c + mfd_npm1300.c set_timer(): 24-bit target in
        // TIMERHIBYTE/MID/LO (16 ms per tick), TIMERTARGETSTROBE latches,
        // TIMERSET starts, TIMERCLR stops, WATCHDOGKICK reloads,
        // TIMERCONFIG selects boot/warn/reset/general mode.

        private byte ReadTimer(byte offset)
        {
            switch(offset)
            {
            case TimerStatusReg:
                return (byte)(timerRunning ? 1 : 0); // BOOTMONITORACTIVE used as busy flag
            case TimerConfigReg:
                return (byte)((timerPrescalerFast ? 0x08 : 0x00) | timerMode);
            case TimerHiByte:
                return (byte)(timerTargetShadow >> 16);
            case TimerMidByte:
                return (byte)(timerTargetShadow >> 8);
            case TimerLoByte:
                return (byte)timerTargetShadow;
            default:
                return ReadStored(PageTimer, offset);
            }
        }

        private void WriteTimer(byte offset, byte value)
        {
            switch(offset)
            {
            case TimerSet:
                if((value & 1) != 0)
                {
                    StartTimer();
                }
                break;
            case TimerClr:
                if((value & 1) != 0)
                {
                    timerRunning = false;
                    watchdogTimer.Enabled = false;
                }
                break;
            case TimerTargetStrobe:
                if((value & 1) != 0)
                {
                    timerTargetLatched = timerTargetShadow;
                }
                break;
            case WatchdogKick:
                if((value & 1) != 0 && timerRunning)
                {
                    StartTimer(); // reload from the latched target
                }
                break;
            case TimerConfigReg:
                timerMode = (byte)(value & 0x7);
                timerPrescalerFast = (value & 0x08) != 0;
                break;
            case TimerHiByte:
                timerTargetShadow = (timerTargetShadow & 0x00FFFF) | ((uint)value << 16);
                break;
            case TimerMidByte:
                timerTargetShadow = (timerTargetShadow & 0xFF00FF) | ((uint)value << 8);
                break;
            case TimerLoByte:
                timerTargetShadow = (timerTargetShadow & 0xFFFF00) | value;
                break;
            default:
                WriteStored(PageTimer, offset, value);
                break;
            }
        }

        private void StartTimer()
        {
            timerRunning = true;
            WatchdogExpired = false;
            watchdogTimer.Enabled = false;
            // 16 ms/tick (2 ms with fast prescaler); TIMER_PRESCALER_MS in
            // mfd_npm1300.c is 16
            var milliseconds = (ulong)Math.Max(1u, timerTargetLatched) * (timerPrescalerFast ? 2UL : 16UL);
            watchdogTimer.Limit = milliseconds;
            watchdogTimer.Value = 0;
            watchdogTimer.Enabled = true;
        }

        private void OnTimerExpired()
        {
            watchdogTimer.Enabled = false;
            timerRunning = false;
            WatchdogExpired = true;
            // EVENTSSHPHLD bit3: watchdog warning
            // (mfd_npm1300.c NPM1300_EVENT_WATCHDOG_WARN {0x12, 0x08})
            RaiseEvent(EventGroupShphld, 0x08);
            if(timerMode == TimerModeWarn || timerMode == TimerModeReset)
            {
                this.Log(LogLevel.Warning, "watchdog expired in mode {0} — host reset would be asserted", timerMode);
            }
            if(timerMode == TimerModeReset && ResetOnWatchdogExpiry)
            {
                WatchdogResetCount++;
                this.Log(LogLevel.Error, "G1PMICRESET watchdog expired in RESET mode — power-cycling the host (count={0})",
                    WatchdogResetCount);
                // Deferred so the reset does not run inside the timer callback.
                machine.RequestReset();
            }
        }

        // -------------------- LDSW / LDO (0x08) --------------------

        private byte ReadLdsw(byte offset)
        {
            switch(offset)
            {
            case LdswStatusReg:
                // npmx LDSW_LDSWSTATUS: bit0/2 LDSWnPWRUPLDSW, bit1/3
                // LDSWnPWRUPLDO, bit4 LDSWENABLE
                byte status = 0;
                var ldoSelected1 = (ReadStored(PageLdsw, Ldsw1LdoSel, quiet: true) & 1) != 0;
                var ldoSelected2 = (ReadStored(PageLdsw, Ldsw2LdoSel, quiet: true) & 1) != 0;
                if(ldswEnabled[0])
                {
                    status |= ldoSelected1 ? (byte)0x02 : (byte)0x01;
                    status |= 0x10;
                }
                if(ldswEnabled[1])
                {
                    status |= ldoSelected2 ? (byte)0x08 : (byte)0x04;
                    status |= 0x10;
                }
                return status;
            default:
                return ReadStored(PageLdsw, offset);
            }
        }

        private void WriteLdsw(byte offset, byte value)
        {
            switch(offset)
            {
            case TaskLdsw1Set:
                if((value & 1) != 0) { ldswEnabled[0] = true; }
                break;
            case TaskLdsw1Clr:
                if((value & 1) != 0) { ldswEnabled[0] = false; }
                break;
            case TaskLdsw2Set:
                if((value & 1) != 0) { ldswEnabled[1] = true; }
                break;
            case TaskLdsw2Clr:
                if((value & 1) != 0) { ldswEnabled[1] = false; }
                break;
            default:
                WriteStored(PageLdsw, offset, value); // GPISEL/CONFIG/LDOSEL/VOUTSEL
                break;
            }
        }

        // -------------------- SHPHLD / ship mode (0x0B) --------------------

        private byte ReadShip(byte offset)
        {
            switch(offset)
            {
            case ShipShphldStatus:
                return 0x01; // SHPHLD pin held high (button not pressed)
            case ShipShphldConfig:
                return ReadStored(PageShip, offset, quiet: true, resetValue: 0x03); // npmx reset 0x03
            default:
                return ReadStored(PageShip, offset);
            }
        }

        private void WriteShip(byte offset, byte value)
        {
            switch(offset)
            {
            case ShipTaskEnterHibernate:
                if((value & 1) != 0)
                {
                    HibernateRequested = true;
                    EnterShipMode("hibernate (wake on timer/VBUS)");
                }
                break;
            case ShipTaskEnterShipMode:
                if((value & 1) != 0)
                {
                    EnterShipMode("ship mode (wake on SHPHLD/VBUS)");
                }
                break;
            case ShipTaskShphldCfgStrobe:
            case ShipTaskResetCfg:
                break;
            default:
                WriteStored(PageShip, offset, value);
                break;
            }
        }

        private void EnterShipMode(string what)
        {
            this.Log(LogLevel.Info, "Entering {0}: I2C interface powers down until VBUS is connected", what);
            ShipModeActive = true;
        }

        // -------------------- stored register file --------------------

        private byte ReadStored(byte page, byte offset, bool quiet = false, byte resetValue = 0)
        {
            if(!storedRegisters.TryGetValue(((ushort)(page << 8)) | offset, out var value))
            {
                if(resetValue == 0 && !storedResetValues.TryGetValue(((ushort)(page << 8)) | offset, out resetValue))
                {
                    resetValue = 0;
                }
                value = resetValue;
            }
            if(!quiet)
            {
                this.Log(LogLevel.Warning, "UNIMPL periph=NPM1300 dir=R width=1 offset=0x{0:X2}{1:X2} value=0x{2:X2} pc=0x0 (stored register file)", page, offset, value);
            }
            return value;
        }

        private void WriteStored(byte page, byte offset, byte value)
        {
            this.Log(LogLevel.Warning, "UNIMPL periph=NPM1300 dir=W width=1 offset=0x{0:X2}{1:X2} value=0x{2:X2} pc=0x0 (stored register file)", page, offset, value);
            storedRegisters[((ushort)(page << 8)) | offset] = value;
        }

        private static int CheckRailIndex(int index)
        {
            if(index < 1 || index > 2)
            {
                throw new Antmicro.Renode.Exceptions.RecoverableException("rail index must be 1 or 2");
            }
            return index - 1;
        }

        private int CheckGpioIndex(int pin)
        {
            if(pin < 0 || pin >= GpioPinCount)
            {
                throw new Antmicro.Renode.Exceptions.RecoverableException($"GPIO pin must be 0..{GpioPinCount - 1}");
            }
            return pin;
        }

        // -------------------- state --------------------

        private TransferState transferState;
        private byte currentPage;
        private byte currentOffset;

        private byte[] eventsPending = new byte[EventGroupCount];
        private byte[] eventsEnabled = new byte[EventGroupCount];

        private bool faultNakInjected;
        private bool vbusConnected;
        private bool batteryDetected = true;
        private byte vbusCurrentLimit;
        private bool vbusSuspended;

        private bool chargingEnabled;
        private bool rechargeDisabled;
        private bool chargerCompletedLatch;
        private bool chargerCompletedEventSent;
        private byte chargerErrorReason;
        private byte chargerErrorSensor;
        private byte chargeCurrentMsb;
        private byte chargeCurrentLsb;
        private byte dischargeCurrentMsb;
        private byte dischargeCurrentLsb;
        private byte terminationVoltage;
        private byte terminationVoltageWarm;
        private byte trickleSelect;
        private byte terminationCurrentSelect;

        private bool[] buckEnabled = new bool[2];
        private bool[] buckPwm = new bool[2];
        private bool[] ldswEnabled = new bool[2];
        private bool[] gpioInputs = new bool[GpioPinCount];

        private bool timerRunning;
        private byte timerMode;
        private bool timerPrescalerFast;
        private uint timerTargetShadow;
        private uint timerTargetLatched;

        private readonly LimitTimer watchdogTimer;
        private readonly IMachine machine;
        private readonly Dictionary<int, byte> storedRegisters = new Dictionary<int, byte>();
        // non-zero hardware reset values for registers kept in the stored
        // file (npmx ADK <REG>_ResetValue)
        private readonly Dictionary<int, byte> storedResetValues = new Dictionary<int, byte>
        {
            { (PageAdc << 8) | 0x0A, 0x01 },  // ADC_ADCNTCRSEL reset 0x01
            { (PageShip << 8) | ShipShphldConfig, 0x03 }, // SHPHLD_SHPHLDCONFIG reset 0x03
        };

        private enum TransferState
        {
            Idle,
            PageReceived,
            OffsetReceived,
            Data,
        }

        // pages
        private const byte PageMain = 0x00;
        private const byte PageVbus = 0x02;
        private const byte PageCharger = 0x03;
        private const byte PageBuck = 0x04;
        private const byte PageAdc = 0x05;
        private const byte PageGpio = 0x06;
        private const byte PageTimer = 0x07;
        private const byte PageLdsw = 0x08;
        private const byte PageShip = 0x0B;

        // MAIN
        private const byte MainTaskSwReset = 0x01;
        private const int EventGroupAdc = 0;
        private const int EventGroupCharger0 = 1;
        private const int EventGroupCharger1 = 2;
        private const int EventGroupCharger2 = 3;
        private const int EventGroupShphld = 4;
        private const int EventGroupVbus0 = 5;
        private const int EventGroupVbus1 = 6;
        private const int EventGroupGpio = 7;
        private const int EventGroupCount = 8;

        // VBUSIN
        private const byte VbusTaskUpdateIlimSw = 0x00;
        private const byte VbusIlim0 = 0x01;
        private const byte VbusSuspendReg = 0x03;
        private const byte VbusUsbcDetectStatus = 0x05;
        private const byte VbusStatus = 0x07;

        // BCHARGER
        private const byte ChargerTaskReleaseErr = 0x00;
        private const byte ChargerTaskClearChgErr = 0x01;
        private const byte ChargerTaskClearSafetyTimer = 0x02;
        private const byte ChargerEnableSet = 0x04;
        private const byte ChargerEnableClr = 0x05;
        private const byte ChargerDisableSet = 0x06;
        private const byte ChargerDisableClr = 0x07;
        private const byte ChargerIsetMsb = 0x08;
        private const byte ChargerIsetLsb = 0x09;
        private const byte ChargerIsetDischargeMsb = 0x0A;
        private const byte ChargerIsetDischargeLsb = 0x0B;
        private const byte ChargerVterm = 0x0C;
        private const byte ChargerVtermR = 0x0D;
        private const byte ChargerTrickleSel = 0x0E;
        private const byte ChargerItermSel = 0x0F;
        private const byte ChargerNtcColdMsb = 0x10;
        private const byte ChargerNtcCoolMsb = 0x12;
        private const byte ChargerNtcWarmMsb = 0x14;
        private const byte ChargerNtcHotMsb = 0x16;
        private const byte ChargerIlimStatus = 0x2D;
        private const byte ChargerNtcStatus = 0x32;
        private const byte ChargerDieTempStatus = 0x33;
        private const byte ChargerChargeStatus = 0x34;
        private const byte ChargerErrReason = 0x36;
        private const byte ChargerErrSensor = 0x37;

        // BUCK
        private const byte Buck1EnaSet = 0x00;
        private const byte Buck1EnaClr = 0x01;
        private const byte Buck2EnaSet = 0x02;
        private const byte Buck2EnaClr = 0x03;
        private const byte Buck1PwmSet = 0x04;
        private const byte Buck1PwmClr = 0x05;
        private const byte Buck2PwmSet = 0x06;
        private const byte Buck2PwmClr = 0x07;
        private const byte Buck1NormVout = 0x08;
        private const byte Buck2NormVout = 0x0A;
        private const byte Buck1VoutStatus = 0x10;
        private const byte Buck2VoutStatus = 0x11;
        private const byte BuckStatusReg = 0x34;

        // ADC
        private const byte AdcTaskVbatMeasure = 0x00;
        private const byte AdcTaskNtcMeasure = 0x01;
        private const byte AdcTaskTempMeasure = 0x02;
        private const byte AdcTaskVsysMeasure = 0x03;
        private const byte AdcTaskIbatMeasure = 0x06;
        private const byte AdcTaskVbusMeasure = 0x07;
        private const byte AdcTaskDelayedVbatMeasure = 0x08;
        private const byte AdcTaskAutoTimUpdate = 0x0C;
        private const byte AdcIbatMeasStatus = 0x10;
        private const byte AdcVbatResultMsb = 0x11;
        private const byte AdcNtcResultMsb = 0x12;
        private const byte AdcTempResultMsb = 0x13;
        private const byte AdcVsysResultMsb = 0x14;
        private const byte AdcGp0ResultLsbs = 0x15;
        private const byte AdcIbatResultMsb = 0x18;
        private const byte AdcVbusResultMsb = 0x19;
        private const byte AdcGp1ResultLsbs = 0x1A;

        // GPIOS
        private const byte GpioModeBase = 0x00;
        private const byte GpioStatusReg = 0x1E;
        private const int GpioPinCount = 5;
        private const byte GpioModeInput = 0;
        private const byte GpioModeInputLogic1 = 1;
        private const byte GpioModeInputLogic0 = 2;
        private const byte GpioModeInputRisingEdge = 3;
        private const byte GpioModeInputFallingEdge = 4;
        private const byte GpioModeOutputIrq = 5;
        private const byte GpioModeOutputReset = 6;
        private const byte GpioModeOutputPowerLossWarning = 7;
        private const byte GpioModeOutputLogic1 = 8;
        private const byte GpioModeOutputLogic0 = 9;

        // TIMER
        private const byte TimerSet = 0x00;
        private const byte TimerClr = 0x01;
        private const byte TimerTargetStrobe = 0x03;
        private const byte WatchdogKick = 0x04;
        private const byte TimerConfigReg = 0x05;
        private const byte TimerStatusReg = 0x06;
        private const byte TimerHiByte = 0x08;
        private const byte TimerMidByte = 0x09;
        private const byte TimerLoByte = 0x0A;
        private const byte TimerModeBoot = 0;
        private const byte TimerModeWarn = 1;
        private const byte TimerModeReset = 2;
        private const byte TimerModeGeneral = 3;
        private const long WatchdogTimerFrequency = 1000; // limits in ms

        // LDSW
        private const byte TaskLdsw1Set = 0x00;
        private const byte TaskLdsw1Clr = 0x01;
        private const byte TaskLdsw2Set = 0x02;
        private const byte TaskLdsw2Clr = 0x03;
        private const byte LdswStatusReg = 0x04;
        private const byte Ldsw1LdoSel = 0x08;
        private const byte Ldsw2LdoSel = 0x09;

        // SHPHLD
        private const byte ShipTaskEnterHibernate = 0x00;
        private const byte ShipTaskShphldCfgStrobe = 0x01;
        private const byte ShipTaskEnterShipMode = 0x02;
        private const byte ShipTaskResetCfg = 0x03;
        private const byte ShipShphldConfig = 0x04;
        private const byte ShipShphldStatus = 0x05;
    }
}
