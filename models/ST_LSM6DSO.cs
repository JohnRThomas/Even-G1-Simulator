//
// ST_LSM6DSO — ST LSM6DSO iNEMO 6-axis IMU (accelerometer + gyroscope), I2C.
//
// Modelled from:
//   - ST LSM6DSO datasheet DS12140 (register map §9, sensitivities §2.1:
//     accel ±2g = 0.061 mg/LSB, gyro ±250 dps = 8.75 mdps/LSB)
//   - pinned STdC driver third_party/hal_st/sensor/stmemsc/lsm6dso_STdC/
//     driver/lsm6dso_reg.h (register addresses + bitfields; NCS v2.5.1 pin)
//   - pinned Zephyr driver third_party/zephyr/drivers/sensor/lsm6dso/
//     (init sequence: mem_bank_set, WHO_AM_I, I3C disable, SHUB master
//     check, SW_RESET, power modes, FS/ODR, FIFO bypass, BDU —
//     lsm6dso.c lsm6dso_init_chip())
//   - upstream Renode LSM6DSO_IMU.cs (I2C framing / register-collection
//     pattern base), adapted to this repository's conventions.
//
// Scope per board map: polling only (no INT pins are modelled; INT1/INT2
// control registers are stored), FIFO bypass mode only (other FIFO modes
// log UNSUPPORTED), accel+gyro power/range/ODR.
//
// Register banks: the user bank is fully modelled; the embedded-functions
// and sensor-hub banks (FUNC_CFG_ACCESS bits 7/6) are register files with
// datasheet reset values for the fields the pinned drivers touch
// (e.g. sensor-hub MASTER_CONFIG) — accesses there are logged.
//
// Injection (monitor-settable):
//   AccelerationX/Y/Z  [g]      e.g.  lsm6dso AccelerationZ 1
//   AngularRateX/Y/Z   [dps]
//   Temperature        [°C]
// Scripted motion playback (monitor-callable):
//   PlayGesture "don"           stream a named (t, accel xyz, gyro xyz) profile
//                               over virtual time so the driver reads a moving
//                               gravity/rate vector at its own ODR, instead of a
//                               single static injection. See the Gestures table.
//   StopGesture                 stop playback (freezes the current sample)
//   GesturePlaying   [bool]     is a profile currently streaming
// Faults:
//   FaultWrongId — WHO_AM_I returns a corrupted value (0xFF)
//   FaultNak     — the TWIM master address-NACKs this device
//                  (II2CDeviceFaultNak, see NRF5340_TWIM.cs)
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class ST_LSM6DSO : II2CPeripheral, IProvidesRegisterCollection<ByteRegisterCollection>, II2CDeviceFaultNak
    {
        public ST_LSM6DSO(IMachine machine)
        {
            this.machine = machine;
            RegistersCollection = new ByteRegisterCollection(this);
            resetTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "swReset",
                SoftwareResetMicroseconds, Antmicro.Renode.Time.Direction.Ascending,
                enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            resetTimer.LimitReached += CompleteSoftwareReset;
            playbackTimer = new LimitTimer(machine.ClockSource, TimerFrequency, this, "gesture",
                PlaybackTickMicroseconds, Antmicro.Renode.Time.Direction.Ascending,
                enabled: false, workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: true);
            playbackTimer.LimitReached += AdvancePlayback;
            DefineRegisters();
            Reset();
        }

        public void Reset()
        {
            ResetRegisters(preserveI3cDisable: false);
            transactionState = TransactionState.Idle;
            registerAddress = 0;
            swResetInProgress = false;
            resetTimer.Enabled = false;
            playbackTimer.Enabled = false;
            activeGesture = null;
            activeGestureName = null;
            playbackTimeMicroseconds = 0;
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            var index = 0;
            if(transactionState == TransactionState.Idle)
            {
                registerAddress = data[0];
                transactionState = TransactionState.AddressReceived;
                index = 1;
            }
            for(; index < data.Length; index++)
            {
                transactionState = TransactionState.Writing;
                WriteRegister(registerAddress, data[index]);
                TryIncrementAddress();
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadRegister(registerAddress);
                TryIncrementAddress();
            }
            return result;
        }

        public void FinishTransmission()
        {
            transactionState = TransactionState.Idle;
        }

        public ByteRegisterCollection RegistersCollection { get; }

        // -------------------- injection & faults --------------------

        public decimal AccelerationX { get; set; }
        public decimal AccelerationY { get; set; }
        public decimal AccelerationZ { get; set; }
        public decimal AngularRateX { get; set; }
        public decimal AngularRateY { get; set; }
        public decimal AngularRateZ { get; set; }
        public decimal Temperature { get; set; } = 25m;

        public bool FaultWrongId { get; set; }
        public bool FaultNak { get; set; }

        // -------------------- scripted motion playback --------------------
        //
        // Streams a named (t, accel xyz [g], gyro xyz [dps]) profile over virtual
        // time. On each tick the interpolated sample is written to the same
        // Acceleration*/AngularRate* fields the driver samples, so the firmware's
        // Zephyr channel reads (SENSOR_CHAN_ACCEL_XYZ / _GYRO_XYZ) see a *moving*
        // gravity/rate vector. The G1 wear/tilt fusion (imu_fusion_thread @0xfe88)
        // integrates this into a pitch angle; a "don" that ends in the head-up-
        // look orientation drives that pitch into the firmware's wake window.

        public void PlayGesture(string name)
        {
            if(!Gestures.TryGetValue(name, out var frames))
            {
                this.Log(LogLevel.Warning, "LSM6DSO: unknown gesture '{0}'; known: {1}",
                    name, string.Join(", ", Gestures.Keys));
                return;
            }
            activeGesture = frames;
            activeGestureName = name;
            playbackTimeMicroseconds = 0;
            ApplySampleAtSeconds(0.0);
            playbackTimer.Enabled = true;
            this.Log(LogLevel.Info, "LSM6DSO: playing gesture '{0}' ({1} keyframes, {2:F2} s)",
                name, frames.Length, frames[frames.Length - 1].T);
        }

        public void StopGesture()
        {
            playbackTimer.Enabled = false;
            activeGesture = null;
            activeGestureName = null;
            this.Log(LogLevel.Info, "LSM6DSO: gesture playback stopped");
        }

        public bool GesturePlaying => activeGesture != null;

        private void AdvancePlayback()
        {
            var frames = activeGesture;
            if(frames == null)
            {
                return;
            }
            playbackTimeMicroseconds += PlaybackTickMicroseconds;
            var t = playbackTimeMicroseconds / 1_000_000.0;
            var end = frames[frames.Length - 1].T;
            if(t >= end)
            {
                // Hold the final worn orientation so the firmware keeps reading it.
                ApplyFrame(frames[frames.Length - 1]);
                playbackTimer.Enabled = false;
                this.Log(LogLevel.Info, "LSM6DSO: gesture '{0}' complete; holding final orientation "
                    + "(accel {1:F2},{2:F2},{3:F2} g)", activeGestureName,
                    AccelerationX, AccelerationY, AccelerationZ);
                activeGesture = null;
                activeGestureName = null;
                return;
            }
            ApplySampleAtSeconds(t);
        }

        private void ApplySampleAtSeconds(double t)
        {
            var f = activeGesture;
            var i = 0;
            while(i < f.Length - 1 && t >= f[i + 1].T)
            {
                i++;
            }
            var a = f[i];
            var b = f[Math.Min(i + 1, f.Length - 1)];
            var span = b.T - a.T;
            var u = span > 1e-9 ? (t - a.T) / span : 0.0;
            if(u < 0.0) u = 0.0;
            if(u > 1.0) u = 1.0;
            AccelerationX = Lerp(a.Ax, b.Ax, u);
            AccelerationY = Lerp(a.Ay, b.Ay, u);
            AccelerationZ = Lerp(a.Az, b.Az, u);
            AngularRateX = Lerp(a.Gx, b.Gx, u);
            AngularRateY = Lerp(a.Gy, b.Gy, u);
            AngularRateZ = Lerp(a.Gz, b.Gz, u);
        }

        private void ApplyFrame(MotionFrame f)
        {
            AccelerationX = f.Ax; AccelerationY = f.Ay; AccelerationZ = f.Az;
            AngularRateX = f.Gx; AngularRateY = f.Gy; AngularRateZ = f.Gz;
        }

        private static decimal Lerp(decimal a, decimal b, double u)
        {
            return a + (b - a) * (decimal)u;
        }

        private struct MotionFrame
        {
            public MotionFrame(double t, decimal ax, decimal ay, decimal az,
                               decimal gx, decimal gy, decimal gz)
            {
                T = t; Ax = ax; Ay = ay; Az = az; Gx = gx; Gy = gy; Gz = gz;
            }
            public readonly double T;                       // seconds
            public readonly decimal Ax, Ay, Az;             // g
            public readonly decimal Gx, Gy, Gz;             // dps
        }

        // "don" — putting the glasses on. Reproduces the temple/head motion of a
        // real donning: at rest on a table (gravity +Z, pitch ~0), picked up and
        // rotated (peak angular rate ~130 dps), raised to the face, seated on the
        // ears/nose (worn, head roughly level), then the wearer tilts their head
        // UP to look at the HUD. The final gravity direction (ax:az ~= 6.3,
        // magnitude ~1 g) puts the fused pitch at about -81 deg — inside the G1
        // wake window, which is read live from the fusion state at
        // device_info+0xeec/+0xef0. NOTE the window is a *persisted* setting: on
        // a first-boot device it is pitch*100 in [-8300, -5300] (-83..-53 deg),
        // but the provisioned seed (fw/g1_extstore_seed.bin) shifts it to about
        // [-9650, -6650] (-96.5..-66.5 deg), so the head-up endpoint tilts to the
        // steeper HUD gaze that a provisioned wearer actually configures.
        private static readonly Dictionary<string, MotionFrame[]> Gestures =
            new Dictionary<string, MotionFrame[]>
        {
            ["don"] = new[]
            {
                //              t     ax     ay     az     gx     gy    gz
                new MotionFrame(0.0,  0.00m, 0.00m, 1.00m,   0m,   0m,  0m),   // at rest on table
                new MotionFrame(0.7,  0.05m, 0.08m, 1.00m,  20m,  12m,  6m),   // fingertip contact
                new MotionFrame(1.3,  0.18m, 0.25m, 0.95m,  75m,  55m, 22m),   // lifted off table
                new MotionFrame(2.0,  0.45m, 0.35m, 0.80m, 130m,  85m, 45m),   // rotating up to face
                new MotionFrame(2.7,  0.70m, 0.22m, 0.55m, 100m,  60m, 30m),   // temples opening
                new MotionFrame(3.3,  0.80m, 0.08m, 0.50m,  45m,  25m, 14m),   // seating on ears/nose
                new MotionFrame(3.9,  0.62m, 0.00m, 0.78m,  12m,   6m,  3m),   // settled, head ~level
                new MotionFrame(4.6,  0.988m,0.00m, 0.156m,-62m, -20m, -8m),   // tilt head UP to HUD (~-81 deg)
                new MotionFrame(5.6,  0.988m,0.00m, 0.156m,  0m,   0m,  0m),   // hold the HUD gaze
            },
            // "head_down" / "head_up" — a wearer looking away from the HUD and
            // back at it: the last step of "don" played backwards, and again
            // forwards without the donning before it.
            ["head_down"] = new[]
            {
                new MotionFrame(0.0,  0.988m,0.00m, 0.156m,  0m,   0m,  0m),   // HUD gaze
                new MotionFrame(0.7,  0.62m, 0.00m, 0.78m,  62m,  20m,  8m),   // tilt head DOWN to ~level
                new MotionFrame(1.7,  0.62m, 0.00m, 0.78m,   0m,   0m,  0m),   // hold
            },
            ["head_up"] = new[]
            {
                new MotionFrame(0.0,  0.62m, 0.00m, 0.78m,   0m,   0m,  0m),   // worn, head ~level
                new MotionFrame(0.7,  0.988m,0.00m, 0.156m,-62m, -20m, -8m),   // tilt head UP to HUD
                new MotionFrame(1.7,  0.988m,0.00m, 0.156m,  0m,   0m,  0m),   // hold the HUD gaze
            },
        };

        public bool AccelerometerEnabled => accelOutputDataRate.Value != 0;
        public bool GyroscopeEnabled => gyroOutputDataRate.Value != 0;

        // -------------------- registers --------------------

        private void DefineRegisters()
        {
            Registers.FuncCfgAccess.Define(this)
                .WithReservedBits(0, 6)
                .WithFlag(6, out shubRegAccess, name: "SHUB_REG_ACCESS")
                .WithFlag(7, out funcCfgAccess, name: "FUNC_CFG_ACCESS");

            Registers.PinCtrl.Define(this, 0x3F)
                .WithValueField(0, 6, FieldMode.Read, name: "RESERVED_READS_0x3F", valueProviderCallback: _ => 0x3F)
                .WithFlag(6, name: "SDO_PU_EN")
                .WithFlag(7, name: "OIS_PU_DIS");

            DefineStored(Registers.FifoCtrl1, "FIFO_CTRL1");
            DefineStored(Registers.FifoCtrl2, "FIFO_CTRL2");
            DefineStored(Registers.FifoCtrl3, "FIFO_CTRL3");

            Registers.FifoCtrl4.Define(this)
                .WithValueField(0, 3, out fifoMode, name: "FIFO_MODE",
                    changeCallback: (_, value) =>
                    {
                        if(value != 0)
                        {
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph=LSM6DSO what=FIFO_MODE detail=only bypass mode (0) is modelled, got {0}; FIFO stays empty", value);
                        }
                    })
                .WithReservedBits(3, 1)
                .WithValueField(4, 2, name: "ODR_T_BATCH")
                .WithValueField(6, 2, name: "DEC_TS_BATCH");

            DefineStored(Registers.CounterBdrReg1, "COUNTER_BDR_REG1");
            DefineStored(Registers.CounterBdrReg2, "COUNTER_BDR_REG2");
            DefineStored(Registers.Int1Ctrl, "INT1_CTRL");   // polling only: stored, no pin
            DefineStored(Registers.Int2Ctrl, "INT2_CTRL");   // polling only: stored, no pin

            Registers.WhoAmI.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "WHO_AM_I",
                    valueProviderCallback: _ => FaultWrongId ? CorruptedWhoAmI : WhoAmIValue);

            Registers.Ctrl1Xl.Define(this)
                .WithReservedBits(0, 1)
                .WithFlag(1, name: "LPF2_XL_EN")
                .WithValueField(2, 2, out accelFullScale, name: "FS_XL")
                .WithValueField(4, 4, out accelOutputDataRate, name: "ODR_XL")
                ;

            Registers.Ctrl2G.Define(this)
                .WithReservedBits(0, 1)
                .WithFlag(1, out gyroFullScale125, name: "FS_125")
                .WithValueField(2, 2, out gyroFullScale, name: "FS_G")
                .WithValueField(4, 4, out gyroOutputDataRate, name: "ODR_G");

            Registers.Ctrl3C.Define(this, 0x04)
                .WithFlag(0, name: "SW_RESET",
                    valueProviderCallback: _ => swResetInProgress,
                    writeCallback: (_, value) => { if(value) StartSoftwareReset(); })
                .WithReservedBits(1, 1)
                .WithFlag(2, out addressAutoIncrement, name: "IF_INC")
                .WithFlag(3, name: "SIM")
                .WithFlag(4, name: "PP_OD")
                .WithFlag(5, name: "H_LACTIVE")
                .WithFlag(6, out blockDataUpdate, name: "BDU")
                .WithFlag(7, name: "BOOT", valueProviderCallback: _ => false);

            DefineStored(Registers.Ctrl4C, "CTRL4_C");
            DefineStored(Registers.Ctrl5C, "CTRL5_C");
            DefineStored(Registers.Ctrl6C, "CTRL6_C");
            DefineStored(Registers.Ctrl7G, "CTRL7_G");
            DefineStored(Registers.Ctrl8Xl, "CTRL8_XL");

            Registers.Ctrl9Xl.Define(this, 0xE0)
                .WithReservedBits(0, 1)
                .WithFlag(1, out i3cDisable, name: "I3C_DISABLE")
                .WithFlag(2, name: "DEN_LH")
                .WithFlag(3, name: "DEN_XL_EN")
                .WithFlag(4, name: "DEN_XL_G")
                .WithFlag(5, name: "DEN_Z")
                .WithFlag(6, name: "DEN_Y")
                .WithFlag(7, name: "DEN_X");

            DefineStored(Registers.Ctrl10C, "CTRL10_C");

            DefineReadOnlyZero(Registers.AllIntSrc, "ALL_INT_SRC");
            DefineReadOnlyZero(Registers.WakeUpSrc, "WAKE_UP_SRC");
            DefineReadOnlyZero(Registers.TapSrc, "TAP_SRC");
            DefineReadOnlyZero(Registers.D6dSrc, "D6D_SRC");

            Registers.StatusReg.Define(this)
                .WithFlag(0, FieldMode.Read, name: "XLDA", valueProviderCallback: _ => AccelerometerEnabled)
                .WithFlag(1, FieldMode.Read, name: "GDA", valueProviderCallback: _ => GyroscopeEnabled)
                .WithFlag(2, FieldMode.Read, name: "TDA", valueProviderCallback: _ => true)
                .WithReservedBits(3, 5);

            Registers.OutTempL.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "OUT_TEMP_L", valueProviderCallback: _ => (byte)TemperatureRaw);
            Registers.OutTempH.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "OUT_TEMP_H", valueProviderCallback: _ => (byte)(TemperatureRaw >> 8));

            DefineOutputPair(Registers.OutXLG, () => GyroscopeEnabled ? GyroRaw(AngularRateX) : (short)0);
            DefineOutputPair(Registers.OutYLG, () => GyroscopeEnabled ? GyroRaw(AngularRateY) : (short)0);
            DefineOutputPair(Registers.OutZLG, () => GyroscopeEnabled ? GyroRaw(AngularRateZ) : (short)0);
            DefineOutputPair(Registers.OutXLA, () => AccelerometerEnabled ? AccelRaw(AccelerationX) : (short)0);
            DefineOutputPair(Registers.OutYLA, () => AccelerometerEnabled ? AccelRaw(AccelerationY) : (short)0);
            DefineOutputPair(Registers.OutZLA, () => AccelerometerEnabled ? AccelRaw(AccelerationZ) : (short)0);

            DefineReadOnlyZero(Registers.EmbFuncStatusMainpage, "EMB_FUNC_STATUS_MAINPAGE");
            DefineReadOnlyZero(Registers.FsmStatusAMainpage, "FSM_STATUS_A_MAINPAGE");
            DefineReadOnlyZero(Registers.FsmStatusBMainpage, "FSM_STATUS_B_MAINPAGE");
            DefineReadOnlyZero(Registers.StatusMasterMainpage, "STATUS_MASTER_MAINPAGE");
            DefineReadOnlyZero(Registers.FifoStatus1, "FIFO_STATUS1"); // bypass: always empty
            DefineReadOnlyZero(Registers.FifoStatus2, "FIFO_STATUS2");

            DefineReadOnlyZero(Registers.Timestamp0, "TIMESTAMP0");
            DefineReadOnlyZero(Registers.Timestamp1, "TIMESTAMP1");
            DefineReadOnlyZero(Registers.Timestamp2, "TIMESTAMP2");
            DefineReadOnlyZero(Registers.Timestamp3, "TIMESTAMP3");

            DefineStored(Registers.TapCfg0, "TAP_CFG0");
            DefineStored(Registers.TapCfg1, "TAP_CFG1");
            DefineStored(Registers.TapCfg2, "TAP_CFG2");
            DefineStored(Registers.TapThs6d, "TAP_THS_6D");
            DefineStored(Registers.IntDur2, "INT_DUR2");
            DefineStored(Registers.WakeUpThs, "WAKE_UP_THS");
            DefineStored(Registers.WakeUpDur, "WAKE_UP_DUR");
            DefineStored(Registers.FreeFall, "FREE_FALL");
            DefineStored(Registers.Md1Cfg, "MD1_CFG");
            DefineStored(Registers.Md2Cfg, "MD2_CFG");

            DefineStored(Registers.I3cBusAvb, "I3C_BUS_AVB");
            DefineReadOnlyZero(Registers.InternalFreqFine, "INTERNAL_FREQ_FINE");

            DefineStored(Registers.IntOis, "INT_OIS");
            DefineStored(Registers.Ctrl1Ois, "CTRL1_OIS");
            DefineStored(Registers.Ctrl2Ois, "CTRL2_OIS");
            DefineStored(Registers.Ctrl3Ois, "CTRL3_OIS");
            DefineStored(Registers.XOfsUsr, "X_OFS_USR");
            DefineStored(Registers.YOfsUsr, "Y_OFS_USR");
            DefineStored(Registers.ZOfsUsr, "Z_OFS_USR");

            DefineReadOnlyZero(Registers.FifoDataOutTag, "FIFO_DATA_OUT_TAG");
            DefineReadOnlyZero(Registers.FifoDataOutXL, "FIFO_DATA_OUT_X_L");
            DefineReadOnlyZero(Registers.FifoDataOutXH, "FIFO_DATA_OUT_X_H");
            DefineReadOnlyZero(Registers.FifoDataOutYL, "FIFO_DATA_OUT_Y_L");
            DefineReadOnlyZero(Registers.FifoDataOutYH, "FIFO_DATA_OUT_Y_H");
            DefineReadOnlyZero(Registers.FifoDataOutZL, "FIFO_DATA_OUT_Z_L");
            DefineReadOnlyZero(Registers.FifoDataOutZH, "FIFO_DATA_OUT_Z_H");
        }

        private void DefineStored(Registers register, string name, byte resetValue = 0x00)
        {
            register.Define(this, resetValue)
                .WithValueField(0, 8, name: name);
        }

        private void DefineReadOnlyZero(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: name, valueProviderCallback: _ => 0);
        }

        private void DefineOutputPair(Registers lowRegister, Func<short> sampleProvider)
        {
            lowRegister.Define(this)
                .WithValueField(0, 8, FieldMode.Read, valueProviderCallback: _ => (byte)sampleProvider());
            ((Registers)((long)lowRegister + 1)).Define(this)
                .WithValueField(0, 8, FieldMode.Read, valueProviderCallback: _ => (byte)(sampleProvider() >> 8));
        }

        // -------------------- transport helpers --------------------

        private byte ReadRegister(byte address)
        {
            if(SelectedBank != RegisterBank.User && address != (byte)Registers.FuncCfgAccess)
            {
                var bank = BankFile(SelectedBank);
                var value = bank.TryGetValue(address, out var stored) ? stored : BankResetValue(SelectedBank, address);
                this.Log(LogLevel.Warning, "UNIMPL periph=LSM6DSO dir=R width=1 offset=0x{0:X2} value=0x{1:X2} pc=0x0 ({2} bank register file)", address, value, SelectedBank);
                return value;
            }
            if(RegistersCollection.TryRead(address, out var result))
            {
                return result;
            }
            this.Log(LogLevel.Warning, "UNIMPL periph=LSM6DSO dir=R width=1 offset=0x{0:X2} value=0x00 pc=0x0", address);
            return 0;
        }

        private void WriteRegister(byte address, byte value)
        {
            if(SelectedBank != RegisterBank.User && address != (byte)Registers.FuncCfgAccess)
            {
                this.Log(LogLevel.Warning, "UNIMPL periph=LSM6DSO dir=W width=1 offset=0x{0:X2} value=0x{1:X2} pc=0x0 ({2} bank register file)", address, value, SelectedBank);
                BankFile(SelectedBank)[address] = value;
                return;
            }
            if(!RegistersCollection.TryWrite(address, value))
            {
                this.Log(LogLevel.Warning, "UNIMPL periph=LSM6DSO dir=W width=1 offset=0x{0:X2} value=0x{1:X2} pc=0x0", address, value);
            }
        }

        private void TryIncrementAddress()
        {
            if(addressAutoIncrement.Value)
            {
                registerAddress = (byte)((registerAddress + 1) & 0x7F);
            }
        }

        private RegisterBank SelectedBank =>
            funcCfgAccess.Value ? RegisterBank.EmbeddedFunctions :
            shubRegAccess.Value ? RegisterBank.SensorHub : RegisterBank.User;

        private Dictionary<byte, byte> BankFile(RegisterBank bank) =>
            bank == RegisterBank.EmbeddedFunctions ? embeddedFunctionsBank : sensorHubBank;

        private byte BankResetValue(RegisterBank bank, byte address)
        {
            // Sensor-hub MASTER_CONFIG (0x14): MASTER_ON=0 after reset — the
            // pinned Zephyr driver reads it before SW_RESET (lsm6dso.c:
            // lsm6dso_sh_master_get()).
            return 0x00;
        }

        // -------------------- reset --------------------

        private void StartSoftwareReset()
        {
            // Datasheet §6.4: SW_RESET restores default register values;
            // the bit is auto-cleared (reset takes ~50 µs). I3C_disable and
            // I3C_BUS_AVB survive (datasheet table 55 note; lsm6dso.c relies
            // on this by disabling I3C before the reset).
            swResetInProgress = true;
            ResetRegisters(preserveI3cDisable: true);
            resetTimer.Enabled = true;
        }

        private void CompleteSoftwareReset()
        {
            swResetInProgress = false;
            resetTimer.Enabled = false;
        }

        private void ResetRegisters(bool preserveI3cDisable)
        {
            var i3cDisableValue = preserveI3cDisable && i3cDisable != null && i3cDisable.Value;
            byte i3cBusAvb = 0;
            if(preserveI3cDisable)
            {
                RegistersCollection.TryRead((long)Registers.I3cBusAvb, out i3cBusAvb);
            }
            RegistersCollection.Reset();
            embeddedFunctionsBank.Clear();
            sensorHubBank.Clear();
            if(preserveI3cDisable)
            {
                if(i3cDisableValue)
                {
                    i3cDisable.Value = true;
                }
                RegistersCollection.TryWrite((long)Registers.I3cBusAvb, i3cBusAvb);
            }
        }

        // -------------------- sample conversion --------------------

        private short AccelRaw(decimal accelerationInG)
        {
            // datasheet §2.1: 0.061 / 0.122 / 0.244 / 0.488 mg/LSB
            // FS_XL: 00=±2g, 01=±16g, 10=±4g, 11=±8g (CTRL1_XL, lsm6dso_reg.h lsm6dso_fs_xl_t)
            decimal microGPerLsb;
            switch(accelFullScale.Value)
            {
            case 0:
                microGPerLsb = 61m;
                break;
            case 1:
                microGPerLsb = 488m;
                break;
            case 2:
                microGPerLsb = 122m;
                break;
            default:
                microGPerLsb = 244m;
                break;
            }
            return ClampToShort(accelerationInG * 1_000_000m / microGPerLsb);
        }

        private short GyroRaw(decimal rateInDps)
        {
            // datasheet §2.1: 4.375 / 8.75 / 17.5 / 35 / 70 mdps/LSB
            // FS_125 overrides FS_G (lsm6dso_reg.h lsm6dso_fs_g_t)
            decimal microDpsPerLsb;
            if(gyroFullScale125.Value)
            {
                microDpsPerLsb = 4375m;
            }
            else
            {
                switch(gyroFullScale.Value)
                {
                case 0:
                    microDpsPerLsb = 8750m;
                    break;
                case 1:
                    microDpsPerLsb = 17500m;
                    break;
                case 2:
                    microDpsPerLsb = 35000m;
                    break;
                default:
                    microDpsPerLsb = 70000m;
                    break;
                }
            }
            return ClampToShort(rateInDps * 1_000_000m / microDpsPerLsb);
        }

        private short TemperatureRaw => ClampToShort((Temperature - 25m) * 256m);

        private static short ClampToShort(decimal value)
        {
            if(value > short.MaxValue)
            {
                return short.MaxValue;
            }
            if(value < short.MinValue)
            {
                return short.MinValue;
            }
            return (short)Math.Round(value);
        }

        private IFlagRegisterField shubRegAccess;
        private IFlagRegisterField funcCfgAccess;
        private IFlagRegisterField addressAutoIncrement;
        private IFlagRegisterField blockDataUpdate;
        private IFlagRegisterField i3cDisable;
        private IFlagRegisterField gyroFullScale125;
        private IValueRegisterField fifoMode;
        private IValueRegisterField accelFullScale;
        private IValueRegisterField accelOutputDataRate;
        private IValueRegisterField gyroFullScale;
        private IValueRegisterField gyroOutputDataRate;

        private TransactionState transactionState;
        private byte registerAddress;
        private bool swResetInProgress;

        private MotionFrame[] activeGesture;
        private string activeGestureName;
        private ulong playbackTimeMicroseconds;

        private readonly IMachine machine;
        private readonly LimitTimer resetTimer;
        private readonly LimitTimer playbackTimer;
        private readonly Dictionary<byte, byte> embeddedFunctionsBank = new Dictionary<byte, byte>();
        private readonly Dictionary<byte, byte> sensorHubBank = new Dictionary<byte, byte>();

        private const byte WhoAmIValue = 0x6C;
        private const byte CorruptedWhoAmI = 0xFF;
        private const long TimerFrequency = 1000000;
        private const ulong SoftwareResetMicroseconds = 50;
        // 50 Hz playback resolution: finer than the driver's poll rate, so the
        // fusion always reads a smoothly interpolated sample.
        private const ulong PlaybackTickMicroseconds = 20000;

        private enum TransactionState
        {
            Idle,
            AddressReceived,
            Writing,
        }

        private enum RegisterBank
        {
            User,
            EmbeddedFunctions,
            SensorHub,
        }

        private enum Registers : long
        {
            FuncCfgAccess = 0x01,
            PinCtrl = 0x02,
            FifoCtrl1 = 0x07,
            FifoCtrl2 = 0x08,
            FifoCtrl3 = 0x09,
            FifoCtrl4 = 0x0A,
            CounterBdrReg1 = 0x0B,
            CounterBdrReg2 = 0x0C,
            Int1Ctrl = 0x0D,
            Int2Ctrl = 0x0E,
            WhoAmI = 0x0F,
            Ctrl1Xl = 0x10,
            Ctrl2G = 0x11,
            Ctrl3C = 0x12,
            Ctrl4C = 0x13,
            Ctrl5C = 0x14,
            Ctrl6C = 0x15,
            Ctrl7G = 0x16,
            Ctrl8Xl = 0x17,
            Ctrl9Xl = 0x18,
            Ctrl10C = 0x19,
            AllIntSrc = 0x1A,
            WakeUpSrc = 0x1B,
            TapSrc = 0x1C,
            D6dSrc = 0x1D,
            StatusReg = 0x1E,
            OutTempL = 0x20,
            OutTempH = 0x21,
            OutXLG = 0x22,
            OutXHG = 0x23,
            OutYLG = 0x24,
            OutYHG = 0x25,
            OutZLG = 0x26,
            OutZHG = 0x27,
            OutXLA = 0x28,
            OutXHA = 0x29,
            OutYLA = 0x2A,
            OutYHA = 0x2B,
            OutZLA = 0x2C,
            OutZHA = 0x2D,
            EmbFuncStatusMainpage = 0x35,
            FsmStatusAMainpage = 0x36,
            FsmStatusBMainpage = 0x37,
            StatusMasterMainpage = 0x39,
            FifoStatus1 = 0x3A,
            FifoStatus2 = 0x3B,
            Timestamp0 = 0x40,
            Timestamp1 = 0x41,
            Timestamp2 = 0x42,
            Timestamp3 = 0x43,
            TapCfg0 = 0x56,
            TapCfg1 = 0x57,
            TapCfg2 = 0x58,
            TapThs6d = 0x59,
            IntDur2 = 0x5A,
            WakeUpThs = 0x5B,
            WakeUpDur = 0x5C,
            FreeFall = 0x5D,
            Md1Cfg = 0x5E,
            Md2Cfg = 0x5F,
            I3cBusAvb = 0x62,
            InternalFreqFine = 0x63,
            IntOis = 0x6F,
            Ctrl1Ois = 0x70,
            Ctrl2Ois = 0x71,
            Ctrl3Ois = 0x72,
            XOfsUsr = 0x73,
            YOfsUsr = 0x74,
            ZOfsUsr = 0x75,
            FifoDataOutTag = 0x78,
            FifoDataOutXL = 0x79,
            FifoDataOutXH = 0x7A,
            FifoDataOutYL = 0x7B,
            FifoDataOutYH = 0x7C,
            FifoDataOutZL = 0x7D,
            FifoDataOutZH = 0x7E,
        }
    }
}
