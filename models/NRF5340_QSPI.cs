//
// NRF5340_QSPI — nRF5340 QSPI (external flash interface 0) at
// 0x4002B000/0x5002B000, IRQ 43, plus the XIP window helper mapped at
// 0x10000000 (NRF5340_QSPI_XIPWindow).
//
// Register offsets, reset values and field positions verbatim from the
// pinned SVD (svd/nrf5340_application.svd, hal_nordic 9784731…).
// Behaviour implemented to cover what the pinned NCS v2.5.1 drivers use
// (third_party/hal_nordic/nrfx/drivers/src/nrfx_qspi.c,
//  third_party/zephyr/drivers/flash/nrf_qspi_nor.c):
//   TASKS_ACTIVATE/READSTART/WRITESTART/ERASESTART/DEACTIVATE,
//   EVENTS_READY + INTEN/INTENSET/INTENCLR, ENABLE, READ.*, WRITE.*,
//   ERASE.*, IFCONFIG0/IFCONFIG1, STATUS, DPMDUR, ADDRCONF,
//   CINSTRCONF/CINSTRDAT0/1 (incl. long frame mode), IFTIMING, XIPOFFSET,
//   XIPEN, PSEL.*.
// SUBSCRIBE_*/PUBLISH_* (DPPI) are stored and log UNSUPPORTED (no DPPI hub
// yet — TODO wire to NRF5340_DPPIC when it lands). XIP/DMA encryption
// registers are stored; enabling them logs UNSUPPORTED.
//
// The external flash attaches as an ISPIPeripheral (repl: `@ qspi`); all
// flash traffic is byte-banged through Transmit()/FinishTransmission().
// Byte-transport contract (see models/Macronix_MX25U.cs): dummy /
// performance-enhance cycles of the fast/dual/quad opcodes are timing
// artifacts and are not represented at byte level.
//
// Timing model: bus transfers complete instantly in virtual time
// (idealized, like the platform clock model). The flash's program/erase
// busy time is real (virtual-time) state in the flash model; the
// controller emulates its hardware WIP polling (before read/write/erase
// tasks, between page-program chunks, and for the WIPWAIT flags) through
// the INorFlashBusyTiming interface, and delays EVENTS_READY until the
// wait would have ended. With a stuck-busy flash (FaultStuckBusy) the
// wait never ends and EVENTS_READY is never generated, matching the
// driver's timeout path. EVENTS_READY is generated for every task except
// DEACTIVATE (SVD note).
//
// Assumptions (documented per docs/conventions.md, "never invent"):
//  - WREN (0x06) is sent automatically before every page-program chunk
//    and before erase operations (required for multi-page writes to work;
//    the flash clears WEL after each program).
//  - The controller WIP-waits before starting READ/WRITE/ERASE tasks.
//  - ADDRCONF's instruction is sent during TASKS_ACTIVATE whenever
//    ADDRCONF.MODE != NoInstr (reset value has MODE=NoInstr, so nothing
//    is sent unless firmware configured it).
//  - DPMDUR units: N * 256 * 31.25 ns = N * 8 us (SVD description).
//
// 24-bit addressing on a larger flash: addresses in READ.SRC/WRITE.DST/
// ERASE.PTR/XIP beyond 16 MiB cannot be expressed on the wire. Policy is
// configurable via Wrap24BitAddresses: true (default) truncates to
// 24 bits (aliasing into the EAR-selected bank, as the wire protocol
// does) and logs a diagnostic; false logs UNSUPPORTED and skips the
// transfer.
//
// Fault injection: FaultDropInterrupt — swallow the IRQ assertion of the
// next READY event (the event flag itself is still set).
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class NRF5340_QSPI : NullRegistrationPointPeripheralContainer<ISPIPeripheral>,
        IDoubleWordPeripheral, IProvidesRegisterCollection<DoubleWordRegisterCollection>, IKnownSize
    {
        public NRF5340_QSPI(IMachine machine) : base(machine)
        {
            this.machine = machine;
            sysbus = machine.GetSystemBus(this);
            IRQ = new GPIO();
            RegistersCollection = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            RegistersCollection.Reset();
            activated = false;
            longFrameActive = false;
            statusBusy = false;
            sreg = 0;
            dpmTarget = false;
            dpmTransitionAt = TimeInterval.Empty;
            readyGeneration++; // cancels any in-flight scheduled READY
            UpdateInterrupts();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(!RegistersCollection.TryRead(offset, out var value))
            {
                LogUnimplemented("R", offset, 0);
                return 0;
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(!RegistersCollection.TryWrite(offset, value))
            {
                LogUnimplemented("W", offset, value);
            }
        }

        public GPIO IRQ { get; }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        public long Size => 0x1000;

        // Fault injection: swallow the IRQ assertion of the next READY event.
        public bool FaultDropInterrupt { get; set; }

        // High-address policy in 24-bit address mode (see header).
        public bool Wrap24BitAddresses { get; set; } = true;

        public bool Activated => activated;

        // ---------------- XIP window ----------------

        public uint XipRead(long offset, int width)
        {
            var flash = RegisteredPeripheral;
            if(!enabled.Value || !activated || !xipEnable.Value)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=xip-read-inactive detail=XIP read at offset 0x{0:X} while {1}; returning 0",
                    offset, !enabled.Value ? "disabled" : !activated ? "not activated" : "XIPEN=0");
                return 0;
            }
            if(flash == null)
            {
                this.Log(LogLevel.Warning, "XIP read at offset 0x{0:X} with no flash attached", offset);
                return 0xFFFFFFFF;
            }
            if(DpmActive)
            {
                this.Log(LogLevel.Warning, "XIP read at offset 0x{0:X} while flash is in DPM; returning 0xFF..", offset);
                return 0xFFFFFFFF;
            }
            var raw = (uint)((ulong)offset + xipOffset.Value);
            if(!ResolveFlashAddress(raw, "XIP", out var address))
            {
                return 0xFFFFFFFF;
            }
            if(longFrameActive)
            {
                this.Log(LogLevel.Warning, "XIP read during an ongoing long-frame custom instruction; aborting the frame");
                flash.FinishTransmission();
                longFrameActive = false;
            }
            if(!TryGetReadOpcode(out var opcode))
            {
                return 0xFFFFFFFF;
            }
            flash.Transmit(opcode);
            SendAddress(flash, address);
            var result = 0u;
            for(var i = 0; i < width; i++)
            {
                result |= (uint)flash.Transmit(0xFF) << (8 * i);
            }
            flash.FinishTransmission();
            return result;
        }

        // ---------------- registers ----------------

        private void DefineRegisters()
        {
            Registers.TasksActivate.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_ACTIVATE", writeCallback: (_, value) => { if(value) DoActivate(); })
                .WithReservedBits(1, 31);

            Registers.TasksReadStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_READSTART", writeCallback: (_, value) => { if(value) DoReadStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksWriteStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_WRITESTART", writeCallback: (_, value) => { if(value) DoWriteStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksEraseStart.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_ERASESTART", writeCallback: (_, value) => { if(value) DoEraseStart(); })
                .WithReservedBits(1, 31);

            Registers.TasksDeactivate.Define(this)
                .WithFlag(0, FieldMode.Write, name: "TASKS_DEACTIVATE", writeCallback: (_, value) => { if(value) DoDeactivate(); })
                .WithReservedBits(1, 31);

            DefineSubscribeRegister(Registers.SubscribeActivate, "SUBSCRIBE_ACTIVATE");
            DefineSubscribeRegister(Registers.SubscribeReadStart, "SUBSCRIBE_READSTART");
            DefineSubscribeRegister(Registers.SubscribeWriteStart, "SUBSCRIBE_WRITESTART");
            DefineSubscribeRegister(Registers.SubscribeEraseStart, "SUBSCRIBE_ERASESTART");
            DefineSubscribeRegister(Registers.SubscribeDeactivate, "SUBSCRIBE_DEACTIVATE");

            Registers.EventsReady.Define(this)
                .WithFlag(0, out eventsReady, name: "EVENTS_READY")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            DefineSubscribeRegister(Registers.PublishReady, "PUBLISH_READY");

            Registers.Inten.Define(this)
                .WithFlag(0, out readyInterruptEnabled, name: "READY")
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.IntenSet.Define(this)
                .WithFlag(0, name: "READY",
                    valueProviderCallback: _ => readyInterruptEnabled.Value,
                    writeCallback: (_, value) => { if(value) readyInterruptEnabled.Value = true; })
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.IntenClr.Define(this)
                .WithFlag(0, name: "READY",
                    valueProviderCallback: _ => readyInterruptEnabled.Value,
                    writeCallback: (_, value) => { if(value) readyInterruptEnabled.Value = false; })
                .WithReservedBits(1, 31)
                .WithWriteCallback((_, __) => UpdateInterrupts());

            Registers.Enable.Define(this)
                .WithFlag(0, out enabled, name: "ENABLE")
                .WithReservedBits(1, 31);

            Registers.ReadSrc.Define(this)
                .WithValueField(0, 32, out readSrc, name: "READ.SRC");
            Registers.ReadDst.Define(this)
                .WithValueField(0, 32, out readDst, name: "READ.DST");
            Registers.ReadCnt.Define(this)
                .WithValueField(0, 21, out readCnt, name: "READ.CNT")
                .WithReservedBits(21, 11);

            Registers.WriteDst.Define(this)
                .WithValueField(0, 32, out writeDst, name: "WRITE.DST");
            Registers.WriteSrc.Define(this)
                .WithValueField(0, 32, out writeSrc, name: "WRITE.SRC");
            Registers.WriteCnt.Define(this)
                .WithValueField(0, 21, out writeCnt, name: "WRITE.CNT")
                .WithReservedBits(21, 11);

            Registers.ErasePtr.Define(this)
                .WithValueField(0, 32, out erasePtr, name: "ERASE.PTR");
            Registers.EraseLen.Define(this)
                .WithValueField(0, 2, out eraseLen, name: "ERASE.LEN")
                .WithReservedBits(2, 30);

            DefinePselRegister(Registers.PselSck, "PSEL.SCK");
            DefinePselRegister(Registers.PselCsn, "PSEL.CSN");
            DefinePselRegister(Registers.PselIo0, "PSEL.IO0");
            DefinePselRegister(Registers.PselIo1, "PSEL.IO1");
            DefinePselRegister(Registers.PselIo2, "PSEL.IO2");
            DefinePselRegister(Registers.PselIo3, "PSEL.IO3");

            Registers.XipOffset.Define(this)
                .WithValueField(0, 32, out xipOffset, name: "XIPOFFSET");

            // Fields per SVD; bits 16/17 are undocumented but written by the
            // nrfx anomaly-121 workaround, so the whole word is stored.
            Registers.IfConfig0.Define(this)
                .WithValueField(0, 32, out ifconfig0, name: "IFCONFIG0");

            Registers.XipEn.Define(this, 0x1)
                .WithFlag(0, out xipEnable, name: "XIPEN")
                .WithReservedBits(1, 31);

            DefineEncRegister(Registers.XipEncKey0, "XIP_ENC.KEY0");
            DefineEncRegister(Registers.XipEncKey1, "XIP_ENC.KEY1");
            DefineEncRegister(Registers.XipEncKey2, "XIP_ENC.KEY2");
            DefineEncRegister(Registers.XipEncKey3, "XIP_ENC.KEY3");
            DefineEncRegister(Registers.XipEncNonce0, "XIP_ENC.NONCE0");
            DefineEncRegister(Registers.XipEncNonce1, "XIP_ENC.NONCE1");
            DefineEncRegister(Registers.XipEncNonce2, "XIP_ENC.NONCE2");
            DefineEncEnableRegister(Registers.XipEncEnable, "XIP_ENC.ENABLE");
            DefineEncRegister(Registers.DmaEncKey0, "DMA_ENC.KEY0");
            DefineEncRegister(Registers.DmaEncKey1, "DMA_ENC.KEY1");
            DefineEncRegister(Registers.DmaEncKey2, "DMA_ENC.KEY2");
            DefineEncRegister(Registers.DmaEncKey3, "DMA_ENC.KEY3");
            DefineEncRegister(Registers.DmaEncNonce0, "DMA_ENC.NONCE0");
            DefineEncRegister(Registers.DmaEncNonce1, "DMA_ENC.NONCE1");
            DefineEncRegister(Registers.DmaEncNonce2, "DMA_ENC.NONCE2");
            DefineEncEnableRegister(Registers.DmaEncEnable, "DMA_ENC.ENABLE");

            Registers.IfConfig1.Define(this, 0x00040480)
                .WithValueField(0, 32, out ifconfig1, name: "IFCONFIG1",
                    writeCallback: (previous, value) => HandleIfConfig1Write((uint)previous, (uint)value));

            Registers.Status.Define(this)
                .WithReservedBits(0, 2)
                .WithFlag(2, FieldMode.Read, name: "DPM", valueProviderCallback: _ => DpmActive)
                .WithFlag(3, FieldMode.Read, name: "READY", valueProviderCallback: _ => !statusBusy)
                .WithReservedBits(4, 20)
                .WithValueField(24, 8, FieldMode.Read, name: "SREG", valueProviderCallback: _ => sreg);

            Registers.DpmDur.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 16, out dpmEnterDuration, name: "ENTER")
                .WithValueField(16, 16, out dpmExitDuration, name: "EXIT");

            Registers.AddrConf.Define(this, 0x000000B7)
                .WithValueField(0, 32, out addrConf, name: "ADDRCONF");

            Registers.CinstrConf.Define(this, 0x00002000)
                .WithValueField(0, 32, out cinstrConf, name: "CINSTRCONF",
                    writeCallback: (_, value) => DoCustomInstruction((uint)value));

            Registers.CinstrDat0.Define(this)
                .WithValueField(0, 32, out cinstrDat0, name: "CINSTRDAT0");
            Registers.CinstrDat1.Define(this)
                .WithValueField(0, 32, out cinstrDat1, name: "CINSTRDAT1");

            Registers.IfTiming.Define(this, 0x00000200)
                .WithValueField(0, 32, out ifTiming, name: "IFTIMING");
        }

        private void DefineSubscribeRegister(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 32, name: name,
                    writeCallback: (_, value) =>
                    {
                        if((value & (1u << 31)) != 0)
                        {
                            // TODO: wire to the DPPI hub once NRF5340_DPPIC lands.
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=dppi detail={0} enabled (0x{1:X8}) but DPPI is not modelled", name, value);
                        }
                    });
        }

        private void DefinePselRegister(Registers register, string name)
        {
            register.Define(this, 0xFFFFFFFF)
                .WithValueField(0, 32, name: name);
        }

        private void DefineEncRegister(Registers register, string name)
        {
            register.Define(this)
                .WithValueField(0, 32, FieldMode.Write, name: name);
        }

        private void DefineEncEnableRegister(Registers register, string name)
        {
            register.Define(this)
                .WithFlag(0, name: name,
                    writeCallback: (_, value) =>
                    {
                        if(value)
                        {
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=encryption detail={0} set but the stream cipher is not modelled; data is passed through unencrypted", name);
                        }
                    })
                .WithReservedBits(1, 31);
        }

        // ---------------- tasks ----------------

        private void DoActivate()
        {
            if(!enabled.Value)
            {
                // The nrfx anomaly-43 workaround (compiled in unconditionally
                // for nRF5340 application builds) triggers ACTIVATE with the
                // peripheral disabled and pins disconnected, then spins on
                // EVENTS_READY. Hardware completes it; do the same but leave
                // the interface inactive.
                this.Log(LogLevel.Debug, "TASKS_ACTIVATE while ENABLE=0 (anomaly-43 workaround pattern): READY generated, interface stays inactive");
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            activated = true;
            var readyAt = TimeInterval.Empty;
            var flash = RegisteredPeripheral;
            var mode = (int)((addrConf.Value >> 24) & 0x3);
            if(flash != null && mode != 0)
            {
                var wipWait = (addrConf.Value & (1u << 26)) != 0;
                if(wipWait && !EmulateControllerWipWait("ACTIVATE/ADDRCONF", out readyAt))
                {
                    return; // stuck busy: no READY, ever
                }
                if((addrConf.Value & (1u << 27)) != 0)
                {
                    SendWriteEnable(flash);
                }
                flash.Transmit((byte)(addrConf.Value & 0xFF));
                if(mode >= 2)
                {
                    flash.Transmit((byte)((addrConf.Value >> 8) & 0xFF));
                }
                if(mode == 3)
                {
                    flash.Transmit((byte)((addrConf.Value >> 16) & 0xFF));
                }
                flash.FinishTransmission();
            }
            this.Log(LogLevel.Debug, "Activated");
            ScheduleReady(readyAt);
        }

        private void DoDeactivate()
        {
            // SVD: EVENTS_READY is generated for all tasks except DEACTIVATE.
            activated = false;
            if(longFrameActive)
            {
                RegisteredPeripheral?.FinishTransmission();
                longFrameActive = false;
            }
            this.Log(LogLevel.Debug, "Deactivated");
        }

        private void DoReadStart()
        {
            if(!CheckActive("READSTART"))
            {
                return;
            }
            var flash = RegisteredPeripheral;
            var count = (int)readCnt.Value;
            if(flash == null)
            {
                this.Log(LogLevel.Warning, "READSTART with no flash attached");
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            if(!ResolveFlashAddress((uint)readSrc.Value, "READSTART", out var address)
                || !TryGetReadOpcode(out var opcode)
                || count == 0)
            {
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            if(!EmulateControllerWipWait("READSTART", out var readyAt))
            {
                return;
            }
            flash.Transmit(opcode);
            SendAddress(flash, address);
            var buffer = new byte[count];
            for(var i = 0; i < count; i++)
            {
                buffer[i] = flash.Transmit(0xFF);
            }
            flash.FinishTransmission();
            sysbus.WriteBytes(buffer, readDst.Value);
            this.Log(LogLevel.Debug, "READ: 0x{0:X} bytes flash@0x{1:X} -> ram@0x{2:X}", count, address, readDst.Value);
            ScheduleReady(readyAt);
        }

        private void DoWriteStart()
        {
            if(!CheckActive("WRITESTART"))
            {
                return;
            }
            var flash = RegisteredPeripheral;
            var count = (int)writeCnt.Value;
            if(flash == null)
            {
                this.Log(LogLevel.Warning, "WRITESTART with no flash attached");
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            if(!ResolveFlashAddress((uint)writeDst.Value, "WRITESTART", out var address)
                || !TryGetWriteOpcode(out var opcode)
                || count == 0)
            {
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            var data = sysbus.ReadBytes(writeSrc.Value, count);
            var pageSize = (ifconfig0.Value & (1u << 12)) != 0 ? 512u : 256u;
            var offset = 0;
            var readyAt = TimeInterval.Empty;
            while(offset < count)
            {
                var current = address + (uint)offset;
                var chunk = (int)Math.Min(pageSize - (current & (pageSize - 1)), (uint)(count - offset));
                // Hardware polls the flash status between page chunks.
                if(!EmulateControllerWipWait("WRITESTART", out readyAt))
                {
                    return; // stuck busy mid-write: no READY
                }
                SendWriteEnable(flash);
                flash.Transmit(opcode);
                SendAddress(flash, current);
                for(var i = 0; i < chunk; i++)
                {
                    flash.Transmit(data[offset + i]);
                }
                flash.FinishTransmission();
                offset += chunk;
            }
            this.Log(LogLevel.Debug, "WRITE: 0x{0:X} bytes ram@0x{1:X} -> flash@0x{2:X}", count, writeSrc.Value, address);
            ScheduleReady(readyAt);
        }

        private void DoEraseStart()
        {
            if(!CheckActive("ERASESTART"))
            {
                return;
            }
            var flash = RegisteredPeripheral;
            if(flash == null)
            {
                this.Log(LogLevel.Warning, "ERASESTART with no flash attached");
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            byte opcode;
            uint alignment;
            switch(eraseLen.Value)
            {
            case 0:
                opcode = 0x20; // SE, 4 KiB
                alignment = 0x1000;
                break;
            case 1:
                opcode = 0xD8; // BE, 64 KiB
                alignment = 0x10000;
                break;
            case 2:
                opcode = 0xC7; // CE
                alignment = 0;
                break;
            default:
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=erase-len detail=ERASE.LEN=3 is reserved; task ignored");
                ScheduleReady(TimeInterval.Empty);
                return;
            }
            var address = 0u;
            if(alignment != 0)
            {
                if(!ResolveFlashAddress((uint)erasePtr.Value, "ERASESTART", out address))
                {
                    ScheduleReady(TimeInterval.Empty);
                    return;
                }
                if((address & (alignment - 1)) != 0)
                {
                    this.Log(LogLevel.Warning, "ERASE.PTR 0x{0:X8} is not aligned to 0x{1:X}; the flash erases the containing unit", address, alignment);
                }
            }
            if(!EmulateControllerWipWait("ERASESTART", out var readyAt))
            {
                return;
            }
            SendWriteEnable(flash);
            flash.Transmit(opcode);
            if(alignment != 0)
            {
                SendAddress(flash, address);
            }
            flash.FinishTransmission();
            this.Log(LogLevel.Debug, "ERASE: len={0} at 0x{1:X}", eraseLen.Value, address);
            ScheduleReady(readyAt);
        }

        // ---------------- custom instructions ----------------

        private void DoCustomInstruction(uint value)
        {
            var opcode = (byte)(value & 0xFF);
            var length = (int)((value >> 8) & 0xF);
            var wipWait = (value & (1u << 14)) != 0;
            var wren = (value & (1u << 15)) != 0;
            var lfen = (value & (1u << 16)) != 0;
            var lfstop = (value & (1u << 17)) != 0;

            if(!CheckActive("CINSTR"))
            {
                return;
            }
            if(length > 9)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=cinstr-length detail=LENGTH={0} out of range; instruction ignored", length);
                return;
            }
            if(length == 0 && !(lfstop && longFrameActive))
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=cinstr-length detail=LENGTH=0; instruction ignored");
                return;
            }
            var flash = RegisteredPeripheral;
            var readyAt = TimeInterval.Empty;
            if(wipWait && !longFrameActive && !EmulateControllerWipWait("CINSTR", out readyAt))
            {
                return; // stuck busy: driver's timeout path
            }
            if(flash == null)
            {
                this.Log(LogLevel.Warning, "CINSTR 0x{0:X2} with no flash attached", opcode);
                var idle = 0xFFFFFFFFu;
                if(length > 1)
                {
                    cinstrDat0.Value = idle;
                    cinstrDat1.Value = idle;
                }
                ScheduleReady(readyAt);
                return;
            }
            if(wren)
            {
                SendWriteEnable(flash);
            }
            var continuation = longFrameActive;
            if(!continuation)
            {
                flash.Transmit(opcode);
            }
            var tx = new byte[8];
            var dat0 = (uint)cinstrDat0.Value;
            var dat1 = (uint)cinstrDat1.Value;
            for(var i = 0; i < 4; i++)
            {
                tx[i] = (byte)(dat0 >> (8 * i));
                tx[4 + i] = (byte)(dat1 >> (8 * i));
            }
            var rx0 = dat0;
            var rx1 = dat1;
            for(var i = 0; i < length - 1; i++)
            {
                var received = flash.Transmit(tx[i]);
                if(i < 4)
                {
                    rx0 = (rx0 & ~(0xFFu << (8 * i))) | (uint)received << (8 * i);
                }
                else
                {
                    rx1 = (rx1 & ~(0xFFu << (8 * (i - 4)))) | (uint)received << (8 * (i - 4));
                }
            }
            cinstrDat0.Value = rx0;
            cinstrDat1.Value = rx1;
            if(lfen && !lfstop)
            {
                longFrameActive = true;
            }
            else
            {
                flash.FinishTransmission();
                longFrameActive = false;
            }
            this.Log(LogLevel.Debug, "CINSTR 0x{0:X2} len={1}{2}{3}", opcode, length,
                lfen ? " LFEN" : "", lfstop ? " LFSTOP" : "");
            ScheduleReady(readyAt);
        }

        // ---------------- DPM ----------------

        private void HandleIfConfig1Write(uint previous, uint value)
        {
            var wasDpm = (previous & (1u << 24)) != 0;
            var isDpm = (value & (1u << 24)) != 0;
            if(wasDpm == isDpm)
            {
                return;
            }
            if((ifconfig0.Value & (1u << 7)) == 0)
            {
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=dpm detail=IFCONFIG1.DPMEN toggled while IFCONFIG0.DPMENABLE=0; ignored");
                return;
            }
            var flash = RegisteredPeripheral;
            var now = machine.LocalTimeSource.ElapsedVirtualTime;
            // DPMDUR unit: 256 * 31.25 ns = 8 us (SVD).
            var durationUs = 8UL * (isDpm ? dpmEnterDuration.Value : dpmExitDuration.Value);
            dpmTarget = isDpm;
            dpmTransitionAt = now + TimeInterval.FromMicroseconds(durationUs);
            if(flash != null)
            {
                flash.Transmit(isDpm ? (byte)0xB9 : (byte)0xAB);
                flash.FinishTransmission();
            }
            this.Log(LogLevel.Info, "DPM {0} requested; effective at {1}", isDpm ? "enter" : "exit", dpmTransitionAt);
        }

        private bool DpmActive
        {
            get
            {
                var now = machine.LocalTimeSource.ElapsedVirtualTime;
                return now >= dpmTransitionAt ? dpmTarget : !dpmTarget;
            }
        }

        // ---------------- helpers ----------------

        private bool CheckEnabled(string what)
        {
            if(!enabled.Value)
            {
                this.Log(LogLevel.Warning, "TASKS_{0} while ENABLE=0; ignored", what);
                return false;
            }
            return true;
        }

        private bool CheckActive(string what)
        {
            if(!CheckEnabled(what))
            {
                return false;
            }
            if(!activated)
            {
                this.Log(LogLevel.Warning, "{0} while not activated; ignored", what);
                return false;
            }
            return true;
        }

        private bool TryGetReadOpcode(out byte opcode)
        {
            switch(ifconfig0.Value & 0x7)
            {
            case 0: opcode = 0x0B; return true; // FASTREAD
            case 1: opcode = 0x3B; return true; // READ2O
            case 2: opcode = 0xBB; return true; // READ2IO
            case 3: opcode = 0x6B; return true; // READ4O
            case 4: opcode = 0xEB; return true; // READ4IO
            default:
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=readoc detail=IFCONFIG0.READOC={0} is reserved; transfer skipped", ifconfig0.Value & 0x7);
                opcode = 0;
                return false;
            }
        }

        private bool TryGetWriteOpcode(out byte opcode)
        {
            switch((ifconfig0.Value >> 3) & 0x7)
            {
            case 0: opcode = 0x02; return true; // PP
            case 1: opcode = 0xA2; return true; // PP2O
            case 2: opcode = 0x32; return true; // PP4O
            case 3: opcode = 0x38; return true; // PP4IO
            default:
                this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=writeoc detail=IFCONFIG0.WRITEOC={0} is reserved; transfer skipped", (ifconfig0.Value >> 3) & 0x7);
                opcode = 0;
                return false;
            }
        }

        private bool Uses32BitAddressing => (ifconfig0.Value & (1u << 6)) != 0;

        private bool ResolveFlashAddress(uint raw, string what, out uint resolved)
        {
            resolved = raw;
            if(Uses32BitAddressing || raw < (1u << 24))
            {
                return true;
            }
            if(Wrap24BitAddresses)
            {
                resolved = raw & 0x00FFFFFF;
                this.Log(LogLevel.Warning, "QSPI {0}: address 0x{1:X8} exceeds the 24-bit addressing range; wrapping/aliasing to 0x{2:X6}",
                    what, raw, resolved);
                return true;
            }
            this.Log(LogLevel.Warning, "UNSUPPORTED periph=QSPI what=addr-overflow detail={0} address 0x{1:X8} exceeds the 24-bit range and Wrap24BitAddresses is false; transfer skipped", what, raw);
            return false;
        }

        private void SendAddress(ISPIPeripheral flash, uint address)
        {
            if(Uses32BitAddressing)
            {
                flash.Transmit((byte)(address >> 24));
            }
            flash.Transmit((byte)(address >> 16));
            flash.Transmit((byte)(address >> 8));
            flash.Transmit((byte)address);
        }

        private void SendWriteEnable(ISPIPeripheral flash)
        {
            flash.Transmit(0x06);
            flash.FinishTransmission();
        }

        // Emulates the hardware's RDSR/WIP polling in virtual time. Returns
        // false (and latches busy) when the wait would never end.
        private bool EmulateControllerWipWait(string what, out TimeInterval readyAt)
        {
            readyAt = TimeInterval.Empty;
            var timing = RegisteredPeripheral as INorFlashBusyTiming;
            if(timing == null)
            {
                return true;
            }
            var busyUntil = timing.BusyUntil;
            var now = machine.LocalTimeSource.ElapsedVirtualTime;
            if(busyUntil == TimeInterval.Maximal)
            {
                this.Log(LogLevel.Warning, "{0}: flash WIP never clears (stuck busy); EVENTS_READY will not be generated", what);
                statusBusy = true;
                readyGeneration++;
                return false;
            }
            if(busyUntil <= now)
            {
                return true;
            }
            timing.NotifyControllerWipWait();
            UpdateSregFromFlash();
            readyAt = busyUntil;
            return true;
        }

        private void UpdateSregFromFlash()
        {
            var flash = RegisteredPeripheral;
            if(flash == null || longFrameActive)
            {
                return;
            }
            flash.Transmit(0x05); // RDSR
            sreg = flash.Transmit(0xFF);
            flash.FinishTransmission();
        }

        private void ScheduleReady(TimeInterval at)
        {
            var timeSource = machine.LocalTimeSource;
            var now = timeSource.ElapsedVirtualTime;
            if(at <= now)
            {
                SignalReady();
                return;
            }
            statusBusy = true;
            var generation = ++readyGeneration;
            timeSource.ExecuteInSyncedState(_ =>
            {
                if(generation == readyGeneration)
                {
                    SignalReady();
                }
            }, new TimeStamp(at, timeSource.Domain));
        }

        private void SignalReady()
        {
            statusBusy = false;
            eventsReady.Value = true;
            if(FaultDropInterrupt)
            {
                FaultDropInterrupt = false;
                this.Log(LogLevel.Warning, "FaultDropInterrupt: READY IRQ swallowed (event flag still set)");
                return;
            }
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            IRQ.Set(eventsReady.Value && readyInterruptEnabled.Value);
        }

        private void LogUnimplemented(string direction, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNIMPL periph=QSPI dir={0} width=4 offset=0x{1:X} value=0x{2:X8} pc=0x{3:X8}",
                direction, offset, value, pc);
        }

        private readonly IMachine machine;
        private readonly IBusController sysbus;

        private IFlagRegisterField eventsReady;
        private IFlagRegisterField readyInterruptEnabled;
        private IFlagRegisterField enabled;
        private IFlagRegisterField xipEnable;
        private IValueRegisterField readSrc;
        private IValueRegisterField readDst;
        private IValueRegisterField readCnt;
        private IValueRegisterField writeDst;
        private IValueRegisterField writeSrc;
        private IValueRegisterField writeCnt;
        private IValueRegisterField erasePtr;
        private IValueRegisterField eraseLen;
        private IValueRegisterField xipOffset;
        private IValueRegisterField ifconfig0;
        private IValueRegisterField ifconfig1;
        private IValueRegisterField dpmEnterDuration;
        private IValueRegisterField dpmExitDuration;
        private IValueRegisterField addrConf;
        private IValueRegisterField cinstrConf;
        private IValueRegisterField cinstrDat0;
        private IValueRegisterField cinstrDat1;
        private IValueRegisterField ifTiming;

        private bool activated;
        private bool longFrameActive;
        private bool statusBusy;
        private byte sreg;
        private bool dpmTarget;
        private TimeInterval dpmTransitionAt;
        private int readyGeneration;

        private enum Registers : long
        {
            TasksActivate = 0x000,
            TasksReadStart = 0x004,
            TasksWriteStart = 0x008,
            TasksEraseStart = 0x00C,
            TasksDeactivate = 0x010,
            SubscribeActivate = 0x080,
            SubscribeReadStart = 0x084,
            SubscribeWriteStart = 0x088,
            SubscribeEraseStart = 0x08C,
            SubscribeDeactivate = 0x090,
            EventsReady = 0x100,
            PublishReady = 0x180,
            Inten = 0x300,
            IntenSet = 0x304,
            IntenClr = 0x308,
            Enable = 0x500,
            ReadSrc = 0x504,
            ReadDst = 0x508,
            ReadCnt = 0x50C,
            WriteDst = 0x510,
            WriteSrc = 0x514,
            WriteCnt = 0x518,
            ErasePtr = 0x51C,
            EraseLen = 0x520,
            PselSck = 0x524,
            PselCsn = 0x528,
            PselIo0 = 0x530,
            PselIo1 = 0x534,
            PselIo2 = 0x538,
            PselIo3 = 0x53C,
            XipOffset = 0x540,
            IfConfig0 = 0x544,
            XipEn = 0x54C,
            XipEncKey0 = 0x560,
            XipEncKey1 = 0x564,
            XipEncKey2 = 0x568,
            XipEncKey3 = 0x56C,
            XipEncNonce0 = 0x570,
            XipEncNonce1 = 0x574,
            XipEncNonce2 = 0x578,
            XipEncEnable = 0x57C,
            DmaEncKey0 = 0x580,
            DmaEncKey1 = 0x584,
            DmaEncKey2 = 0x588,
            DmaEncKey3 = 0x58C,
            DmaEncNonce0 = 0x590,
            DmaEncNonce1 = 0x594,
            DmaEncNonce2 = 0x598,
            DmaEncEnable = 0x59C,
            IfConfig1 = 0x600,
            Status = 0x604,
            DpmDur = 0x614,
            AddrConf = 0x624,
            CinstrConf = 0x634,
            CinstrDat0 = 0x638,
            CinstrDat1 = 0x63C,
            IfTiming = 0x640,
        }
    }

    // Execute-in-place window: maps the external flash contents at
    // 0x10000000 (read-only). Reads work while the QSPI is enabled,
    // activated and XIPEN is set; XIPOFFSET is added to the address.
    public class NRF5340_QSPI_XIPWindow : IDoubleWordPeripheral, IWordPeripheral, IBytePeripheral, IKnownSize
    {
        public NRF5340_QSPI_XIPWindow(IMachine machine, NRF5340_QSPI controller, long size = 0x2000000)
        {
            this.machine = machine;
            this.controller = controller;
            this.size = size;
        }

        public void Reset()
        {
            // no state
        }

        public uint ReadDoubleWord(long offset) => controller.XipRead(offset, 4);
        public ushort ReadWord(long offset) => (ushort)controller.XipRead(offset, 2);
        public byte ReadByte(long offset) => (byte)controller.XipRead(offset, 1);

        public void WriteDoubleWord(long offset, uint value) => LogWrite(offset, value, 4);
        public void WriteWord(long offset, ushort value) => LogWrite(offset, value, 2);
        public void WriteByte(long offset, byte value) => LogWrite(offset, value, 1);

        public long Size => size;

        private void LogWrite(long offset, uint value, int width)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            this.Log(LogLevel.Warning,
                "UNSUPPORTED periph=QSPI what=xip-write detail=the XIP window is read-only (dir=W width={0} offset=0x{1:X} value=0x{2:X8} pc=0x{3:X8})",
                width, offset, value, pc);
        }

        private readonly IMachine machine;
        private readonly NRF5340_QSPI controller;
        private readonly long size;
    }
}
