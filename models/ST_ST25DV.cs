//
// ST_ST25DV — ST ST25DV04K/16K/64K dynamic NFC/RFID tag with I2C EEPROM
// and fast-transfer mailbox.
//
// Modelled from the ST25DV04K/16K/64K datasheet (DS11501) with register
// addresses/bit masks cross-checked against ST's official BSP driver
// (github.com/STMicroelectronics/stm32-st25dv, st25dv_reg.h / st25dv.h):
//   - user memory EEPROM at I2C device address 0x53 (0xA6 8-bit,
//     ST25DV_ADDR_DATA_I2C), 16-bit big-endian byte address, sequential
//     read/write with auto-increment
//   - dynamic registers at 0x2000..0x2007 (GPO_CTRL_Dyn, EH_CTRL_Dyn,
//     RF_MNGT_Dyn, I2C_SSO_Dyn, IT_STS_Dyn, MB_CTRL_Dyn, MB_LEN_Dyn)
//   - fast-transfer mailbox RAM 0x2008..0x2107 (256 bytes)
//   - system configuration area (GPO 0x0000 .. IC_REV 0x0020, I2C_PWD
//     0x0900) normally behind device address 0x57 (0xAE,
//     ST25DV_ADDR_SYST_I2C)
// The pinned Zephyr v3.4.99-ncs1 tree has no st25dv driver (verified),
// so the datasheet + ST BSP are the sole behavioural references.
//
// Density/package is configurable (constructor `density`: 4, 16 or 64
// kbit -> 512/2048/8192 B user memory; IC_REF 0x24 for 04K, 0x26 for
// 16/64K per st25dv.h I_AM_ST25DV04/I_AM_ST25DV64; MEM_SIZE = block
// count - 1, BLK_SIZE = 0x03).
//
// System-area access: the board uses only device address 0x53 (per
// docs/board-map.md the 0x57 system address must NOT be assumed), so by
// default the system area is reachable only through monitor helpers
// (ReadSystemRegister/WriteSystemRegister, ChipId, MemorySizeBlocks —
// the chip-ID/area-size queries). Boards that do wire the system address
// can additionally register the companion `ST_ST25DV_SystemPort` at any
// address (e.g. 0x57); it forwards to the same device state.
//
// NDEF: the user EEPROM holds the capability container + NDEF TLVs.
// ProgramNdefMessage(hex) writes a type-5-tag layout (CC at 0x0000,
// NDEF message TLV 0x03, terminator 0xFE); ReadUserMemory/WriteUserMemory
// give tests raw EEPROM access.
//
// Mailbox: MB_CTRL_Dyn.MB_EN is writable only while system MB_MODE.MB_MODE
// is 1 (datasheet MB_CTRL_Dyn description). Host messages are written to
// 0x2008.. in one transaction; at STOP the model latches MB_LEN_Dyn =
// length-1 and HOST_PUT_MSG. The RF side is driven from the monitor:
// RfPutMailboxMessage(hex) / RfReadMailboxMessage() (reading the last
// byte of an RF message clears RF_PUT_MSG, as completing the read does
// on hardware).
//
// GPO line and RF events (added for the simulator, from the v1.66 firmware
// source). The G1 does not poll the tag: the GPO output (open drain, active
// low, P1.10) interrupts it, and it then reads IT_STS_Dyn to see why. The
// charging case is the RF side. Driven from the monitor:
//   RfField true|false        the case's field appears / disappears
//   RfEnableMailbox           what the case does before talking: switches
//                             fast transfer mode on (the firmware never does)
//   RfPutMailboxMessage hex   a message from the case
// Each raises its bit in IT_STS_Dyn (FIELD_RISING 0x10, FIELD_FALLING 0x08,
// RF_PUT_MSG 0x20) and, if the firmware enabled that event in the GPO
// register, pulses GPO low.
//
// Faults: FaultWrongId (IC_REF/UID product code read corrupted 0xFF),
// FaultNak (address NACK via II2CDeviceFaultNak).
//
using System;
using System.Linq;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class ST_ST25DV : II2CPeripheral, II2CDeviceFaultNak
    {
        public ST_ST25DV(IMachine machine, int density = 4)
        {
            switch(density)
            {
            case 4:
                memorySize = 512;
                chipId = 0x24; // I_AM_ST25DV04
                break;
            case 16:
                memorySize = 2048;
                chipId = 0x26; // I_AM_ST25DV64 (16/64K share the IC ref)
                break;
            case 64:
                memorySize = 8192;
                chipId = 0x26;
                break;
            default:
                throw new ConstructionException("density must be 4, 16 or 64 (kbit)");
            }
            eeprom = new byte[memorySize];
            for(var i = 0; i < eeprom.Length; i++)
            {
                eeprom[i] = 0x00; // delivery state: user memory initialized to 0
            }
            GPO = new GPIO();
            gpoPulseTimer = new LimitTimer(machine.ClockSource, 1000000, this, "gpoPulse",
                GpoPulseMicroseconds, Direction.Ascending, enabled: false, workMode: WorkMode.OneShot, eventEnabled: true);
            gpoPulseTimer.LimitReached += () =>
            {
                gpoPulseTimer.Enabled = false;
                GPO.Set(true);
            };
            Reset();
        }

        // GPO output, active low: wire it to the SoC pin (P1.10 on the G1).
        public GPIO GPO { get; }

        public void Reset()
        {
            transferState = TransferState.Idle;
            currentAddress = 0;
            // dynamic registers
            gpoCtrl = 0x00;
            ehCtrl = 0x00;
            rfManagement = 0x00;
            securitySessionOpen = false;
            interruptStatus = 0x00;
            gpoPulseTimer.Enabled = false;
            gpoSynced = false;      // the SoC's pin state was reset as well
            mailboxEnabled = false;
            hostPutMessage = false;
            rfPutMessage = false;
            hostMissMessage = false;
            rfMissMessage = false;
            mailboxLength = 0;
            mailboxWriteCount = 0;
            Array.Clear(mailbox, 0, mailbox.Length);
            // system registers (delivery state)
            systemGpo = 0x00;
            systemItTime = 0x00;
            systemEhMode = 0x00;
            systemRfManagement = 0x00;
            Array.Clear(areaSecurity, 0, areaSecurity.Length);
            var lastBlock = MemorySizeBlocks; // whole memory in area 1 (delivery state)
            endArea1 = (byte)Math.Min(lastBlock / 8, 0xFF);
            endArea2 = endArea1;
            endArea3 = endArea1;
            systemI2cSecurity = 0x00;
            lockCcFile = 0x00;
            mailboxMode = 0x00;
            mailboxWatchdog = 0x00;
            lockConfiguration = 0x00;
            lockDsfid = 0x00;
            lockAfi = 0x00;
            dsfid = 0x00;
            afi = 0x00;
            password = new byte[8];
        }

        // -------------------- I2C transport (device address 0x53) --------------------

        public void Write(byte[] data)
        {
            SyncGpo();
            var index = 0;
            if(transferState == TransferState.Idle && index < data.Length)
            {
                addressHigh = data[index++];
                transferState = TransferState.AddressHighReceived;
            }
            if(transferState == TransferState.AddressHighReceived && index < data.Length)
            {
                currentAddress = (ushort)((addressHigh << 8) | data[index++]);
                transferState = TransferState.AddressReceived;
                mailboxWriteCount = 0;
            }
            for(; index < data.Length; index++)
            {
                transferState = TransferState.Data;
                WriteLocation(currentAddress, data[index], systemArea: false);
                currentAddress++;
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = ReadLocation(currentAddress, systemArea: false);
                currentAddress++;
            }
            return result;
        }

        public void FinishTransmission()
        {
            if(transferState == TransferState.Data && mailboxWriteCount > 0)
            {
                // host message committed at STOP
                mailboxLength = mailboxWriteCount;
                hostPutMessage = true;
                rfPutMessage = false;
                this.Log(LogLevel.Debug, "Mailbox: host put a {0}-byte message", mailboxWriteCount);
            }
            mailboxWriteCount = 0;
            transferState = TransferState.Idle;
        }

        // -------------------- configuration / injection --------------------

        public bool FaultWrongId { get; set; }
        public bool FaultNak { get; set; }

        // chip-ID / area-size queries (also reachable via the system area)
        public byte ChipId => FaultWrongId ? CorruptedId : chipId;
        public int MemorySizeBlocks => memorySize / BlockSizeBytes - 1;
        public int MemorySizeBytes => memorySize;
        public byte IcRevision => 0x11;
        public string Uid => BitConverter.ToString(BuildUid());

        public ulong UidSerial { get; set; } = 0x0123456789;

        public void WriteUserMemory(uint address, byte value)
        {
            if(address >= memorySize)
            {
                throw new RecoverableException($"address 0x{address:X} outside user memory (size 0x{memorySize:X})");
            }
            eeprom[address] = value;
        }

        public byte ReadUserMemory(uint address)
        {
            if(address >= memorySize)
            {
                throw new RecoverableException($"address 0x{address:X} outside user memory (size 0x{memorySize:X})");
            }
            return eeprom[address];
        }

        public void ProgramNdefMessage(string hexBytes)
        {
            var payload = ParseHex(hexBytes);
            if(payload.Length + 7 > memorySize)
            {
                throw new RecoverableException("NDEF message does not fit in user memory");
            }
            // type-5 tag capability container (4 bytes): magic E1, version/access
            // 0x40, MLEN = user memory / 8, features 0x05 (MBREAD)
            eeprom[0] = 0xE1;
            eeprom[1] = 0x40;
            eeprom[2] = (byte)Math.Min(memorySize / 8, 0xFF);
            eeprom[3] = 0x05;
            var offset = 4;
            eeprom[offset++] = 0x03; // NDEF message TLV
            if(payload.Length < 0xFF)
            {
                eeprom[offset++] = (byte)payload.Length;
            }
            else
            {
                eeprom[offset++] = 0xFF;
                eeprom[offset++] = (byte)(payload.Length >> 8);
                eeprom[offset++] = (byte)payload.Length;
            }
            Array.Copy(payload, 0, eeprom, offset, payload.Length);
            eeprom[offset + payload.Length] = 0xFE; // terminator TLV
        }

        public string ReadNdefMessage()
        {
            if(eeprom[0] != 0xE1)
            {
                return "";
            }
            var offset = 4;
            if(eeprom[offset] != 0x03)
            {
                return "";
            }
            int length = eeprom[offset + 1];
            var start = offset + 2;
            if(length == 0xFF)
            {
                length = (eeprom[offset + 2] << 8) | eeprom[offset + 3];
                start = offset + 4;
            }
            return BitConverter.ToString(eeprom, start, length);
        }

        public void OpenSecuritySession()
        {
            securitySessionOpen = true;
        }

        public byte ReadSystemRegister(uint address) => ReadLocation((ushort)address, systemArea: true);

        public void WriteSystemRegister(uint address, byte value) => WriteLocation((ushort)address, value, systemArea: true);

        // RF side: the reader's field appears or disappears.
        public void RfField(bool present)
        {
            RaiseRfEvent(present ? ItFieldRising : ItFieldFalling, GpoFieldChangeEnable);
        }

        // RF side: the reader switches fast transfer mode on, as it has to
        // before exchanging mailbox messages.
        public void RfEnableMailbox()
        {
            mailboxMode = 0x01;
            if(!mailboxEnabled)
            {
                mailboxEnabled = true;
                hostPutMessage = false;
                rfPutMessage = false;
                hostMissMessage = false;
                rfMissMessage = false;
                mailboxLength = 0;
            }
        }

        // RF-side mailbox access (monitor-driven)
        public void RfPutMailboxMessage(string hexBytes)
        {
            var payload = ParseHex(hexBytes);
            if(!mailboxEnabled)
            {
                this.Log(LogLevel.Warning, "Mailbox: RF put ignored, mailbox not enabled");
                return;
            }
            if(payload.Length == 0 || payload.Length > MailboxSize)
            {
                throw new RecoverableException($"message must be 1..{MailboxSize} bytes");
            }
            if(hostPutMessage)
            {
                hostMissMessage = true; // host message overwritten before being read
            }
            Array.Copy(payload, mailbox, payload.Length);
            mailboxLength = payload.Length;
            rfPutMessage = true;
            hostPutMessage = false;
            rfReadProgress = 0;
            RaiseRfEvent(ItRfPutMessage, GpoRfPutMessageEnable);
        }

        // Records an RF event in IT_STS_Dyn and, if GPO is enabled for it,
        // pulses the GPO line low.
        private void RaiseRfEvent(byte statusBit, byte gpoEnableBit)
        {
            interruptStatus |= statusBit;
            if((systemGpo & GpoEnable) == 0 || (systemGpo & gpoEnableBit) == 0)
            {
                return;
            }
            SyncGpo();
            GPO.Set(false);
            gpoPulseTimer.Enabled = false;
            gpoPulseTimer.Value = 0;
            gpoPulseTimer.Enabled = true;
        }

        // Makes the SoC pin see the line's idle (high) level: the level set
        // before the pin was connected, or before the SoC's GPIO port was
        // reset, never reached it, and a pulse would then start without an edge.
        private void SyncGpo()
        {
            if(gpoSynced)
            {
                return;
            }
            gpoSynced = true;
            GPO.Set(false);
            GPO.Set(true);
        }

        public string RfReadMailboxMessage()
        {
            if(!hostPutMessage)
            {
                return "";
            }
            hostPutMessage = false; // RF completed the read
            return BitConverter.ToString(mailbox, 0, mailboxLength);
        }

        // -------------------- location dispatch --------------------

        private byte ReadLocation(ushort address, bool systemArea)
        {
            if(systemArea)
            {
                return ReadSystem(address);
            }
            if(address < memorySize)
            {
                return eeprom[address];
            }
            if(address >= DynamicRegistersBase && address < MailboxBase)
            {
                return ReadDynamic(address);
            }
            if(address >= MailboxBase && address < MailboxBase + MailboxSize)
            {
                var index = address - MailboxBase;
                var value = mailbox[index];
                if(rfPutMessage && index == mailboxLength - 1)
                {
                    // host finished reading the RF message
                    rfPutMessage = false;
                }
                return value;
            }
            this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=R width=1 offset=0x{0:X4} value=0xFF pc=0x0 (outside user memory/dynamic/mailbox)", address);
            return 0xFF;
        }

        private void WriteLocation(ushort address, byte value, bool systemArea)
        {
            if(systemArea)
            {
                WriteSystem(address, value);
                return;
            }
            if(address < memorySize)
            {
                // area write protection (I2CSS) is modelled only as
                // security-session gating of protected areas when configured
                if(IsUserAreaWriteProtected(address) && !securitySessionOpen)
                {
                    this.Log(LogLevel.Warning, "EEPROM write to 0x{0:X4} blocked: area write-protected and no I2C security session", address);
                    return;
                }
                eeprom[address] = value;
                return;
            }
            if(address >= DynamicRegistersBase && address < MailboxBase)
            {
                WriteDynamic(address, value);
                return;
            }
            if(address >= MailboxBase && address < MailboxBase + MailboxSize)
            {
                if(!mailboxEnabled)
                {
                    this.Log(LogLevel.Warning, "Mailbox write to 0x{0:X4} ignored: MB_EN is 0", address);
                    return;
                }
                if(rfPutMessage)
                {
                    rfMissMessage = true; // RF message overwritten before host read it
                    rfPutMessage = false;
                }
                mailbox[address - MailboxBase] = value;
                mailboxWriteCount = Math.Max(mailboxWriteCount, address - MailboxBase + 1);
                return;
            }
            this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=W width=1 offset=0x{0:X4} value=0x{1:X2} pc=0x0 (outside user memory/dynamic/mailbox)", address, value);
        }

        private bool IsUserAreaWriteProtected(ushort address)
        {
            // I2CSS (system 0x000B): bits1/3/5/7 = write protection of areas
            // 1..4; area boundaries from ENDAn (8-block granularity)
            var block = address / BlockSizeBytes / 8;
            int area;
            if(block <= endArea1)
            {
                area = 0;
            }
            else if(block <= endArea2)
            {
                area = 1;
            }
            else if(block <= endArea3)
            {
                area = 2;
            }
            else
            {
                area = 3;
            }
            return ((systemI2cSecurity >> (2 * area + 1)) & 1) != 0;
        }

        // -------------------- dynamic registers (0x2000..0x2007) --------------------

        private byte ReadDynamic(ushort address)
        {
            switch(address)
            {
            case GpoCtrlDyn:
                return gpoCtrl;
            case EhCtrlDyn:
                return ehCtrl;
            case RfMngtDyn:
                return rfManagement;
            case I2cSsoDyn:
                return (byte)(securitySessionOpen ? 0x01 : 0x00);
            case ItStsDyn:
            {
                var value = interruptStatus;
                interruptStatus = 0; // clear-on-read
                return value;
            }
            case MbCtrlDyn:
                // st25dv_reg.h: bit0 MBEN, bit1 HOSTPUTMSG, bit2 RFPUTMSG,
                // bit4 HOSTMISSMSG, bit5 RFMISSMSG, bits6-7 CURRENTMSG
                return (byte)((mailboxEnabled ? 0x01 : 0x00)
                    | (hostPutMessage ? 0x02 : 0x00)
                    | (rfPutMessage ? 0x04 : 0x00)
                    | (hostMissMessage ? 0x10 : 0x00)
                    | (rfMissMessage ? 0x20 : 0x00)
                    | (hostPutMessage ? 0x40 : 0x00)
                    | (rfPutMessage ? 0x80 : 0x00));
            case MbLenDyn:
                return (byte)(mailboxLength > 0 ? mailboxLength - 1 : 0);
            default:
                this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=R width=1 offset=0x{0:X4} value=0x00 pc=0x0 (dynamic)", address);
                return 0x00;
            }
        }

        private void WriteDynamic(ushort address, byte value)
        {
            switch(address)
            {
            case GpoCtrlDyn:
                gpoCtrl = value;
                break;
            case EhCtrlDyn:
                ehCtrl = (byte)(value & 0x01); // only EH_EN is writable
                break;
            case RfMngtDyn:
                rfManagement = (byte)(value & 0x03);
                break;
            case MbCtrlDyn:
                if((value & 0x01) != 0)
                {
                    if((mailboxMode & 0x01) == 0)
                    {
                        // MB_EN can only be set while MB_MODE.MB_MODE = 1
                        this.Log(LogLevel.Warning, "MB_CTRL_Dyn.MB_EN=1 ignored: system MB_MODE.MB_MODE is 0");
                    }
                    else if(!mailboxEnabled)
                    {
                        mailboxEnabled = true;
                        // enabling resets the mailbox status
                        hostPutMessage = false;
                        rfPutMessage = false;
                        hostMissMessage = false;
                        rfMissMessage = false;
                        mailboxLength = 0;
                    }
                }
                else
                {
                    mailboxEnabled = false;
                    hostPutMessage = false;
                    rfPutMessage = false;
                    hostMissMessage = false;
                    rfMissMessage = false;
                    mailboxLength = 0;
                }
                break;
            case I2cSsoDyn:
            case ItStsDyn:
            case MbLenDyn:
                this.Log(LogLevel.Warning, "Write to read-only dynamic register 0x{0:X4}, ignoring", address);
                break;
            default:
                this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=W width=1 offset=0x{0:X4} value=0x{1:X2} pc=0x0 (dynamic)", address, value);
                break;
            }
        }

        // -------------------- system area (device address 0x57 / helpers) ----------

        private byte ReadSystem(ushort address)
        {
            switch(address)
            {
            case SystemGpo:
                return systemGpo;
            case SystemItTime:
                return systemItTime;
            case SystemEhMode:
                return systemEhMode;
            case SystemRfMngt:
                return systemRfManagement;
            case SystemRfA1Ss:
                return areaSecurity[0];
            case SystemEndA1:
                return endArea1;
            case SystemRfA2Ss:
                return areaSecurity[1];
            case SystemEndA2:
                return endArea2;
            case SystemRfA3Ss:
                return areaSecurity[2];
            case SystemEndA3:
                return endArea3;
            case SystemRfA4Ss:
                return areaSecurity[3];
            case SystemI2cSs:
                return systemI2cSecurity;
            case SystemLockCcFile:
                return lockCcFile;
            case SystemMbMode:
                return mailboxMode;
            case SystemMbWdg:
                return mailboxWatchdog;
            case SystemLockCfg:
                return lockConfiguration;
            case SystemLockDsfid:
                return lockDsfid;
            case SystemLockAfi:
                return lockAfi;
            case SystemDsfid:
                return dsfid;
            case SystemAfi:
                return afi;
            case SystemMemSizeLsb:
                return (byte)MemorySizeBlocks;
            case SystemMemSizeMsb:
                return (byte)(MemorySizeBlocks >> 8);
            case SystemBlkSize:
                return BlockSizeBytes - 1;
            case SystemIcRef:
                return ChipId;
            case SystemIcRev:
                return IcRevision;
            default:
                if(address >= SystemUid && address < SystemUid + 8)
                {
                    return BuildUid()[address - SystemUid];
                }
                if(address >= SystemI2cPwd && address < SystemI2cPwd + 8)
                {
                    return 0x00; // password always reads as zero
                }
                this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=R width=1 offset=0x{0:X4} value=0x00 pc=0x0 (system)", address);
                return 0x00;
            }
        }

        private void WriteSystem(ushort address, byte value)
        {
            // static configuration registers require an open security session
            // on hardware; enforced except for the password presentation flow
            if(address < SystemI2cPwd && !securitySessionOpen)
            {
                this.Log(LogLevel.Warning, "System register write 0x{0:X4}=0x{1:X2} blocked: I2C security session closed", address, value);
                return;
            }
            switch(address)
            {
            case SystemGpo:
                systemGpo = value;
                break;
            case SystemItTime:
                systemItTime = value;
                break;
            case SystemEhMode:
                systemEhMode = value;
                break;
            case SystemRfMngt:
                systemRfManagement = value;
                break;
            case SystemRfA1Ss:
                areaSecurity[0] = value;
                break;
            case SystemEndA1:
                endArea1 = value;
                break;
            case SystemRfA2Ss:
                areaSecurity[1] = value;
                break;
            case SystemEndA2:
                endArea2 = value;
                break;
            case SystemRfA3Ss:
                areaSecurity[2] = value;
                break;
            case SystemEndA3:
                endArea3 = value;
                break;
            case SystemRfA4Ss:
                areaSecurity[3] = value;
                break;
            case SystemI2cSs:
                systemI2cSecurity = value;
                break;
            case SystemLockCcFile:
                lockCcFile = value;
                break;
            case SystemMbMode:
                mailboxMode = (byte)(value & 0x01);
                break;
            case SystemMbWdg:
                mailboxWatchdog = (byte)(value & 0x07);
                break;
            case SystemLockCfg:
                lockConfiguration = (byte)(value & 0x01);
                break;
            case SystemDsfid:
                dsfid = value;
                break;
            case SystemAfi:
                afi = value;
                break;
            default:
                if(address >= SystemI2cPwd && address < SystemI2cPwd + 17)
                {
                    HandlePasswordByte(address - SystemI2cPwd, value);
                    return;
                }
                this.Log(LogLevel.Warning, "UNIMPL periph=ST25DV dir=W width=1 offset=0x{0:X4} value=0x{1:X2} pc=0x0 (system)", address, value);
                break;
            }
        }

        private void HandlePasswordByte(int index, byte value)
        {
            // presentation sequence: 8 password bytes, validation code
            // (0x09 present / 0x07 write), 8 password bytes again
            if(index < 8)
            {
                passwordCandidate[index] = value;
            }
            else if(index == 8)
            {
                passwordValidationCode = value;
            }
            else
            {
                passwordConfirmation[index - 9] = value;
                if(index == 16)
                {
                    if(!passwordCandidate.SequenceEqual(passwordConfirmation))
                    {
                        this.Log(LogLevel.Warning, "I2C password sequence halves differ, ignoring");
                        return;
                    }
                    if(passwordValidationCode == 0x09)
                    {
                        securitySessionOpen = passwordCandidate.SequenceEqual(password);
                        this.Log(LogLevel.Debug, "I2C password presented, session {0}", securitySessionOpen ? "open" : "REJECTED");
                    }
                    else if(passwordValidationCode == 0x07 && securitySessionOpen)
                    {
                        Array.Copy(passwordCandidate, password, 8);
                        this.Log(LogLevel.Debug, "I2C password changed");
                    }
                }
            }
        }

        private byte[] BuildUid()
        {
            // ISO/IEC 15693 UID: E0 (byte 7), 0x02 ST (byte 6), product code
            // (byte 5), 40-bit serial — returned MSB first
            var uid = new byte[8];
            uid[0] = 0xE0;
            uid[1] = 0x02;
            uid[2] = ChipId;
            for(var i = 0; i < 5; i++)
            {
                uid[3 + i] = (byte)(UidSerial >> (8 * (4 - i)));
            }
            return uid;
        }

        private static byte[] ParseHex(string hexBytes)
        {
            var cleaned = hexBytes.Replace(" ", "").Replace("-", "").Replace("0x", "").Replace(",", "");
            if(cleaned.Length % 2 != 0)
            {
                throw new RecoverableException("hex string must contain an even number of digits");
            }
            return Enumerable.Range(0, cleaned.Length / 2)
                .Select(i => Convert.ToByte(cleaned.Substring(2 * i, 2), 16))
                .ToArray();
        }

        // -------------------- state --------------------

        private TransferState transferState;
        private byte addressHigh;
        private ushort currentAddress;
        private int mailboxWriteCount;
        private int rfReadProgress;

        private byte gpoCtrl;
        private byte ehCtrl;
        private byte rfManagement;
        private bool securitySessionOpen;
        private byte interruptStatus;
        private bool gpoSynced;
        private readonly LimitTimer gpoPulseTimer;

        // GPO register enables and IT_STS_Dyn bits
        private const byte GpoFieldChangeEnable = 0x08;
        private const byte GpoRfPutMessageEnable = 0x10;
        private const byte GpoEnable = 0x80;
        private const byte ItFieldFalling = 0x08;
        private const byte ItFieldRising = 0x10;
        private const byte ItRfPutMessage = 0x20;
        private const ulong GpoPulseMicroseconds = 300;
        private bool mailboxEnabled;
        private bool hostPutMessage;
        private bool rfPutMessage;
        private bool hostMissMessage;
        private bool rfMissMessage;
        private int mailboxLength;

        private byte systemGpo;
        private byte systemItTime;
        private byte systemEhMode;
        private byte systemRfManagement;
        private byte[] areaSecurity = new byte[4];
        private byte endArea1;
        private byte endArea2;
        private byte endArea3;
        private byte systemI2cSecurity;
        private byte lockCcFile;
        private byte mailboxMode;
        private byte mailboxWatchdog;
        private byte lockConfiguration;
        private byte lockDsfid;
        private byte lockAfi;
        private byte dsfid;
        private byte afi;
        private byte[] password = new byte[8];
        private readonly byte[] passwordCandidate = new byte[8];
        private readonly byte[] passwordConfirmation = new byte[8];
        private byte passwordValidationCode;

        private readonly byte[] eeprom;
        private readonly byte[] mailbox = new byte[MailboxSize];
        private readonly int memorySize;
        private readonly byte chipId;

        private const byte CorruptedId = 0xFF;
        private const byte BlockSizeBytes = 4;
        private const ushort DynamicRegistersBase = 0x2000;
        private const ushort MailboxBase = 0x2008;
        private const int MailboxSize = 256;

        // dynamic registers (st25dv_reg.h)
        private const ushort GpoCtrlDyn = 0x2000;
        private const ushort EhCtrlDyn = 0x2002;
        private const ushort RfMngtDyn = 0x2003;
        private const ushort I2cSsoDyn = 0x2004;
        private const ushort ItStsDyn = 0x2005;
        private const ushort MbCtrlDyn = 0x2006;
        private const ushort MbLenDyn = 0x2007;

        // system registers (st25dv_reg.h)
        private const ushort SystemGpo = 0x0000;
        private const ushort SystemItTime = 0x0001;
        private const ushort SystemEhMode = 0x0002;
        private const ushort SystemRfMngt = 0x0003;
        private const ushort SystemRfA1Ss = 0x0004;
        private const ushort SystemEndA1 = 0x0005;
        private const ushort SystemRfA2Ss = 0x0006;
        private const ushort SystemEndA2 = 0x0007;
        private const ushort SystemRfA3Ss = 0x0008;
        private const ushort SystemEndA3 = 0x0009;
        private const ushort SystemRfA4Ss = 0x000A;
        private const ushort SystemI2cSs = 0x000B;
        private const ushort SystemLockCcFile = 0x000C;
        private const ushort SystemMbMode = 0x000D;
        private const ushort SystemMbWdg = 0x000E;
        private const ushort SystemLockCfg = 0x000F;
        private const ushort SystemLockDsfid = 0x0010;
        private const ushort SystemLockAfi = 0x0011;
        private const ushort SystemDsfid = 0x0012;
        private const ushort SystemAfi = 0x0013;
        private const ushort SystemMemSizeLsb = 0x0014;
        private const ushort SystemMemSizeMsb = 0x0015;
        private const ushort SystemBlkSize = 0x0016;
        private const ushort SystemIcRef = 0x0017;
        private const ushort SystemUid = 0x0018;
        private const ushort SystemIcRev = 0x0020;
        private const ushort SystemI2cPwd = 0x0900;

        private enum TransferState
        {
            Idle,
            AddressHighReceived,
            AddressReceived,
            Data,
        }
    }

    // Companion peripheral for boards that additionally wire the ST25DV
    // system device address (0x57 on hardware; configurable here by choosing
    // the registration point):
    //   st25dv_sys: Sensors.ST_ST25DV_SystemPort @ twim1 0x57
    //       target: st25dv
    // It forwards to the same device state as the main ST_ST25DV instance.
    public class ST_ST25DV_SystemPort : II2CPeripheral
    {
        public ST_ST25DV_SystemPort(ST_ST25DV target)
        {
            this.target = target;
        }

        public void Reset()
        {
            state = 0;
        }

        public void Write(byte[] data)
        {
            var index = 0;
            if(state == 0 && index < data.Length)
            {
                high = data[index++];
                state = 1;
            }
            if(state == 1 && index < data.Length)
            {
                address = (ushort)((high << 8) | data[index++]);
                state = 2;
            }
            for(; index < data.Length; index++)
            {
                target.WriteSystemRegister(address, data[index]);
                address++;
            }
        }

        public byte[] Read(int count = 1)
        {
            var result = new byte[count];
            for(var i = 0; i < count; i++)
            {
                result[i] = target.ReadSystemRegister(address);
                address++;
            }
            return result;
        }

        public void FinishTransmission()
        {
            state = 0;
        }

        private readonly ST_ST25DV target;
        private int state;
        private byte high;
        private ushort address;
    }
}
