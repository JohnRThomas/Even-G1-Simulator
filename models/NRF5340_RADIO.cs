//
// NRF5340_RADIO — nRF5340 network-core 2.4 GHz radio (0x41008000, IRQ 8).
//
// Register layout verbatim from the pinned network-core SVD
// (svd/nrf5340_network.svd, hal_nordic 9784731461018d3e983604698fbbed6af2bea801);
// see reference/nrf5340_regs.rs `net::radio`. Behavioural semantics from the
// nRF5340 Product Specification, RADIO chapter
// (https://docs.nordicsemi.com/bundle/ps_nrf5340/page/radio.html), cross-checked
// against the pinned consumer sdk-nrf v2.5.1 subsys/esb/esb.c.
//
// Modelled behaviour
//   - Full task/event/short set with the SVD bit assignments; INTENSET/INTENCLR;
//     all SUBSCRIBE_*/PUBLISH_* endpoints wired to the network-core DPPI hub
//     (models/NRF5340_DPPIC.cs, see models/DPPI_README.md).
//   - STATE machine: DISABLED/RXRU/RXIDLE/RX/RXDISABLE/TXRU/TXIDLE/TX/TXDISABLE
//     with deterministic virtual-time ramp-ups and packet timing (see the
//     "Timing model" comment below for values and sources).
//   - EasyDMA frame assembly/parse at PACKETPTR per PCNF0/PCNF1
//     (S0/LENGTH/S1/payload, STATLEN, MAXLEN clamp, CRCINC).
//   - Renode wireless medium via IRadio: FrameSent on TX, ReceiveFrame on RX,
//     with address/pipe filtering (BASE0/BASE1 + PREFIX0/PREFIX1, BALEN 2..4 —
//     3-byte on-air addresses, BALEN=2, are fully supported), RXMATCH,
//     real CRC computation per CRCCNF/CRCPOLY/CRCINIT (CRCCNF.LEN is the ONLY
//     source of CRC length; it is never derived from the address length).
//   - RSSI (RSSISAMPLE injectable via the RssiSample property), bit counter
//     (BCC/BCSTART/BCMATCH), 802.15.4-ish ED/CCA (EDSAMPLE injectable via
//     EdLevel, CCA verdict via CcaBusy), POWER reset semantics.
//
// On-air ("wire") format between two instances of this model — a logical
// byte-level representation, not a bit-exact air capture:
//   [address: BALEN base bytes (most-significant bytes of BASEx, in
//    transmission order) + 1 prefix byte] [S0 bytes] [LENGTH byte]
//   [S1 byte, if present in RAM] [payload] [CRC, CRCCNF.LEN bytes, MSB first]
//   (Ieee802154_250Kbit mode: [SFD byte] [LENGTH byte] [payload] [FCS]).
// Preamble is implicit (timing only). Whitening (WHITEEN/DATAWHITEIV) is
// symmetric on air and therefore transparent in this representation; the
// registers are stored but no whitening transform is applied. The CRC is
// computed bit-serially MSB-first over the logical bytes selected by
// CRCCNF.SKIPADDR with the configured CRCPOLY/CRCINIT — for the classic ESB
// CRC16-CCITT configuration this matches hardware values; for configurations
// whose air header is not byte-aligned (e.g. ESB DPL 6-bit LENGTH) the CRC
// *value* differs from real air capture but match/mismatch semantics between
// two instances of this model are exact.
//
// Timing model (virtual time, deterministic; 16 MHz tick base):
//   ramp-up TXEN->READY: 129 us, RXEN->READY: 124 us (MODECNF0.RU=Default),
//     40 us both for RU=Fast — nRF5340 PS "Radio timing parameters"
//     (tTXEN/tRXEN, tTXEN,FAST/tRXEN,FAST) as used by the pinned consumer
//     sdk-nrf v2.5.1 subsys/esb/esb.c:51-57 (TX_RAMP_UP_TIME_US=129,
//     RX_RAMP_UP_TIME_US=124, TX_FAST_RAMP_UP_TIME_US=40).
//   DISABLE: TX* -> DISABLED 6 us, RX* -> DISABLED 1 us (PS tTXDIS ~4-6 us,
//     tRXDIS ~0.5 us, rounded to whole us) when a shortcut or DPPI triggers
//     it; at once when the firmware writes TASKS_DISABLE (see
//     DisabledByFirmware). TASKS_DISABLE in DISABLED re-raises
//     EVENTS_DISABLED synchronously.
//   byte time per MODE: 1 Mbit modes 8 us, 2 Mbit 4 us, LR500Kbit 16 us,
//     LR125Kbit 64 us, Ieee802154_250Kbit 32 us.
//   preamble length from PCNF0.PLEN: 8bit=1, 16bit=2, 32bitZero=4 bytes,
//     LongRange=10 byte-times (approximation of the 80 us coded preamble).
//   EVENTS_ADDRESS at (preamble+address) time after START; PAYLOAD/END/PHYEND
//     at packet end; CCA verdict after 128 us (8 symbols); ED after
//     (EDCNT+1)*128 us; RSSISTART->RSSIEND instant (hardware: 0.25 us).
//   TIFS is stored but not enforced (UNSUPPORTED log on first nonzero write);
//     inter-frame spacing emerges from ramp-up times only.
//
// Not behaviourally modelled (registers present, values stored, honest
// UNSUPPORTED log when a non-reset value is written / the feature is armed):
//   device-address matching (DAB/DAP/DACNF -> no DEVMATCH/DEVMISS),
//   MHRMATCH, SYNC, RATEBOOST, CTE/DFE block (0x900+), PDUSTAT/CTESTATUS/
//   DFESTATUS read as 0, TIFS enforcement, EDCNT semantics beyond duration.
//
// Fault injection (monitor-settable, default off):
//   FaultCrcError      — the next received frame reports CRCERROR/CRCSTATUS=0
//   FaultDropInterrupt — swallow the next rising IRQ edge
// Injection helpers: RssiSample, EdLevel, CcaBusy properties and
//   InjectFrame("AABB...") to feed a frame without a second machine.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Wireless
{
    public class NRF5340_RADIO : BasicDoubleWordPeripheral, IRadio, IKnownSize, INRFEventProvider
    {
        public NRF5340_RADIO(IMachine machine, NRF5340_DPPIC dppic = null) : base(machine)
        {
            IRQ = new GPIO();

            subscribeTask = new DPPISubscribeEndpoint[TaskCount];
            for(var i = 0; i < TaskCount; i++)
            {
                var task = (Tasks)i;
                subscribeTask[i] = new DPPISubscribeEndpoint(dppic, this, task.ToString(), () => RunTask(task));
            }
            publishEvent = new DPPIPublishEndpoint[EventBitCount];
            foreach(Events ev in Enum.GetValues(typeof(Events)))
            {
                publishEvent[(int)ev] = new DPPIPublishEndpoint(dppic, this, ev.ToString());
            }

            rampTimer = new DelayedAction(machine, this, "radioRamp");
            addressTimer = new DelayedAction(machine, this, "radioAddress");
            endTimer = new DelayedAction(machine, this, "radioEnd");
            disableTimer = new DelayedAction(machine, this, "radioDisable");
            bcTimer = new DelayedAction(machine, this, "radioBitCounter");
            edTimer = new DelayedAction(machine, this, "radioEnergyDetect");
            ccaTimer = new DelayedAction(machine, this, "radioCca");

            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            radioState = State.Disabled;
            eventPending = 0;
            interruptEnable = 0;
            rxMatch = 0;
            rxCrc = 0;
            crcStatusOk = false;
            tifsWarned = false;
            dacnfWarned = false;
            dacnfRaw = 0;
            edSampleValue = 0;
            Array.Clear(addressPrefixes, 0, addressPrefixes.Length);
            Array.Clear(deviceAddressBase, 0, deviceAddressBase.Length);
            Array.Clear(deviceAddressPrefix, 0, deviceAddressPrefix.Length);
            for(var i = 0; i < pselDfeGpio.Length; i++)
            {
                pselDfeGpio[i] = 0xFFFFFFFF;
            }
            foreach(var endpoint in subscribeTask)
            {
                endpoint.Reset();
            }
            foreach(var endpoint in publishEvent)
            {
                endpoint?.Reset();
            }
            rampTimer.Cancel();
            addressTimer.Cancel();
            endTimer.Cancel();
            disableTimer.Cancel();
            bcTimer.Cancel();
            edTimer.Cancel();
            ccaTimer.Cancel();
            pendingFrame = null;
            pendingSender = null;
            IRQ.Unset();
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; }

        public event Action<uint> EventTriggered;
        public event Action<IRadio, byte[]> FrameSent;

        // Fired when the radio begins ramping up for reception (TASKS_RXEN),
        // i.e. an RX window is opening. A virtual peer (BLE_VirtualCentral)
        // uses this to inject a frame that will be buffered as pendingFrame and
        // delivered when the receiver reaches RX — the way a real peer's packet
        // would already be on the air at the anchor point.
        public event Action RxWindowOpened;

        // ---- virtual-peer helpers (used by BLE_VirtualCentral) ----

        // On-air access-address bytes currently configured for a logical pipe
        // (LSB-first, BALEN+1 bytes). Pipe 0 is the one the SDC uses for both
        // advertising RX and connection-event RX.
        public byte[] OnAirAddressPublic(int pipe) => OnAirAddress((uint)pipe);

        // The 3-byte (or CRCCNF.LEN-byte) CRC, computed with the radio's CURRENT
        // CRCPOLY/CRCINIT over a frame's PDU (everything after the on-air
        // address), in the same MSB-first order the TX path appends and the RX
        // path verifies. Pass the frame WITHOUT its trailing CRC.
        public byte[] BuildFrameCrc(byte[] frameWithoutCrc)
        {
            var crcStart = CrcStartIndex(AddressLength);
            var crc = ComputeCrc(frameWithoutCrc, crcStart, frameWithoutCrc.Length - crcStart);
            var len = (int)crcLength.Value;
            var outb = new byte[len];
            for(var i = 0; i < len; i++)
            {
                outb[i] = (byte)(crc >> (8 * (len - 1 - i)));
            }
            return outb;
        }

        // Build a complete on-air frame addressed to pipe 0 (the master's own
        // address), carrying `payload`, using the radio's LIVE PCNF (S0/LEN/S1
        // field sizes) and CRCCNF/CRCPOLY/CRCINIT — the same layout BeginTransmission
        // produces. Lets a virtual peer emit an ESB ACK (typically zero-length
        // payload) that this radio's RX path parses and CRC-verifies identically.
        public byte[] BuildAckFrame(byte[] payload)
        {
            payload = payload ?? new byte[0];
            var s0Bytes = (int)s0Length.Value;
            var lengthBytes = lengthFieldLength.Value == 0 ? 0 : 1;
            var s1BytesAir = (s1Length.Value > 0) ? 1 : 0;
            var crcLen = (int)crcLength.Value;
            var frame = new System.Collections.Generic.List<byte>();
            frame.AddRange(OnAirAddress(0));
            for(var i = 0; i < s0Bytes; i++)
            {
                frame.Add(0);
            }
            if(lengthBytes > 0)
            {
                frame.Add((byte)(payload.Length & ((1 << (int)lengthFieldLength.Value) - 1)));
            }
            if(s1BytesAir > 0)
            {
                frame.Add(0);
            }
            frame.AddRange(payload);
            var crcStart = CrcStartIndex(AddressLength);
            var crc = ComputeCrc(frame.ToArray(), crcStart, frame.Count - crcStart);
            for(var i = crcLen - 1; i >= 0; i--)
            {
                frame.Add((byte)(crc >> (8 * i)));
            }
            return frame.ToArray();
        }

        // Radio state as an int (0=Disabled,1=RxRu,2=RxIdle,3=Rx,...,9=TxRu,10=TxIdle,11=Tx).
        public int RadioStateRaw => (int)radioState;

        // CRC length (bytes) currently programmed (CRCCNF.LEN). ESB uses 2,
        // BLE uses 3. Lets a virtual peer strip/rebuild the trailing CRC of a
        // captured frame with the live configuration.
        public int CrcLengthPublic => (int)crcLength.Value;

        // ---- injection / fault knobs (docs/conventions.md) ----

        // RSSISAMPLE value (positive dBm loss: 60 => -60 dBm). Injectable.
        public uint RssiSample { get; set; } = 60;
        // EDSAMPLE.EDLVL reported after an ED scan. Injectable.
        public uint EdLevel { get; set; } = 0;
        // CCA verdict: false -> CCAIDLE, true -> CCABUSY.
        public bool CcaBusy { get; set; }
        // Force the next received frame to fail CRC.
        public bool FaultCrcError { get; set; }
        // Swallow the next rising IRQ edge.
        public bool FaultDropInterrupt { get; set; }

        // Observability for tests/UI: number of frames transmitted since
        // reset and the last transmitted frame as a hex string.
        public ulong TransmittedFrames { get; private set; }
        public string LastTransmittedFrame { get; private set; } = "";

        // Frequency in MHz: 2400 + FREQUENCY, or 2360 + FREQUENCY with MAP=Low.
        public uint FrequencyMHz => (frequencyMap.Value ? 2360u : 2400u) + (uint)frequency.Value;

        public int Channel
        {
            get => (int)FrequencyMHz - 2400;
            set => throw new RecoverableException("Channel follows the FREQUENCY register; write FREQUENCY instead");
        }

        // Feed a frame from the monitor without a peer machine, e.g.:
        //   radio InjectFrame "E7E7E7E701AA"
        public void InjectFrame(string hexBytes)
        {
            var clean = hexBytes.Replace(" ", "").Replace("_", "");
            if(clean.Length % 2 != 0)
            {
                throw new RecoverableException("Expected an even number of hex digits");
            }
            var frame = new byte[clean.Length / 2];
            for(var i = 0; i < frame.Length; i++)
            {
                frame[i] = Convert.ToByte(clean.Substring(2 * i, 2), 16);
            }
            ReceiveFrame(frame, null);
        }

        public void ReceiveFrame(byte[] frame, IRadio sender)
        {
            if(sender != null && sender.Channel != Channel)
            {
                this.Log(LogLevel.Noisy, "Dropping frame from channel {0} (listening on {1})", sender.Channel, Channel);
                return;
            }
            if(radioState == State.RxRu || radioState == State.RxIdle)
            {
                // The receiver is ramping up / about to START while this frame
                // is still on the air. Hardware syncs as long as enough of the
                // preamble+address remains; model that by buffering one frame
                // for the preamble+address+header window and delivering it if
                // the radio reaches RX within that window (see TaskStart).
                // Only a frame the receiver would sync to: one for another
                // address must not take the place of one that is waiting.
                if(mode.Value != Mode.Ieee802154_250Kbit && MatchingPipe(frame) < 0)
                {
                    this.Log(LogLevel.Noisy, "Dropping frame that arrived in state {0}: no enabled logical address matched", radioState);
                    return;
                }
                pendingFrame = frame;
                pendingSender = sender;
                pendingSince = machine.LocalTimeSource.ElapsedVirtualTime;
                this.Log(LogLevel.Noisy, "Frame arrived in state {0}; buffering for the sync window", radioState);
                return;
            }
            if(radioState != State.Rx)
            {
                this.Log(LogLevel.Noisy, "Dropping frame received in state {0} (receiver not started)", radioState);
                return;
            }
            ProcessRxFrame(frame, sender);
        }

        // The enabled logical address (pipe) a frame is addressed to, or -1.
        private int MatchingPipe(byte[] frame)
        {
            var addressLength = AddressLength;
            if(frame.Length < addressLength)
            {
                return -1;
            }
            for(var p = 0; p < 8; p++)
            {
                if(!rxAddressEnabled[p].Value)
                {
                    continue;
                }
                var expected = OnAirAddress((uint)p);
                var match = true;
                for(var i = 0; i < addressLength; i++)
                {
                    if(frame[i] != expected[i])
                    {
                        match = false;
                        break;
                    }
                }
                if(match)
                {
                    return p;
                }
            }
            return -1;
        }

        private void ProcessRxFrame(byte[] frame, IRadio sender)
        {
            if(mode.Value == Mode.Ieee802154_250Kbit)
            {
                Receive802154Frame(frame);
                return;
            }

            var addressLength = AddressLength;
            if(frame.Length < addressLength)
            {
                this.Log(LogLevel.Warning, "Dropping runt frame ({0} bytes, address alone is {1})", frame.Length, addressLength);
                return;
            }
            var pipe = MatchingPipe(frame);
            if(pipe < 0)
            {
                this.Log(LogLevel.Noisy, "Dropping frame: no enabled logical address matched");
                return;
            }

            // Header parse (wire mirrors the RAM header layout, see file header)
            var offset = addressLength;
            var s0Bytes = (int)s0Length.Value;
            var lengthBytes = lengthFieldLength.Value == 0 ? 0 : 1;
            var s1Bytes = (s1Length.Value > 0) ? 1 : 0;
            var header = new byte[s0Bytes + lengthBytes + s1Bytes];
            for(var i = 0; i < header.Length && offset + i < frame.Length; i++)
            {
                header[i] = frame[offset + i];
            }
            uint airPayloadLength;
            if(lengthBytes > 0)
            {
                airPayloadLength = header[s0Bytes] & ((1u << (int)lengthFieldLength.Value) - 1);
            }
            else
            {
                airPayloadLength = (uint)staticLength.Value;
            }
            var crcLen = (int)crcLength.Value;
            var declared = airPayloadLength;
            if(crcIncludedInLength.Value && declared >= (uint)crcLen)
            {
                declared -= (uint)crcLen;
            }
            var payloadLength = declared;
            if(lengthBytes > 0)
            {
                payloadLength += (uint)staticLength.Value;
            }
            if(payloadLength > maxPacketLength.Value)
            {
                this.Log(LogLevel.Warning, "Received payload length {0} exceeds MAXLEN {1}, clamping", payloadLength, maxPacketLength.Value);
                payloadLength = (uint)maxPacketLength.Value;
            }
            offset += header.Length;
            var payload = new byte[payloadLength];
            for(var i = 0; i < payloadLength && offset + i < frame.Length; i++)
            {
                payload[i] = frame[offset + i];
            }
            offset += (int)payloadLength;

            // CRC check: CRCCNF.LEN is the only CRC-length source.
            var receivedCrc = 0u;
            for(var i = 0; i < crcLen && offset + i < frame.Length; i++)
            {
                receivedCrc = (receivedCrc << 8) | frame[offset + i];
            }
            var computedCrc = ComputeCrc(frame, CrcStartIndex(addressLength), offset - CrcStartIndex(addressLength));
            var ok = crcLen == 0 || computedCrc == receivedCrc;
            if(FaultCrcError)
            {
                this.Log(LogLevel.Warning, "FaultCrcError active: forcing CRC failure on this frame");
                FaultCrcError = false;
                ok = false;
            }

            // EasyDMA writeback: S0, LENGTH, S1 (RAM layout), then payload.
            var ram = new byte[RamHeaderLength() + payloadLength];
            var ri = 0;
            for(var i = 0; i < s0Bytes; i++)
            {
                ram[ri++] = header[i];
            }
            if(lengthBytes > 0)
            {
                ram[ri++] = header[s0Bytes];
            }
            if(s1Bytes > 0 || s1Include.Value)
            {
                ram[ri++] = s1Bytes > 0 ? header[s0Bytes + lengthBytes] : (byte)0;
            }
            Array.Copy(payload, 0, ram, ri, payloadLength);
            sysbus.WriteBytes(ram, packetPointer.Value, true);

            this.Log(LogLevel.Noisy, "RX frame on pipe {0}: {1} payload bytes, crc {2}", pipe, payloadLength, ok ? "ok" : "ERROR");

            var byteTime = ByteTimeUs();
            var addressTime = (PreambleBytes() + addressLength) * byteTime;
            var endTime = addressTime + (header.Length + payloadLength + crcLen) * byteTime;
            var thisPipe = pipe;
            var thisCrcOk = ok;
            var thisCrc = receivedCrc;

            addressTimer.Schedule(addressTime, () =>
            {
                SetEvent(Events.Address);
                HandleAddressShorts();
                if(dacnfRaw != 0 && !dacnfWarned)
                {
                    dacnfWarned = true;
                    this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=device-address-match detail=DACNF=0x{1:X} armed but DAB/DAP matching is not modelled; DEVMATCH/DEVMISS will not fire", LabelForLog, dacnfRaw);
                }
            });
            endTimer.Schedule(endTime, () =>
            {
                rxMatch = (uint)thisPipe;
                rxCrc = thisCrc;
                crcStatusOk = thisCrcOk;
                radioState = State.RxIdle;
                SetEvent(Events.Payload);
                SetEvent(Events.End);
                SetEvent(Events.PhyEnd);
                SetEvent(thisCrcOk ? Events.CrcOk : Events.CrcError);
                HandleEndShorts();
            });
        }

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

        // ------------------------- register map -------------------------

        private void DefineRegisters()
        {
            // TASKS_* 0x000..0x030 and SUBSCRIBE_* 0x080..0x0B0 (task + 0x80)
            for(var i = 0; i < TaskCount; i++)
            {
                var task = (Tasks)i;
                var index = i;
                ((Registers)(4 * i)).Define(this, name: $"TASKS_{task}")
                    .WithFlag(0, FieldMode.Write, name: $"TASKS_{task}", writeCallback: (_, value) => { if(value) RunTask(task); })
                    .WithReservedBits(1, 31);
                ((Registers)(0x80 + 4 * i)).Define(this, name: $"SUBSCRIBE_{task}")
                    .WithValueField(0, 32, name: $"SUBSCRIBE_{task}",
                        valueProviderCallback: _ => subscribeTask[index].RegisterValue,
                        writeCallback: (_, value) => subscribeTask[index].RegisterValue = (uint)value);
            }

            // EVENTS_* 0x100 + 4*bit and PUBLISH_* 0x180 + 4*bit; the SVD event
            // register index equals the INTEN bit position for every event.
            foreach(Events ev in Enum.GetValues(typeof(Events)))
            {
                var bit = (int)ev;
                ((Registers)(0x100 + 4 * bit)).Define(this, name: $"EVENTS_{ev}")
                    .WithFlag(0, name: $"EVENTS_{ev}",
                        valueProviderCallback: _ => (eventPending & (1u << bit)) != 0,
                        writeCallback: (_, value) =>
                        {
                            if(value)
                            {
                                eventPending |= 1u << bit;
                            }
                            else
                            {
                                eventPending &= ~(1u << bit);
                            }
                            UpdateInterrupts();
                        })
                    .WithReservedBits(1, 31);
                ((Registers)(0x180 + 4 * bit)).Define(this, name: $"PUBLISH_{ev}")
                    .WithValueField(0, 32, name: $"PUBLISH_{ev}",
                        valueProviderCallback: _ => publishEvent[bit].RegisterValue,
                        writeCallback: (_, value) => publishEvent[bit].RegisterValue = (uint)value);
            }

            Registers.Shorts.Define(this)
                .WithFlag(0, out shortReadyStart, name: "READY_START")
                .WithFlag(1, out shortEndDisable, name: "END_DISABLE")
                .WithFlag(2, out shortDisabledTxen, name: "DISABLED_TXEN")
                .WithFlag(3, out shortDisabledRxen, name: "DISABLED_RXEN")
                .WithFlag(4, out shortAddressRssistart, name: "ADDRESS_RSSISTART")
                .WithFlag(5, out shortEndStart, name: "END_START")
                .WithFlag(6, out shortAddressBcstart, name: "ADDRESS_BCSTART")
                .WithReservedBits(7, 1)
                .WithFlag(8, out shortDisabledRssistop, name: "DISABLED_RSSISTOP")
                .WithReservedBits(9, 2)
                .WithFlag(11, out shortRxreadyCcastart, name: "RXREADY_CCASTART")
                .WithFlag(12, out shortCcaidleTxen, name: "CCAIDLE_TXEN")
                .WithFlag(13, out shortCcabusyDisable, name: "CCABUSY_DISABLE")
                .WithFlag(14, out shortFramestartBcstart, name: "FRAMESTART_BCSTART")
                .WithFlag(15, out shortReadyEdstart, name: "READY_EDSTART")
                .WithFlag(16, out shortEdendDisable, name: "EDEND_DISABLE")
                .WithFlag(17, out shortCcaidleStop, name: "CCAIDLE_STOP")
                .WithFlag(18, out shortTxreadyStart, name: "TXREADY_START")
                .WithFlag(19, out shortRxreadyStart, name: "RXREADY_START")
                .WithFlag(20, out shortPhyendDisable, name: "PHYEND_DISABLE")
                .WithFlag(21, out shortPhyendStart, name: "PHYEND_START")
                .WithReservedBits(22, 10);

            Registers.IntenSet.Define(this)
                .WithValueField(0, 32, name: "INTENSET",
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable |= (uint)value & EventMask; UpdateInterrupts(); });

            Registers.IntenClr.Define(this)
                .WithValueField(0, 32, name: "INTENCLR",
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable &= ~(uint)value; UpdateInterrupts(); });

            Registers.CrcStatus.Define(this)
                .WithFlag(0, FieldMode.Read, name: "CRCSTATUS", valueProviderCallback: _ => crcStatusOk)
                .WithReservedBits(1, 31);

            Registers.RxMatch.Define(this)
                .WithValueField(0, 3, FieldMode.Read, name: "RXMATCH", valueProviderCallback: _ => rxMatch)
                .WithReservedBits(3, 29);

            Registers.RxCrc.Define(this)
                .WithValueField(0, 24, FieldMode.Read, name: "RXCRC", valueProviderCallback: _ => rxCrc)
                .WithReservedBits(24, 8);

            Registers.Dai.Define(this)
                .WithValueField(0, 3, FieldMode.Read, name: "DAI", valueProviderCallback: _ => 0)
                .WithReservedBits(3, 29);

            Registers.PduStat.Define(this)
                .WithValueField(0, 3, FieldMode.Read, name: "PDUSTAT", valueProviderCallback: _ => 0)
                .WithReservedBits(3, 29);

            Registers.CteStatus.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "CTESTATUS", valueProviderCallback: _ => 0)
                .WithReservedBits(8, 24);

            Registers.DfeStatus.Define(this)
                .WithValueField(0, 5, FieldMode.Read, name: "DFESTATUS", valueProviderCallback: _ => 0)
                .WithReservedBits(5, 27);

            Registers.PacketPtr.Define(this, 0x01000000)
                .WithValueField(0, 32, out packetPointer, name: "PACKETPTR");

            Registers.Frequency.Define(this, 0x00000002)
                .WithValueField(0, 7, out frequency, name: "FREQUENCY")
                .WithReservedBits(7, 1)
                .WithFlag(8, out frequencyMap, name: "MAP")
                .WithReservedBits(9, 23);

            Registers.TxPower.Define(this)
                .WithValueField(0, 8, out txPower, name: "TXPOWER")
                .WithReservedBits(8, 24);

            Registers.Mode.Define(this)
                .WithEnumField(0, 4, out mode, name: "MODE")
                .WithReservedBits(4, 28);

            Registers.Pcnf0.Define(this)
                .WithValueField(0, 4, out lengthFieldLength, name: "LFLEN")
                .WithReservedBits(4, 4)
                .WithValueField(8, 1, out s0Length, name: "S0LEN")
                .WithReservedBits(9, 7)
                .WithValueField(16, 4, out s1Length, name: "S1LEN")
                .WithFlag(20, out s1Include, name: "S1INCL")
                .WithReservedBits(21, 1)
                .WithValueField(22, 2, out codeIndicatorLength, name: "CILEN")
                .WithValueField(24, 2, out preambleLength, name: "PLEN")
                .WithFlag(26, out crcIncludedInLength, name: "CRCINC")
                .WithReservedBits(27, 2)
                .WithValueField(29, 2, out termLength, name: "TERMLEN")
                .WithReservedBits(31, 1);

            Registers.Pcnf1.Define(this)
                .WithValueField(0, 8, out maxPacketLength, name: "MAXLEN")
                .WithValueField(8, 8, out staticLength, name: "STATLEN")
                .WithValueField(16, 3, out baseAddressLength, name: "BALEN")
                .WithReservedBits(19, 5)
                .WithFlag(24, out bigEndian, name: "ENDIAN")
                .WithFlag(25, out whiteningEnabled, name: "WHITEEN")
                .WithReservedBits(26, 6);

            Registers.Base0.Define(this)
                .WithValueField(0, 32, out baseAddress0, name: "BASE0");

            Registers.Base1.Define(this)
                .WithValueField(0, 32, out baseAddress1, name: "BASE1");

            Registers.Prefix0.Define(this)
                .WithValueFields(0, 8, 4, name: "AP",
                    valueProviderCallback: (i, _) => addressPrefixes[i],
                    writeCallback: (i, _, value) => addressPrefixes[i] = (byte)value);

            Registers.Prefix1.Define(this)
                .WithValueFields(0, 8, 4, name: "AP",
                    valueProviderCallback: (i, _) => addressPrefixes[4 + i],
                    writeCallback: (i, _, value) => addressPrefixes[4 + i] = (byte)value);

            Registers.TxAddress.Define(this)
                .WithValueField(0, 3, out txAddress, name: "TXADDRESS")
                .WithReservedBits(3, 29);

            Registers.RxAddresses.Define(this)
                .WithFlags(0, 8, out rxAddressEnabled, name: "ADDR")
                .WithReservedBits(8, 24);

            Registers.CrcCnf.Define(this)
                .WithValueField(0, 2, out crcLength, name: "LEN")
                .WithReservedBits(2, 6)
                .WithEnumField(8, 2, out crcSkipAddress, name: "SKIPADDR")
                .WithReservedBits(10, 22);

            Registers.CrcPoly.Define(this)
                .WithValueField(0, 24, out crcPolynomial, name: "CRCPOLY")
                .WithReservedBits(24, 8);

            Registers.CrcInit.Define(this)
                .WithValueField(0, 24, out crcInitialValue, name: "CRCINIT")
                .WithReservedBits(24, 8);

            Registers.Tifs.Define(this)
                .WithValueField(0, 10, out tifs, name: "TIFS", writeCallback: (_, value) =>
                {
                    if(value != 0 && !tifsWarned)
                    {
                        tifsWarned = true;
                        this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=TIFS detail=value {1} us stored; interframe spacing is not enforced, turnaround uses ramp-up times only", LabelForLog, value);
                    }
                })
                .WithReservedBits(10, 22);

            Registers.RssiSampleReg.Define(this)
                .WithValueField(0, 7, FieldMode.Read, name: "RSSISAMPLE", valueProviderCallback: _ => RssiSample & 0x7F)
                .WithReservedBits(7, 25);

            Registers.StateReg.Define(this)
                .WithEnumField<DoubleWordRegister, State>(0, 4, FieldMode.Read, name: "STATE", valueProviderCallback: _ => radioState)
                .WithReservedBits(4, 28);

            Registers.DataWhiteIv.Define(this, 0x00000040)
                .WithValueField(0, 7, out dataWhiteIv, name: "DATAWHITEIV")
                .WithReservedBits(7, 25);

            Registers.Bcc.Define(this)
                .WithValueField(0, 32, out bitCounterCompare, name: "BCC");

            Registers.Dab0.DefineMany(this, 8, (register, index) =>
            {
                register.WithValueField(0, 32, name: $"DAB[{index}]",
                    valueProviderCallback: _ => deviceAddressBase[index],
                    writeCallback: (_, value) => deviceAddressBase[index] = (uint)value);
            });

            Registers.Dap0.DefineMany(this, 8, (register, index) =>
            {
                register.WithValueField(0, 16, name: $"DAP[{index}]",
                        valueProviderCallback: _ => deviceAddressPrefix[index],
                        writeCallback: (_, value) => deviceAddressPrefix[index] = (uint)value)
                    .WithReservedBits(16, 16);
            });

            Registers.Dacnf.Define(this)
                .WithValueField(0, 16, name: "DACNF",
                    valueProviderCallback: _ => dacnfRaw,
                    writeCallback: (_, value) =>
                    {
                        dacnfRaw = (uint)value;
                        dacnfWarned = false;
                    })
                .WithReservedBits(16, 16);

            DefineStoredRegister(Registers.MhrMatchConf, "MHRMATCHCONF", 0, 32, 0);
            DefineStoredRegister(Registers.MhrMatchMas, "MHRMATCHMAS", 0, 32, 0);

            Registers.ModeCnf0.Define(this, 0x00000200)
                .WithFlag(0, out fastRampUp, name: "RU")
                .WithReservedBits(1, 7)
                .WithValueField(8, 2, out defaultTxValue, name: "DTX")
                .WithReservedBits(10, 22);

            Registers.Sfd.Define(this, 0x000000A7)
                .WithValueField(0, 8, out sfd, name: "SFD")
                .WithReservedBits(8, 24);

            Registers.EdCnt.Define(this)
                .WithValueField(0, 21, out edCount, name: "EDCNT")
                .WithReservedBits(21, 11);

            Registers.EdSample.Define(this)
                .WithValueField(0, 8, FieldMode.Read, name: "EDLVL", valueProviderCallback: _ => edSampleValue)
                .WithReservedBits(8, 24);

            Registers.CcaCtrl.Define(this, 0x052D0000)
                .WithEnumField(0, 3, out ccaMode, name: "CCAMODE")
                .WithReservedBits(3, 5)
                .WithValueField(8, 8, name: "CCAEDTHRES")
                .WithValueField(16, 8, name: "CCACORRTHRES")
                .WithValueField(24, 8, name: "CCACORRCNT");

            // CTE/DFE block — register presence only (values stored, honest logs)
            DefineStoredRegister(Registers.DfeMode, "DFEMODE", 0, 2, 0);
            DefineStoredRegister(Registers.CteInlineConf, "CTEINLINECONF", 0x2800, 32, 0x2800);
            DefineStoredRegister(Registers.DfeCtrl1, "DFECTRL1", 0x00023282, 28, 0x00023282);
            DefineStoredRegister(Registers.DfeCtrl2, "DFECTRL2", 0, 28, 0);
            DefineStoredRegister(Registers.SwitchPattern, "SWITCHPATTERN", 0, 8, 0);
            Registers.ClearPattern.Define(this)
                .WithFlag(0, FieldMode.Write, name: "CLEARPATTERN")
                .WithReservedBits(1, 31);
            Registers.PselDfeGpio0.DefineMany(this, 8, (register, index) =>
            {
                register.WithValueField(0, 32, name: $"PSEL_DFEGPIO[{index}]",
                    valueProviderCallback: _ => pselDfeGpio[index],
                    writeCallback: (_, value) => pselDfeGpio[index] = (uint)value);
            });
            DefineStoredRegister(Registers.DfePacketPtr, "DFEPACKET_PTR", 0x01000000, 32, 0x01000000);
            DefineStoredRegister(Registers.DfePacketMaxCnt, "DFEPACKET_MAXCNT", 0x00001000, 14, 0x00001000);
            Registers.DfePacketAmount.Define(this)
                .WithValueField(0, 16, FieldMode.Read, name: "DFEPACKET_AMOUNT", valueProviderCallback: _ => 0)
                .WithReservedBits(16, 16);

            Registers.Power.Define(this, 0x00000001)
                .WithFlag(0, name: "POWER",
                    valueProviderCallback: _ => powerOn,
                    changeCallback: (_, value) =>
                    {
                        powerOn = value;
                        if(!value)
                        {
                            // Powering the peripheral down resets it (PS: POWER)
                            Reset();
                            powerOn = false;
                        }
                    });
        }

        // Register-presence helper for features that are not behaviourally
        // modelled: value stored and read back; a non-reset write logs
        // UNSUPPORTED (docs/conventions.md — never silently succeed).
        private void DefineStoredRegister(Registers register, string name, uint resetValue, int width, uint quietValue)
        {
            var definition = register.Define(this, resetValue)
                .WithValueField(0, width, name: name,
                    writeCallback: (_, value) =>
                    {
                        if((uint)value != quietValue)
                        {
                            this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what={1} detail=value 0x{2:X} stored, feature not behaviourally modelled", LabelForLog, name, (uint)value);
                        }
                    });
            if(width < 32)
            {
                definition.WithReservedBits(width, 32 - width);
            }
        }

        // ------------------------- task dispatch -------------------------

        private void RunTask(Tasks task)
        {
            this.Log(LogLevel.Noisy, "Task {0} in state {1}", task, radioState);
            switch(task)
            {
            case Tasks.TxEn:
                TaskTxEn();
                break;
            case Tasks.RxEn:
                TaskRxEn();
                break;
            case Tasks.Start:
                TaskStart();
                break;
            case Tasks.Stop:
                TaskStop();
                break;
            case Tasks.Disable:
                TaskDisable();
                break;
            case Tasks.RssiStart:
                // sample period is 0.25 us on hardware — modelled as instant
                SetEvent(Events.RssiEnd);
                break;
            case Tasks.RssiStop:
                // nothing to stop: the single sample completed instantly
                break;
            case Tasks.BcStart:
                bcTimer.Schedule(bitCounterCompare.Value * BitTimeUs(), () => SetEvent(Events.BcMatch));
                break;
            case Tasks.BcStop:
                bcTimer.Cancel();
                break;
            case Tasks.EdStart:
                edTimer.Schedule((edCount.Value + 1) * CcaEdDurationUs, () =>
                {
                    edSampleValue = EdLevel & 0xFF;
                    SetEvent(Events.EdEnd);
                    if(shortEdendDisable.Value)
                    {
                        TaskDisable();
                    }
                });
                break;
            case Tasks.EdStop:
                if(edTimer.Cancel())
                {
                    SetEvent(Events.EdStopped);
                }
                break;
            case Tasks.CcaStart:
                ccaTimer.Schedule(CcaEdDurationUs, () =>
                {
                    if(CcaBusy)
                    {
                        SetEvent(Events.CcaBusy);
                        if(shortCcabusyDisable.Value)
                        {
                            TaskDisable();
                        }
                    }
                    else
                    {
                        SetEvent(Events.CcaIdle);
                        if(shortCcaidleTxen.Value)
                        {
                            TaskTxEn();
                        }
                        if(shortCcaidleStop.Value)
                        {
                            TaskStop();
                        }
                    }
                });
                break;
            case Tasks.CcaStop:
                if(ccaTimer.Cancel())
                {
                    SetEvent(Events.CcaStopped);
                }
                break;
            }
        }

        private void TaskTxEn()
        {
            if(radioState != State.Disabled)
            {
                this.Log(LogLevel.Warning, "TASKS_TXEN in state {0} (hardware expects DISABLED); forcing TX ramp-up", radioState);
                CancelPacketTimers();
            }
            radioState = State.TxRu;
            rampTimer.Schedule(fastRampUp.Value ? FastRampUpUs : TxRampUpUs, () =>
            {
                radioState = State.TxIdle;
                SetEvent(Events.Ready);
                SetEvent(Events.TxReady);
                if(shortReadyStart.Value || shortTxreadyStart.Value)
                {
                    TaskStart();
                }
                if(shortReadyEdstart.Value)
                {
                    RunTask(Tasks.EdStart);
                }
            });
        }

        private void TaskRxEn()
        {
            if(radioState != State.Disabled)
            {
                this.Log(LogLevel.Warning, "TASKS_RXEN in state {0} (hardware expects DISABLED); forcing RX ramp-up", radioState);
                CancelPacketTimers();
            }
            radioState = State.RxRu;
            // An RX window is opening: let a virtual peer present a frame now so
            // it is buffered and delivered when the receiver reaches RX.
            RxWindowOpened?.Invoke();
            rampTimer.Schedule(fastRampUp.Value ? FastRampUpUs : RxRampUpUs, () =>
            {
                radioState = State.RxIdle;
                SetEvent(Events.Ready);
                SetEvent(Events.RxReady);
                if(shortReadyStart.Value || shortRxreadyStart.Value)
                {
                    TaskStart();
                }
                if(shortReadyEdstart.Value)
                {
                    RunTask(Tasks.EdStart);
                }
                if(shortRxreadyCcastart.Value)
                {
                    RunTask(Tasks.CcaStart);
                }
            });
        }

        private void TaskStart()
        {
            switch(radioState)
            {
            case State.TxIdle:
                BeginTransmission();
                break;
            case State.RxIdle:
                radioState = State.Rx;
                TryDeliverPendingFrame();
                break;
            default:
                this.Log(LogLevel.Warning, "TASKS_START in state {0}, ignored (hardware expects TXIDLE/RXIDLE)", radioState);
                break;
            }
        }

        private void TaskStop()
        {
            CancelPacketTimers();
            switch(radioState)
            {
            case State.Tx:
                radioState = State.TxIdle;
                break;
            case State.Rx:
                radioState = State.RxIdle;
                break;
            default:
                // STOP in idle/ramp states is a no-op
                break;
            }
        }

        private void TryDeliverPendingFrame()
        {
            if(pendingFrame == null)
            {
                return;
            }
            var frame = pendingFrame;
            var sender = pendingSender;
            pendingFrame = null;
            pendingSender = null;
            var elapsedUs = (machine.LocalTimeSource.ElapsedVirtualTime - pendingSince).TotalSeconds * 1e6;
            // A peer that responds to an RX window (RxWindowOpened fires at ramp
            // START) transmits a full frame that stays on the air while THIS
            // receiver ramps up and syncs. The frame is deliverable if the radio
            // reaches RX within the ramp-up plus the sync (preamble+address+header)
            // window of the frame arriving. Without the ramp term a fast (2 Mbit)
            // link whose sync window (~32 us) is shorter than the RX ramp-up
            // (40 us fast / 124 us normal) would drop every peer response — this is
            // exactly the Nordic ESB ACK case (2 Mbit, address+PCF ~= 32 us).
            var rampUs = fastRampUp.Value ? FastRampUpUs : RxRampUpUs;
            var windowUs = rampUs + (PreambleBytes() + AddressLength + RamHeaderLength()) * ByteTimeUs();
            if(elapsedUs > windowUs)
            {
                this.Log(LogLevel.Noisy, "Buffered frame expired ({0} us > {1} us sync window), dropping", elapsedUs, windowUs);
                return;
            }
            ProcessRxFrame(frame, sender);
        }

        private void TaskDisable()
        {
            CancelPacketTimers();
            rampTimer.Cancel();
            pendingFrame = null;
            pendingSender = null;
            switch(radioState)
            {
            case State.Disabled:
                // Hardware re-raises DISABLED; ESB relies on this when it
                // busy-waits after an unconditional TASKS_DISABLE
                // (sdk-nrf v2.5.1 subsys/esb/esb.c clear_events_restart_rx).
                FinishDisable();
                return;
            case State.TxRu:
            case State.TxIdle:
            case State.Tx:
                if(DisabledByFirmware())
                {
                    FinishDisable();
                    return;
                }
                radioState = State.TxDisable;
                disableTimer.Schedule(TxDisableUs, FinishDisable);
                return;
            case State.RxRu:
            case State.RxIdle:
            case State.Rx:
                if(DisabledByFirmware())
                {
                    FinishDisable();
                    return;
                }
                radioState = State.RxDisable;
                disableTimer.Schedule(RxDisableUs, FinishDisable);
                return;
            case State.TxDisable:
            case State.RxDisable:
                // already on the way down
                return;
            }
        }

        // True when TASKS_DISABLE comes from code the core is executing right
        // now, not from a shortcut or a timer through DPPI. The radio is then
        // disabled at once, without the few microseconds it takes: a core
        // runs on to the next timer event that was due when it set off, so a
        // timer armed here would not expire while the firmware spins waiting
        // for EVENTS_DISABLED, and a controller that allows that wait only so
        // many iterations (the SoftDevice Controller asserts after 255)
        // would give up.
        private bool DisabledByFirmware()
        {
            return machine.SystemBus.TryGetCurrentCPU(out var cpu);
        }

        private void FinishDisable()
        {
            radioState = State.Disabled;
            SetEvent(Events.Disabled);
            if(shortDisabledTxen.Value)
            {
                TaskTxEn();
            }
            else if(shortDisabledRxen.Value)
            {
                TaskRxEn();
            }
            // DISABLED_RSSISTOP: RSSI sampling is instantaneous here, no-op
        }

        private void CancelPacketTimers()
        {
            addressTimer.Cancel();
            endTimer.Cancel();
            bcTimer.Cancel();
        }

        // ------------------------- TX path -------------------------

        private void BeginTransmission()
        {
            radioState = State.Tx;

            var isIeee = mode.Value == Mode.Ieee802154_250Kbit;
            var addressLength = isIeee ? 1 : AddressLength;
            var s0Bytes = (int)s0Length.Value;
            var lengthBytes = lengthFieldLength.Value == 0 ? 0 : 1;
            var s1BytesAir = (s1Length.Value > 0) ? 1 : 0;
            var s1BytesRam = s1BytesAir > 0 || s1Include.Value ? 1 : 0;
            var crcLen = (int)crcLength.Value;

            var ramHeader = sysbus.ReadBytes(packetPointer.Value, s0Bytes + lengthBytes + s1BytesRam, true);
            uint airPayloadLength;
            if(lengthBytes > 0)
            {
                airPayloadLength = ramHeader[s0Bytes] & ((1u << (int)lengthFieldLength.Value) - 1);
            }
            else
            {
                airPayloadLength = (uint)staticLength.Value;
            }
            var declared = airPayloadLength;
            if(crcIncludedInLength.Value && declared >= (uint)crcLen)
            {
                declared -= (uint)crcLen;
            }
            var payloadLength = declared;
            if(lengthBytes > 0)
            {
                payloadLength += (uint)staticLength.Value;
            }
            if(payloadLength > maxPacketLength.Value)
            {
                this.Log(LogLevel.Warning, "TX payload length {0} exceeds MAXLEN {1}, clamping", payloadLength, maxPacketLength.Value);
                payloadLength = (uint)maxPacketLength.Value;
            }
            var payload = sysbus.ReadBytes(packetPointer.Value + (ulong)(s0Bytes + lengthBytes + s1BytesRam), (int)payloadLength, true);

            var frame = new List<byte>();
            if(isIeee)
            {
                frame.Add((byte)sfd.Value);
            }
            else
            {
                frame.AddRange(OnAirAddress((uint)txAddress.Value));
            }
            for(var i = 0; i < s0Bytes; i++)
            {
                frame.Add(ramHeader[i]);
            }
            if(lengthBytes > 0)
            {
                frame.Add(ramHeader[s0Bytes]);
            }
            if(s1BytesAir > 0)
            {
                frame.Add(s1BytesRam > 0 ? ramHeader[s0Bytes + lengthBytes] : (byte)0);
            }
            frame.AddRange(payload);

            var crcStart = CrcStartIndex(addressLength);
            var crc = ComputeCrc(frame.ToArray(), crcStart, frame.Count - crcStart);
            for(var i = crcLen - 1; i >= 0; i--)
            {
                frame.Add((byte)(crc >> (8 * i)));
            }
            var wire = frame.ToArray();

            this.Log(LogLevel.Noisy, "TX frame on channel {0} ({1} bytes, crc=0x{2:X})", Channel, wire.Length, crc);
            TransmittedFrames++;
            LastTransmittedFrame = BitConverter.ToString(wire).Replace("-", "");
            FrameSent?.Invoke(this, wire);

            var byteTime = ByteTimeUs();
            var addressTime = (PreambleBytes() + addressLength) * byteTime;
            var headerAirBytes = s0Bytes + lengthBytes + s1BytesAir;
            var endTime = addressTime + (headerAirBytes + payloadLength + crcLen) * byteTime;

            addressTimer.Schedule(addressTime, () =>
            {
                SetEvent(isIeee ? Events.FrameStart : Events.Address);
                HandleAddressShorts();
            });
            endTimer.Schedule(endTime, () =>
            {
                radioState = State.TxIdle;
                SetEvent(Events.Payload);
                SetEvent(Events.End);
                SetEvent(Events.PhyEnd);
                HandleEndShorts();
            });
        }

        private void HandleAddressShorts()
        {
            if(shortAddressRssistart.Value)
            {
                SetEvent(Events.RssiEnd);
            }
            if(shortAddressBcstart.Value || shortFramestartBcstart.Value)
            {
                RunTask(Tasks.BcStart);
            }
        }

        private void HandleEndShorts()
        {
            if(shortEndDisable.Value || shortPhyendDisable.Value)
            {
                TaskDisable();
            }
            else if(shortEndStart.Value || shortPhyendStart.Value)
            {
                TaskStart();
            }
        }

        // ------------------------- 802.15.4 RX -------------------------

        private void Receive802154Frame(byte[] frame)
        {
            if(frame.Length < 2 || frame[0] != (byte)sfd.Value)
            {
                this.Log(LogLevel.Noisy, "Dropping 802.15.4 frame: SFD mismatch or runt");
                return;
            }
            var crcLen = (int)crcLength.Value;
            var length = Math.Min(frame[1] & ((1 << (int)lengthFieldLength.Value) - 1), (int)maxPacketLength.Value);
            var payloadLen = crcIncludedInLength.Value && length >= crcLen ? length - crcLen : length;
            var ram = new byte[1 + payloadLen];
            ram[0] = frame[1];
            for(var i = 0; i < payloadLen && 2 + i < frame.Length; i++)
            {
                ram[1 + i] = frame[2 + i];
            }
            sysbus.WriteBytes(ram, packetPointer.Value, true);

            var receivedCrc = 0u;
            for(var i = 0; i < crcLen && 2 + payloadLen + i < frame.Length; i++)
            {
                receivedCrc = (receivedCrc << 8) | frame[2 + payloadLen + i];
            }
            var computed = ComputeCrc(frame, 2, payloadLen);
            var ok = crcLen == 0 || computed == receivedCrc;
            if(FaultCrcError)
            {
                this.Log(LogLevel.Warning, "FaultCrcError active: forcing CRC failure on this frame");
                FaultCrcError = false;
                ok = false;
            }

            var byteTime = ByteTimeUs();
            var thisCrc = receivedCrc;
            addressTimer.Schedule((PreambleBytes() + 1) * byteTime, () =>
            {
                SetEvent(Events.FrameStart);
                HandleAddressShorts();
            });
            endTimer.Schedule((PreambleBytes() + 2 + payloadLen + crcLen) * byteTime, () =>
            {
                rxCrc = thisCrc;
                crcStatusOk = ok;
                radioState = State.RxIdle;
                SetEvent(Events.Payload);
                SetEvent(Events.End);
                SetEvent(Events.PhyEnd);
                SetEvent(ok ? Events.CrcOk : Events.CrcError);
                HandleEndShorts();
            });
        }

        // ------------------------- helpers -------------------------

        // On-air logical address of a pipe: the BALEN most-significant bytes
        // of BASE0 (pipe 0) / BASE1 (pipes 1..7) — the base is left-aligned,
        // the low 4-BALEN bytes are ignored — transmitted low-byte-first,
        // then the pipe's prefix byte (PS "Address configuration"; matches
        // the BLE convention BASE0 = AccessAddress << 8, PREFIX0.AP0 =
        // AccessAddress >> 24 with on-air LSB-first order, and the upstream
        // NRF52840_Radio interpretation). BALEN is runtime configurable
        // 2..4 — 3-byte addresses (BALEN=2) are first-class.
        private byte[] OnAirAddress(uint pipe)
        {
            var balen = (int)baseAddressLength.Value;
            if(balen < 2 || balen > 4)
            {
                this.Log(LogLevel.Warning, "BALEN={0} outside the hardware-legal 2..4 range, clamping", balen);
                balen = Math.Min(Math.Max(balen, 2), 4);
            }
            var baseValue = pipe == 0 ? (uint)baseAddress0.Value : (uint)baseAddress1.Value;
            var result = new byte[balen + 1];
            for(var i = 0; i < balen; i++)
            {
                result[i] = (byte)(baseValue >> (8 * (4 - balen + i)));
            }
            result[balen] = addressPrefixes[pipe];
            return result;
        }

        private int AddressLength => (int)Math.Min(Math.Max(baseAddressLength.Value, 2), 4) + 1;

        private int RamHeaderLength()
        {
            var length = (int)s0Length.Value + (lengthFieldLength.Value == 0 ? 0 : 1);
            if(s1Length.Value > 0 || s1Include.Value)
            {
                length += 1;
            }
            return length;
        }

        private int CrcStartIndex(int addressLength)
        {
            switch(crcSkipAddress.Value)
            {
            case CrcAddressHandling.Include:
                return 0;
            case CrcAddressHandling.Skip:
                return addressLength;
            case CrcAddressHandling.Ieee802154:
                // FCS over the MAC payload: skip SFD and the length byte
                return addressLength + 1;
            default:
                this.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=CRCCNF.SKIPADDR detail=reserved value {1}, treating as Include", LabelForLog, crcSkipAddress.Value);
                return 0;
            }
        }

        // Bit-serial CRC, MSB first, width = 8*CRCCNF.LEN, poly/init from
        // CRCPOLY/CRCINIT. CRCCNF.LEN is the ONLY source of the CRC length.
        private uint ComputeCrc(byte[] data, int start, int count)
        {
            var lenBytes = (int)crcLength.Value;
            if(lenBytes == 0)
            {
                return 0;
            }
            var width = 8 * lenBytes;
            var mask = width == 32 ? 0xFFFFFFFFu : (1u << width) - 1;
            var topBit = 1u << (width - 1);
            var poly = (uint)crcPolynomial.Value & mask;
            var crc = (uint)crcInitialValue.Value & mask;
            for(var i = start; i < start + count && i < data.Length; i++)
            {
                for(var bit = 7; bit >= 0; bit--)
                {
                    var input = ((data[i] >> bit) & 1) != 0;
                    var feedback = ((crc & topBit) != 0) ^ input;
                    crc = (crc << 1) & mask;
                    if(feedback)
                    {
                        crc ^= poly;
                    }
                }
            }
            return crc & mask;
        }

        private double ByteTimeUs()
        {
            switch(mode.Value)
            {
            case Mode.Nrf2Mbit:
            case Mode.Ble2Mbit:
                return 4;
            case Mode.BleLr500Kbit:
                return 16;
            case Mode.BleLr125Kbit:
                return 64;
            case Mode.Ieee802154_250Kbit:
                return 32;
            default: // Nrf_1Mbit, Ble_1Mbit and reserved values
                return 8;
            }
        }

        private double BitTimeUs() => ByteTimeUs() / 8;

        private int PreambleBytes()
        {
            switch(preambleLength.Value)
            {
            case 1:
                return 2;   // 16bit
            case 2:
                return 4;   // 32bitZero
            case 3:
                return 10;  // LongRange coded preamble, ~80 us at 125 Kbit
            default:
                return 1;   // 8bit
            }
        }

        private void SetEvent(Events @event)
        {
            eventPending |= 1u << (int)@event;
            EventTriggered?.Invoke((uint)(0x100 + 4 * (int)@event));
            publishEvent[(int)@event].Publish();
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var asserted = (eventPending & interruptEnable) != 0;
            if(asserted && !IRQ.IsSet && FaultDropInterrupt)
            {
                FaultDropInterrupt = false;
                this.Log(LogLevel.Warning, "FaultDropInterrupt active: swallowing IRQ assertion");
                return;
            }
            IRQ.Set(asserted);
        }

        private string LabelForLog
        {
            get
            {
                try
                {
                    if(machine.TryGetLocalName(this, out var name))
                    {
                        return name;
                    }
                }
                catch(Exception)
                {
                    // registration not finished yet — fall through
                }
                return GetType().Name;
            }
        }

        // ------------------------- state -------------------------

        private State radioState;
        private uint eventPending;
        private uint interruptEnable;
        private uint rxMatch;
        private uint rxCrc;
        private bool crcStatusOk;
        private bool powerOn = true;
        private bool tifsWarned;
        private bool dacnfWarned;
        private uint dacnfRaw;
        private uint edSampleValue;
        private byte[] pendingFrame;
        private IRadio pendingSender;
        private TimeInterval pendingSince;
        private readonly byte[] addressPrefixes = new byte[8];
        private readonly uint[] deviceAddressBase = new uint[8];
        private readonly uint[] deviceAddressPrefix = new uint[8];
        private readonly uint[] pselDfeGpio = new uint[8];

        private IValueRegisterField packetPointer;
        private IValueRegisterField frequency;
        private IFlagRegisterField frequencyMap;
        private IValueRegisterField txPower;
        private IEnumRegisterField<Mode> mode;
        private IValueRegisterField lengthFieldLength;
        private IValueRegisterField s0Length;
        private IValueRegisterField s1Length;
        private IFlagRegisterField s1Include;
        private IValueRegisterField codeIndicatorLength;
        private IValueRegisterField preambleLength;
        private IFlagRegisterField crcIncludedInLength;
        private IValueRegisterField termLength;
        private IValueRegisterField maxPacketLength;
        private IValueRegisterField staticLength;
        private IValueRegisterField baseAddressLength;
        private IFlagRegisterField bigEndian;
        private IFlagRegisterField whiteningEnabled;
        private IValueRegisterField baseAddress0;
        private IValueRegisterField baseAddress1;
        private IValueRegisterField txAddress;
        private IFlagRegisterField[] rxAddressEnabled;
        private IValueRegisterField crcLength;
        private IEnumRegisterField<CrcAddressHandling> crcSkipAddress;
        private IValueRegisterField crcPolynomial;
        private IValueRegisterField crcInitialValue;
        private IValueRegisterField tifs;
        private IValueRegisterField dataWhiteIv;
        private IValueRegisterField bitCounterCompare;
        private IFlagRegisterField fastRampUp;
        private IValueRegisterField defaultTxValue;
        private IValueRegisterField sfd;
        private IValueRegisterField edCount;
        private IEnumRegisterField<CcaModeSelect> ccaMode;

        private IFlagRegisterField shortReadyStart;
        private IFlagRegisterField shortEndDisable;
        private IFlagRegisterField shortDisabledTxen;
        private IFlagRegisterField shortDisabledRxen;
        private IFlagRegisterField shortAddressRssistart;
        private IFlagRegisterField shortEndStart;
        private IFlagRegisterField shortAddressBcstart;
        private IFlagRegisterField shortDisabledRssistop;
        private IFlagRegisterField shortRxreadyCcastart;
        private IFlagRegisterField shortCcaidleTxen;
        private IFlagRegisterField shortCcabusyDisable;
        private IFlagRegisterField shortFramestartBcstart;
        private IFlagRegisterField shortReadyEdstart;
        private IFlagRegisterField shortEdendDisable;
        private IFlagRegisterField shortCcaidleStop;
        private IFlagRegisterField shortTxreadyStart;
        private IFlagRegisterField shortRxreadyStart;
        private IFlagRegisterField shortPhyendDisable;
        private IFlagRegisterField shortPhyendStart;

        private readonly DPPISubscribeEndpoint[] subscribeTask;
        private readonly DPPIPublishEndpoint[] publishEvent;
        private readonly DelayedAction rampTimer;
        private readonly DelayedAction addressTimer;
        private readonly DelayedAction endTimer;
        private readonly DelayedAction disableTimer;
        private readonly DelayedAction bcTimer;
        private readonly DelayedAction edTimer;
        private readonly DelayedAction ccaTimer;

        // Timing constants — see the "Timing model" header comment for sources.
        private const double TxRampUpUs = 129;
        private const double RxRampUpUs = 124;
        private const double FastRampUpUs = 40;
        private const double TxDisableUs = 6;
        private const double RxDisableUs = 1;
        private const double CcaEdDurationUs = 128;

        private const int TaskCount = 13;
        private const int EventBitCount = 29;
        private const uint EventMask = 0x1CFFF4FF; // bits 0..7,10,12..23,26..28

        // One-shot virtual-time delayed action on a 16 MHz LimitTimer.
        private sealed class DelayedAction
        {
            public DelayedAction(IMachine machine, IPeripheral owner, string name)
            {
                timer = new LimitTimer(machine.ClockSource, TimerFrequency, owner, name,
                    limit: 1, direction: Direction.Ascending, enabled: false,
                    workMode: WorkMode.OneShot, eventEnabled: true);
                timer.LimitReached += () =>
                {
                    var current = action;
                    action = null;
                    current?.Invoke();
                };
            }

            public void Schedule(double microseconds, Action what)
            {
                Cancel();
                action = what;
                timer.Limit = (ulong)Math.Max(1, Math.Round(microseconds * TicksPerMicrosecond));
                timer.Value = 0;
                timer.Enabled = true;
            }

            // Returns true when a pending action was cancelled.
            public bool Cancel()
            {
                var wasPending = action != null;
                timer.Enabled = false;
                action = null;
                return wasPending;
            }

            private Action action;
            private readonly LimitTimer timer;

            private const long TimerFrequency = 16000000;
            private const double TicksPerMicrosecond = 16;
        }

        private enum Tasks
        {
            TxEn = 0,
            RxEn = 1,
            Start = 2,
            Stop = 3,
            Disable = 4,
            RssiStart = 5,
            RssiStop = 6,
            BcStart = 7,
            BcStop = 8,
            EdStart = 9,
            EdStop = 10,
            CcaStart = 11,
            CcaStop = 12,
        }

        // Value == INTEN bit position == (event register offset - 0x100)/4
        private enum Events
        {
            Ready = 0,
            Address = 1,
            Payload = 2,
            End = 3,
            Disabled = 4,
            DevMatch = 5,
            DevMiss = 6,
            RssiEnd = 7,
            BcMatch = 10,
            CrcOk = 12,
            CrcError = 13,
            FrameStart = 14,
            EdEnd = 15,
            EdStopped = 16,
            CcaIdle = 17,
            CcaBusy = 18,
            CcaStopped = 19,
            RateBoost = 20,
            TxReady = 21,
            RxReady = 22,
            MhrMatch = 23,
            Sync = 26,
            PhyEnd = 27,
            CtePresent = 28,
        }

        private enum State
        {
            Disabled = 0,
            RxRu = 1,
            RxIdle = 2,
            Rx = 3,
            RxDisable = 4,
            TxRu = 9,
            TxIdle = 10,
            Tx = 11,
            TxDisable = 12,
        }

        private enum Mode
        {
            Nrf1Mbit = 0,
            Nrf2Mbit = 1,
            Ble1Mbit = 3,
            Ble2Mbit = 4,
            BleLr125Kbit = 5,
            BleLr500Kbit = 6,
            Ieee802154_250Kbit = 15,
        }

        private enum CrcAddressHandling
        {
            Include = 0,
            Skip = 1,
            Ieee802154 = 2,
        }

        private enum CcaModeSelect
        {
            EdMode = 0,
            CarrierMode = 1,
            CarrierAndEdMode = 2,
            CarrierOrEdMode = 3,
            EdModeTest1 = 4,
        }

        private enum Registers : long
        {
            TasksTxEn = 0x000,
            TasksRxEn = 0x004,
            TasksStart = 0x008,
            TasksStop = 0x00C,
            TasksDisable = 0x010,
            TasksRssiStart = 0x014,
            TasksRssiStop = 0x018,
            TasksBcStart = 0x01C,
            TasksBcStop = 0x020,
            TasksEdStart = 0x024,
            TasksEdStop = 0x028,
            TasksCcaStart = 0x02C,
            TasksCcaStop = 0x030,
            // SUBSCRIBE_* = task + 0x80, EVENTS_* = 0x100 + 4*bit,
            // PUBLISH_* = event + 0x80 (defined programmatically above)
            Shorts = 0x200,
            IntenSet = 0x304,
            IntenClr = 0x308,
            CrcStatus = 0x400,
            RxMatch = 0x408,
            RxCrc = 0x40C,
            Dai = 0x410,
            PduStat = 0x414,
            CteStatus = 0x44C,
            DfeStatus = 0x458,
            PacketPtr = 0x504,
            Frequency = 0x508,
            TxPower = 0x50C,
            Mode = 0x510,
            Pcnf0 = 0x514,
            Pcnf1 = 0x518,
            Base0 = 0x51C,
            Base1 = 0x520,
            Prefix0 = 0x524,
            Prefix1 = 0x528,
            TxAddress = 0x52C,
            RxAddresses = 0x530,
            CrcCnf = 0x534,
            CrcPoly = 0x538,
            CrcInit = 0x53C,
            Tifs = 0x544,
            RssiSampleReg = 0x548,
            StateReg = 0x550,
            DataWhiteIv = 0x554,
            Bcc = 0x560,
            Dab0 = 0x600,
            Dap0 = 0x620,
            Dacnf = 0x640,
            MhrMatchConf = 0x644,
            MhrMatchMas = 0x648,
            ModeCnf0 = 0x650,
            Sfd = 0x660,
            EdCnt = 0x664,
            EdSample = 0x668,
            CcaCtrl = 0x66C,
            DfeMode = 0x900,
            CteInlineConf = 0x904,
            DfeCtrl1 = 0x910,
            DfeCtrl2 = 0x914,
            SwitchPattern = 0x928,
            ClearPattern = 0x92C,
            PselDfeGpio0 = 0x930,
            DfePacketPtr = 0x950,
            DfePacketMaxCnt = 0x954,
            DfePacketAmount = 0x958,
            Power = 0xFFC,
        }
    }
}
