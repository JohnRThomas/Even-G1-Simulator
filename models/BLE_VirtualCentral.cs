//
// BLE_VirtualCentral: "the phone", a BLE central that talks to the emulated
// lens over the modelled radio (NRF5340_RADIO). The lens's controller, host
// and GATT server are the firmware's own; this peer puts on the air what a
// phone would, and answers what the lens sends.
//
// It rides the radio model:
//   radio.FrameSent          every frame the lens transmits: its ADV_IND, and
//                            its PDUs on the connection
//   radio.RxWindowOpened     the lens is opening a receive window (after an
//                            advertisement, or at a connection event). Our
//                            frame is handed over now; the radio holds it and
//                            delivers it when the receiver is up
//   radio.ReceiveFrame(f, null)
//                            the hand-over. A null sender skips the channel
//                            check, so channel hopping is not tracked
//   radio.OnAirAddressPublic / BuildFrameCrc
//                            the access address the lens is listening on, and
//                            a CRC its receiver accepts
//
// What a connection goes through:
//
//   link layer   CONNECT_IND after an ADV_IND; stop-and-wait SN/NESN with
//                retransmission; answers to the lens's control procedures
//                (features, version, data length, PHY, ping); data length
//                update; the encryption start procedure and AES-CCM on every
//                PDU after it; LL_TERMINATE_IND in both directions
//   L2CAP        fragmentation and reassembly; the signalling channel
//                (connection parameter update requests are declined)
//   ATT          as a client: MTU exchange, discovery of the Nordic UART
//                Service by UUID, notifications switched on, protocol packets
//                written to its RX characteristic. As a server: an empty
//                attribute table, so the lens's own discovery finds nothing
//   SMP          LE Secure Connections pairing, Just Works, started by the
//                phone or on the lens's Security Request; bonding, with the
//                phone's identity key handed over
//
// The bond (the long-term key) outlives a link and a lens reboot: the next
// connection is encrypted with it straight away. If the lens no longer has
// the key, the phone forgets it too and pairs again.
//
// A link the lens drops, by LL_TERMINATE_IND or by going silent, is noticed
// and the phone connects again at the lens's next advertisement.
//
// Everything worth reading is logged at Info level, protocol packets as
// "tx <hex>" and "rx <hex>"; the PDU trace is at Debug level.
//
// Disabled by default: `vcentral Enabled true` switches the phone on.
//
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Numerics;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Utilities.Crypto;

namespace Antmicro.Renode.Peripherals.Wireless
{
    public class BLE_VirtualCentral : BasicDoubleWordPeripheral, IKnownSize
    {
        public BLE_VirtualCentral(IMachine machine, NRF5340_RADIO radio) : base(machine)
        {
            this.radio = radio;
            radio.FrameSent += OnFrameSent;
            radio.RxWindowOpened += OnRxWindowOpened;
        }

        // The lens was reset: the link is gone, the bond is not.
        public override void Reset()
        {
            base.Reset();
            lock(sync)
            {
                if(link == Link.Connected)
                {
                    Say("link dropped: the lens was reset");
                }
                link = Link.Idle;
                haveAdv = false;
                reconnectNotBeforeMs = 0;
            }
        }

        public long Size => 0x1000;

        // ------------------------------ settings ------------------------------

        // The phone's Bluetooth, off by default. Switching it off closes an
        // open link the way a phone does, with LL_TERMINATE_IND.
        public bool Enabled
        {
            get
            {
                return enabled;
            }
            set
            {
                lock(sync)
                {
                    if(enabled == value)
                    {
                        return;
                    }
                    enabled = value;
                    if(value)
                    {
                        return;
                    }
                    haveAdv = false;
                    pendingPackets.Clear();
                    if(link == Link.Connected)
                    {
                        Terminate();
                    }
                    else
                    {
                        link = Link.Idle;
                    }
                }
            }
        }

        // Pair as soon as the Nordic UART Service is set up. If false, the
        // phone pairs only when the lens asks with a Security Request. A bonded
        // phone encrypts the link on connecting either way.
        public bool PairOnConnect { get; set; } = true;

        // The ATT MTU the phone asks for at its next connection.
        public uint Mtu
        {
            get
            {
                return mtu;
            }
            set
            {
                if(value < DefaultAttMtu || value > 517)
                {
                    throw new RecoverableException("an ATT MTU is between 23 and 517");
                }
                mtu = value;
            }
        }

        // A heartbeat packet is written after this many seconds without any
        // other write, so that the lens does not drop a phone it has not
        // heard from. 0 switches it off.
        public double HeartbeatSeconds { get; set; } = 10;

        // ------------------------------- state --------------------------------

        public bool Connected
        {
            get
            {
                lock(sync)
                {
                    return link == Link.Connected && LinkAlive(NowMs);
                }
            }
        }

        public bool Encrypted => encrypted && Connected;
        public bool Bonded => bondKey != null;
        public bool NusReady => nusReady && Connected;
        public uint AttMtu => (uint)attMtu;
        public uint NusRxValueHandle => nusRxHandle;
        public uint NusTxValueHandle => nusTxHandle;
        public uint NusTxCccHandle => nusCccHandle;
        public int ConnectIndsSent => connectIndsSent;
        public int DataEventsAnswered => dataEventsAnswered;
        public int PacketsSent => packetsSent;
        public int PacketsReceived => packetsReceived;

        // The state in a few words, for a status line.
        public string Summary
        {
            get
            {
                lock(sync)
                {
                    if(!enabled && link != Link.Connected)
                    {
                        return "off";
                    }
                    if(link != Link.Connected || !LinkAlive(NowMs))
                    {
                        return link == Link.Connecting ? "connecting" : "waiting for advertising";
                    }
                    var text = new StringBuilder("connected");
                    text.Append(", MTU ").Append(attMtu);
                    if(nusReady)
                    {
                        text.Append(", notifications on");
                    }
                    if(smp != Smp.Idle)
                    {
                        text.Append(", pairing");
                    }
                    else if(encrypted)
                    {
                        text.Append(bondKey != null ? ", encrypted, bonded" : ", encrypted");
                    }
                    else
                    {
                        text.Append(bondKey != null ? ", bonded, not encrypted" : ", not paired");
                    }
                    return text.ToString();
                }
            }
        }

        // ------------------------------ commands ------------------------------

        // Write a protocol packet to the lens, e.g. `vcentral SendPacket "3E"`.
        // It goes out once the link is set up; until then it waits.
        public void SendPacket(string hex)
        {
            var packet = ParseHex(hex);
            if(packet.Length == 0)
            {
                throw new RecoverableException("an empty packet");
            }
            lock(sync)
            {
                if(!enabled)
                {
                    throw new RecoverableException("the phone is switched off");
                }
                if(nusReady && packet.Length > attMtu - 3)
                {
                    throw new RecoverableException(string.Format(
                        "the packet is {0} bytes; with an ATT MTU of {1} a write carries {2}",
                        packet.Length, attMtu, attMtu - 3));
                }
                if(pendingPackets.Count >= MaxPendingPackets)
                {
                    throw new RecoverableException("too many packets are waiting for the link");
                }
                pendingPackets.Enqueue(packet);
            }
        }

        // Start pairing now, whatever PairOnConnect says.
        public void Pair()
        {
            lock(sync)
            {
                if(link != Link.Connected)
                {
                    throw new RecoverableException("the phone is not connected");
                }
                if(SecurityBusy)
                {
                    throw new RecoverableException("pairing or encryption is already in progress");
                }
                if(encrypted)
                {
                    throw new RecoverableException("the link is encrypted already; to pair afresh: ForgetBond, then Disconnect");
                }
                StartPairing();
            }
        }

        // Forget the long-term key, as "forget this device" on a phone does.
        public void ForgetBond()
        {
            lock(sync)
            {
                bondKey = null;
            }
        }

        // Close the link. The phone connects again at the next advertisement
        // unless it is switched off.
        public void Disconnect()
        {
            lock(sync)
            {
                if(link != Link.Connected)
                {
                    throw new RecoverableException("the phone is not connected");
                }
                Terminate();
            }
        }

        // ---------------------------- radio callbacks ---------------------------

        private void OnFrameSent(IRadio sender, byte[] frame)
        {
            lock(sync)
            {
                if(frame == null || frame.Length < AddrLen + 2 + CrcLen)
                {
                    return;
                }
                var header = frame[AddrLen];
                int length = frame[AddrLen + 1];
                var now = NowMs;

                if(AaEquals(frame, AdvAccessAddress))
                {
                    if((header & 0x0F) != AdvIndType || frame.Length < AddrLen + 2 + 6 + CrcLen)
                    {
                        return;
                    }
                    if(link == Link.Connected)
                    {
                        // Advertising again while our link should be up.
                        if(!peerTerminated && LinkAlive(now))
                        {
                            return;
                        }
                        Disconnected(peerTerminated ? "the lens closed the link" : "the lens stopped answering", now);
                    }
                    link = Link.Idle;        // a CONNECT_IND it did not take
                    if(!enabled)
                    {
                        return;
                    }
                    advA = new byte[6];
                    Array.Copy(frame, AddrLen + 2, advA, 0, 6);
                    advARandom = (header & 0x40) != 0;
                    haveAdv = true;
                    return;
                }

                if(link != Link.Connected || !AaEquals(frame, accessAddress)
                   || frame.Length < AddrLen + 2 + length + CrcLen)
                {
                    return;
                }
                var llid = header & 0x03;
                var peerNesn = (header >> 2) & 1;
                var peerSn = (header >> 3) & 1;

                if(lastTx != null && peerNesn != txSn)
                {
                    txSn ^= 1;                          // our PDU was acknowledged
                    var acked = lastTx;
                    lastTx = null;
                    if(acked.OnAcked != null)
                    {
                        acked.OnAcked();
                    }
                    if(link != Link.Connected)
                    {
                        return;
                    }
                }
                if(peerSn != nextRxSn)
                {
                    return;                             // a retransmission of one we have
                }
                nextRxSn ^= 1;
                if(length == 0)
                {
                    return;
                }
                var payload = new byte[length];
                Array.Copy(frame, AddrLen + 2, payload, 0, length);
                if(rxEncrypted)
                {
                    payload = length > MicLen ? Ccm(payload, (byte)llid, rxCounter, false) : null;
                    if(payload == null)
                    {
                        this.Log(LogLevel.Error, "a PDU from the lens failed its integrity check (packet {0})", rxCounter);
                        Disconnected("message integrity check failed", now);
                        return;
                    }
                    rxCounter++;
                }
                HandlePdu(llid, payload);
            }
        }

        private void OnRxWindowOpened()
        {
            lock(sync)
            {
                var listening = radio.OnAirAddressPublic(0);
                if(listening == null || listening.Length < AddrLen)
                {
                    return;
                }
                var now = NowMs;

                if(AaEquals(listening, AdvAccessAddress))
                {
                    if(link == Link.Idle && enabled && haveAdv && now >= reconnectNotBeforeMs)
                    {
                        haveAdv = false;
                        SendConnectInd();
                    }
                    return;
                }
                if(link == Link.Idle || !AaEquals(listening, accessAddress))
                {
                    return;
                }
                if(link == Link.Connecting)
                {
                    link = Link.Connected;
                    lastWindowMs = now;
                    OnConnected(now);
                }
                lastWindowMs = now;
                Tick(now);
                if(lastTx == null)
                {
                    lastTx = NextPdu();
                }
                Transmit(lastTx);
                dataEventsAnswered++;
                if(peerTerminated)
                {
                    // That PDU acknowledged the lens's LL_TERMINATE_IND.
                    Disconnected(string.Format("the lens closed the link (reason 0x{0:X2})", peerTerminateReason), now);
                }
            }
        }

        // ------------------------------ link layer ------------------------------

        private void SendConnectInd()
        {
            if(random == null)
            {
                // Reproducible, and not the same for the phones of two lenses.
                random = new Random(0x4731 ^ (advA[0] | advA[1] << 8 | advA[2] << 16 | advA[3] << 24));
            }
            // A link of its own: the lenses of a pair share the air, and two
            // links with one access address, CRC seed and hop sequence would
            // take each other's packets for their own.
            do
            {
                accessAddress = RandomBytes(4);
            }
            while(!ValidAccessAddress(accessAddress));
            var hop = (byte)(5 + RandomBytes(1)[0] % 12);           // 5..16, sleep clock accuracy 251-500 ppm

            var pdu = new List<byte>();
            pdu.AddRange(AdvAccessAddress);
            pdu.Add((byte)(0x05 | (advARandom ? 0x80 : 0x00)));     // CONNECT_IND, public InitA
            pdu.Add(34);
            pdu.AddRange(InitA);
            pdu.AddRange(advA);
            pdu.AddRange(accessAddress);
            pdu.AddRange(RandomBytes(3));                           // CRC initialisation value
            pdu.Add(WinSize);
            pdu.AddRange(WinOffset);
            pdu.AddRange(Interval);
            pdu.AddRange(Latency);
            pdu.AddRange(Timeout);
            pdu.AddRange(ChannelMap);
            pdu.Add(hop);
            SendFrame(pdu);
            connectIndsSent++;
            link = Link.Connecting;
        }

        // What the Core specification asks of an access address (Vol 6 Part B
        // 2.1.2): not the advertising one or one bit away from it, its four
        // octets not all equal, at most six equal bits in a row, at most 24
        // transitions, at least two of them in the six most significant bits.
        private static bool ValidAccessAddress(byte[] address)
        {
            var value = (uint)(address[0] | address[1] << 8 | address[2] << 16 | address[3] << 24);
            var difference = value ^ 0x8E89BED6;
            if((difference & (difference - 1)) == 0
               || (address[0] == address[1] && address[1] == address[2] && address[2] == address[3]))
            {
                return false;
            }
            var run = 1;
            var transitions = 0;
            var transitionsOnTop = 0;
            for(var bit = 1; bit < 32; bit++)
            {
                if(((value >> bit) & 1) == ((value >> (bit - 1)) & 1))
                {
                    if(++run > 6)
                    {
                        return false;
                    }
                    continue;
                }
                run = 1;
                transitions++;
                if(bit >= 27)
                {
                    transitionsOnTop++;
                }
            }
            return transitions <= 24 && transitionsOnTop >= 2;
        }

        private void OnConnected(double now)
        {
            txSn = 0;
            nextRxSn = 0;
            lastTx = null;
            control.Clear();
            data.Clear();
            peerTerminated = false;
            terminating = false;
            versionSent = false;
            maxTxOctets = DefaultOctets;
            txEncrypted = false;
            rxEncrypted = false;
            encrypted = false;
            enc = Enc.None;
            smp = Smp.Idle;
            reassembly = null;
            attQueue.Clear();
            attPending = null;
            attMtu = DefaultAttMtu;
            nusReady = false;
            nusRxHandle = 0;
            nusTxHandle = 0;
            nusCccHandle = 0;
            characteristics.Clear();
            heartbeatSent = null;
            lastWriteMs = now;
            connectedAtMs = now;
            securityRequested = false;

            Say("connected to {0}", AddressText(advA));
            // As a phone does on a new link: ask for long PDUs, then encrypt
            // if the lens is one we are bonded with. GATT runs alongside.
            lengthPending = true;
            SendControl(new byte[] { LlLengthReq, MaxOctets, 0x00, 0x48, 0x08, MaxOctets, 0x00, 0x48, 0x08 });
            StartGattSetup();
        }

        private void Disconnected(string why, double now)
        {
            if(link == Link.Connected)
            {
                Say("disconnected: {0}", why);
            }
            link = Link.Idle;
            haveAdv = false;
            encrypted = false;
            nusReady = false;
            smp = Smp.Idle;
            enc = Enc.None;
            reconnectNotBeforeMs = now + ReconnectDelayMs;
        }

        private void Terminate()
        {
            if(terminating)
            {
                return;
            }
            terminating = true;
            control.Clear();
            data.Clear();
            control.Add(new Pdu
            {
                Llid = LlidControl,
                Payload = new byte[] { LlTerminateInd, 0x13 },      // remote user terminated connection
                DuringEncryption = true,
                OnAcked = () => Disconnected("the phone closed the link", NowMs),
            });
        }

        private bool LinkAlive(double now)
        {
            return now - lastWindowMs < SupervisionTimeoutMs;
        }

        // The next PDU for the lens: control before data; while encryption is
        // being started, only what that procedure allows.
        private Pdu NextPdu()
        {
            Pdu pdu = null;
            for(var i = 0; i < control.Count; i++)
            {
                if(enc == Enc.None || control[i].DuringEncryption)
                {
                    pdu = control[i];
                    control.RemoveAt(i);
                    break;
                }
            }
            if(pdu == null && enc == Enc.None && data.Count > 0)
            {
                pdu = data.Dequeue();
            }
            if(pdu == null)
            {
                return new Pdu { Llid = LlidContinuation, Payload = new byte[0] };
            }
            if(txEncrypted)
            {
                pdu.Payload = Ccm(pdu.Payload, (byte)pdu.Llid, txCounter, true);
                txCounter++;
            }
            return pdu;
        }

        private bool MoreToSend()
        {
            if(enc == Enc.None)
            {
                return control.Count > 0 || data.Count > 0;
            }
            foreach(var pdu in control)
            {
                if(pdu.DuringEncryption)
                {
                    return true;
                }
            }
            return false;
        }

        private void Transmit(Pdu pdu)
        {
            var frame = new List<byte>(AddrLen + 2 + pdu.Payload.Length + CrcLen);
            frame.AddRange(accessAddress);
            frame.Add((byte)(pdu.Llid | (nextRxSn << 2) | (txSn << 3) | (MoreToSend() ? 0x10 : 0x00)));
            frame.Add((byte)pdu.Payload.Length);
            frame.AddRange(pdu.Payload);
            SendFrame(frame);
        }

        private void SendFrame(List<byte> frameWithoutCrc)
        {
            var body = frameWithoutCrc.ToArray();
            var crc = radio.BuildFrameCrc(body);
            var frame = new byte[body.Length + crc.Length];
            Array.Copy(body, frame, body.Length);
            Array.Copy(crc, 0, frame, body.Length, crc.Length);
            radio.ReceiveFrame(frame, null);
        }

        private void SendControl(byte[] payload, bool duringEncryption = false)
        {
            this.Log(LogLevel.Debug, "LL control to the lens: {0}", Hex(payload));
            control.Add(new Pdu { Llid = LlidControl, Payload = payload, DuringEncryption = duringEncryption });
        }

        private void HandlePdu(int llid, byte[] payload)
        {
            if(llid == LlidControl)
            {
                HandleControl(payload);
            }
            else if(llid == LlidStart)
            {
                if(payload.Length < 4)
                {
                    return;
                }
                reassembly = new List<byte>(payload);
                DeliverIfComplete();
            }
            else if(llid == LlidContinuation && reassembly != null)
            {
                reassembly.AddRange(payload);
                DeliverIfComplete();
            }
        }

        private void HandleControl(byte[] p)
        {
            this.Log(LogLevel.Debug, "LL control from the lens: {0}", Hex(p));
            switch(p[0])
            {
            case LlTerminateInd:
                peerTerminated = true;
                peerTerminateReason = p.Length > 1 ? p[1] : (byte)0;
                break;
            case LlPeripheralFeatureReq:
                SendControl(new byte[] { LlFeatureRsp, Features, 0, 0, 0, 0, 0, 0, 0 });
                break;
            case LlVersionInd:
                if(!versionSent)
                {
                    versionSent = true;
                    SendControl(new byte[] { LlVersionInd, 0x0D, 0xFF, 0xFF, 0x00, 0x00 });    // Bluetooth 5.4
                }
                break;
            case LlLengthReq:
            case LlLengthRsp:
                if(p.Length >= 9)
                {
                    // What we may send is what the lens can receive.
                    maxTxOctets = Math.Max(DefaultOctets, Math.Min((int)MaxOctets, p[1] | (p[2] << 8)));
                }
                if(p[0] == LlLengthReq)
                {
                    SendControl(new byte[] { LlLengthRsp, MaxOctets, 0x00, 0x48, 0x08, MaxOctets, 0x00, 0x48, 0x08 });
                }
                else
                {
                    LengthProcedureDone();
                }
                break;
            case LlPhyReq:
                // The central decides, and decides to stay on the 1M PHY.
                SendControl(new byte[] { LlPhyUpdateInd, 0x00, 0x00, 0x00, 0x00 });
                break;
            case LlConnectionParamReq:
                SendControl(new byte[] { LlRejectExtInd, LlConnectionParamReq, ErrUnsupportedRemoteFeature });
                break;
            case LlPingReq:
                SendControl(new byte[] { LlPingRsp });
                break;
            case LlEncRsp:
                if(enc == Enc.WaitEncRsp && p.Length >= 13)
                {
                    // SK = e(LTK, SKD), SKD and IV with the central's part in
                    // the least significant octets.
                    var skd = new byte[16];
                    Array.Copy(skdCentral, 0, skd, 0, 8);
                    Array.Copy(p, 1, skd, 8, 8);
                    sessionKey = AesEncrypt(encKey, Reversed(skd));
                    Array.Copy(p, 9, sessionIv, 4, 4);
                    enc = Enc.WaitStartEncReq;
                }
                break;
            case LlStartEncReq:
                if(enc == Enc.WaitStartEncReq)
                {
                    txCounter = 0;
                    rxCounter = 0;
                    txEncrypted = true;
                    rxEncrypted = true;
                    SendControl(new byte[] { LlStartEncRsp }, true);
                    enc = Enc.WaitStartEncRsp;
                }
                break;
            case LlStartEncRsp:
                if(enc == Enc.WaitStartEncRsp)
                {
                    enc = Enc.None;
                    encrypted = true;
                    OnEncrypted();
                }
                break;
            case LlRejectInd:
                OnRejected(enc != Enc.None ? LlEncReq : (byte)0xFF, p.Length > 1 ? p[1] : (byte)0);
                break;
            case LlRejectExtInd:
                if(p.Length >= 3)
                {
                    OnRejected(p[1], p[2]);
                }
                break;
            case LlUnknownRsp:
                if(p.Length >= 2 && p[1] == LlLengthReq)
                {
                    LengthProcedureDone();
                }
                break;
            case LlFeatureRsp:
            case LlPingRsp:
            case LlMinUsedChannelsInd:
                break;
            default:
                SendControl(new byte[] { LlUnknownRsp, p[0] });
                break;
            }
        }

        private void LengthProcedureDone()
        {
            if(!lengthPending)
            {
                return;
            }
            lengthPending = false;
            if(bondKey != null && !encrypted && !SecurityBusy)
            {
                StartEncryption(bondKey);
            }
        }

        private void OnRejected(byte opcode, byte error)
        {
            if(opcode == LlLengthReq)
            {
                LengthProcedureDone();
                return;
            }
            if(opcode != LlEncReq || enc == Enc.None)
            {
                return;
            }
            enc = Enc.None;
            txEncrypted = false;
            rxEncrypted = false;
            if(smp == Smp.WaitEncryption)
            {
                smp = Smp.Idle;
                Say("pairing failed: the lens refused to encrypt with the new key (0x{0:X2})", error);
                return;
            }
            // 0x06, "PIN or key missing": the lens has lost its half of the bond.
            Say("the lens has no key for this phone (0x{0:X2}): bond forgotten", error);
            bondKey = null;
            if(PairOnConnect || securityRequested)
            {
                StartPairing();
            }
        }

        private void StartEncryption(byte[] key)
        {
            encKey = key;
            skdCentral = RandomBytes(8);
            var ivCentral = RandomBytes(4);
            Array.Copy(ivCentral, 0, sessionIv, 0, 4);
            var pdu = new byte[23];                 // Rand and EDIV are zero for a Secure Connections key
            pdu[0] = LlEncReq;
            Array.Copy(skdCentral, 0, pdu, 11, 8);
            Array.Copy(ivCentral, 0, pdu, 19, 4);
            enc = Enc.WaitEncRsp;
            SendControl(pdu, true);
        }

        private void OnEncrypted()
        {
            if(smp != Smp.WaitEncryption)
            {
                Say("link encrypted with the bond's key");
                return;
            }
            // Phase 3: the lens hands over its keys, then the phone its own.
            smp = Smp.KeyDistribution;
            keysFromPeer &= ~DistEncKey;            // a Secure Connections LTK is derived, not sent
            keysToPeer &= ~DistEncKey;
            FinishKeyDistribution();
        }

        // AES-CCM as the link layer uses it (Core spec Vol 6 Part E): a 13-byte
        // nonce of the 39-bit packet counter, the direction bit and the IV; a
        // 4-byte MIC; one byte of associated data, the PDU header with NESN,
        // SN and MD masked out. Returns payload + MIC when encrypting, and the
        // payload, or null if the MIC is wrong, when decrypting.
        private byte[] Ccm(byte[] input, byte headerBits, ulong counter, bool fromCentral)
        {
            var nonce = new byte[13];
            for(var i = 0; i < 4; i++)
            {
                nonce[i] = (byte)(counter >> (8 * i));
            }
            nonce[4] = (byte)(((counter >> 32) & 0x7F) | (fromCentral ? 0x80u : 0u));
            Array.Copy(sessionIv, 0, nonce, 5, 8);

            var length = fromCentral ? input.Length : input.Length - MicLen;
            using(var aes = AesProvider.GetEcbProvider(sessionKey))
            {
                Func<byte[], byte[]> cipher = block =>
                {
                    var result = (byte[])block.Clone();
                    aes.EncryptBlockInSitu(Block.UsingBytes(result));
                    return result;
                };
                Func<int, byte[]> counterBlock = index =>
                {
                    var block = new byte[16];
                    block[0] = 0x01;
                    Array.Copy(nonce, 0, block, 1, 13);
                    block[14] = (byte)(index >> 8);
                    block[15] = (byte)index;
                    return block;
                };

                var output = new byte[length];
                for(var i = 0; i < length; i += 16)
                {
                    var keystream = cipher(counterBlock(i / 16 + 1));
                    for(var j = i; j < Math.Min(i + 16, length); j++)
                    {
                        output[j] = (byte)(input[j] ^ keystream[j - i]);
                    }
                }
                var plain = fromCentral ? input : output;

                var b0 = new byte[16];
                b0[0] = 0x49;
                Array.Copy(nonce, 0, b0, 1, 13);
                b0[14] = (byte)(length >> 8);
                b0[15] = (byte)length;
                var x = cipher(b0);
                x[1] ^= 0x01;                       // the associated data: its length, 1 ...
                x[2] ^= (byte)(headerBits & 0xE3);  // ... and the masked header
                x = cipher(x);
                for(var i = 0; i < length; i += 16)
                {
                    for(var j = i; j < Math.Min(i + 16, length); j++)
                    {
                        x[j - i] ^= plain[j];
                    }
                    x = cipher(x);
                }
                var s0 = cipher(counterBlock(0));
                for(var i = 0; i < MicLen; i++)
                {
                    x[i] ^= s0[i];
                }

                if(fromCentral)
                {
                    var result = new byte[length + MicLen];
                    Array.Copy(output, result, length);
                    Array.Copy(x, 0, result, length, MicLen);
                    return result;
                }
                for(var i = 0; i < MicLen; i++)
                {
                    if(x[i] != input[length + i])
                    {
                        return null;
                    }
                }
                return output;
            }
        }

        // --------------------------------- L2CAP --------------------------------

        private void SendL2cap(int channel, byte[] payload)
        {
            var frame = new byte[4 + payload.Length];
            frame[0] = (byte)payload.Length;
            frame[1] = (byte)(payload.Length >> 8);
            frame[2] = (byte)channel;
            frame[3] = (byte)(channel >> 8);
            Array.Copy(payload, 0, frame, 4, payload.Length);
            for(var offset = 0; offset < frame.Length; offset += maxTxOctets)
            {
                var part = new byte[Math.Min(maxTxOctets, frame.Length - offset)];
                Array.Copy(frame, offset, part, 0, part.Length);
                data.Enqueue(new Pdu { Llid = offset == 0 ? LlidStart : LlidContinuation, Payload = part });
            }
        }

        private void DeliverIfComplete()
        {
            var length = reassembly[0] | (reassembly[1] << 8);
            if(reassembly.Count < 4 + length)
            {
                return;
            }
            var channel = reassembly[2] | (reassembly[3] << 8);
            var payload = reassembly.GetRange(4, length).ToArray();
            reassembly = null;
            if(payload.Length == 0)
            {
                return;
            }
            switch(channel)
            {
            case ChannelAtt:
                HandleAtt(payload);
                break;
            case ChannelSignalling:
                HandleSignalling(payload);
                break;
            case ChannelSmp:
                HandleSmp(payload);
                break;
            }
        }

        private void HandleSignalling(byte[] p)
        {
            if(p.Length < 4)
            {
                return;
            }
            this.Log(LogLevel.Debug, "L2CAP signalling from the lens: {0}", Hex(p));
            switch(p[0])
            {
            case 0x12:      // Connection Parameter Update Request: the phone keeps its own
                SendL2cap(ChannelSignalling, new byte[] { 0x13, p[1], 0x02, 0x00, 0x01, 0x00 });
                break;
            case 0x01:      // Command Reject
            case 0x13:
            case 0x15:
            case 0x16:
                break;
            default:        // Command Reject: command not understood
                SendL2cap(ChannelSignalling, new byte[] { 0x01, p[1], 0x02, 0x00, 0x00, 0x00 });
                break;
            }
        }

        // ---------------------------------- ATT ---------------------------------

        private void AttRequest(byte[] pdu, Action<byte[]> onResponse)
        {
            attQueue.Enqueue(new AttTransaction { Request = pdu, OnResponse = onResponse });
            AttPump();
        }

        private void AttPump()
        {
            if(attPending == null && attQueue.Count > 0)
            {
                attPending = attQueue.Dequeue();
                this.Log(LogLevel.Debug, "ATT to the lens: {0}", Hex(attPending.Request));
                SendL2cap(ChannelAtt, attPending.Request);
            }
        }

        private void HandleAtt(byte[] p)
        {
            var opcode = p[0];
            if(opcode == AttNotification)
            {
                if(p.Length >= 3 && (p[1] | (p[2] << 8)) == nusTxHandle && nusTxHandle != 0)
                {
                    var packet = new byte[p.Length - 3];
                    Array.Copy(p, 3, packet, 0, packet.Length);
                    OnPacket(packet);
                }
                return;
            }
            this.Log(LogLevel.Debug, "ATT from the lens: {0}", Hex(p));
            if(opcode == AttIndication)
            {
                SendL2cap(ChannelAtt, new byte[] { AttConfirmation });
                return;
            }
            if(Array.IndexOf(AttResponses, opcode) >= 0)
            {
                if(attPending == null || (opcode == AttError && (p.Length < 5 || p[1] != attPending.Request[0])))
                {
                    return;
                }
                var done = attPending;
                attPending = null;
                done.OnResponse(p);
                AttPump();
                return;
            }
            if(opcode == AttMtuReq && p.Length >= 3)
            {
                attMtu = Math.Max(DefaultAttMtu, Math.Min((int)Mtu, p[1] | (p[2] << 8)));
                SendL2cap(ChannelAtt, new byte[] { AttMtuRsp, (byte)Mtu, (byte)(Mtu >> 8) });
                return;
            }
            if((opcode & 0x40) != 0)
            {
                return;                 // a command: nothing to answer
            }
            // The phone's own attribute table is empty.
            var handle = p.Length >= 3 ? new[] { p[1], p[2] } : new byte[] { 0, 0 };
            byte error;
            switch(opcode)
            {
            case 0x04:      // Find Information
            case 0x06:      // Find By Type Value
            case 0x08:      // Read By Type
            case 0x10:      // Read By Group Type
                error = 0x0A;           // attribute not found
                break;
            case 0x0A:      // Read
            case 0x0C:      // Read Blob
            case 0x0E:      // Read Multiple
            case 0x12:      // Write
            case 0x16:      // Prepare Write
                error = 0x01;           // invalid handle
                break;
            default:
                error = 0x06;           // request not supported
                handle = new byte[] { 0, 0 };
                break;
            }
            SendL2cap(ChannelAtt, new byte[] { AttError, opcode, handle[0], handle[1], error });
        }

        // MTU, then the Nordic UART Service: find its two characteristics by
        // UUID, find the TX characteristic's CCC descriptor, switch
        // notifications on.
        private void StartGattSetup()
        {
            AttRequest(new byte[] { AttMtuReq, (byte)Mtu, (byte)(Mtu >> 8) }, response =>
            {
                if(response[0] == AttMtuRsp && response.Length >= 3)
                {
                    attMtu = Math.Max(DefaultAttMtu, Math.Min((int)Mtu, response[1] | (response[2] << 8)));
                }
                Say("ATT MTU {0}", attMtu);
                DiscoverCharacteristics(0x0001);
            });
        }

        private void DiscoverCharacteristics(int from)
        {
            // Read By Type: every characteristic declaration from that handle on.
            AttRequest(new byte[] { 0x08, (byte)from, (byte)(from >> 8), 0xFF, 0xFF, 0x03, 0x28 }, response =>
            {
                var last = 0xFFFF;
                if(response[0] == 0x09 && response.Length >= 2 && response[1] >= 7)
                {
                    int size = response[1];
                    for(var at = 2; at + size <= response.Length; at += size)
                    {
                        var uuid = new byte[size - 5];
                        Array.Copy(response, at + 5, uuid, 0, uuid.Length);
                        last = response[at] | (response[at + 1] << 8);
                        characteristics.Add(new Characteristic
                        {
                            Declaration = last,
                            Value = response[at + 3] | (response[at + 4] << 8),
                            Uuid = uuid,
                        });
                    }
                }
                if(last < 0xFFFF)
                {
                    DiscoverCharacteristics(last + 1);
                    return;
                }
                foreach(var characteristic in characteristics)
                {
                    if(SameBytes(characteristic.Uuid, NusRxUuid))
                    {
                        nusRxHandle = (uint)characteristic.Value;
                    }
                    else if(SameBytes(characteristic.Uuid, NusTxUuid))
                    {
                        nusTxHandle = (uint)characteristic.Value;
                    }
                }
                // The TX characteristic's descriptors end where the next
                // characteristic starts.
                var end = 0xFFFF;
                foreach(var characteristic in characteristics)
                {
                    if(characteristic.Declaration > nusTxHandle && characteristic.Declaration <= end)
                    {
                        end = characteristic.Declaration - 1;
                    }
                }
                if(nusRxHandle == 0 || nusTxHandle == 0)
                {
                    this.Log(LogLevel.Warning, "the lens has no Nordic UART Service ({0} characteristics found): "
                             + "packets cannot be exchanged", characteristics.Count);
                    return;
                }
                FindCcc((int)nusTxHandle + 1, end);
            });
        }

        private void FindCcc(int from, int to)
        {
            if(from > to)
            {
                this.Log(LogLevel.Warning, "the Nordic UART TX characteristic has no CCC descriptor");
                return;
            }
            // Find Information: the descriptors behind the TX value.
            AttRequest(new byte[] { 0x04, (byte)from, (byte)(from >> 8), (byte)to, (byte)(to >> 8) }, response =>
            {
                var last = to;
                if(response[0] == 0x05 && response.Length >= 6 && response[1] == 0x01)
                {
                    for(var at = 2; at + 4 <= response.Length; at += 4)
                    {
                        last = response[at] | (response[at + 1] << 8);
                        var uuid = response[at + 2] | (response[at + 3] << 8);
                        if(uuid == 0x2902)
                        {
                            nusCccHandle = (uint)last;
                            EnableNotifications();
                            return;
                        }
                        if(uuid == 0x2800 || uuid == 0x2801 || uuid == 0x2803)
                        {
                            last = to;          // the next service or characteristic
                            break;
                        }
                    }
                }
                FindCcc(last + 1, to);
            });
        }

        private void EnableNotifications()
        {
            AttRequest(new byte[] { 0x12, (byte)nusCccHandle, (byte)(nusCccHandle >> 8), 0x01, 0x00 }, response =>
            {
                if(response[0] != 0x13)
                {
                    this.Log(LogLevel.Warning, "the lens refused the write that switches notifications on: {0}",
                             Hex(response));
                    return;
                }
                nusReady = true;
                lastWriteMs = NowMs;
                Say("Nordic UART Service: RX 0x{0:X4}, TX 0x{1:X4}, notifications on", nusRxHandle, nusTxHandle);
                if(PairOnConnect && bondKey == null && !encrypted && !SecurityBusy)
                {
                    StartPairing();
                }
            });
        }

        private void WritePacket(byte[] packet)
        {
            var pdu = new byte[3 + packet.Length];
            pdu[0] = 0x52;                          // Write Command
            pdu[1] = (byte)nusRxHandle;
            pdu[2] = (byte)(nusRxHandle >> 8);
            Array.Copy(packet, 0, pdu, 3, packet.Length);
            SendL2cap(ChannelAtt, pdu);
            lastWriteMs = NowMs;
        }

        private void OnPacket(byte[] packet)
        {
            if(heartbeatSent != null && packet.Length >= heartbeatSent.Length
               && SameBytes(heartbeatSent, Slice(packet, 0, heartbeatSent.Length)))
            {
                heartbeatSent = null;               // the echo of a heartbeat
                this.Log(LogLevel.Debug, "heartbeat answered");
                return;
            }
            packetsReceived++;
            Say("rx {0}", Hex(packet));
        }

        // Once per connection event: write what is waiting, keep the lens
        // from concluding that the phone has gone.
        private void Tick(double now)
        {
            if(lengthPending && now - connectedAtMs >= LengthProcedureMs)
            {
                LengthProcedureDone();          // the lens left the request unanswered
            }
            if(!nusReady || SecurityBusy || terminating)
            {
                return;
            }
            while(pendingPackets.Count > 0)
            {
                var packet = pendingPackets.Dequeue();
                if(packet.Length > attMtu - 3)
                {
                    this.Log(LogLevel.Warning, "packet of {0} bytes dropped: with an ATT MTU of {1} a write carries {2}",
                             packet.Length, attMtu, attMtu - 3);
                    continue;
                }
                WritePacket(packet);
                packetsSent++;
                Say("tx {0}", Hex(packet));
            }
            if(HeartbeatSeconds > 0 && now - lastWriteMs >= HeartbeatSeconds * 1000)
            {
                heartbeatSent = new byte[] { 0x25, 0x06, 0x00, heartbeatSequence, 0x04, heartbeatSequence };
                heartbeatSequence++;
                WritePacket(heartbeatSent);
                this.Log(LogLevel.Debug, "heartbeat {0}", Hex(heartbeatSent));
            }
        }

        // ---------------------------------- SMP ---------------------------------

        private bool SecurityBusy => smp != Smp.Idle || enc != Enc.None;

        private void StartPairing()
        {
            privateKey = P256.NewPrivateKey(RandomBytes(32));
            publicKey = P256.Multiply(privateKey, P256.G);
            pairingRequest = new byte[] { 0x01, IoKeyboardDisplay, 0x00, AuthReq, 16, KeyDistribution, KeyDistribution };
            smp = Smp.WaitResponse;
            Say("pairing: started");
            SendSmp(pairingRequest);
        }

        private void SendSmp(byte[] pdu)
        {
            this.Log(LogLevel.Debug, "SMP to the lens: {0}", Hex(pdu));
            SendL2cap(ChannelSmp, pdu);
        }

        private void PairingFailed(byte reason, string why)
        {
            smp = Smp.Idle;
            Say("pairing failed: {0}", why);
            SendSmp(new byte[] { 0x05, reason });
        }

        private void HandleSmp(byte[] p)
        {
            this.Log(LogLevel.Debug, "SMP from the lens: {0}", Hex(p));
            switch(p[0])
            {
            case 0x0B:      // Security Request
                securityRequested = true;
                if(encrypted || SecurityBusy)
                {
                    break;
                }
                if(bondKey != null)
                {
                    StartEncryption(bondKey);
                }
                else
                {
                    StartPairing();
                }
                break;

            case 0x05:      // Pairing Failed
                if(smp != Smp.Idle)
                {
                    smp = Smp.Idle;
                    Say("pairing failed: the lens gave reason 0x{0:X2}", p.Length > 1 ? p[1] : 0);
                }
                break;

            case 0x02:      // Pairing Response
                if(smp != Smp.WaitResponse || p.Length < 7)
                {
                    break;
                }
                if((p[3] & AuthSecureConnections) == 0)
                {
                    PairingFailed(0x03, "the lens does not offer LE Secure Connections");
                    break;
                }
                if(p[4] < 7 || p[4] > 16)
                {
                    PairingFailed(0x06, "the lens offers an unusable key size");
                    break;
                }
                pairingResponse = Slice(p, 0, 7);
                keysToPeer = p[5] & KeyDistribution;
                keysFromPeer = p[6] & KeyDistribution;
                smp = Smp.WaitPublicKey;
                var key = new byte[65];
                key[0] = 0x0C;
                Array.Copy(Reversed(P256.Bytes(publicKey[0])), 0, key, 1, 32);
                Array.Copy(Reversed(P256.Bytes(publicKey[1])), 0, key, 33, 32);
                SendSmp(key);
                break;

            case 0x0C:      // Pairing Public Key
                if(smp != Smp.WaitPublicKey || p.Length < 65)
                {
                    break;
                }
                peerPublicKey = new[]
                {
                    P256.Number(Reversed(Slice(p, 1, 32))),
                    P256.Number(Reversed(Slice(p, 33, 32))),
                };
                if(!P256.OnCurve(peerPublicKey))
                {
                    PairingFailed(0x0A, "the lens's public key is not on the curve");
                    break;
                }
                smp = Smp.WaitConfirm;
                break;

            case 0x03:      // Pairing Confirm
                if(smp != Smp.WaitConfirm || p.Length < 17)
                {
                    break;
                }
                peerConfirm = Reversed(Slice(p, 1, 16));
                nonce = RandomBytes(16);
                smp = Smp.WaitRandom;
                SendSmp(Concat(new byte[] { 0x04 }, Reversed(nonce)));
                break;

            case 0x04:      // Pairing Random
                if(smp != Smp.WaitRandom || p.Length < 17)
                {
                    break;
                }
                peerNonce = Reversed(Slice(p, 1, 16));
                // Cb = f4(PKbx, PKax, Nb, 0)
                var confirm = Cmac(peerNonce, Concat(P256.Bytes(peerPublicKey[0]), P256.Bytes(publicKey[0]), new byte[1]));
                if(!SameBytes(confirm, peerConfirm))
                {
                    PairingFailed(0x04, "the lens's confirm value does not match");
                    break;
                }
                // f5: MacKey and LTK out of the shared secret, both nonces
                // and both addresses.
                var secret = P256.Bytes(P256.Multiply(privateKey, peerPublicKey)[0]);
                var t = Cmac(F5Salt, secret);
                var a = Concat(new byte[] { 0x00 }, Reversed(InitA));
                var b = Concat(new byte[] { (byte)(advARandom ? 0x01 : 0x00) }, Reversed(advA));
                var tail = Concat(new byte[] { 0x62, 0x74, 0x6C, 0x65 }, nonce, peerNonce, a, b, new byte[] { 0x01, 0x00 });
                macKey = Cmac(t, Concat(new byte[] { 0x00 }, tail));
                newKey = Cmac(t, Concat(new byte[] { 0x01 }, tail));
                // Ea = f6(MacKey, Na, Nb, 0, IOcapA, A, B), and the Eb to expect.
                var zero = new byte[16];
                var ioA = new[] { pairingRequest[3], pairingRequest[2], pairingRequest[1] };
                var ioB = new[] { pairingResponse[3], pairingResponse[2], pairingResponse[1] };
                var check = Cmac(macKey, Concat(nonce, peerNonce, zero, ioA, a, b));
                peerCheck = Cmac(macKey, Concat(peerNonce, nonce, zero, ioB, b, a));
                smp = Smp.WaitDhKeyCheck;
                SendSmp(Concat(new byte[] { 0x0D }, Reversed(check)));
                break;

            case 0x0D:      // Pairing DHKey Check
                if(smp != Smp.WaitDhKeyCheck || p.Length < 17)
                {
                    break;
                }
                if(!SameBytes(Reversed(Slice(p, 1, 16)), peerCheck))
                {
                    PairingFailed(0x0B, "the lens's DHKey check does not match");
                    break;
                }
                smp = Smp.WaitEncryption;
                StartEncryption(newKey);
                break;

            case 0x07:      // Central Identification: ends the lens's encryption key
                KeyFromPeer(DistEncKey);
                break;
            case 0x09:      // Identity Address Information: ends its identity key
                KeyFromPeer(DistIdKey);
                break;
            case 0x0A:      // Signing Information
                KeyFromPeer(DistSignKey);
                break;
            }
        }

        private void KeyFromPeer(int which)
        {
            if(smp == Smp.KeyDistribution)
            {
                keysFromPeer &= ~which;
                FinishKeyDistribution();
            }
        }

        private void FinishKeyDistribution()
        {
            if(keysFromPeer != 0)
            {
                return;             // the lens has more to send
            }
            if((keysToPeer & DistIdKey) != 0)
            {
                SendSmp(Concat(new byte[] { 0x08 }, IdentityKey));
                SendSmp(Concat(new byte[] { 0x09, 0x00 }, InitA));
            }
            if((keysToPeer & DistSignKey) != 0)
            {
                SendSmp(Concat(new byte[] { 0x0A }, RandomBytes(16)));
            }
            bondKey = newKey;
            smp = Smp.Idle;
            Say("pairing: complete, link encrypted, bonded");
        }

        // AES-CMAC (RFC 4493), which the Secure Connections functions f4, f5
        // and f6 are built on. Key, message and result most significant octet
        // first, as the specification writes them.
        private static byte[] Cmac(byte[] key, byte[] message)
        {
            var k1 = CmacSubkey(AesEncrypt(key, new byte[16]));
            var k2 = CmacSubkey(k1);
            var blocks = Math.Max(1, (message.Length + 15) / 16);
            var whole = message.Length > 0 && message.Length % 16 == 0;
            var x = new byte[16];
            for(var n = 0; n < blocks; n++)
            {
                var block = new byte[16];
                var count = Math.Min(16, message.Length - 16 * n);
                Array.Copy(message, 16 * n, block, 0, count);
                if(n == blocks - 1)
                {
                    if(!whole)
                    {
                        block[count] = 0x80;
                    }
                    for(var i = 0; i < 16; i++)
                    {
                        block[i] ^= whole ? k1[i] : k2[i];
                    }
                }
                for(var i = 0; i < 16; i++)
                {
                    block[i] ^= x[i];
                }
                x = AesEncrypt(key, block);
            }
            return x;
        }

        private static byte[] CmacSubkey(byte[] input)
        {
            var output = new byte[16];
            for(var i = 0; i < 16; i++)
            {
                output[i] = (byte)((input[i] << 1) | (i < 15 ? input[i + 1] >> 7 : 0));
            }
            if((input[0] & 0x80) != 0)
            {
                output[15] ^= 0x87;
            }
            return output;
        }

        private static byte[] AesEncrypt(byte[] key, byte[] block)
        {
            var result = (byte[])block.Clone();
            using(var aes = AesProvider.GetEcbProvider(key))
            {
                aes.EncryptBlockInSitu(Block.UsingBytes(result));
            }
            return result;
        }

        // The curve of Secure Connections pairing, in affine coordinates. A
        // point is { x, y }, or null for the point at infinity.
        private static class P256
        {
            public static BigInteger Number(byte[] bigEndian)
            {
                var littleEndian = new byte[bigEndian.Length + 1];      // a zero on top: not negative
                for(var i = 0; i < bigEndian.Length; i++)
                {
                    littleEndian[i] = bigEndian[bigEndian.Length - 1 - i];
                }
                return new BigInteger(littleEndian);
            }

            public static byte[] Bytes(BigInteger number)
            {
                var littleEndian = number.ToByteArray();
                var bigEndian = new byte[32];
                for(var i = 0; i < Math.Min(32, littleEndian.Length); i++)
                {
                    bigEndian[31 - i] = littleEndian[i];
                }
                return bigEndian;
            }

            public static BigInteger NewPrivateKey(byte[] random)
            {
                return Number(random) % (N - 1) + 1;
            }

            public static bool OnCurve(BigInteger[] point)
            {
                var x = point[0];
                var y = point[1];
                return x.Sign >= 0 && x < P && y.Sign >= 0 && y < P
                    && Mod(y * y - (x * x * x - 3 * x + B)).IsZero;
            }

            public static BigInteger[] Multiply(BigInteger scalar, BigInteger[] point)
            {
                BigInteger[] result = null;
                for(var bit = 255; bit >= 0; bit--)
                {
                    result = Add(result, result);
                    if(!((scalar >> bit) & 1).IsZero)
                    {
                        result = Add(result, point);
                    }
                }
                return result;
            }

            private static BigInteger[] Add(BigInteger[] p, BigInteger[] q)
            {
                if(p == null)
                {
                    return q;
                }
                if(q == null)
                {
                    return p;
                }
                BigInteger slope;
                if(p[0] == q[0])
                {
                    if(Mod(p[1] + q[1]).IsZero)
                    {
                        return null;
                    }
                    slope = (3 * p[0] * p[0] - 3) * Inverse(2 * p[1]);
                }
                else
                {
                    slope = (q[1] - p[1]) * Inverse(q[0] - p[0]);
                }
                var x = Mod(slope * slope - p[0] - q[0]);
                return new[] { x, Mod(slope * (p[0] - x) - p[1]) };
            }

            private static BigInteger Mod(BigInteger number)
            {
                var result = number % P;
                return result.Sign < 0 ? result + P : result;
            }

            private static BigInteger Inverse(BigInteger number)
            {
                return BigInteger.ModPow(Mod(number), P - 2, P);
            }

            private static BigInteger Constant(string hex)
            {
                return BigInteger.Parse("0" + hex, NumberStyles.AllowHexSpecifier);
            }

            private static readonly BigInteger P = Constant("FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF");
            private static readonly BigInteger N = Constant("FFFFFFFF00000000FFFFFFFFFFFFFFFFBCE6FAADA7179E84F3B9CAC2FC632551");
            private static readonly BigInteger B = Constant("5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B");

            public static readonly BigInteger[] G =
            {
                Constant("6B17D1F2E12C4247F8BCE6E563A440F277037D812DEB33A0F4A13945D898C296"),
                Constant("4FE342E2FE1A7F9B8EE7EB4A7C0F9E162BCE33576B315ECECBB6406837BF51F5"),
            };
        }

        // -------------------------------- helpers -------------------------------

        private double NowMs => machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds * 1000.0;

        // A line for whoever watches the phone: prefixed with the emulated
        // time, which also keeps equal lines apart in the log.
        private void Say(string format, params object[] arguments)
        {
            this.Log(LogLevel.Info, "{0}  {1}",
                     string.Format(CultureInfo.InvariantCulture, "{0,8:F2}s", NowMs / 1000.0),
                     string.Format(CultureInfo.InvariantCulture, format, arguments));
        }

        // Reproducible: an emulation that is run twice uses the same access
        // addresses and pairs with the same keys.
        private byte[] RandomBytes(int count)
        {
            var bytes = new byte[count];
            random.NextBytes(bytes);
            return bytes;
        }

        private static bool AaEquals(byte[] frame, byte[] address)
        {
            for(var i = 0; i < AddrLen; i++)
            {
                if(frame[i] != address[i])
                {
                    return false;
                }
            }
            return true;
        }

        private static bool SameBytes(byte[] a, byte[] b)
        {
            if(a.Length != b.Length)
            {
                return false;
            }
            for(var i = 0; i < a.Length; i++)
            {
                if(a[i] != b[i])
                {
                    return false;
                }
            }
            return true;
        }

        private static byte[] Slice(byte[] source, int offset, int count)
        {
            var result = new byte[count];
            Array.Copy(source, offset, result, 0, count);
            return result;
        }

        private static byte[] Reversed(byte[] source)
        {
            var result = (byte[])source.Clone();
            Array.Reverse(result);
            return result;
        }

        private static byte[] Concat(params byte[][] parts)
        {
            var result = new List<byte>();
            foreach(var part in parts)
            {
                result.AddRange(part);
            }
            return result.ToArray();
        }

        private static string Hex(byte[] bytes)
        {
            return BitConverter.ToString(bytes).Replace("-", " ");
        }

        // An address as it is written, from its on-air (least significant
        // byte first) order.
        private static string AddressText(byte[] address)
        {
            return BitConverter.ToString(Reversed(address)).Replace("-", ":");
        }

        private static byte[] ParseHex(string hex)
        {
            var digits = (hex ?? "").Replace(" ", "").Replace("_", "");
            if(digits.Length % 2 != 0)
            {
                throw new RecoverableException("expected an even number of hex digits");
            }
            var bytes = new byte[digits.Length / 2];
            for(var i = 0; i < bytes.Length; i++)
            {
                if(!byte.TryParse(digits.Substring(2 * i, 2), NumberStyles.AllowHexSpecifier,
                                  CultureInfo.InvariantCulture, out bytes[i]))
                {
                    throw new RecoverableException("not hex: " + digits.Substring(2 * i, 2));
                }
            }
            return bytes;
        }

        private class Pdu
        {
            public int Llid;
            public byte[] Payload;
            public bool DuringEncryption;       // may be sent while encryption is being started
            public Action OnAcked;
        }

        private class AttTransaction
        {
            public byte[] Request;
            public Action<byte[]> OnResponse;
        }

        private class Characteristic
        {
            public int Declaration;
            public int Value;
            public byte[] Uuid;
        }

        private enum Link { Idle, Connecting, Connected }

        // The encryption start procedure, by the PDU that is awaited.
        private enum Enc { None, WaitEncRsp, WaitStartEncReq, WaitStartEncRsp }

        private enum Smp
        {
            Idle, WaitResponse, WaitPublicKey, WaitConfirm, WaitRandom, WaitDhKeyCheck,
            WaitEncryption, KeyDistribution,
        }

        private readonly NRF5340_RADIO radio;
        private readonly object sync = new object();
        private Random random;
        private bool enabled;
        private uint mtu = 251;

        // advertising and the link
        private Link link;
        private byte[] advA;
        private byte[] accessAddress = new byte[AddrLen];
        private bool advARandom;
        private bool haveAdv;
        private double reconnectNotBeforeMs;
        private double lastWindowMs;
        private double connectedAtMs;
        private int connectIndsSent;
        private int dataEventsAnswered;
        private int txSn, nextRxSn;
        private Pdu lastTx;
        private readonly List<Pdu> control = new List<Pdu>();
        private readonly Queue<Pdu> data = new Queue<Pdu>();
        private bool peerTerminated;
        private byte peerTerminateReason;
        private bool terminating;
        private bool versionSent;
        private bool lengthPending;
        private int maxTxOctets = DefaultOctets;

        // encryption
        private Enc enc;
        private bool encrypted;
        private bool txEncrypted, rxEncrypted;
        private ulong txCounter, rxCounter;
        private byte[] encKey;
        private byte[] skdCentral;
        private byte[] sessionKey;
        private readonly byte[] sessionIv = new byte[8];

        // L2CAP and ATT
        private List<byte> reassembly;
        private readonly Queue<AttTransaction> attQueue = new Queue<AttTransaction>();
        private AttTransaction attPending;
        private int attMtu = DefaultAttMtu;
        private readonly List<Characteristic> characteristics = new List<Characteristic>();
        private bool nusReady;
        private uint nusRxHandle, nusTxHandle, nusCccHandle;
        private readonly Queue<byte[]> pendingPackets = new Queue<byte[]>();
        private double lastWriteMs;
        private byte[] heartbeatSent;
        private byte heartbeatSequence;
        private int packetsSent;
        private int packetsReceived;

        // pairing; keys and nonces most significant octet first
        private Smp smp;
        private bool securityRequested;
        private byte[] bondKey;
        private byte[] newKey;
        private byte[] macKey;
        private BigInteger privateKey;
        private BigInteger[] publicKey;
        private BigInteger[] peerPublicKey;
        private byte[] pairingRequest;
        private byte[] pairingResponse;
        private byte[] nonce;
        private byte[] peerNonce;
        private byte[] peerConfirm;
        private byte[] peerCheck;
        private int keysToPeer;
        private int keysFromPeer;

        private const int AddrLen = 4;
        private const int CrcLen = 3;
        private const int MicLen = 4;
        private const int AdvIndType = 0x0;
        private const int LlidContinuation = 1;
        private const int LlidStart = 2;
        private const int LlidControl = 3;
        private const int DefaultOctets = 27;
        private const byte MaxOctets = 251;
        private const int DefaultAttMtu = 23;
        private const int MaxPendingPackets = 64;
        private const double ReconnectDelayMs = 1000;
        private const double LengthProcedureMs = 1000;
        private const double SupervisionTimeoutMs = 2000;       // as in the CONNECT_IND

        private const byte LlTerminateInd = 0x02;
        private const byte LlEncReq = 0x03;
        private const byte LlEncRsp = 0x04;
        private const byte LlStartEncReq = 0x05;
        private const byte LlStartEncRsp = 0x06;
        private const byte LlUnknownRsp = 0x07;
        private const byte LlFeatureRsp = 0x09;
        private const byte LlVersionInd = 0x0C;
        private const byte LlRejectInd = 0x0D;
        private const byte LlPeripheralFeatureReq = 0x0E;
        private const byte LlConnectionParamReq = 0x0F;
        private const byte LlRejectExtInd = 0x11;
        private const byte LlPingReq = 0x12;
        private const byte LlPingRsp = 0x13;
        private const byte LlLengthReq = 0x14;
        private const byte LlLengthRsp = 0x15;
        private const byte LlPhyReq = 0x16;
        private const byte LlPhyUpdateInd = 0x18;
        private const byte LlMinUsedChannelsInd = 0x19;
        private const byte ErrUnsupportedRemoteFeature = 0x1A;
        // LE Encryption, Extended Reject Indication, Peripheral-initiated
        // Features Exchange, LE Ping, LE Data Packet Length Extension
        private const byte Features = 0x3D;

        private const int ChannelAtt = 0x0004;
        private const int ChannelSignalling = 0x0005;
        private const int ChannelSmp = 0x0006;

        private const byte AttError = 0x01;
        private const byte AttMtuReq = 0x02;
        private const byte AttMtuRsp = 0x03;
        private const byte AttNotification = 0x1B;
        private const byte AttIndication = 0x1D;
        private const byte AttConfirmation = 0x1E;
        private static readonly byte[] AttResponses =
            { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x11, 0x13, 0x17, 0x19, 0x21 };

        private const byte IoKeyboardDisplay = 0x04;
        private const byte AuthSecureConnections = 0x08;
        private const byte AuthReq = 0x0D;                      // bonding, MITM, Secure Connections
        private const int DistEncKey = 0x01;
        private const int DistIdKey = 0x02;
        private const int DistSignKey = 0x04;
        private const byte KeyDistribution = DistEncKey | DistIdKey;

        private static readonly byte[] AdvAccessAddress = { 0xD6, 0xBE, 0x89, 0x8E };

        // The phone's address, public, 66:55:44:33:22:11: the one the lens's
        // settings seed (g1_seed.py) has it bound to.
        private static readonly byte[] InitA = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };
        private static readonly byte[] IdentityKey =
            { 0x50, 0x48, 0x4F, 0x4E, 0x45, 0x2D, 0x49, 0x52, 0x4B, 0x2D, 0x47, 0x31, 0x53, 0x49, 0x4D, 0x21 };

        // CONNECT_IND connection parameters.
        private const byte WinSize = 0x03;
        private static readonly byte[] WinOffset = { 0x00, 0x00 };
        private static readonly byte[] Interval = { 0x18, 0x00 };   // 30 ms
        private static readonly byte[] Latency = { 0x00, 0x00 };
        private static readonly byte[] Timeout = { 0xC8, 0x00 };    // 2 s
        private static readonly byte[] ChannelMap = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F };

        // Nordic UART Service characteristics, as their UUIDs are sent.
        private static readonly byte[] NusRxUuid =
            { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E };
        private static readonly byte[] NusTxUuid =
            { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E };

        // The salt of f5.
        private static readonly byte[] F5Salt =
            { 0x6C, 0x88, 0x83, 0x91, 0xAA, 0xF5, 0xA5, 0x38, 0x60, 0x37, 0x0B, 0xDB, 0x5A, 0x60, 0x83, 0xBE };
    }
}
