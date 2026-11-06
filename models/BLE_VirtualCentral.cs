//
// BLE_VirtualCentral — a Renode-side BLE central ("the phone") that talks to
// the emulated G1 ONLY over the modeled radio PHY (NRF5340_RADIO). The G1's
// SoftDevice Controller, Bluetooth host and GATT server stay 100% real
// firmware; this peer just puts the frames a real central would put on the air.
//
// It rides the radio's IRadio-ish interface:
//   * radio.FrameSent            — receive the G1's TX (ADV_IND, and later the
//                                  peripheral's data-channel PDUs)
//   * radio.RxWindowOpened       — the G1 is opening an RX window (advertising
//                                  RX or a connection-event RX); present our
//                                  frame now so the radio buffers it and
//                                  delivers it when it reaches RX
//   * radio.ReceiveFrame(f,null) — inject our frame (null sender skips the
//                                  channel check, so we needn't track hopping)
//   * radio.OnAirAddressPublic/BuildFrameCrc — match the G1's current access
//                                  address and compute a CRC the RX will accept
//
// Sequence: see an ADV_IND -> inject a CONNECT_IND in the following RX window
// -> the SDC forms a connection and reports LE (Enhanced) Connection Complete
// UP to the host -> at each connection event the G1 (peripheral) RXes first, so
// we inject a master LL data PDU with correct stop-and-wait SN/NESN: first
// enable the NUS TX CCC (so the firmware treats the phone as subscribed), then
// ATT-Write a G1 protocol frame to the NUS RX value; empty PDUs otherwise keep
// the link alive.
//
// Disabled by default; the self-drive resc sets `vcentral Enabled true`.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

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

        public long Size => 0x1000;

        // Master toggle (default off so shared g1.resc / tests are unaffected).
        public bool Enabled { get; set; }

        // NUS RX value handle (write target) and TX CCC handle (notifications).
        // Discovered by sweep against the live G1 GATT DB.
        public uint NusRxValueHandle { get; set; } = 0x0014;
        public uint NusTxCccHandle { get; set; } = 0x0012;

        // Queue a G1 protocol frame to deliver by ATT Write to the NUS RX value
        // (after enabling the TX CCC). e.g. "0a06000000" = navigation-startup.
        public void QueueAttWrite(string hex)
        {
            payload = ParseHex(hex);
            this.Log(LogLevel.Info, "vcentral: queued ATT write ({0} bytes) to NUS RX handle 0x{1:X}", payload.Length, NusRxValueHandle);
        }

        // Handle-discovery mode (single incrementing write per event).
        public bool SweepHandles { get; set; }
        public bool SweepLoop { get; set; }
        public uint SweepStartHandle { get; set; } = 0x0001;
        public uint SweepMaxHandle { get; set; } = 0x0040;
        public int SweepDwell { get; set; } = 6;

        public int ConnectIndsSent => connectIndsSent;
        public int DataEventsAnswered => dataEventsAnswered;
        public bool Connected => connected;

        // ---- radio-facing callbacks ----

        private void OnFrameSent(IRadio sender, byte[] frame)
        {
            if(!Enabled || frame == null || frame.Length < 6)
            {
                return;
            }
            var aa = new byte[AddrLen];
            Array.Copy(frame, 0, aa, 0, AddrLen);
            var hdr = frame[AddrLen];
            var pduType = hdr & 0x0F;

            if(!connected && pduType == AdvIndType && AaEquals(aa, AdvAccessAddress))
            {
                if(frame.Length >= AddrLen + 2 + 6)
                {
                    advA = new byte[6];
                    Array.Copy(frame, AddrLen + 2, advA, 0, 6);
                    haveAdv = true;
                }
                return;
            }

            if(connected && AaEquals(aa, DataAccessAddress))
            {
                // Peripheral responded — LL stop-and-wait acking.
                var pSn = (hdr >> 3) & 1;
                var pNesn = (hdr >> 2) & 1;
                if(pNesn != txSeq)
                {
                    txSeq ^= 1;                     // our last TX was acked
                    OnMasterTxAcked();
                }
                rxSeq = (byte)(pSn ^ 1);           // expect the other SN next
            }
        }

        private void OnRxWindowOpened()
        {
            if(!Enabled)
            {
                return;
            }
            var rxAa = radio.OnAirAddressPublic(0);
            if(rxAa == null || rxAa.Length < AddrLen)
            {
                return;
            }

            if(!connected && haveAdv && AaEquals(rxAa, AdvAccessAddress))
            {
                InjectConnectInd();
                return;
            }

            if(AaEquals(rxAa, DataAccessAddress))
            {
                if(!connected)
                {
                    connected = true;
                    this.Log(LogLevel.Info, "vcentral: CONNECTED — G1 switched to data access address");
                }
                // Re-arm the ATT script periodically (host GATT readiness).
                if(!txOutstanding && rearmAt != 0 && dataEventsAnswered >= rearmAt)
                {
                    rearmAt = 0;
                    attIndex = 0;
                    txOutstanding = attScript.Count > 0;
                }
                InjectMasterDataPdu();
            }
        }

        // ---- link-layer / GATT state machine ----

        private void OnMasterTxAcked()
        {
            if(!txOutstanding)
            {
                return;
            }
            if(SweepHandles)
            {
                sweepDwellCount++;
                if(sweepDwellCount < SweepDwell)
                {
                    return; // keep writing the same handle for SweepDwell acks
                }
                sweepDwellCount = 0;
                this.Log(LogLevel.Noisy, "vcentral: swept handle 0x{0:X} done", sweepHandle);
                if(sweepHandle < SweepMaxHandle)
                {
                    sweepHandle++;   // try the next handle
                }
                else if(SweepLoop)
                {
                    sweepHandle = SweepStartHandle; // loop the bounded range
                }
                else
                {
                    txOutstanding = false;
                }
                return;
            }

            attIndex++;
            if(attIndex < attScript.Count)
            {
                return; // send the next scripted PDU
            }
            // Whole script sent; repeat a few times, spaced out, so it lands once
            // the host's GATT server is servicing writes.
            txOutstanding = false;
            if(scriptRepeatsLeft > 0)
            {
                scriptRepeatsLeft--;
                rearmAt = dataEventsAnswered + RepeatGapEvents;
                this.Log(LogLevel.Info, "vcentral: ATT script sent (repeats left {0})", scriptRepeatsLeft);
            }
        }

        private void InjectConnectInd()
        {
            var pdu = new List<byte>();
            pdu.AddRange(AdvAccessAddress);
            pdu.Add(0x85);                                  // CONNECT_IND, TxAdd=0(public InitA), RxAdd=1(random AdvA)
            pdu.Add(0x22);                                  // length = 34
            pdu.AddRange(InitA);
            pdu.AddRange(advA);
            pdu.AddRange(DataAccessAddress);
            pdu.AddRange(CrcInit);
            pdu.Add(WinSize);
            pdu.AddRange(WinOffset);
            pdu.AddRange(Interval);
            pdu.AddRange(Latency);
            pdu.AddRange(Timeout);
            pdu.AddRange(ChannelMap);
            pdu.Add(HopSca);
            AppendCrcAndSend(pdu);
            connectIndsSent++;
            txSeq = 0;
            rxSeq = 0;
            BuildAttScript();
            attIndex = 0;
            txOutstanding = (SweepHandles || attScript.Count > 0);
            sweepHandle = SweepStartHandle;
            this.Log(LogLevel.Info, "vcentral: sent CONNECT_IND (#{0}) to AdvA {1}", connectIndsSent, BitReverseStr(advA));
        }

        private void BuildAttScript()
        {
            attScript = new List<byte[]>();
            if(SweepHandles)
            {
                return; // sweep builds its PDU on the fly
            }
            if(payload != null)
            {
                // Enable NUS TX notifications, then write the frame to NUS RX.
                attScript.Add(AttWriteCmd(NusTxCccHandle, new byte[] { 0x01, 0x00 }));
                attScript.Add(AttWriteCmd(NusRxValueHandle, payload));
            }
        }

        private void InjectMasterDataPdu()
        {
            var pdu = new List<byte>();
            pdu.AddRange(DataAccessAddress);

            byte[] llPayload;
            byte llid;
            if(txOutstanding && SweepHandles)
            {
                llPayload = L2capWrap(AttWriteCmd(sweepHandle, payload ?? new byte[] { 0x00 }));
                llid = 0x02;
            }
            else if(txOutstanding && attIndex < attScript.Count)
            {
                llPayload = L2capWrap(attScript[attIndex]);
                llid = 0x02;
                if(!loggedSend)
                {
                    loggedSend = true;
                    this.Log(LogLevel.Info, "vcentral: sending ATT script over the connection (CCC 0x{0:X} + RX 0x{1:X})", NusTxCccHandle, NusRxValueHandle);
                }
            }
            else
            {
                llPayload = new byte[0];
                llid = 0x01;                              // empty keep-alive
            }

            pdu.Add((byte)(llid | (txSeq << 3) | (rxSeq << 2)));
            pdu.Add((byte)llPayload.Length);
            pdu.AddRange(llPayload);
            AppendCrcAndSend(pdu);
            dataEventsAnswered++;
        }

        // ---- helpers ----

        private static byte[] AttWriteCmd(uint handle, byte[] value)
        {
            var b = new byte[3 + value.Length];
            b[0] = 0x52;                                  // ATT Write Command
            b[1] = (byte)(handle & 0xFF);
            b[2] = (byte)((handle >> 8) & 0xFF);
            Array.Copy(value, 0, b, 3, value.Length);
            return b;
        }

        private static byte[] L2capWrap(byte[] att)
        {
            var b = new byte[4 + att.Length];
            b[0] = (byte)(att.Length & 0xFF);
            b[1] = (byte)((att.Length >> 8) & 0xFF);
            b[2] = 0x04;                                  // CID 0x0004 = ATT
            b[3] = 0x00;
            Array.Copy(att, 0, b, 4, att.Length);
            return b;
        }

        private void AppendCrcAndSend(List<byte> pduNoCrc)
        {
            var arr = pduNoCrc.ToArray();
            var crc = radio.BuildFrameCrc(arr);
            var full = new byte[arr.Length + crc.Length];
            Array.Copy(arr, full, arr.Length);
            Array.Copy(crc, 0, full, arr.Length, crc.Length);
            radio.ReceiveFrame(full, null);
        }

        private static bool AaEquals(byte[] a, byte[] b)
        {
            if(a.Length < AddrLen || b.Length < AddrLen) return false;
            for(var i = 0; i < AddrLen; i++)
            {
                if(a[i] != b[i]) return false;
            }
            return true;
        }

        private static string BitReverseStr(byte[] a)
        {
            var s = "";
            for(var i = a.Length - 1; i >= 0; i--)
            {
                s += a[i].ToString("X2");
                if(i > 0) s += ":";
            }
            return s;
        }

        private static byte[] ParseHex(string hex)
        {
            hex = hex.Replace(" ", "").Replace("_", "");
            var b = new byte[hex.Length / 2];
            for(var i = 0; i < b.Length; i++)
            {
                b[i] = Convert.ToByte(hex.Substring(2 * i, 2), 16);
            }
            return b;
        }

        private readonly NRF5340_RADIO radio;
        private byte[] advA;
        private bool haveAdv;
        private bool connected;
        private byte txSeq, rxSeq;
        private bool txOutstanding;
        private int connectIndsSent;
        private int dataEventsAnswered;

        private byte[] payload;
        private List<byte[]> attScript = new List<byte[]>();
        private int attIndex;
        private int scriptRepeatsLeft = 12;
        private int rearmAt;
        private bool loggedSend;
        private uint sweepHandle = 1;
        private int sweepDwellCount;

        private const int AddrLen = 4;
        private const int AdvIndType = 0x0;
        private const int RepeatGapEvents = 15;
        private static readonly byte[] AdvAccessAddress = { 0xD6, 0xBE, 0x89, 0x8E };

        // Central (InitA), public type — the connection peer the G1 records;
        // ancs_connected memcmp's it against the seed (00 11 22 33 44 55 66).
        private static readonly byte[] InitA = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

        // CONNECT_IND connection parameters.
        private static readonly byte[] DataAccessAddress = { 0x29, 0x41, 0x76, 0x71 }; // 0x71764129
        private static readonly byte[] CrcInit = { 0x55, 0x55, 0x55 };
        private const byte WinSize = 0x03;
        private static readonly byte[] WinOffset = { 0x00, 0x00 };
        private static readonly byte[] Interval = { 0x18, 0x00 }; // 30 ms
        private static readonly byte[] Latency = { 0x00, 0x00 };
        private static readonly byte[] Timeout = { 0xC8, 0x00 };  // 2 s
        private static readonly byte[] ChannelMap = { 0xFF, 0xFF, 0xFF, 0xFF, 0x1F };
        private const byte HopSca = 0x0D;
    }
}
