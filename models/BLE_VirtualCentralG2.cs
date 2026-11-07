//
// BLE_VirtualCentralG2: "the phone" as the Even app speaks to G2 glasses. The
// link layer, L2CAP, ATT and SMP are BLE_VirtualCentral's; this class puts the
// G2 app protocol on top of them:
//
//   GATT        the Even service, base UUID 00002760-08c2-11e1-9073-0e8ac72eXXXX.
//               As the app does after discovery: one byte 0x00 is written (with
//               response) to characteristic 5403; a GATT "insufficient
//               authentication/encryption" error to that write makes the phone
//               pair and write again. Then notifications are switched on, one
//               by one, for 5402 (protocol replies), 0002 (OTA), 6402
//               (microphone audio) and 7402 (file service). Protocol frames are
//               written to 5401, file-service frames (services 0xC4..0xC7) to
//               7401, as the app writes them
//   transport   frames "AA 21 seq len total num sid flag <data>", data being the
//               protobuf payload with its CRC-16/CCITT-FALSE (little-endian)
//               appended and cut into fragments; frames from the lens start
//               "AA 12". Fragments are put together again and the CRC checked
//   session     the Even app's bring-up as captured from it, in order: auth
//               (service 0x80, type 4, retried) to both lenses, then to the
//               right lens only: pipe role, time sync, Even AI config, QuickList,
//               app sync, settings read, units, onboarding finished, wear status,
//               watchface info, case info, notification control, gestures (the
//               table in BringUpSteps); after that a heartbeat (0x80 type 14) to
//               both lenses whenever HeartbeatSeconds pass without a write. Each
//               request carries a "magic" (protobuf field 2) that the lens echoes
//               in its reply; the reply is matched by it (the lens's sequence
//               byte is its own). A reply has flag 0x00, a push from the lens
//               flag 0x01, and a header-only frame (length 0) is a control frame
//               carrying a status in its flag byte
//
// Pairing is driven by the write to 5403 (above), as in the app; a lens without
// 5403 is paired as the G1 phone does if PairOnConnect is set. A Security
// Request from the lens starts pairing too.
//
// Every frame is logged at Info level, decoded: "tx"/"rx", service, flag,
// sequence, fragment, CRC result, the protobuf bytes, and which request a
// reply answers. `SendG2 sid flag "pb hex"` writes any request; `SendPacket
// "hex"` writes raw bytes to 5401 as they are.
//
// Disabled by default: `vcentral_g2 Enabled true` switches the phone on.
//
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Wireless
{
    public class BLE_VirtualCentralG2 : BLE_VirtualCentral
    {
        public BLE_VirtualCentralG2(IMachine machine, NRF5340_RADIO radio) : base(machine, radio)
        {
            Mtu = 247;                  // what the Even app asks for
            HeartbeatSeconds = 120;     // the app's keep-alive period
        }

        // ------------------------------ settings ------------------------------

        // Which lens this phone talks to: the app sends everything to the
        // "right" lens and only auth and heartbeats to the "left" one (the G1
        // firmware names the master lens _R_ and the slave _L_).
        public string Side
        {
            get
            {
                return side;
            }
            set
            {
                var text = (value ?? "").Trim().ToLowerInvariant();
                if(text != "right" && text != "left")
                {
                    throw new RecoverableException("Side is \"right\" or \"left\"");
                }
                side = text;
            }
        }

        // After a successful auth on the right lens, send what the app sends
        // next (BringUpSteps). Auth itself always runs.
        public bool BringUp { get; set; } = true;

        // Switch notifications on for the OTA (0002), audio (6402) and file
        // (7402) characteristics as well as the protocol one (5402), as the
        // app does.
        public bool OtaNotifications { get; set; } = true;
        public bool AudioNotifications { get; set; } = true;
        public bool FileNotifications { get; set; } = true;

        // The most data bytes in one fragment (the length field is a byte).
        // 232 is what the app's frames have been seen to carry; a fragment is
        // also kept within one ATT write (MTU - 11).
        public uint FragmentSize
        {
            get
            {
                return fragmentSize;
            }
            set
            {
                if(value < 1 || value > 255)
                {
                    throw new RecoverableException("a fragment carries 1 to 255 bytes");
                }
                fragmentSize = value;
            }
        }

        // Header byte 7 of the bring-up requests: 0x20 is what the app sends on
        // every request except auth and the heartbeat, which carry 0x00.
        public uint RequestFlag { get; set; } = 0x20;

        // How long a request waits for the reply that echoes its magic (the
        // app: 1 s), and how often an unanswered request is sent again (the
        // app: once; auth twice, AuthRetries counts attempts).
        public double ReplyTimeoutSeconds { get; set; } = 1;
        public uint RequestRetries { get; set; } = 1;
        public uint AuthRetries { get; set; } = 3;

        // What time sync tells the lens: seconds since 1970 (0: the host's
        // clock when the request is sent) and the time zone in quarter hours.
        public long UnixTime { get; set; }
        public int TimeZoneQuarterHours { get; set; }

        // ------------------------------- state --------------------------------

        public bool Authenticated => authenticated && Connected;
        public bool BringUpComplete => bringUpComplete && Connected;
        public int FramesSent => framesSent;
        public int FramesReceived => framesReceived;
        public int CrcErrors => crcErrors;
        public int AudioNotificationsReceived => audioCount;
        public uint OtaNotifyHandle => otaHandle;
        public uint AudioNotifyHandle => audioHandle;
        public uint FileNotifyHandle => fileHandle;
        public uint FileWriteHandle => fileWriteHandle;
        public string LensFirmware => lensFirmware ?? "";
        public int LensBattery => lensBattery;

        // ------------------------------ commands ------------------------------

        // Write a request: service id, header flag byte and the protobuf
        // payload as hex (may be empty). The phone adds the header, sequence,
        // fragmentation and CRC. If the payload has a field 2 (magic), the
        // reply that echoes it is reported as the answer.
        //   vcentral_g2 SendG2 0x80 0x20 "08 0e 10 65 6a 00"
        public void SendG2(uint sid, uint flag, string pbHex)
        {
            if(sid > 0xFF || flag > 0xFF)
            {
                throw new RecoverableException("the service id and the flag are one byte each");
            }
            var pb = string.IsNullOrWhiteSpace(pbHex) ? new byte[0] : ParseHex(pbHex);
            lock(sync)
            {
                if(!Enabled)
                {
                    throw new RecoverableException("the phone is switched off");
                }
                if(requests.Count >= MaxPendingPackets)
                {
                    throw new RecoverableException("too many requests are waiting for the link");
                }
                ulong magic;
                var hasMagic = TryReadVarint(pb, 2, out magic);
                Enqueue(new Request
                {
                    Sid = (byte)sid, Flag = (byte)flag, Pb = pb, Label = "SendG2",
                    HasMagic = hasMagic, Magic = (uint)magic, RetriesLeft = RequestRetries,
                    OnReply = hasMagic ? (Action<byte[]>)(reply => { }) : null,
                });
            }
        }

        // Run the auth handshake (and the bring-up after it) again now.
        public void Authenticate()
        {
            lock(sync)
            {
                if(!nusReady || !Connected)
                {
                    throw new RecoverableException("the phone is not connected with notifications on");
                }
                authAttempts = 0;
                SendAuth();
            }
        }

        // ------------------------------- hooks --------------------------------

        protected override string ServiceName => "Even G2 service";
        protected override byte[] WriteUuid => EvenUuid(0x5401);
        protected override byte[] NotifyUuid => EvenUuid(0x5402);

        protected override void OnLinkStart()
        {
            ClearSession();
        }

        protected override void OnLinkEnd()
        {
            ClearSession();
        }

        private void ClearSession()
        {
            authenticated = false;
            authAttempts = 0;
            bringUpStep = 0;
            bringUpComplete = false;
            requests.Clear();
            pending.Clear();
            reassembly.Clear();
            audioHandle = 0;
            otaHandle = 0;
            fileHandle = 0;
            fileWriteHandle = 0;
            audioCount = 0;
            checkHandle = 0;
            checkPending = false;
            checkRetried = false;
            setupContinuation = null;
        }

        // The app's first act after discovery: one byte 0x00 written with
        // response to 5403. The lens refuses it until the link is encrypted,
        // which is what makes the phone pair.
        protected override void OnServiceFound(Action continueSetup)
        {
            setupContinuation = continueSetup;
            var check = FindCharacteristic(EvenUuid(0x5403));
            if(check == null)
            {
                Say("the lens has no characteristic 5403: the app's check write is skipped");
                if(PairOnConnect && bondKey == null && !encrypted && !SecurityBusy)
                {
                    StartPairing();
                }
                continueSetup();
                return;
            }
            checkHandle = (uint)check.Value;
            WriteCheck();
        }

        private void WriteCheck()
        {
            AttRequest(new byte[] { 0x12, (byte)checkHandle, (byte)(checkHandle >> 8), 0x00 }, response =>
            {
                var resume = setupContinuation;
                if(response[0] == 0x13)
                {
                    Say("check write of 00 to 5403: accepted");
                    setupContinuation = null;
                    if(resume != null)
                    {
                        resume();
                    }
                    return;
                }
                var error = response.Length >= 5 ? response[4] : (byte)0;
                var security = error == 0x05 || error == 0x08 || error == 0x0C || error == 0x0F;
                if(security && !checkRetried)
                {
                    Say("check write to 5403 refused (ATT error 0x{0:X2}): pairing, as the app does", error);
                    checkPending = true;
                    checkRetried = true;
                    if(encrypted)
                    {
                        OnLinkSecured();            // already encrypted: write again at once
                    }
                    else if(!SecurityBusy)
                    {
                        StartPairing();
                    }
                    return;
                }
                Say("check write to 5403 failed ({0}); going on without it", Hex(response));
                setupContinuation = null;
                if(resume != null)
                {
                    resume();
                }
            });
        }

        protected override void OnLinkSecured()
        {
            if(checkPending)
            {
                checkPending = false;
                WriteCheck();
            }
        }

        protected override void OnTransportReady()
        {
            // Not base.OnTransportReady(): pairing is driven by the 5403 write.
            var fileWrite = FindCharacteristic(EvenUuid(0x7401));
            fileWriteHandle = fileWrite == null ? 0 : (uint)fileWrite.Value;
            extraNotifications.Clear();
            if(OtaNotifications)
            {
                extraNotifications.Enqueue(0x0002);
            }
            if(AudioNotifications)
            {
                extraNotifications.Enqueue(0x6402);
            }
            if(FileNotifications)
            {
                extraNotifications.Enqueue(0x7402);
            }
            EnableNextNotification();
        }

        private void EnableNextNotification()
        {
            if(extraNotifications.Count == 0)
            {
                authAttempts = 0;
                SendAuth();
                return;
            }
            var shortUuid = extraNotifications.Dequeue();
            var characteristic = FindCharacteristic(EvenUuid(shortUuid));
            if(characteristic == null)
            {
                Say("the lens has no characteristic {0:X4}; its notifications stay off", shortUuid);
                EnableNextNotification();
                return;
            }
            var valueHandle = (uint)characteristic.Value;
            FindCcc(valueHandle, string.Format("characteristic {0:X4}", shortUuid), ccc =>
            {
                if(ccc == 0)
                {
                    EnableNextNotification();
                    return;
                }
                WriteCcc(ccc, ok =>
                {
                    if(ok)
                    {
                        if(shortUuid == 0x6402)
                        {
                            audioHandle = valueHandle;
                        }
                        else if(shortUuid == 0x0002)
                        {
                            otaHandle = valueHandle;
                        }
                        else
                        {
                            fileHandle = valueHandle;
                        }
                        Say("characteristic {0:X4}: value 0x{1:X4}, notifications on", shortUuid, valueHandle);
                    }
                    EnableNextNotification();
                });
            });
        }

        protected override void OnNotification(uint handle, byte[] value)
        {
            if(handle == nusTxHandle && nusTxHandle != 0)
            {
                OnPacket(value);
            }
            else if(handle == audioHandle && audioHandle != 0)
            {
                audioCount++;
                if(audioCount == 1 || audioCount % 100 == 0)
                {
                    Say("audio notification {0}: {1} bytes", audioCount, value.Length);
                }
                else
                {
                    this.Log(LogLevel.Debug, "audio notification {0}: {1}", audioCount, Hex(value));
                }
            }
            else if(handle == fileHandle && fileHandle != 0)
            {
                OnPacket(value);                    // the file service frames the same way
            }
            else if(handle == otaHandle && otaHandle != 0)
            {
                Say("rx OTA notification: {0}", Hex(value));
            }
            else
            {
                Say("rx notification of handle 0x{0:X4}: {1}", handle, Hex(value));
            }
        }

        protected override void OnTick(double now)
        {
            while(requests.Count > 0)
            {
                WriteRequest(requests.Dequeue(), now);
            }
            for(var i = pending.Count - 1; i >= 0; i--)
            {
                var waiting = pending[i];
                if(now < waiting.DeadlineMs)
                {
                    continue;
                }
                pending.RemoveAt(i);
                if(waiting.Request != null && waiting.Request.RetriesLeft > 0)
                {
                    // Sent again with a new magic, as the app does.
                    waiting.Request.RetriesLeft--;
                    waiting.Request.Magic = NextMagic();
                    waiting.Request.Pb = waiting.Request.Rebuild != null
                        ? waiting.Request.Rebuild(waiting.Request.Magic) : waiting.Request.Pb;
                    Say("no answer to {0} (service 0x{1:X2}, magic {2}) within {3}s: sent again",
                        waiting.Label, waiting.Sid, waiting.Magic, ReplyTimeoutSeconds);
                    WriteRequest(waiting.Request, now);
                    continue;
                }
                Say("no answer to {0} (service 0x{1:X2}, magic {2}) within {3}s",
                    waiting.Label, waiting.Sid, waiting.Magic, ReplyTimeoutSeconds);
                if(waiting.OnReply != null)
                {
                    waiting.OnReply(null);
                }
            }
        }

        protected override void SendHeartbeat()
        {
            // {1: 14, 2: magic, 13: {}}, flag 0x00 as the app sends it (every 60 s).
            var magic = NextMagic();
            var pb = Concat(Varint(1, 14), Varint(2, magic), Message(13, new byte[0]));
            WriteRequest(new Request
            {
                Sid = 0x80, Flag = 0x00, Pb = pb, Label = "heartbeat",
                HasMagic = true, Magic = magic, Quiet = true,
                OnReply = reply =>
                {
                    if(reply != null)
                    {
                        this.Log(LogLevel.Debug, "heartbeat answered");
                    }
                },
            }, NowMs);
        }

        protected override string ProtocolSummary()
        {
            if(authenticated)
            {
                return ", authenticated";
            }
            return authAttempts > 0 ? ", authenticating" : ", not authenticated";
        }

        // ------------------------------ session -------------------------------

        private void SendAuth()
        {
            authenticated = false;
            authAttempts++;
            // {1: 4, 2: magic, 3: {1: 1, 2: 4}}: as the app sends it, flag 0x00.
            var magic = NextMagic();
            var pb = Concat(Varint(1, 4), Varint(2, magic), Message(3, Concat(Varint(1, 1), Varint(2, 4))));
            Enqueue(new Request
            {
                Sid = 0x80, Flag = 0x00, Pb = pb, Label = "auth", HasMagic = true, Magic = magic,
                OnReply = reply =>
                {
                    if(reply == null)
                    {
                        if(authAttempts < AuthRetries)
                        {
                            SendAuth();
                        }
                        else
                        {
                            Say("auth: no answer after {0} attempts; the lens is not authenticated "
                                + "(`Authenticate` tries again)", authAttempts);
                        }
                        return;
                    }
                    byte[] result;
                    if(TryReadBytes(reply, 3, out result) && result.Length == 0)
                    {
                        Say("auth: OK");
                    }
                    else if(reply.Length == 0)
                    {
                        Say("auth: acknowledged by a control frame, no echo (the G2 glasses echo 08 04 10 mm 1A 00)");
                    }
                    else
                    {
                        Say("auth: answered, but not with the expected empty field 3 (1A 00): {0}", Hex(reply));
                    }
                    authenticated = true;
                    bringUpStep = 0;
                    if(!BringUp)
                    {
                        return;
                    }
                    if(side != "right")
                    {
                        Say("left lens: auth and heartbeats only, as the app does");
                        return;
                    }
                    NextBringUpStep();
                },
            });
        }

        // After auth on the right lens: what the app was captured sending, in
        // this order, each with flag RequestFlag. The lens answers each with the
        // same type and magic; the answer, or its absence after
        // ReplyTimeoutSeconds, moves on to the next step.
        private struct Step
        {
            public byte Sid;
            public string Label;
            public Func<uint, byte[]> Pb;
            public Action<byte[]> OnReply;
        }

        private Step[] BringUpSteps()
        {
            return new[]
            {
                // {1: 5, 2: m, 4: {1: 1}}: this lens takes the command role
                new Step { Sid = 0x80, Label = "pipe role", Pb = m => Concat(Varint(1, 5), Varint(2, m), Message(4, Varint(1, 1))) },
                // {1: 128, 2: m, 128: {1: utc seconds, 2: time zone in quarter hours}}
                new Step { Sid = 0x80, Label = "time sync", Pb = m =>
                    Concat(Varint(1, 128), Varint(2, m), Message(128, Concat(
                        Varint(1, UnixTime != 0 ? UnixTime : DateTimeOffset.UtcNow.ToUnixTimeSeconds()),
                        Varint(2, TimeZoneQuarterHours)))) },
                // {1: 10, 2: m, 13: {1: voice switch 0, 2: stream speed 80, 4: 0}}
                new Step { Sid = 0x07, Label = "Even AI config", Pb = m =>
                    Concat(Varint(1, 10), Varint(2, m), Message(13, Concat(Varint(1, 0), Varint(2, 80), Varint(4, 0)))) },
                // {1: 2, 2: m, 4: {1: 1, 2: 0}}: an empty QuickList
                new Step { Sid = 0x0C, Label = "QuickList full update", Pb = m =>
                    Concat(Varint(1, 2), Varint(2, m), Message(4, Concat(Varint(1, 1), Varint(2, 0)))) },
                // {1: 0, 2: m}: which app is in the foreground (the reply is not known)
                new Step { Sid = 0x0D, Label = "app sync", Pb = m => Concat(Varint(1, 0), Varint(2, m)) },
                // {1: 2, 2: m, 4: {1: 1}}: read the basic settings
                new Step { Sid = 0x09, Label = "settings read", Pb = m =>
                    Concat(Varint(1, 2), Varint(2, m), Message(4, Varint(1, 1))), OnReply = ReadSettings },
                // {1: 1, 2: m, 3: {9: {1: 0, 2: 0, 3: 1, 4: 0, 5: 1}}}: units and formats
                new Step { Sid = 0x09, Label = "units", Pb = m =>
                    Concat(Varint(1, 1), Varint(2, m), Message(3, Message(9, Concat(
                        Varint(1, 0), Varint(2, 0), Varint(3, 1), Varint(4, 0), Varint(5, 1))))) },
                // {1: 1, 2: m, 3: {1: 4}}: onboarding process 4 finished
                new Step { Sid = 0x10, Label = "onboarding finished", Pb = m =>
                    Concat(Varint(1, 1), Varint(2, m), Message(3, Varint(1, 4))) },
                // {1: 3, 2: m, 5: {1: 1}}: event 1, the wear status (the reply says worn or not)
                new Step { Sid = 0x10, Label = "wear status event", Pb = m =>
                    Concat(Varint(1, 3), Varint(2, m), Message(5, Varint(1, 1))) },
                // {1: 0, 2: m, 3: {1: 1}}
                new Step { Sid = 0x1F, Label = "watchface info", Pb = m =>
                    Concat(Varint(1, 0), Varint(2, m), Message(3, Varint(1, 1))) },
                // {1: 1, 2: m, 3: {}} (the reply carries the case battery)
                new Step { Sid = 0x81, Label = "case info", Pb = m =>
                    Concat(Varint(1, 1), Varint(2, m), Message(3, new byte[0])) },
                // {1: 1, 2: m, 3: {1: notifications on, 2: auto display off, 3: display 5 s, 5: do not disturb off}}
                new Step { Sid = 0x04, Label = "notification control", Pb = m =>
                    Concat(Varint(1, 1), Varint(2, m), Message(3, Concat(
                        Varint(1, 1), Varint(2, 0), Varint(3, 5), Varint(5, 0)))) },
                // {1: 1, 2: m, 3: {10: {1: [{1: 0, 2: op, 3: 0}] for op 0, 1, 2}}}: the gesture list
                new Step { Sid = 0x09, Label = "gestures", Pb = m =>
                    Concat(Varint(1, 1), Varint(2, m), Message(3, Message(10, Concat(
                        Message(1, Concat(Varint(1, 0), Varint(2, 0), Varint(3, 0))),
                        Message(1, Concat(Varint(1, 0), Varint(2, 1), Varint(3, 0))),
                        Message(1, Concat(Varint(1, 0), Varint(2, 2), Varint(3, 0))))))) },
            };
        }

        private void NextBringUpStep()
        {
            var steps = BringUpSteps();
            if(bringUpStep >= steps.Length)
            {
                bringUpComplete = true;
                Say("bring-up complete");
                return;
            }
            var step = steps[bringUpStep++];
            var magic = NextMagic();
            Enqueue(new Request
            {
                Sid = step.Sid, Flag = (byte)RequestFlag, Pb = step.Pb(magic), Label = step.Label,
                HasMagic = true, Magic = magic, RetriesLeft = RequestRetries, Rebuild = step.Pb,
                OnReply = reply =>
                {
                    if(reply != null && step.OnReply != null)
                    {
                        step.OnReply(reply);
                    }
                    NextBringUpStep();
                },
            });
        }

        // The settings reply: field 4 carries the lens's state, among it the
        // firmware versions (5, 6) and the battery (12), charging (13).
        private void ReadSettings(byte[] reply)
        {
            byte[] state;
            if(!TryReadBytes(reply, 4, out state))
            {
                return;
            }
            var words = new List<string>();
            byte[] text;
            ulong number;
            if(TryReadBytes(state, 5, out text) && text.Length > 0)
            {
                lensFirmware = Encoding.UTF8.GetString(text);
                words.Add("firmware " + lensFirmware);
            }
            if(TryReadBytes(state, 6, out text) && text.Length > 0)
            {
                words.Add("other lens firmware " + Encoding.UTF8.GetString(text));
            }
            if(TryReadVarint(state, 12, out number))
            {
                lensBattery = (int)number;
                words.Add("battery " + number + "%");
            }
            if(TryReadVarint(state, 13, out number))
            {
                words.Add(number != 0 ? "charging" : "not charging");
            }
            if(words.Count > 0)
            {
                Say("settings: {0}", string.Join(", ", words));
            }
        }

        private void SendRequest(byte sid, string label, uint magic, byte[] pb, Action<byte[]> onReply)
        {
            Enqueue(new Request
            {
                Sid = sid, Flag = (byte)RequestFlag, Pb = pb, Label = label,
                HasMagic = true, Magic = magic, OnReply = onReply,
            });
        }

        private void Enqueue(Request request)
        {
            requests.Enqueue(request);
        }

        // ----------------------------- transport ------------------------------

        private void WriteRequest(Request request, double now)
        {
            // The app's sequence byte is the request's magic.
            var seq = request.HasMagic ? (byte)request.Magic : NextSeq();
            var crc = Crc16(request.Pb);
            var body = Concat(request.Pb, new[] { (byte)crc, (byte)(crc >> 8) });
            var chunk = Math.Max(1, Math.Min((int)fragmentSize, attMtu - 3 - HeaderLen));
            var total = Math.Max(1, (body.Length + chunk - 1) / chunk);
            if(total > 255)
            {
                Say("{0}: a payload of {1} bytes needs {2} fragments, more than 255; dropped",
                    request.Label, request.Pb.Length, total);
                return;
            }
            // The file service has its own characteristic, 7401.
            var isFile = request.Sid >= FileServiceFirst && request.Sid <= FileServiceLast && fileWriteHandle != 0;
            if(isFile && fileHandle == 0 && !fileHintGiven)
            {
                fileHintGiven = true;
                Say("file service: notifications of 7402 are off (FileNotifications), its replies will not be seen");
            }
            for(var i = 0; i < total; i++)
            {
                var part = Slice(body, i * chunk, Math.Min(chunk, body.Length - i * chunk));
                var frame = Concat(new[] { FrameMagic, DirectionToLens, seq, (byte)part.Length, (byte)total, (byte)(i + 1),
                                           request.Sid, request.Flag }, part);
                WritePacket(frame, isFile ? fileWriteHandle : nusRxHandle);
                framesSent++;
                if(!request.Quiet)
                {
                    Say("tx service 0x{0:X2} flag 0x{1:X2} seq {2}{3} {4}{5}: {6}",
                        request.Sid, request.Flag, seq,
                        total > 1 ? string.Format(" fragment {0}/{1}", i + 1, total) : "",
                        request.Label, isFile ? " (to 7401)" : "", i == 0 ? Hex(request.Pb) : Hex(part));
                }
                else
                {
                    this.Log(LogLevel.Debug, "tx service 0x{0:X2} flag 0x{1:X2} seq {2} {3}: {4}",
                             request.Sid, request.Flag, seq, request.Label, Hex(request.Pb));
                }
            }
            if(request.OnReply != null && request.HasMagic)
            {
                pending.Add(new Pending
                {
                    Sid = request.Sid, Magic = request.Magic, Label = request.Label,
                    DeadlineMs = now + ReplyTimeoutSeconds * 1000, OnReply = request.OnReply,
                    Quiet = request.Quiet, Request = request,
                });
            }
        }

        protected override void OnPacket(byte[] packet)
        {
            packetsReceived++;
            if(packet.Length < HeaderLen || packet[0] != FrameMagic)
            {
                Say("rx (not a G2 frame): {0}", Hex(packet));
                return;
            }
            var direction = packet[1];
            var seq = packet[2];
            int length = packet[3];
            int total = packet[4];
            int number = packet[5];
            var sid = packet[6];
            var flag = packet[7];
            if(length == 0 && packet.Length == HeaderLen)
            {
                // A header-only control frame: the status is in the flag byte,
                // no CRC, no magic. It counts as the answer to the oldest request
                // waiting on that service (the real glasses echo the request
                // instead; a firmware that only acknowledges shows up here).
                framesReceived++;
                Pending acked = null;
                foreach(var waiting in pending)
                {
                    if(waiting.Sid == sid)
                    {
                        acked = waiting;
                        break;
                    }
                }
                if(acked != null)
                {
                    pending.Remove(acked);
                }
                Say("rx control frame service 0x{0:X2} seq {1} status {2} (flag 0x{3:X2}){4}",
                    sid, seq, (flag >> 1) & 0x0F, flag,
                    acked != null ? " acknowledges " + acked.Label + " (no echo)" : "");
                if(acked != null && acked.OnReply != null)
                {
                    acked.OnReply(new byte[0]);
                }
                return;
            }
            var data = Slice(packet, HeaderLen, Math.Min(length, packet.Length - HeaderLen));
            if(length > packet.Length - HeaderLen)
            {
                Say("rx frame says {0} data bytes, carries {1}", length, packet.Length - HeaderLen);
            }
            byte[] body;
            if(total <= 1)
            {
                body = data;
            }
            else
            {
                var key = sid << 8 | seq;
                List<byte> parts;
                Fragments fragments;
                if(number == 1)
                {
                    parts = new List<byte>(data);
                    reassembly[key] = new Fragments { Parts = parts, Next = 2 };
                }
                else if(reassembly.TryGetValue(key, out fragments) && fragments.Next == number)
                {
                    fragments.Parts.AddRange(data);
                    fragments.Next++;
                    parts = fragments.Parts;
                }
                else
                {
                    reassembly.Remove(key);
                    Say("rx fragment {0}/{1} of service 0x{2:X2} seq {3} out of order; message dropped",
                        number, total, sid, seq);
                    return;
                }
                this.Log(LogLevel.Debug, "rx fragment {0}/{1} of service 0x{2:X2} seq {3}", number, total, sid, seq);
                if(number < total)
                {
                    return;
                }
                reassembly.Remove(key);
                body = parts.ToArray();
            }
            if(body.Length < 2)
            {
                Say("rx service 0x{0:X2} seq {1}: frame too short for a CRC: {2}", sid, seq, Hex(packet));
                return;
            }
            var pb = Slice(body, 0, body.Length - 2);
            var crcOk = Crc16(pb) == (body[body.Length - 2] | body[body.Length - 1] << 8);
            if(!crcOk)
            {
                crcErrors++;
            }
            framesReceived++;

            // Which request does it answer?
            string answers = "";
            Pending answered = null;
            ulong magic;
            // A file-service reply carries no magic: it answers the oldest
            // request waiting on its service.
            var byService = sid >= FileServiceFirst && sid <= FileServiceLast;
            if(TryReadVarint(pb, 2, out magic) || byService)
            {
                foreach(var waiting in pending)
                {
                    if(waiting.Sid == sid && (byService || waiting.Magic == magic))
                    {
                        answered = waiting;
                        break;
                    }
                }
            }
            if(answered != null)
            {
                pending.Remove(answered);
                answers = " answers " + answered.Label;
            }
            var status = (flag >> 1) & 0x0F;
            var line = string.Format(CultureInfo.InvariantCulture,
                "rx service 0x{0:X2} flag 0x{1:X2} seq {2}{3} {4}{5}{6}{7}: {8}",
                sid, flag, seq, total > 1 ? string.Format(" ({0} fragments)", total) : "",
                (flag & 1) != 0 ? "push" : "reply", status != 0 ? string.Format(" status {0}", status) : "",
                crcOk ? "" : " CRC BAD", direction == DirectionFromLens ? "" : string.Format(" direction 0x{0:X2}", direction),
                pb.Length > 0 ? Hex(pb) : "(empty)");
            if(answered != null && answered.Quiet)
            {
                this.Log(LogLevel.Debug, "{0}{1}", line, answers);
            }
            else
            {
                Say("{0}{1}", line, answers);
            }
            if(answered != null && answered.OnReply != null)
            {
                answered.OnReply(pb);
            }
        }

        private byte NextSeq()
        {
            seqCounter = (byte)(seqCounter == 255 ? 1 : seqCounter + 1);
            return seqCounter;
        }

        private uint NextMagic()
        {
            magicCounter++;
            if(magicCounter > 0x7F)
            {
                magicCounter = 1;       // keeps the magic a one-byte varint
            }
            return magicCounter;
        }

        // CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF.
        private static ushort Crc16(byte[] data)
        {
            var crc = 0xFFFF;
            foreach(var b in data)
            {
                crc ^= b << 8;
                for(var i = 0; i < 8; i++)
                {
                    crc = (crc & 0x8000) != 0 ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
                }
            }
            return (ushort)crc;
        }

        // ------------------------------ protobuf ------------------------------

        private static void PutVarint(List<byte> output, ulong value)
        {
            while(value >= 0x80)
            {
                output.Add((byte)(value & 0x7F | 0x80));
                value >>= 7;
            }
            output.Add((byte)value);
        }

        // A varint field; a negative value is sent as protobuf sends an int32,
        // in ten bytes.
        private static byte[] Varint(int field, long value)
        {
            var output = new List<byte>();
            PutVarint(output, (ulong)(field << 3));
            PutVarint(output, (ulong)value);
            return output.ToArray();
        }

        private static byte[] Message(int field, byte[] inner)
        {
            var output = new List<byte>();
            PutVarint(output, (ulong)(field << 3 | 2));
            PutVarint(output, (ulong)inner.Length);
            output.AddRange(inner);
            return output.ToArray();
        }

        private static bool ReadVarint(byte[] pb, ref int at, out ulong value)
        {
            value = 0;
            var shift = 0;
            while(at < pb.Length && shift < 64)
            {
                var b = pb[at++];
                value |= (ulong)(b & 0x7F) << shift;
                if((b & 0x80) == 0)
                {
                    return true;
                }
                shift += 7;
            }
            return false;
        }

        // The top-level fields of a message, in order.
        private static List<Field> Fields(byte[] pb)
        {
            var fields = new List<Field>();
            var at = 0;
            while(at < pb.Length)
            {
                ulong key;
                if(!ReadVarint(pb, ref at, out key))
                {
                    break;
                }
                var field = new Field { Number = (int)(key >> 3), Wire = (int)(key & 7) };
                switch(field.Wire)
                {
                case 0:
                    if(!ReadVarint(pb, ref at, out field.Varint))
                    {
                        return fields;
                    }
                    break;
                case 1:
                    at += 8;
                    break;
                case 2:
                {
                    ulong length;
                    if(!ReadVarint(pb, ref at, out length) || at + (int)length > pb.Length)
                    {
                        return fields;
                    }
                    field.Bytes = Slice(pb, at, (int)length);
                    at += (int)length;
                    break;
                }
                case 5:
                    at += 4;
                    break;
                default:
                    return fields;
                }
                fields.Add(field);
            }
            return fields;
        }

        private static bool TryReadVarint(byte[] pb, int number, out ulong value)
        {
            foreach(var field in Fields(pb))
            {
                if(field.Number == number && field.Wire == 0)
                {
                    value = field.Varint;
                    return true;
                }
            }
            value = 0;
            return false;
        }

        private static bool TryReadBytes(byte[] pb, int number, out byte[] value)
        {
            foreach(var field in Fields(pb))
            {
                if(field.Number == number && field.Wire == 2)
                {
                    value = field.Bytes;
                    return true;
                }
            }
            value = null;
            return false;
        }

        // ------------------------------- helpers ------------------------------

        // 00002760-08c2-11e1-9073-0e8ac72eXXXX as it goes on the air (least
        // significant byte first).
        private static byte[] EvenUuid(ushort shortUuid)
        {
            var uuid = (byte[])EvenBaseUuid.Clone();
            uuid[0] = (byte)shortUuid;
            uuid[1] = (byte)(shortUuid >> 8);
            return uuid;
        }

        private class Request
        {
            public byte Sid;
            public byte Flag;
            public byte[] Pb;
            public string Label;
            public bool HasMagic;
            public uint Magic;
            public bool Quiet;              // heartbeats: Debug level only
            public uint RetriesLeft;        // sent again that often without a reply
            public Func<uint, byte[]> Rebuild;  // the payload for a new magic, if it holds one
            public Action<byte[]> OnReply;  // null when no reply came
        }

        private class Pending
        {
            public byte Sid;
            public uint Magic;
            public string Label;
            public double DeadlineMs;
            public bool Quiet;
            public Request Request;
            public Action<byte[]> OnReply;
        }

        private class Fragments
        {
            public List<byte> Parts;
            public int Next;
        }

        private class Field
        {
            public int Number;
            public int Wire;
            public ulong Varint;
            public byte[] Bytes;
        }

        private uint fragmentSize = 232;
        private string side = "right";
        private bool authenticated;
        private bool bringUpComplete;
        private uint authAttempts;
        private int bringUpStep;
        private byte seqCounter;
        private uint magicCounter = 100;    // the first magic is 101, as in the app's traces
        private int framesSent;
        private int framesReceived;
        private int crcErrors;
        private int audioCount;
        private uint audioHandle;
        private uint otaHandle;
        private uint fileHandle;
        private uint fileWriteHandle;
        private bool fileHintGiven;
        private uint checkHandle;
        private bool checkPending;
        private bool checkRetried;
        private Action setupContinuation;
        private string lensFirmware;
        private int lensBattery = -1;
        private readonly Queue<Request> requests = new Queue<Request>();
        private readonly List<Pending> pending = new List<Pending>();
        private readonly Dictionary<int, Fragments> reassembly = new Dictionary<int, Fragments>();
        private readonly Queue<ushort> extraNotifications = new Queue<ushort>();

        private const int HeaderLen = 8;
        private const byte FileServiceFirst = 0xC4;
        private const byte FileServiceLast = 0xC7;
        private const byte FrameMagic = 0xAA;
        private const byte DirectionToLens = 0x21;
        private const byte DirectionFromLens = 0x12;

        private static readonly byte[] EvenBaseUuid =
            { 0x00, 0x00, 0x2E, 0xC7, 0x8A, 0x0E, 0x73, 0x90, 0xE1, 0x11, 0xC2, 0x08, 0x60, 0x27, 0x00, 0x00 };
    }
}
