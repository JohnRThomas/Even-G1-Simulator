//
// NRF5340_IPC — nRF5340 interprocessor communication peripheral.
// Application instance at 0x5002A000/0x4002A000, network at 0x41012000.
//
// Register offsets verbatim from the pinned nRF5340 MDK/SVD
// (hal_nordic 9784731461018d3e983604698fbbed6af2bea801).
//
// The two instances are paired: instantiate the first without a peer and the
// second with `peer:` pointing at the first; the constructor links both ways.
// TASKS_SEND[i] signals every IPC channel selected in SEND_CNF[i]; each
// instance (sender included, as in hardware) raises EVENTS_RECEIVE[j] for
// every j whose RECEIVE_CNF[j] listens on a signalled channel.
//
// Thread safety: the paired instances share one lock (syncRoot). Each
// instance's registers are accessed from its own core's thread, but
// TASKS_SEND additionally mutates the PEER's receiveEvents and toggles the
// peer's IRQ from the sender core's thread. Without serialization the two
// threads' UpdateInterrupt() calls can interleave and emit a transient
// IRQ.Set(false) while events are actually pending; if that glitch lands
// between the peer CPU's exception-entry decision and its NVIC acknowledge,
// Renode's NVIC finds no pending interrupt and returns spurious exception
// number 528, vectoring through VTOR+0x840 into garbage (observed with the
// Zephyr v3.4.99-ncs1-1 RPMsg ipc_service sample: both cores jumped to the
// literal word stored at image offset 0x840).
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_IPC : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_IPC(IMachine machine, NRF5340_IPC peer = null, NRF5340_DPPIC dppic = null) : base(machine)
        {
            if(peer != null)
            {
                this.peer = peer;
                peer.peer = this;
                this.syncRoot = peer.syncRoot;
            }
            else
            {
                this.syncRoot = new object();
            }

            // nRF5340 IPC couples to the DPPI: TASKS_SEND[i] can be triggered by
            // a DPPI channel (SUBSCRIBE_SEND[i]) with no CPU write, and
            // EVENTS_RECEIVE[j] can publish to a DPPI channel (PUBLISH_RECEIVE[j])
            // to drive another peripheral (e.g. an EGU IRQ) with no CPU read.
            // The G1 net-core SoftDevice Controller HCI signalling uses exactly
            // this path (channels 7/8/10/11) — without it the inter-core BT
            // vring kicks never propagate and bt_enable stalls (host -120).
            subscribeSend = new DPPISubscribeEndpoint[ChannelCount];
            publishReceive = new DPPIPublishEndpoint[ChannelCount];
            for(var i = 0; i < ChannelCount; i++)
            {
                var index = i;
                subscribeSend[index] = new DPPISubscribeEndpoint(dppic, this, $"SEND[{index}]", () => TriggerSend(index));
                publishReceive[index] = new DPPIPublishEndpoint(dppic, this, $"RECEIVE[{index}]");
            }

            DefineRegisters();
        }

        public override uint ReadDoubleWord(long offset)
        {
            lock(syncRoot)
            {
                return base.ReadDoubleWord(offset);
            }
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            lock(syncRoot)
            {
                base.WriteDoubleWord(offset, value);
            }
        }

        public override void Reset()
        {
            lock(syncRoot)
            {
                base.Reset();
                for(var i = 0; i < ChannelCount; i++)
                {
                    sendConfig[i] = 0;
                    receiveConfig[i] = 0;
                    receiveEvents[i] = false;
                    subscribeSend[i].Reset();
                    publishReceive[i].Reset();
                }
                interruptEnable = 0;
                gpMem0 = 0;
                gpMem1 = 0;
                UpdateInterrupt();
            }
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; } = new GPIO();

        private void DefineRegisters()
        {
            Registers.TasksSend.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithFlag(0, FieldMode.Write, writeCallback: (_, value) => { if(value) TriggerSend(index); })
                    .WithReservedBits(1, 31);
            });

            Registers.SubscribeSend.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithValueField(0, 32, name: "SUBSCRIBE_SEND",
                        valueProviderCallback: _ => subscribeSend[index].RegisterValue,
                        writeCallback: (_, value) => subscribeSend[index].RegisterValue = (uint)value);
            });

            Registers.EventsReceive.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithFlag(0,
                        valueProviderCallback: _ => receiveEvents[index],
                        writeCallback: (_, value) => { receiveEvents[index] = value; UpdateInterrupt(); })
                    .WithReservedBits(1, 31);
            });

            Registers.PublishReceive.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithValueField(0, 32, name: "PUBLISH_RECEIVE",
                        valueProviderCallback: _ => publishReceive[index].RegisterValue,
                        writeCallback: (_, value) => publishReceive[index].RegisterValue = (uint)value);
            });

            Registers.InterruptEnable.Define(this)
                .WithValueField(0, ChannelCount,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable = (uint)value; UpdateInterrupt(); })
                .WithReservedBits(ChannelCount, 32 - ChannelCount);

            Registers.InterruptEnableSet.Define(this)
                .WithValueField(0, ChannelCount,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable |= (uint)value; UpdateInterrupt(); })
                .WithReservedBits(ChannelCount, 32 - ChannelCount);

            Registers.InterruptEnableClear.Define(this)
                .WithValueField(0, ChannelCount,
                    valueProviderCallback: _ => interruptEnable,
                    writeCallback: (_, value) => { interruptEnable &= ~(uint)value; UpdateInterrupt(); })
                .WithReservedBits(ChannelCount, 32 - ChannelCount);

            Registers.InterruptPending.Define(this)
                .WithValueField(0, ChannelCount, FieldMode.Read, valueProviderCallback: _ => PendingInterrupts())
                .WithReservedBits(ChannelCount, 32 - ChannelCount);

            Registers.SendConfig.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithValueField(0, ChannelCount,
                        valueProviderCallback: _ => sendConfig[index],
                        writeCallback: (_, value) => sendConfig[index] = (uint)value)
                    .WithReservedBits(ChannelCount, 32 - ChannelCount);
            });

            Registers.ReceiveConfig.DefineMany(this, ChannelCount, (register, index) =>
            {
                register
                    .WithValueField(0, ChannelCount,
                        valueProviderCallback: _ => receiveConfig[index],
                        writeCallback: (_, value) => receiveConfig[index] = (uint)value)
                    .WithReservedBits(ChannelCount, 32 - ChannelCount);
            });

            Registers.GpMem0.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => gpMem0, writeCallback: (_, value) => gpMem0 = (uint)value);
            Registers.GpMem1.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => gpMem1, writeCallback: (_, value) => gpMem1 = (uint)value);
        }

        private void TriggerSend(int taskIndex)
        {
            // Reentrant lock (Monitor): register-write callers already hold it;
            // the DPPI SUBSCRIBE_SEND path enters here without it, so take it
            // to keep the cross-core receiveEvents/IRQ mutation serialized.
            lock(syncRoot)
            {
                var channels = sendConfig[taskIndex];
                if(channels == 0)
                {
                    this.Log(LogLevel.Noisy, "TASKS_SEND[{0}] with empty SEND_CNF, ignoring", taskIndex);
                    return;
                }
                DeliverChannels(channels);
                peer?.DeliverChannels(channels);
            }
        }

        private void DeliverChannels(uint channels)
        {
            var any = false;
            for(var j = 0; j < ChannelCount; j++)
            {
                if((receiveConfig[j] & channels) != 0 && !receiveEvents[j])
                {
                    receiveEvents[j] = true;
                    any = true;
                    // PUBLISH_RECEIVE[j]: route the event edge onto the DPPI,
                    // independent of INTEN (hardware publishes unconditionally).
                    publishReceive[j].Publish();
                }
            }
            if(any)
            {
                UpdateInterrupt();
            }
        }

        private uint PendingInterrupts()
        {
            uint pending = 0;
            for(var j = 0; j < ChannelCount; j++)
            {
                if(receiveEvents[j])
                {
                    pending |= 1u << j;
                }
            }
            return pending & interruptEnable;
        }

        private void UpdateInterrupt()
        {
            IRQ.Set(PendingInterrupts() != 0);
        }

        private NRF5340_IPC peer;
        private readonly object syncRoot;
        private readonly uint[] sendConfig = new uint[ChannelCount];
        private readonly uint[] receiveConfig = new uint[ChannelCount];
        private readonly bool[] receiveEvents = new bool[ChannelCount];
        private readonly DPPISubscribeEndpoint[] subscribeSend;
        private readonly DPPIPublishEndpoint[] publishReceive;
        private uint interruptEnable;
        private uint gpMem0, gpMem1;

        private const int ChannelCount = 16;

        private enum Registers : long
        {
            TasksSend = 0x000,        // ..0x03C, stride 4
            SubscribeSend = 0x080,    // ..0x0BC, stride 4
            EventsReceive = 0x100,    // ..0x13C, stride 4
            PublishReceive = 0x180,   // ..0x1BC, stride 4
            InterruptEnable = 0x300,
            InterruptEnableSet = 0x304,
            InterruptEnableClear = 0x308,
            InterruptPending = 0x30C,
            SendConfig = 0x510,       // ..0x54C, stride 4
            ReceiveConfig = 0x590,    // ..0x5CC, stride 4
            GpMem0 = 0x610,
            GpMem1 = 0x614,
        }
    }
}
