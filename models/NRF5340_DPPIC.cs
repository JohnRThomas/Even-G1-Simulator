//
// NRF5340_DPPIC — nRF5340 Distributed programmable peripheral interconnect
// controller, one instance per DPPI domain:
//   application core: 0x40017000 (NS) / 0x50017000 (S)
//   network core:     0x4100F000
//
// Register offsets verbatim from the pinned SVDs (hal_nordic
// 9784731461018d3e983604698fbbed6af2bea801), svd/nrf5340_application.svd and
// svd/nrf5340_network.svd. Both instances have 32 channels (CHEN CH0..CH31)
// and 6 channel groups:
//   TASKS_CHG[6].EN/.DIS      0x000, stride 8
//   SUBSCRIBE_CHG[6].EN/.DIS  0x080, stride 8
//   CHEN/CHENSET/CHENCLR      0x500/0x504/0x508 (SET/CLR read back CHEN)
//   CHG[6]                    0x800, stride 4
//
// The instance doubles as the DPPI channel hub for the whole domain: any
// peripheral model can attach its PUBLISH_*/SUBSCRIBE_* endpoint registers
// via DPPIPublishEndpoint/DPPISubscribeEndpoint (see models/DPPI_README.md
// for the API contract). An event published on a channel reaches every
// enabled subscription of that channel if — and only if — the channel is
// enabled in CHEN, matching hardware (publishers are never gated, the
// interconnect is).
//
// The DPPIC's own group EN/DIS tasks are themselves subscribable through
// SUBSCRIBE_CHG[n], wired to the same hub.
//
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_DPPIC : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_DPPIC(IMachine machine) : base(machine)
        {
            groupSubscribeEnable = new DPPISubscribeEndpoint[GroupCount];
            groupSubscribeDisable = new DPPISubscribeEndpoint[GroupCount];
            for(var i = 0; i < GroupCount; i++)
            {
                var group = i;
                groupSubscribeEnable[group] = new DPPISubscribeEndpoint(this, this, $"CHG[{group}].EN", () => EnableGroup(group));
                groupSubscribeDisable[group] = new DPPISubscribeEndpoint(this, this, $"CHG[{group}].DIS", () => DisableGroup(group));
            }
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            channelEnable = 0;
            for(var i = 0; i < GroupCount; i++)
            {
                channelGroups[i] = 0;
                groupSubscribeEnable[i].Reset();
                groupSubscribeDisable[i].Reset();
            }
        }

        // Publish an event on a DPPI channel. Called by DPPIPublishEndpoint
        // (and usable directly, e.g. from the monitor for fault injection).
        // Delivery is gated by CHEN, as on hardware.
        public void Publish(int channelIndex)
        {
            if(channelIndex < 0 || channelIndex >= NumberOfChannels)
            {
                this.Log(LogLevel.Warning, "Publish on nonexistent DPPI channel {0} (this DPPIC has {1} channels), ignoring", channelIndex, NumberOfChannels);
                return;
            }
            if((channelEnable & (1u << channelIndex)) == 0)
            {
                this.Log(LogLevel.Noisy, "Event published on disabled DPPI channel {0}, not routed", channelIndex);
                return;
            }
            if(publishDepth >= MaxPublishDepth)
            {
                this.Log(LogLevel.Error, "DPPI publish depth exceeded {0} on channel {1} — event loop? Dropping event", MaxPublishDepth, channelIndex);
                return;
            }
            publishDepth++;
            try
            {
                // Snapshot: a fired task may reconfigure subscriptions.
                foreach(var subscription in subscriptions.ToArray())
                {
                    if(subscription.IsListeningTo(channelIndex))
                    {
                        this.Log(LogLevel.Noisy, "DPPI channel {0} -> task {1}", channelIndex, subscription.Name);
                        subscription.FireTask();
                    }
                }
            }
            finally
            {
                publishDepth--;
            }
        }

        public int NumberOfChannels => ChannelCount;
        public int NumberOfGroups => GroupCount;

        public long Size => 0x1000;

        internal void RegisterSubscription(DPPISubscribeEndpoint endpoint)
        {
            subscriptions.Add(endpoint);
        }

        internal static string LabelOf(IMachine machine, IPeripheral peripheral)
        {
            try
            {
                if(machine != null && peripheral != null && machine.TryGetLocalName(peripheral, out var name))
                {
                    return name;
                }
            }
            catch(Exception)
            {
                // registration not finished yet — fall through
            }
            return peripheral?.GetType().Name ?? "?";
        }

        // Mandatory UNIMPL logging (docs/conventions.md) shared by the
        // nRF5340 fork models; format consumed by scripts/coverage_report.py.
        public static void LogUnimplementedAccess(IPeripheral peripheral, IMachine machine, string dir, long offset, uint value)
        {
            var pc = 0UL;
            if(machine.SystemBus.TryGetCurrentCPU(out var cpu))
            {
                pc = cpu.PC.RawValue;
            }
            peripheral.Log(LogLevel.Warning,
                "UNIMPL periph={0} dir={1} width=4 offset=0x{2:X} value=0x{3:X8} pc=0x{4:X8}",
                LabelOf(machine, peripheral), dir, offset, value, pc);
        }

        public override uint ReadDoubleWord(long offset)
        {
            if(RegistersCollection.TryRead(offset, out var result))
            {
                return result;
            }
            LogUnimplementedAccess(this, machine, "R", offset, 0);
            return 0;
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            if(!RegistersCollection.TryWrite(offset, value))
            {
                LogUnimplementedAccess(this, machine, "W", offset, value);
            }
        }

        private void DefineRegisters()
        {
            Registers.TasksChgEn.DefineMany(this, GroupCount, (register, index) =>
            {
                register
                    .WithFlag(0, FieldMode.Write, name: "EN", writeCallback: (_, value) => { if(value) EnableGroup(index); })
                    .WithReservedBits(1, 31);
            }, GroupStride);

            Registers.TasksChgDis.DefineMany(this, GroupCount, (register, index) =>
            {
                register
                    .WithFlag(0, FieldMode.Write, name: "DIS", writeCallback: (_, value) => { if(value) DisableGroup(index); })
                    .WithReservedBits(1, 31);
            }, GroupStride);

            Registers.SubscribeChgEn.DefineMany(this, GroupCount, (register, index) =>
            {
                register.WithValueField(0, 32, name: "SUBSCRIBE_CHG_EN",
                    valueProviderCallback: _ => groupSubscribeEnable[index].RegisterValue,
                    writeCallback: (_, value) => groupSubscribeEnable[index].RegisterValue = (uint)value);
            }, GroupStride);

            Registers.SubscribeChgDis.DefineMany(this, GroupCount, (register, index) =>
            {
                register.WithValueField(0, 32, name: "SUBSCRIBE_CHG_DIS",
                    valueProviderCallback: _ => groupSubscribeDisable[index].RegisterValue,
                    writeCallback: (_, value) => groupSubscribeDisable[index].RegisterValue = (uint)value);
            }, GroupStride);

            Registers.ChannelEnable.Define(this)
                .WithValueField(0, 32, name: "CHEN",
                    valueProviderCallback: _ => channelEnable,
                    writeCallback: (_, value) => channelEnable = (uint)value);

            Registers.ChannelEnableSet.Define(this)
                .WithValueField(0, 32, name: "CHENSET",
                    valueProviderCallback: _ => channelEnable,
                    writeCallback: (_, value) => channelEnable |= (uint)value);

            Registers.ChannelEnableClear.Define(this)
                .WithValueField(0, 32, name: "CHENCLR",
                    valueProviderCallback: _ => channelEnable,
                    writeCallback: (_, value) => channelEnable &= ~(uint)value);

            Registers.ChannelGroup.DefineMany(this, GroupCount, (register, index) =>
            {
                register.WithValueField(0, 32, name: "CHG",
                    valueProviderCallback: _ => channelGroups[index],
                    writeCallback: (_, value) => channelGroups[index] = (uint)value);
            });
        }

        private void EnableGroup(int group)
        {
            channelEnable |= channelGroups[group];
            this.Log(LogLevel.Noisy, "CHG[{0}].EN: CHEN |= 0x{1:X8} -> 0x{2:X8}", group, channelGroups[group], channelEnable);
        }

        private void DisableGroup(int group)
        {
            channelEnable &= ~channelGroups[group];
            this.Log(LogLevel.Noisy, "CHG[{0}].DIS: CHEN &= ~0x{1:X8} -> 0x{2:X8}", group, channelGroups[group], channelEnable);
        }

        private uint channelEnable;
        private int publishDepth;
        private readonly uint[] channelGroups = new uint[GroupCount];
        private readonly DPPISubscribeEndpoint[] groupSubscribeEnable;
        private readonly DPPISubscribeEndpoint[] groupSubscribeDisable;
        private readonly List<DPPISubscribeEndpoint> subscriptions = new List<DPPISubscribeEndpoint>();

        private const int ChannelCount = 32;    // CHEN CH0..CH31, both SVDs
        private const int GroupCount = 6;       // TASKS_CHG/SUBSCRIBE_CHG/CHG dim, both SVDs
        private const uint GroupStride = 0x8;
        private const int MaxPublishDepth = 32;

        private enum Registers : long
        {
            TasksChgEn = 0x000,         // TASKS_CHG[0..5].EN, stride 8
            TasksChgDis = 0x004,        // TASKS_CHG[0..5].DIS, stride 8
            SubscribeChgEn = 0x080,     // SUBSCRIBE_CHG[0..5].EN, stride 8
            SubscribeChgDis = 0x084,    // SUBSCRIBE_CHG[0..5].DIS, stride 8
            ChannelEnable = 0x500,      // CHEN
            ChannelEnableSet = 0x504,   // CHENSET
            ChannelEnableClear = 0x508, // CHENCLR
            ChannelGroup = 0x800,       // CHG[0..5], stride 4
        }
    }

    // One PUBLISH_* register of a peripheral: holds CHIDX (bits 7:0) and EN
    // (bit 31), exactly the SVD field layout. The owning model calls
    // Publish() at the point where the corresponding event fires.
    public sealed class DPPIPublishEndpoint
    {
        public DPPIPublishEndpoint(NRF5340_DPPIC dppic, IPeripheral owner, string name)
        {
            this.dppic = dppic;
            this.owner = owner;
            Name = name;
        }

        public void Publish()
        {
            if(!Enabled)
            {
                return;
            }
            if(dppic == null)
            {
                // TODO(dppi): peripheral instantiated without a hub reference
                owner.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=dppi-publish detail=PUBLISH_{1} enabled but no DPPIC hub connected, event not routed", Owner, Name);
                return;
            }
            dppic.Publish(ChannelIndex);
        }

        public void Reset()
        {
            registerValue = 0;
        }

        public uint RegisterValue
        {
            get => registerValue;
            set => registerValue = value & FieldMask;
        }

        public bool Enabled => (registerValue & EnableMask) != 0;
        public int ChannelIndex => (int)(registerValue & ChannelMask);
        public string Name { get; }

        private string Owner
        {
            get
            {
                try
                {
                    return NRF5340_DPPIC.LabelOf(owner.GetMachine(), owner);
                }
                catch(Exception)
                {
                    return owner.GetType().Name;
                }
            }
        }

        private uint registerValue;
        private readonly NRF5340_DPPIC dppic;
        private readonly IPeripheral owner;

        private const uint ChannelMask = 0xFF;      // CHIDX bits 7:0
        private const uint EnableMask = 1u << 31;   // EN bit 31
        private const uint FieldMask = ChannelMask | EnableMask;
    }

    // One SUBSCRIBE_* register of a peripheral: same CHIDX/EN layout. While
    // enabled, an event published on the selected (and CHEN-enabled) channel
    // invokes the task callback given at construction.
    public sealed class DPPISubscribeEndpoint
    {
        public DPPISubscribeEndpoint(NRF5340_DPPIC dppic, IPeripheral owner, string name, Action task)
        {
            this.dppic = dppic;
            this.owner = owner;
            this.task = task;
            Name = name;
            dppic?.RegisterSubscription(this);
        }

        public void Reset()
        {
            registerValue = 0;
        }

        public bool IsListeningTo(int channelIndex)
        {
            return Enabled && ChannelIndex == channelIndex;
        }

        public void FireTask()
        {
            task();
        }

        public uint RegisterValue
        {
            get => registerValue;
            set
            {
                registerValue = value & FieldMask;
                if(Enabled && dppic == null)
                {
                    // TODO(dppi): peripheral instantiated without a hub reference
                    owner.Log(LogLevel.Warning, "UNSUPPORTED periph={0} what=dppi-subscribe detail=SUBSCRIBE_{1} enabled but no DPPIC hub connected, task will never fire", Owner, Name);
                }
            }
        }

        public bool Enabled => (registerValue & EnableMask) != 0;
        public int ChannelIndex => (int)(registerValue & ChannelMask);
        public string Name { get; }

        private string Owner
        {
            get
            {
                try
                {
                    return NRF5340_DPPIC.LabelOf(owner.GetMachine(), owner);
                }
                catch(Exception)
                {
                    return owner.GetType().Name;
                }
            }
        }

        private uint registerValue;
        private readonly NRF5340_DPPIC dppic;
        private readonly IPeripheral owner;
        private readonly Action task;

        private const uint ChannelMask = 0xFF;      // CHIDX bits 7:0
        private const uint EnableMask = 1u << 31;   // EN bit 31
        private const uint FieldMask = ChannelMask | EnableMask;
    }
}
