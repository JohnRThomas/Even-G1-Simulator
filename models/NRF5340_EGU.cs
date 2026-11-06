//
// NRF5340_EGU — nRF5340 event generator unit.
//
// Fork of Renode's NRF52840_EGU (reference/renode-models/NRF52840_EGU.cs,
// Antmicro, MIT) extended with the nRF5340 DPPI endpoint registers, offsets
// verbatim from the pinned SVDs (hal_nordic 9784731…), EGU0 map (all EGU
// instances derive from it in both SVDs):
//   TASKS_TRIGGER[16]     0x000
//   SUBSCRIBE_TRIGGER[16] 0x080   (task offset + 0x80)
//   EVENTS_TRIGGERED[16]  0x100
//   PUBLISH_TRIGGERED[16] 0x180   (event offset + 0x80)
//   INTEN/INTENSET/INTENCLR 0x300/0x304/0x308
//
// Pass `dppic:` in the platform description to attach the instance to its
// DPPI domain hub (see models/DPPI_README.md).
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_EGU : BasicDoubleWordPeripheral, IKnownSize, INRFEventProvider
    {
        public NRF5340_EGU(IMachine machine, NRF5340_DPPIC dppic = null) : base(machine)
        {
            interruptManager = new InterruptManager<Events>(this, IRQ, "EGU_IRQ");

            subscribeTrigger = new DPPISubscribeEndpoint[NumberOfTasks];
            publishTriggered = new DPPIPublishEndpoint[NumberOfTasks];
            for(var i = 0; i < NumberOfTasks; i++)
            {
                var index = i;
                subscribeTrigger[index] = new DPPISubscribeEndpoint(dppic, this, $"TRIGGER[{index}]", () => TriggerTask(index));
                publishTriggered[index] = new DPPIPublishEndpoint(dppic, this, $"TRIGGERED[{index}]");
            }

            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            interruptManager.Reset();
            base.Reset();
            for(var i = 0; i < NumberOfTasks; i++)
            {
                subscribeTrigger[i].Reset();
                publishTriggered[i].Reset();
            }
        }

        public long Size => 0x1000;

        public GPIO IRQ { get; } = new GPIO();

        public event Action<uint> EventTriggered;

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

        private void TriggerTask(int i)
        {
            interruptManager.SetInterrupt((Events)i);
            events[i].Value = true;

            this.NoisyLog("Triggering event {0}.", i);

            // Triggering a task results in raising an interrupt and generating an event
            EventTriggered?.Invoke((uint)Registers.EventsTriggered0 + (uint)i * EventsTriggerRegisterSize);
            publishTriggered[i].Publish();
        }

        private void DefineRegisters()
        {
            Registers.TasksTrigger0.DefineMany(this, NumberOfTasks, (reg, i) =>
            {
                reg.WithFlag(0, FieldMode.Write, writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        TriggerTask(i);
                    }
                })
                .WithReservedBits(1, 31);
            }, TasksTriggerRegisterSize, name: "TasksTrigger");

            Registers.SubscribeTrigger0.DefineMany(this, NumberOfTasks, (reg, i) =>
            {
                reg.WithValueField(0, 32, name: "SUBSCRIBE_TRIGGER",
                    valueProviderCallback: _ => subscribeTrigger[i].RegisterValue,
                    writeCallback: (_, value) => subscribeTrigger[i].RegisterValue = (uint)value);
            }, TasksTriggerRegisterSize, name: "SubscribeTrigger");

            // These registers are used to check if an event has been generated for a particular task (if a task has been triggered)
            // and to reset event state on write
            Registers.EventsTriggered0.DefineMany(this, NumberOfTasks, (reg, i) =>
            {
                reg.WithFlag(0, flagField: out events[i], writeCallback: (_, value) =>
                {
                    if(!value)
                    {
                        interruptManager.SetInterrupt((Events)i, false);
                    }
                    this.NoisyLog("Change event {0} to state {1}.", i, value);
                })
                .WithReservedBits(1, 31);
            }, EventsTriggerRegisterSize, name: "EventsTriggered");

            Registers.PublishTriggered0.DefineMany(this, NumberOfTasks, (reg, i) =>
            {
                reg.WithValueField(0, 32, name: "PUBLISH_TRIGGERED",
                    valueProviderCallback: _ => publishTriggered[i].RegisterValue,
                    writeCallback: (_, value) => publishTriggered[i].RegisterValue = (uint)value);
            }, EventsTriggerRegisterSize, name: "PublishTriggered");

            RegistersCollection.AddRegister((long)Registers.InterruptEnableDisable, interruptManager.GetInterruptEnableRegister<DoubleWordRegister>());
            RegistersCollection.AddRegister((long)Registers.InterruptSet, interruptManager.GetInterruptEnableSetRegister<DoubleWordRegister>());
            RegistersCollection.AddRegister((long)Registers.InterruptClear, interruptManager.GetInterruptEnableClearRegister<DoubleWordRegister>());
        }

        private readonly IFlagRegisterField[] events = new IFlagRegisterField[NumberOfTasks];
        private readonly InterruptManager<Events> interruptManager;
        private readonly DPPISubscribeEndpoint[] subscribeTrigger;
        private readonly DPPIPublishEndpoint[] publishTriggered;

        private const uint NumberOfTasks = 16;
        private const uint TasksTriggerRegisterSize = 0x4;
        private const uint EventsTriggerRegisterSize = 0x4;

        private enum Events
        {
            Triggered0 = 0,
            Triggered1,
            Triggered2,
            Triggered3,
            Triggered4,
            Triggered5,
            Triggered6,
            Triggered7,
            Triggered8,
            Triggered9,
            Triggered10,
            Triggered11,
            Triggered12,
            Triggered13,
            Triggered14,
            Triggered15,
        }

        private enum Registers
        {
            TasksTrigger0 = 0x0,
            SubscribeTrigger0 = 0x080,
            EventsTriggered0 = 0x100,
            PublishTriggered0 = 0x180,
            InterruptEnableDisable = 0x300,
            InterruptSet = 0x304,
            InterruptClear = 0x308
        }
    }
}
