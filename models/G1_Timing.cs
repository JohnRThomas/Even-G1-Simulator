//
// G1_Timing: the accurate timing mode (g1sim.py --timing accurate).
//
// By default the four cores (app and net core of each lens) are handles of
// the emulation's one time source. They run one after the other within each
// time quantum, and a lens's clock (the one every timer peripheral runs on)
// advances only as far as the slowest core has run, so every core but the
// last runs its quantum on a stopped clock and takes its timer interrupts up
// to a quantum late. Radio frames between the lenses reach the other lens only
// at its next sync point. Both hold the quantum at 50 us (README, "Simulated
// Phone").
//
// The accurate mode gives each lens a time source of its own (g1_timing.py
// creates the machines) and
//   - keeps the net core last in its lens's serial order (KeepLast), so the
//     lens's clock moves exactly with the net core, where the SoftDevice
//     Controller and ESB keep their radio timing;
//   - lets a polling app core see its UART transmission end on its own clock
//     (NRF5340_UARTE.TxLocalPoll);
//   - hands radio frames from one lens straight to the other, on the
//     receiver's clock at the sender's time (NRF5340_RADIO.DirectPeerDelivery);
//   - picks the quantum afresh at every sync point (AdaptiveQuantum): short
//     whenever anything happens on either lens, long only while both are
//     asleep. The app core still runs its slice before its lens's clock
//     moves, so it sees what happens around it (a transfer done, a timer, an
//     IPC message) at the next sync point, at most a short quantum late, as
//     with the default timing.
//
// The lens's own time source and the serial order are internals of Renode's
// time framework (TimeSourceBase.handles, HandlesCollection.ready/locker),
// reached by reflection; checked with Renode 1.16.1 and 1.17.0.
//
using System;
using System.Collections.Generic;
using System.Reflection;
using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Wireless;

namespace Antmicro.Renode.Time
{
    public static class G1_Timing
    {
        // Keep the core named `cpuName` last in `machine`'s serial order, at
        // every sync point of the machine's own time source.
        public static string KeepLast(IMachine machine, string cpuName)
        {
            var source = machine.LocalTimeSource as TimeSourceBase;
            if(!(source is SlaveTimeSource))
            {
                throw new RecoverableException("the machine has no time source of its own");
            }
            if(!machine.TryGetByName<ICPU>("sysbus." + cpuName, out var cpu))
            {
                throw new RecoverableException("no CPU " + cpuName);
            }
            var handlesField = typeof(TimeSourceBase).GetField("handles", BindingFlags.Instance | BindingFlags.NonPublic);
            var handles = handlesField?.GetValue(source);
            var readyField = handles?.GetType().GetField("ready", BindingFlags.Instance | BindingFlags.NonPublic);
            var lockerField = handles?.GetType().GetField("locker", BindingFlags.Instance | BindingFlags.NonPublic);
            var ready = readyField?.GetValue(handles) as LinkedList<TimeHandle>;
            var locker = lockerField?.GetValue(handles);
            if(ready == null || locker == null)
            {
                throw new RecoverableException("this Renode's time framework is not the one G1_Timing.cs knows (1.16/1.17)");
            }
            var sink = (ITimeSink)cpu;
            Action keepLast = () =>
            {
                lock(locker)
                {
                    var last = ready.Last;
                    if(last == null || last.Value.TimeSink == sink)
                    {
                        return;
                    }
                    for(var node = ready.First; node != null; node = node.Next)
                    {
                        if(node.Value.TimeSink == sink)
                        {
                            ready.Remove(node);
                            ready.AddLast(node);
                            return;
                        }
                    }
                }
            };
            keepLast();
            source.SyncHook += _ => keepLast();
            return string.Format("{0} runs last of {1} cores", cpuName, ready.Count);
        }

        // The quantum, chosen afresh at every sync point of the emulation's
        // master time source, for the master and each lens's own time source:
        //   small  while something happens on either lens, and for `hold`
        //          after it: its radio is enabled (what one lens sends reaches
        //          the other only as far as the other has run, and the ESB
        //          acknowledgement is due 150-300 us after the packet), one of
        //          its cores ran in the last quantum (rather than sleeping in
        //          WFI), or one of its scheduled events (a timer, a transfer's
        //          end) has come due;
        //   up to  the next scheduled event on either lens (plus `small`), so
        //          that the event is seen at most `small` late and whatever it
        //          starts begins in small quanta;
        //   big    otherwise: all four cores asleep and nothing due. Only an
        //          input from outside (a monitor command) can then wake a core,
        //          at most `big` late.
        public static string AdaptiveQuantum(double smallUs, double bigUs, double holdUs)
        {
            var emulation = EmulationManager.Instance.CurrentEmulation;
            small = TimeInterval.FromMicroseconds((ulong)smallUs);
            big = TimeInterval.FromMicroseconds((ulong)bigUs);
            hold = TimeInterval.FromMicroseconds((ulong)holdUs);
            lenses.Clear();
            foreach(var machine in emulation.Machines)
            {
                var lens = new Lens { Machine = machine, Source = machine.LocalTimeSource as SlaveTimeSource };
                machine.TryGetByName<NRF5340_RADIO>("sysbus.radio", out lens.Radio);
                foreach(var cpu in machine.GetPeripheralsOfType<ICPU>())
                {
                    lens.Cores.Add(cpu);
                    lens.Instructions.Add(cpu.ExecutedInstructions);
                }
                lenses.Add(lens);
            }
            holdUntil = TimeInterval.Empty;
            timerDue = null;
            if(!installed)
            {
                emulation.MasterTimeSource.SyncHook += Choose;
                installed = true;
            }
            Choose(emulation.MasterTimeSource.ElapsedVirtualTime);
            return string.Format("adaptive quantum {0}..{1} us, hold {2} us, {3} lenses", smallUs, bigUs, holdUs, lenses.Count);
        }

        public static string Stats()
        {
            return string.Format("sync points {0}: small {1}, up to an event {2}, big {3}; virtual {4:F3} s",
                syncs, smallCount, cappedCount, bigCount, EmulationManager.Instance.CurrentEmulation.MasterTimeSource.ElapsedVirtualTime.TotalSeconds);
        }

        private static void Choose(TimeInterval now)
        {
            syncs++;
            foreach(var lens in lenses)
            {
                var busy = lens.Radio != null && lens.Radio.Busy;
                for(var i = 0; i < lens.Cores.Count; i++)
                {
                    var executed = lens.Cores[i].ExecutedInstructions;
                    busy |= executed != lens.Instructions[i];
                    lens.Instructions[i] = executed;
                }
                if(busy)
                {
                    holdUntil = now + hold;
                }
            }
            if(timerDue.HasValue && now >= timerDue.Value)
            {
                var until = timerDue.Value + hold;
                if(until > holdUntil)
                {
                    holdUntil = until;
                }
                timerDue = null;
            }

            TimeInterval quantum;
            var next = NextEvent();
            if(now < holdUntil)
            {
                quantum = small;
                smallCount++;
            }
            else if(next.HasValue && next.Value + small < big)
            {
                quantum = next.Value + small;
                cappedCount++;
            }
            else
            {
                quantum = big;
                bigCount++;
            }
            // An event due within this quantum is followed by `hold` of small
            // quanta from the next sync point on (above), even when it falls
            // at the quantum's end: what it starts must not run on into a big
            // quantum.
            if(next.HasValue && next.Value <= quantum)
            {
                timerDue = now + next.Value;
            }
            EmulationManager.Instance.CurrentEmulation.MasterTimeSource.Quantum = quantum;
            foreach(var lens in lenses)
            {
                if(lens.Source != null)
                {
                    lens.Source.Quantum = quantum;
                }
            }
        }

        // Time to the nearest scheduled event (a clock entry reaching its
        // limit) on either lens.
        private static TimeInterval? NextEvent()
        {
            TimeInterval? best = null;
            foreach(var lens in lenses)
            {
                var clock = lens.Machine.ClockSource as BaseClockSource;
                if(clock == null)
                {
                    continue;
                }
                foreach(var entry in clock.GetAllClockEntries())
                {
                    if(!entry.Enabled || entry.Frequency == 0)
                    {
                        continue;
                    }
                    ulong ticks = entry.Direction == Direction.Ascending
                        ? (entry.Period > entry.Value ? entry.Period - entry.Value : 0)
                        : entry.Value;
                    var step = entry.Step == 0 ? 1UL : entry.Step;
                    var us = ticks / (double)step * 1e6 / entry.Frequency;
                    if(us > 1e7)
                    {
                        continue;
                    }
                    var t = TimeInterval.FromMicroseconds((ulong)Math.Ceiling(us));
                    if(!best.HasValue || t < best.Value)
                    {
                        best = t;
                    }
                }
            }
            return best;
        }

        private class Lens
        {
            public IMachine Machine;
            public SlaveTimeSource Source;
            public NRF5340_RADIO Radio;
            public readonly List<ICPU> Cores = new List<ICPU>();
            public readonly List<ulong> Instructions = new List<ulong>();
        }

        private static readonly List<Lens> lenses = new List<Lens>();
        private static TimeInterval small, big, hold, holdUntil;
        private static TimeInterval? timerDue;
        private static ulong syncs, smallCount, cappedCount, bigCount;
        private static bool installed;
    }
}
