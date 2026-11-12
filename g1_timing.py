# Monitor commands for the accurate timing mode (g1sim.py --timing accurate,
# models/G1_Timing.cs). g1sim.py loads this with "i @g1_timing.py" after
# g1.resc and G1_Timing.cs.
import System
from Antmicro.Renode.Core import EmulationManager, Machine


def _timing():
    for assembly in System.AppDomain.CurrentDomain.GetAssemblies():
        t = assembly.GetType("Antmicro.Renode.Time.G1_Timing")
        if t is not None:
            return t
    raise Exception("models/G1_Timing.cs is not loaded")


def _call(method, *args):
    return _timing().GetMethod(method).Invoke(None, System.Array[System.Object](list(args)))


def mc_g1_machine_own_time(name):
    """Create machine NAME with a time source of its own ("mach create" gives
    it the emulation's)."""
    emulation = EmulationManager.Instance.CurrentEmulation
    emulation.AddMachine(Machine(True), name)


def mc_g1_keep_last(name, cpu):
    """Keep core CPU last in machine NAME's serial order (G1_Timing.KeepLast)."""
    ok, machine = EmulationManager.Instance.CurrentEmulation.TryGetMachineByName(name)
    if not ok:
        raise Exception("no machine " + name)
    print("g1_timing: %s: %s" % (name, _call("KeepLast", machine, cpu)))


def mc_g1_adaptive_quantum(small_us, big_us, hold_us):
    """Choose the time quantum at every sync point (G1_Timing.AdaptiveQuantum)."""
    print("g1_timing: " + _call("AdaptiveQuantum", float(small_us), float(big_us), float(hold_us)))


def mc_g1_timing_stats():
    """How often each quantum was chosen so far."""
    print("g1_timing: " + _call("Stats"))
