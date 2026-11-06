//
// NRF5340_VREQCTRL — nRF5340 network-core voltage request control
// (0x41004000, no IRQ).
//
// Register layout verbatim from svd/nrf5340_network.svd (hal_nordic 9784731…,
// see reference/nrf5340_regs.rs net::vreqctrl):
//   VREGRADIO.VREQH      0x500  RW  request high voltage for the radio
//   VREGRADIO.VREQHREADY 0x508  RO  READY when the requested voltage is on
//
// Behaviour per the nRF5340 PS VREQCTRL chapter: firmware (MPSL, ESB's
// update_radio_tx_power for +3 dBm output) sets VREQH.VREQH and polls
// VREQHREADY.READY. The grant is modelled as instantaneous: READY follows
// VREQH in the same write (hardware takes tVREQH ≈ a few us; nothing in the
// pinned stack measures it). Clearing VREQH clears READY.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_VREQCTRL : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_VREQCTRL(IMachine machine) : base(machine)
        {
            DefineRegisters();
        }

        public long Size => 0x1000;

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

        private void DefineRegisters()
        {
            Registers.VregRadioVreqH.Define(this)
                .WithFlag(0, out highVoltageRequested, name: "VREQH")
                .WithReservedBits(1, 31);

            Registers.VregRadioVreqHReady.Define(this)
                .WithFlag(0, FieldMode.Read, name: "READY",
                    valueProviderCallback: _ => highVoltageRequested.Value)
                .WithReservedBits(1, 31);
        }

        private IFlagRegisterField highVoltageRequested;

        private enum Registers : long
        {
            VregRadioVreqH = 0x500,      // VREGRADIO.VREQH
            VregRadioVreqHReady = 0x508, // VREGRADIO.VREQHREADY
        }
    }
}
