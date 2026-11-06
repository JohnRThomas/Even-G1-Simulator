//
// GPIO_Pin_Driver — monitor-controlled driver for a single GPIO input pin.
//
// Follows the Miscellaneous.Button pattern (register at a GPIO port pin,
// connect with `-> gpioX@N`), but with explicit level semantics instead of
// press/release, for driving board straps and other plain digital inputs:
//
//   strap_p0_25: Miscellaneous.GPIO_Pin_Driver @ gpio0 25
//       -> gpio0@25
//
// Monitor usage:
//   sysbus.strap_p0_25 SetHigh
//   sysbus.strap_p0_25 SetLow
//   sysbus.strap_p0_25 Set true
//   sysbus.strap_p0_25 Toggle
//   sysbus.strap_p0_25 State
//
// Electrical meaning of the driven pin is intentionally out of scope
// (board straps are board-specific); this model only drives a logic level.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class GPIO_Pin_Driver : IPeripheral, IGPIOSender
    {
        public GPIO_Pin_Driver(bool defaultState = false)
        {
            this.defaultState = defaultState;
            IRQ = new GPIO();
            Reset();
        }

        public void Reset()
        {
            IRQ.Set(defaultState);
        }

        public void SetHigh()
        {
            Set(true);
        }

        public void SetLow()
        {
            Set(false);
        }

        public void Set(bool state)
        {
            IRQ.Set(state);
            this.Log(LogLevel.Info, "Pin driven {0}", state ? "high" : "low");
        }

        public void Toggle()
        {
            Set(!IRQ.IsSet);
        }

        public bool State => IRQ.IsSet;

        public GPIO IRQ { get; }

        private readonly bool defaultState;
    }
}
