//
// NRF5340_AAR — nRF5340 network-core accelerated address resolver engine.
//
// The nRF5340 AAR and CCM are two SVD peripherals sharing one register page
// (0x4100E000, IRQ 14) and one hardware block; the sysbus-registered
// container is models/NRF5340_CCM.cs (class NRF5340_CCM_AAR), which owns the
// union register map and delegates AAR behaviour to this engine class
// ("two cooperating models" — the register offsets of the two personalities
// overlap, so only one bus peripheral can exist).
//
// Behaviour per the nRF5340 Product Specification, AAR chapter
// (https://docs.nordicsemi.com/bundle/ps_nrf5340/page/aar.html) and the
// Bluetooth Core Specification random-address hash function ah()
// (Vol 3 Part H 2.2.2): for each of NIRK 16-byte IRKs at IRKPTR, compute
//   hash = e(irk, 0^104 || prand) mod 2^24
// and compare with the hash part of the resolvable private address.
//
// Byte-order conventions (verified):
//   - IRK list entries are stored big-endian (most significant octet first) —
//     the Zephyr LL controller explicitly swaps HCI little-endian IRKs before
//     handing them to the hardware ("AAR requires big-endian IRKs",
//     zephyr subsys/bluetooth/controller/ll_sw/ull_filter.c:331-333, pinned
//     v3.4.99-ncs1-1). The IRK bytes are therefore used directly as the AES key.
//   - The 6-byte on-air address is hash[0..2] (LSB first) then prand[0..2]
//     (LSB first, two top bits of the last byte = 0b01).
//   - The ah() implementation is validated against the Bluetooth Core
//     specification sample data (IRK ec0234a357c8ad05341010a60a397d9b,
//     prand 0x708194 -> ah 0x0dfbaa).
//   - Hardware quirk: the 6 address bytes are fetched at ADDRPTR + 3, not at
//     ADDRPTR. The Zephyr controller programs ADDRPTR = PACKETPTR - 1 for
//     legacy PDUs (AdvA at PACKETPTR+2) and PACKETPTR + 1 for extended PDUs
//     (AdvA at PACKETPTR+4) — zephyr .../nordic/hal/nrf5/radio/radio.c,
//     radio_ar_configure(), pinned v3.4.99-ncs1-1 — so the block resolves the
//     bytes 3 past ADDRPTR. Modelled identically.
//
// Timing: resolution completes after (1 + 6*n) us of virtual time, n = number
// of IRKs tried until match (or NIRK when unresolved) — PS: 6.1 us typical
// per IRK, rounded to whole microseconds for determinism.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Utilities.Crypto;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Result of one resolution run, to be signalled by the container after
    // the deterministic virtual-time delay has elapsed.
    public struct AarResolution
    {
        public bool Resolved;
        public int Index;        // matching IRK index (STATUS) when Resolved
        public double DurationUs;
    }

    public class NRF5340_AAR
    {
        public NRF5340_AAR(IPeripheral owner)
        {
            this.owner = owner;
        }

        // Runs the Resolvable Private Address Resolution Procedure over the
        // IRK list. Reads RAM through the given bus. nirk is already clamped
        // by the caller to 1..16.
        public AarResolution Resolve(IBusController bus, ulong irkPointer, ulong addressPointer, int nirk)
        {
            // Hardware fetches the 6 address bytes at ADDRPTR + 3 (see header)
            var address = bus.ReadBytes(addressPointer + AddressOffsetQuirk, 6);
            var result = new AarResolution { Resolved = false, Index = 0 };
            var tried = 0;
            for(var i = 0; i < nirk; i++)
            {
                var irk = bus.ReadBytes(irkPointer + (ulong)(16 * i), 16);
                tried++;
                if(MatchesIrk(irk, address))
                {
                    result.Resolved = true;
                    result.Index = i;
                    break;
                }
            }
            result.DurationUs = 1 + PerIrkUs * tried;
            owner.Log(LogLevel.Noisy, "AAR: address {0} {1} after {2} IRK(s)",
                BitConverter.ToString(address), result.Resolved ? $"resolved with IRK {result.Index}" : "not resolved", tried);
            return result;
        }

        // ah() per BT Core Vol 3 Part H 2.2.2: hash = e(irk, 0^104 || prand)
        // mod 2^24. IRK bytes are big-endian in RAM and used directly as the
        // AES-128 key; prand/hash travel LSB-first in the address bytes.
        public static bool MatchesIrk(byte[] irk, byte[] address)
        {
            var block = new byte[16];
            block[13] = address[5];  // prand MSB (contains the 0b01 marker bits)
            block[14] = address[4];
            block[15] = address[3];  // prand LSB
            using(var aes = AesProvider.GetEcbProvider(irk))
            {
                aes.EncryptBlockInSitu(Block.UsingBytes(block));
            }
            return block[15] == address[0] && block[14] == address[1] && block[13] == address[2];
        }

        private readonly IPeripheral owner;

        private const ulong AddressOffsetQuirk = 3;
        private const double PerIrkUs = 6; // PS: 6.1 us typical, rounded
    }
}
