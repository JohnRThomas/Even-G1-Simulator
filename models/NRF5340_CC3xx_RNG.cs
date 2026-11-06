//
// NRF5340_CC3xx_RNG — ARM CryptoCell CC3xx (CC-312) core register block on
// the nRF5340. The CC-312 core base is 0x50845000 (the CRYPTOCELL wrapper at
// 0x50844000 only holds the ENABLE gate at +0x500). This model covers the
// whole core: secure alias 0x50845000 / non-secure alias 0x40845000, size
// 0x2000 (both the 0x50845xxx and 0x50846xxx pages). It implements two
// sub-blocks that share the page:
//
//   * the TRNG (offsets 0x100..0x1C8) — synthetic seeded entropy, below;
//   * the symmetric AES/HASH DMA engine + HOST_RGF interrupt block
//     (offsets 0x400..0xF34) — see the "CC3xx symmetric engine" section
//     lower in this file.
//
// == TRNG sub-block =========================================================
//
// Purpose: unblock the nrf_cc3xx_platform_init startup path. The CC3xx
// software TRNG self-test (LLF_RND_RunTrngStartupTest → getTrngSource →
// startTrngHW) enables a ring oscillator, waits for RNG_ISR.EHR_VALID, reads
// six EHR_DATA words, runs von-Neumann + autocorrelation checks, and repeats
// across the four ring oscillators. With no peripheral mapped here the poll
// never completes and CPUAPP busy-loops in startTrngHW (never clearing
// RESET.NETWORK.FORCEOFF, so CPUNET also stays halted).
//
// This is a SYNTHETIC, SEEDED entropy source for the virtual device — the
// same approach Renode/QEMU take for hardware RNGs (cf. the reused
// NRF52840_RNG model in the tree). It is NOT a real entropy source and must
// not be relied on for cryptographic strength; it exists purely so the
// firmware's self-test passes deterministically and boot proceeds.
//
// Register map (offsets from base, ARM CC3xx TRNG):
//   0x100 RNG_IMR              0x114..0x128 EHR_DATA_0..5 (6 entropy words)
//   0x104 RNG_ISR  (bit0 = EHR_VALID, bit2 = AUTOCORR_ERR, bit3 = VN_ERR)
//   0x108 RNG_ICR  (write-1-to-clear ISR)
//   0x10C TRNG_CONFIG          0x12C RND_SOURCE_ENABLE
//   0x110 TRNG_VALID           0x130 SAMPLE_CNT1
//   0x138 TRNG_DEBUG_CONTROL   0x140 RNG_SW_RESET
//   0x1B8 RNG_BUSY             0x1C4 RNG_CLK_ENABLE   0x1C8 RNG_DMA_ENABLE
//
// Behaviour: when the source is enabled (RND_SOURCE_ENABLE bit0, with the
// clock enabled), the six EHR_DATA words are refilled from a seeded PRNG and
// RNG_ISR.EHR_VALID / TRNG_VALID are asserted — immediately, or after
// SamplingDelayUs of virtual time when set. RNG_ICR write-1-to-clear on
// EHR_VALID re-arms the next sample (fresh words) while the source stays
// enabled, so each of the four ROSC iterations sees an independent stream.
// RNG_BUSY always reads not-busy. Statistical-error bits (AUTOCORR/VN) are
// never asserted — the synthetic stream is designed to pass.
//
// Determinism: the PRNG is a SplitMix64 seeded from the Seed property
// (default 0x5340CC3105340CC3, monitor/repl-settable and recorded). Identical
// Seed + identical firmware access order ⇒ identical EHR_DATA stream.
//
using System;
using System.Collections.Generic;
using System.Security.Cryptography;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class NRF5340_CC3xx_RNG : BasicDoubleWordPeripheral, IKnownSize
    {
        public NRF5340_CC3xx_RNG(IMachine machine, ulong seed = DefaultSeed) : base(machine)
        {
            this.seed = seed;
            prngState = seed;
            DefineRegisters();
            RefillEntropy();
        }

        public override void Reset()
        {
            base.Reset();
            clockEnabled = false;
            dmaEnabled = false;
            sourceEnabled = false;
            ehrValid = false;
            interruptMask = 0;
            trngConfig = 0;
            sampleCount = 0;
            debugControl = 0;
            // Re-seed to the recorded seed so every run is reproducible.
            prngState = seed;
            RefillEntropy();
            cryptoRegs.Clear();
            hostIrr = 0;
        }

        public long Size => 0x2000;

        // ---- CC3xx core dispatch -------------------------------------------
        // The TRNG registers (0x100..0x1C8) are served by the base
        // BasicDoubleWordPeripheral register collection defined in
        // DefineRegisters(); everything else in the core page (HOST_RGF, the
        // AES/HASH engine, the DMA descriptors and misc config) is served by
        // the symmetric-engine handler below.

        public override uint ReadDoubleWord(long offset)
        {
            return IsTrngRegister(offset) ? base.ReadDoubleWord(offset) : CryptoRead(offset);
        }

        public override void WriteDoubleWord(long offset, uint value)
        {
            if(IsTrngRegister(offset))
            {
                base.WriteDoubleWord(offset, value);
                return;
            }
            CryptoWrite(offset, value);
        }

        private static bool IsTrngRegister(long offset)
        {
            return TrngRegisterOffsets.Contains(offset);
        }

        // Seed of the synthetic entropy PRNG. Reproducibility: setting this
        // (repl `seed:` or monitor `sysbus.cc3xx_rng Seed <n>`) fixes the
        // EHR_DATA stream. Read it back to record which seed a run used.
        public ulong Seed
        {
            get => seed;
            set
            {
                seed = value;
                prngState = value;
                RefillEntropy();
            }
        }

        // Optional sampling latency in microseconds of virtual time before
        // EHR_VALID asserts. Default 0 (assert immediately) keeps the boot
        // path fast and fully deterministic; a small value mimics ROSC
        // sampling if a test wants to observe the poll spin briefly.
        public uint SamplingDelayUs { get; set; }

        private void DefineRegisters()
        {
            Registers.RngImr.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => interruptMask,
                    writeCallback: (_, value) => interruptMask = (uint)value);

            // RNG_ISR: bit0 EHR_VALID. Autocorrelation (bit2) and von-Neumann
            // (bit3) error bits are wired read-only-zero — the synthetic
            // stream never fails, so the firmware takes the success path.
            Registers.RngIsr.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => ehrValid, name: "EHR_VALID")
                .WithReservedBits(1, 31);

            // RNG_ICR: write-1-to-clear the ISR bits. Clearing EHR_VALID
            // consumes the current sample; if the source is still enabled we
            // re-arm with a fresh word set (the next ROSC iteration).
            Registers.RngIcr.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, clear) =>
                {
                    if(!clear)
                    {
                        return;
                    }
                    SetEhrValid(false);
                    if(sourceEnabled && clockEnabled)
                    {
                        ArmSample();
                    }
                })
                .WithReservedBits(1, 31);

            Registers.TrngConfig.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => trngConfig,
                    writeCallback: (_, value) => trngConfig = (uint)value);

            // TRNG_VALID mirrors EHR_VALID (some CC3xx paths poll it instead).
            Registers.TrngValid.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => ehrValid)
                .WithReservedBits(1, 31);

            // EHR_DATA_0..5 — six 32-bit entropy words. Reading is
            // side-effect free; the words are refilled when a sample is armed.
            for(var i = 0; i < EhrWordCount; i++)
            {
                var index = i;
                (Registers.EhrData0 + (long)(4 * i)).Define(this)
                    .WithValueField(0, 32, FieldMode.Read,
                        valueProviderCallback: _ => ehrData[index]);
            }

            Registers.RndSourceEnable.Define(this)
                .WithFlag(0, valueProviderCallback: _ => sourceEnabled,
                    writeCallback: (_, value) =>
                    {
                        sourceEnabled = value;
                        if(sourceEnabled && clockEnabled)
                        {
                            ArmSample();
                        }
                        else
                        {
                            SetEhrValid(false);
                        }
                    })
                .WithReservedBits(1, 31);

            Registers.SampleCnt1.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => sampleCount,
                    writeCallback: (_, value) => sampleCount = (uint)value);

            Registers.TrngDebugControl.Define(this)
                .WithValueField(0, 32, valueProviderCallback: _ => debugControl,
                    writeCallback: (_, value) => debugControl = (uint)value);

            // RNG_SW_RESET: writing 1 resets the TRNG core state (not the
            // recorded seed — reproducibility is preserved).
            Registers.RngSwReset.Define(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, doReset) =>
                {
                    if(!doReset)
                    {
                        return;
                    }
                    sourceEnabled = false;
                    SetEhrValid(false);
                    prngState = seed;
                    RefillEntropy();
                })
                .WithReservedBits(1, 31);

            // RNG_BUSY: always reports not-busy (sampling is instantaneous in
            // virtual time by default).
            Registers.RngBusy.Define(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => false)
                .WithReservedBits(1, 31);

            Registers.RngClkEnable.Define(this)
                .WithFlag(0, valueProviderCallback: _ => clockEnabled,
                    writeCallback: (_, value) => clockEnabled = value)
                .WithReservedBits(1, 31);

            Registers.RngDmaEnable.Define(this)
                .WithFlag(0, valueProviderCallback: _ => dmaEnabled,
                    writeCallback: (_, value) => dmaEnabled = value)
                .WithReservedBits(1, 31);
        }

        // Arm a fresh sample: regenerate the entropy words and assert
        // EHR_VALID, optionally after SamplingDelayUs of virtual time.
        private void ArmSample()
        {
            RefillEntropy();
            if(SamplingDelayUs == 0)
            {
                SetEhrValid(true);
                return;
            }
            SetEhrValid(false);
            machine.ScheduleAction(
                TimeInterval.FromMicroseconds(SamplingDelayUs),
                _ =>
                {
                    if(sourceEnabled && clockEnabled)
                    {
                        SetEhrValid(true);
                    }
                });
        }

        // Assert/deassert RNG_ISR.EHR_VALID (0x104 bit0) together with the
        // HOST_RGF RNG interrupt bit (HOST_IRR bit10, RngInt). The CC3xx
        // entropy collector waits on the HOST_IRR bit and reads EHR_VALID; both
        // must move together so the wait returns "RNG ready", not "SYM DMA".
        private void SetEhrValid(bool valid)
        {
            ehrValid = valid;
            if(valid)
            {
                hostIrr |= RngInt;
            }
            else
            {
                hostIrr &= ~RngInt;
            }
        }

        private void RefillEntropy()
        {
            for(var i = 0; i < EhrWordCount; i++)
            {
                ehrData[i] = NextRandom();
            }
        }

        // SplitMix64 — a small, well-distributed seeded PRNG. The upper 32
        // bits feed each EHR word (they mix better than the low bits).
        private uint NextRandom()
        {
            prngState += 0x9E3779B97F4A7C15UL;
            var z = prngState;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            z ^= z >> 31;
            return (uint)(z >> 32);
        }

        private readonly ulong[] ehrData = new ulong[EhrWordCount];
        private ulong seed;
        private ulong prngState;
        private bool clockEnabled;
        private bool dmaEnabled;
        private bool sourceEnabled;
        private bool ehrValid;
        private uint interruptMask;
        private uint trngConfig;
        private uint sampleCount;
        private uint debugControl;

        private const int EhrWordCount = 6;
        private const ulong DefaultSeed = 0x5340CC3105340CC3UL;

        // ====================================================================
        // == CC3xx symmetric engine (AES / HASH / DMA / HOST_RGF) ============
        // ====================================================================
        //
        // Register map confirmed against BOTH the shipped firmware disassembly
        // (G1disasm2: cc_aes_process/ProcessAesDrv @0x7b694+, ProcessHashDrv
        // @0x7b1e4, InitAes @0x7b468, FinishHashDrv @0x7b3f4, and the HAL
        // helpers CC_HalClearInterruptBit @0x7a290 / CC_HalMaskInterrupt
        // @0x7a2ac / the HOST_IRR wait loop @0x7a374) AND the public ARM
        // CryptoCell-312 Technical Reference Manual (HOST_RGF IRR/IMR/ICR,
        // the CRY_KERNEL AES_KEY/AES_IV/AES_CTR/AES_CONTROL bank, the HASH
        // H0..H7/CONTROL/CUR_LEN bank, and the DIN/DOUT DLLI descriptor
        // registers). Offsets are relative to the CC core base 0x50845000.
        //
        //   HOST_RGF   0xA00 HOST_IRR  (bit8 = SYM_DMA_COMPLETED, polled)
        //              0xA04 HOST_IMR  (interrupt mask)
        //              0xA08 HOST_ICR  (write-1-to-clear IRR)
        //   busy       0x910 CRYPTO_BUSY (reads 0 — engine idle, ops finish
        //                     synchronously in virtual time)
        //   engine     0x900 CRYPTO_CTL: 1 = AES, 7 = HASH (selects datapath)
        //   AES key    0x400..0x41C AES_KEY0[0..7]  (up to 256-bit)
        //   AES iv     0x440..0x44C AES_IV0[0..3]    (CBC chain state)
        //   AES ctr    0x460..0x46C AES_CTR0[0..3]   (CTR counter state)
        //   AES ctrl   0x4C0 AES_CONTROL: bit0 = decrypt, bits[4:2] = mode
        //                    (0 ECB, 1 CBC, 2 CTR), bits[13:12] = key size
        //                    (0=128,1=192,2=256)
        //   HASH state 0x640..0x65C HASH_H0[0..7]    (SHA midstate in/out)
        //   HASH ctrl  0x7C0 HASH_CONTROL: 1 = SHA-1, 2 = SHA-256/224
        //   HASH len   0x7CC/0x7D0 HASH_CUR_LEN low/high (message bit length)
        //   HASH pad   0x684 / 0x7C8 auto-pad enable (append SHA padding)
        //   DIN  DLLI  0xC28 src addr / 0xC2C size ; MLLI 0xC30 / 0xC34
        //   DOUT DLLI  0xD28 dst addr / 0xD2C size ; MLLI 0xD30 / 0xD34
        //
        // Datapath: writing the DIN size register (0xC2C/0xC34) starts the
        // transfer, as on real hardware. The engine reads the input from
        // system RAM via the DIN descriptor (machine.GetSystemBus), runs a
        // REAL AES (System.Security.Cryptography) or a REAL SHA compression
        // (implemented below so the firmware-managed midstate in HASH_H0..7
        // is honoured for multi-block / interleaved hashing), writes the
        // result back via the DOUT descriptor, updates the CBC IV / CTR
        // counter registers for chaining, and latches HOST_IRR.
        //
        // Completion: the firmware's symmetric-DMA wait
        // (cc_crypto_dispatch_or_error @0x7a2b8 -> the HOST_IRR poll @0x7a3a4)
        // waits for HOST_IRR bit11 (0x800, SYM_DMA_COMPLETED) and treats bit8
        // (0x100) as an abort/error indicator: the poll checks bit8 FIRST and,
        // if set, returns a non-zero (error) status that ProcessHashDrv reports
        // as a hash failure. So the symmetric engine latches ONLY bit11 on a
        // successful op and never bit8. Because every op completes within its
        // triggering MMIO write (virtual time is not advanced), bit11 is
        // asserted synchronously. HOST_ICR (0xA08) write-1-to-clear clears it.
        // (The RNG interrupt is a separate bit, RngInt = 0x400; see SetEhrValid.)
        //
        // Key source: the observed AES path loads the key words directly into
        // AES_KEY0 from firmware memory (cc_aes_process @0x7b854/0x7b94a), so
        // no KMU key-pull is exercised; the KMU is therefore not modelled.

        private uint CryptoRead(long offset)
        {
            switch(offset)
            {
            case HostIrr:
                // Report the genuinely-pending HOST_RGF interrupt sources:
                // bit11 (SymDmaCompleted, 0x800) latched by the symmetric engine
                // on AES/HASH DMA completion, and bit10 (RngInt, 0x400) latched
                // when the TRNG has a fresh EHR sample. Both are write-1-to-clear
                // via HOST_ICR. bit8 (0x100) is an abort/error indicator and is
                // NEVER asserted on success.
                //
                // This must NOT force any bit on unconditionally: the firmware's
                // interrupt polls (CC_HalWaitInterruptRND for RNG @0x7a368,
                // cc_crypto_dispatch_or_error for SYM @0x7a3a4) both test bit8
                // FIRST and, if it is set, return an error status. Forcing bit8
                // made the TRNG entropy collector loop without reading EHR_DATA
                // (CC_RND error 0x00F10C32) and the entropy-pool hash report a
                // failure (-55), so the crypto@50844000 CTR-DRBG entropy device
                // init returned -EINVAL and Zephyr panicked (FATAL ERROR 4).
                return hostIrr;
            case CryptoBusy:
                return 0u;
            default:
                if(cryptoRegs.TryGetValue(offset, out var v))
                {
                    return v;
                }
                // Genuinely-unimplemented read: log once-ish, return 0. These
                // are version/probe reads the firmware does not gate boot on
                // (e.g. the 0x1E34 HW-signature probe during CC_LibInit).
                this.Log(LogLevel.Noisy,
                    "UNIMPL periph=CC3XX dir=R width=4 offset=0x{0:X} value=0x00000000", offset);
                return 0u;
            }
        }

        private void CryptoWrite(long offset, uint value)
        {
            switch(offset)
            {
            case HostIcr:
                // Write-1-to-clear the latched interrupt-request bits.
                hostIrr &= ~value;
                return;
            case HashPadFinalize: // 0x7C8: value 4 = pad-only finalize
                cryptoRegs[offset] = value;
                if(value == HashPadFinalizeValue)
                {
                    RunHash(dinAddr: 0, size: 0, pad: true);
                    hostIrr |= SymDmaCompleted;
                }
                return;
            case DinSizeDlli:
                cryptoRegs[offset] = value;
                if(value != 0)
                {
                    RunSymmetric(GetReg(DinAddrDlli), value);
                }
                return;
            case DinSizeMlli:
                cryptoRegs[offset] = value;
                if(value != 0)
                {
                    RunSymmetric(GetReg(DinAddrMlli), value);
                }
                return;
            default:
                cryptoRegs[offset] = value;
                return;
            }
        }

        private uint GetReg(long offset)
        {
            return cryptoRegs.TryGetValue(offset, out var v) ? v : 0u;
        }

        // Kick the selected engine for a DIN transfer of `size` bytes.
        private void RunSymmetric(uint dinAddr, uint size)
        {
            var engine = GetReg(CryptoCtl);
            if(engine == HashEngineSelect)
            {
                RunHash(dinAddr, size, pad: GetReg(HashPadEnable) != 0);
            }
            else
            {
                RunAes(dinAddr, size);
            }
            hostIrr |= SymDmaCompleted;
        }

        private void RunAes(uint dinAddr, uint size)
        {
            var ctrl = GetReg(AesControl);
            var decrypt = (ctrl & 0x1u) != 0;
            var mode = (int)((ctrl >> 2) & 0x7u);
            var keySizeSel = (int)((ctrl >> 12) & 0x3u);
            var keyLen = keySizeSel == 0 ? 16 : (keySizeSel == 1 ? 24 : 32);

            var key = WordsToBytes(AesKey0, keyLen / 4);
            var input = machine.GetSystemBus(this).ReadBytes(dinAddr, (int)size);

            var doutAddr = GetReg(DoutAddrDlli);
            if(doutAddr == 0)
            {
                doutAddr = GetReg(DoutAddrMlli);
            }

            byte[] output;
            switch(mode)
            {
            case AesModeEcb:
                output = AesEcb(key, input, decrypt);
                break;
            case AesModeCbc:
            {
                var iv = WordsToBytes(AesIv0, 4);
                output = AesCbc(key, iv, input, decrypt, out var nextIv);
                BytesToWords(AesIv0, nextIv); // chain: IV = last cipher block
                break;
            }
            case AesModeCtr:
            {
                var ctr = WordsToBytes(AesCtr0, 4);
                output = AesCtr(key, ctr, input, out var nextCtr);
                BytesToWords(AesCtr0, nextCtr);
                break;
            }
            default:
                this.Log(LogLevel.Warning,
                    "UNIMPL periph=CC3XX AES mode={0} (unhandled), treating as ECB", mode);
                output = AesEcb(key, input, decrypt);
                break;
            }

            if(doutAddr != 0 && output.Length > 0)
            {
                machine.GetSystemBus(this).WriteBytes(output, doutAddr);
            }
        }

        private void RunHash(uint dinAddr, uint size, bool pad)
        {
            var hashType = GetReg(HashControl);
            var isSha1 = hashType == HashSelectSha1;
            var stateWords = isSha1 ? 5 : 8;

            var state = new uint[stateWords];
            for(var i = 0; i < stateWords; i++)
            {
                state[i] = GetReg(HashH0 + 4 * i);
            }

            var data = size > 0
                ? machine.GetSystemBus(this).ReadBytes(dinAddr, (int)size)
                : new byte[0];

            if(pad)
            {
                var totalBits = ((ulong)GetReg(HashCurLenHigh) << 32) | GetReg(HashCurLenLow);
                if(totalBits == 0)
                {
                    totalBits = (ulong)data.Length * 8;
                }
                data = AppendShaPadding(data, totalBits);
            }

            for(var off = 0; off + 64 <= data.Length; off += 64)
            {
                if(isSha1)
                {
                    Sha1Compress(state, data, off);
                }
                else
                {
                    Sha256Compress(state, data, off);
                }
            }

            for(var i = 0; i < stateWords; i++)
            {
                cryptoRegs[HashH0 + 4 * i] = state[i];
            }
        }

        // ---- helpers: register <-> byte marshalling ------------------------
        // The CC register words hold the key/IV/data in native little-endian
        // word order (the driver copies 32-bit words straight from the byte
        // buffers in RAM on this LE core), so a word maps to 4 LE bytes.

        private byte[] WordsToBytes(long baseOffset, int wordCount)
        {
            var bytes = new byte[wordCount * 4];
            for(var i = 0; i < wordCount; i++)
            {
                var w = GetReg(baseOffset + 4 * i);
                bytes[4 * i + 0] = (byte)w;
                bytes[4 * i + 1] = (byte)(w >> 8);
                bytes[4 * i + 2] = (byte)(w >> 16);
                bytes[4 * i + 3] = (byte)(w >> 24);
            }
            return bytes;
        }

        private void BytesToWords(long baseOffset, byte[] bytes)
        {
            for(var i = 0; i * 4 + 3 < bytes.Length; i++)
            {
                cryptoRegs[baseOffset + 4 * i] = (uint)(bytes[4 * i]
                    | (bytes[4 * i + 1] << 8)
                    | (bytes[4 * i + 2] << 16)
                    | (bytes[4 * i + 3] << 24));
            }
        }

        // ---- AES primitives (real, via System.Security.Cryptography) -------

        private static byte[] AesEcb(byte[] key, byte[] input, bool decrypt)
        {
            using(var aes = Aes.Create())
            {
                aes.Key = key;
                aes.Mode = CipherMode.ECB;
                aes.Padding = PaddingMode.None;
                var xform = decrypt ? aes.CreateDecryptor() : aes.CreateEncryptor();
                return xform.TransformFinalBlock(input, 0, input.Length);
            }
        }

        private static byte[] AesCbc(byte[] key, byte[] iv, byte[] input, bool decrypt, out byte[] nextIv)
        {
            using(var aes = Aes.Create())
            {
                aes.Key = key;
                aes.Mode = CipherMode.CBC;
                aes.Padding = PaddingMode.None;
                aes.IV = iv;
                var xform = decrypt ? aes.CreateDecryptor() : aes.CreateEncryptor();
                var output = xform.TransformFinalBlock(input, 0, input.Length);
                // Chain: next IV is the last ciphertext block.
                nextIv = (byte[])iv.Clone();
                if(input.Length >= 16)
                {
                    var src = decrypt ? input : output;
                    Array.Copy(src, src.Length - 16, nextIv, 0, 16);
                }
                return output;
            }
        }

        // CTR: keystream = ECB-encrypt(counter); XOR with data; counter++ per
        // block. Big-endian 128-bit counter increment (CC3xx convention).
        private static byte[] AesCtr(byte[] key, byte[] counter, byte[] input, out byte[] nextCounter)
        {
            var output = new byte[input.Length];
            var ctr = (byte[])counter.Clone();
            using(var aes = Aes.Create())
            {
                aes.Key = key;
                aes.Mode = CipherMode.ECB;
                aes.Padding = PaddingMode.None;
                using(var enc = aes.CreateEncryptor())
                {
                    for(var off = 0; off < input.Length; off += 16)
                    {
                        var ks = enc.TransformFinalBlock(ctr, 0, 16);
                        var n = Math.Min(16, input.Length - off);
                        for(var i = 0; i < n; i++)
                        {
                            output[off + i] = (byte)(input[off + i] ^ ks[i]);
                        }
                        IncrementBigEndian(ctr);
                    }
                }
            }
            nextCounter = ctr;
            return output;
        }

        private static void IncrementBigEndian(byte[] counter)
        {
            for(var i = counter.Length - 1; i >= 0; i--)
            {
                if(++counter[i] != 0)
                {
                    break;
                }
            }
        }

        // ---- SHA padding + compression (real) ------------------------------

        private static byte[] AppendShaPadding(byte[] data, ulong totalBits)
        {
            var padded = new List<byte>(data);
            padded.Add(0x80);
            while(padded.Count % 64 != 56)
            {
                padded.Add(0x00);
            }
            for(var i = 7; i >= 0; i--)
            {
                padded.Add((byte)(totalBits >> (8 * i)));
            }
            return padded.ToArray();
        }

        private static uint Ror(uint x, int n) => (x >> n) | (x << (32 - n));

        private static void Sha256Compress(uint[] h, byte[] block, int off)
        {
            var w = new uint[64];
            for(var t = 0; t < 16; t++)
            {
                w[t] = (uint)((block[off + 4 * t] << 24) | (block[off + 4 * t + 1] << 16)
                    | (block[off + 4 * t + 2] << 8) | block[off + 4 * t + 3]);
            }
            for(var t = 16; t < 64; t++)
            {
                var s0 = Ror(w[t - 15], 7) ^ Ror(w[t - 15], 18) ^ (w[t - 15] >> 3);
                var s1 = Ror(w[t - 2], 17) ^ Ror(w[t - 2], 19) ^ (w[t - 2] >> 10);
                w[t] = w[t - 16] + s0 + w[t - 7] + s1;
            }
            uint a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
            for(var t = 0; t < 64; t++)
            {
                var s1 = Ror(e, 6) ^ Ror(e, 11) ^ Ror(e, 25);
                var ch = (e & f) ^ ((~e) & g);
                var t1 = hh + s1 + ch + K256[t] + w[t];
                var s0 = Ror(a, 2) ^ Ror(a, 13) ^ Ror(a, 22);
                var maj = (a & b) ^ (a & c) ^ (b & c);
                var t2 = s0 + maj;
                hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
            }
            h[0] += a; h[1] += b; h[2] += c; h[3] += d;
            h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
        }

        private static void Sha1Compress(uint[] h, byte[] block, int off)
        {
            var w = new uint[80];
            for(var t = 0; t < 16; t++)
            {
                w[t] = (uint)((block[off + 4 * t] << 24) | (block[off + 4 * t + 1] << 16)
                    | (block[off + 4 * t + 2] << 8) | block[off + 4 * t + 3]);
            }
            for(var t = 16; t < 80; t++)
            {
                w[t] = Rol(w[t - 3] ^ w[t - 8] ^ w[t - 14] ^ w[t - 16], 1);
            }
            uint a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
            for(var t = 0; t < 80; t++)
            {
                uint f, k;
                if(t < 20) { f = (b & c) | ((~b) & d); k = 0x5A827999; }
                else if(t < 40) { f = b ^ c ^ d; k = 0x6ED9EBA1; }
                else if(t < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
                else { f = b ^ c ^ d; k = 0xCA62C1D6; }
                var tmp = Rol(a, 5) + f + e + k + w[t];
                e = d; d = c; c = Rol(b, 30); b = a; a = tmp;
            }
            h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
        }

        private static uint Rol(uint x, int n) => (x << n) | (x >> (32 - n));

        // ---- symmetric-engine state ----------------------------------------

        private readonly Dictionary<long, uint> cryptoRegs = new Dictionary<long, uint>();
        private uint hostIrr;

        // HOST_RGF interrupt-request bits. The firmware's completion polls
        // (@0x7a368 RNG, @0x7a3a4 SYM) test bit8 (0x100) first as an abort/error
        // flag, then the per-op completion bit below; bit8 is never asserted on
        // success. Confirmed against the shipped disassembly: the SYM DMA wait
        // uses mask 0x800 and the RNG wait uses mask 0x400.
        private const uint SymDmaCompleted = 0x800; // bit11 SYM_DMA_COMPLETED
        // RNG interrupt bit polled by the CC3xx TRNG entropy collector
        // (WaitRngInterrupt passes mask 0x400 to CC_HalWaitInterruptRND). It is
        // latched together with RNG_ISR.EHR_VALID and cleared via HOST_ICR.
        private const uint RngInt = 0x400; // bit10

        // Engine select values written to CRYPTO_CTL (0x900).
        private const uint HashEngineSelect = 7;
        private const uint HashPadFinalizeValue = 4;

        // HASH_CONTROL (0x7C0) values.
        private const uint HashSelectSha1 = 1; // 2 = SHA-256/224

        private const int AesModeEcb = 0;
        private const int AesModeCbc = 1;
        private const int AesModeCtr = 2;

        // Symmetric-engine register offsets (from CC core base 0x50845000).
        private const long AesKey0 = 0x400;      // ..0x41C
        private const long AesIv0 = 0x440;       // ..0x44C
        private const long AesCtr0 = 0x460;      // ..0x46C
        private const long AesControl = 0x4C0;
        private const long HashH0 = 0x640;       // ..0x65C
        private const long HashPadEnable = 0x684;
        private const long CryptoCtl = 0x900;
        private const long CryptoBusy = 0x910;
        private const long HostIrr = 0xA00;
        private const long HostImr = 0xA04;
        private const long HostIcr = 0xA08;
        private const long HashControl = 0x7C0;
        private const long HashPadFinalize = 0x7C8;
        private const long HashCurLenLow = 0x7CC;
        private const long HashCurLenHigh = 0x7D0;
        private const long DinAddrDlli = 0xC28;
        private const long DinSizeDlli = 0xC2C;
        private const long DinAddrMlli = 0xC30;
        private const long DinSizeMlli = 0xC34;
        private const long DoutAddrDlli = 0xD28;
        private const long DoutAddrMlli = 0xD30;

        private static readonly uint[] K256 =
        {
            0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
        };

        // TRNG register offsets served by the base register collection.
        private static readonly HashSet<long> TrngRegisterOffsets = new HashSet<long>
        {
            0x100, 0x104, 0x108, 0x10C, 0x110,
            0x114, 0x118, 0x11C, 0x120, 0x124, 0x128,
            0x12C, 0x130, 0x138, 0x140, 0x1B8, 0x1C4, 0x1C8,
        };

        private enum Registers : long
        {
            RngImr = 0x100,
            RngIsr = 0x104,
            RngIcr = 0x108,
            TrngConfig = 0x10C,
            TrngValid = 0x110,
            EhrData0 = 0x114, // ..0x128 (EHR_DATA_0..5)
            RndSourceEnable = 0x12C,
            SampleCnt1 = 0x130,
            TrngDebugControl = 0x138,
            RngSwReset = 0x140,
            RngBusy = 0x1B8,
            RngClkEnable = 0x1C4,
            RngDmaEnable = 0x1C8,
        }
    }
}
