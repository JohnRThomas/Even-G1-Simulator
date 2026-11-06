#!/usr/bin/env python3
"""Find where a G1 app-core image keeps the state the browser front end shows.

    g1_state.py application_update.zip
    g1_state.py app_update.bin

g1_sim_web.py reads a lens's role, ESB pairing, MACs and case battery level
out of the firmware's RAM. Where those are belongs to one particular build,
so they are looked up in each image. This script does that without symbols,
from things that survive a rebuild:

  GLOBAL_STATE        the address of the pointer to the firmware's state: the
                      one returned by the one-load getter called most by the
                      function that loads the "%s_%d_R_%02X%02X%02X"
                      advertising-name format (bt_start, get_dashboard_state)
  master_mac offset   the three consecutive byte offsets that function reads
                      off the state for the name's %02X%02X%02X; the slave
                      MAC follows the master MAC
  case battery byte   the byte written by the image's only setter of a
                      7-bit field (set_pt_level)

Not finding something is reported on stderr and makes the exit status 1;
nothing is guessed.

Python 3, standard library only.
"""
import argparse
import os
import struct
import sys
import zipfile

MCUBOOT_MAGIC = 0x96F3B83D
APP_VECTORS = 0x0000C200             # address of the image body on the G1
APP_RAM = (0x20000000, 0x20080000)

NAME_FORMAT = b"\0%s_%d_R_%02X%02X%02X\0"


def read_app_image(path):
    """Return the app-core image body (MCUboot header removed) from a zip,
    or from a .bin file. Raises ValueError if there is none."""
    if not os.path.isfile(path):
        raise ValueError("no such file: " + path)
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as archive:
            blobs = [archive.read(m) for m in archive.namelist() if m.lower().endswith(".bin")]
    else:
        with open(path, "rb") as f:
            blobs = [f.read()]
    for data in blobs:
        if len(data) >= 32 and struct.unpack_from("<I", data)[0] == MCUBOOT_MAGIC:
            data = data[struct.unpack_from("<H", data, 8)[0]:]
        if len(data) >= 8:
            sp, pc = struct.unpack_from("<II", data)
            if APP_RAM[0] < sp <= APP_RAM[1] and APP_VECTORS <= pc < APP_VECTORS + len(data):
                return data
    raise ValueError("no G1 app-core image in " + path)


class Image:
    """The app image as Thumb code at its run address."""

    def __init__(self, body):
        self.body = body
        self.base = APP_VECTORS
        self.end = self.base + (len(body) & ~1)
        self.halfwords = struct.unpack_from("<%dH" % (len(body) // 2), body)
        self.calls = []                      # (address of BL, target)
        hw = self.halfwords
        for i in range(len(hw) - 1):
            if hw[i] & 0xF800 == 0xF000 and hw[i + 1] & 0xD000 == 0xD000:
                s = (hw[i] >> 10) & 1
                i1 = 1 - (((hw[i + 1] >> 13) & 1) ^ s)
                i2 = 1 - (((hw[i + 1] >> 11) & 1) ^ s)
                offset = (i1 << 23) | (i2 << 22) | ((hw[i] & 0x3FF) << 12) | ((hw[i + 1] & 0x7FF) << 1)
                address = self.base + 2 * i
                target = address + 4 + offset - (s << 24)
                if self.base <= target < self.end:
                    self.calls.append((address, target))
        self.call_targets = {target for _, target in self.calls}

    def u16(self, address):
        return self.halfwords[(address - self.base) // 2]

    def u32(self, address):
        return struct.unpack_from("<I", self.body, address - self.base)[0]

    def string_address(self, fragment):
        """Address of the one string containing fragment (None if absent or
        ambiguous). A leading NUL in fragment anchors it to the string start."""
        at = self.body.find(fragment)
        if at < 0 or self.body.find(fragment, at + 1) >= 0:
            return None
        return self.base + self.body.rfind(b"\0", 0, at + 1) + 1

    def literal_loads(self, value):
        """Addresses of the 'ldr rX, [pc, #n]' instructions that load value
        from a literal pool."""
        loads = []
        needle = struct.pack("<I", value)
        at = self.body.find(needle)
        while at >= 0:
            pool = self.base + at
            if at % 4 == 0:
                for address in range(max(self.base, pool - 4096), pool, 2):
                    hw = self.u16(address)
                    origin = (address + 4) & ~3
                    if hw & 0xF800 == 0x4800 and origin + (hw & 0xFF) * 4 == pool:
                        loads.append(address)
                    elif hw == 0xF8DF and origin + (self.u16(address + 2) & 0xFFF) == pool:
                        loads.append(address)
            at = self.body.find(needle, at + 1)
        return sorted(loads)

    def function_start(self, address):
        """Start of the function containing address: the nearest preceding
        call target that begins with a push of LR."""
        for start in range(address & ~1, max(self.base, address - 0x4000), -2):
            hw = self.u16(start)
            pushes_lr = hw & 0xFF00 == 0xB500 or (hw == 0xE92D and self.u16(start + 2) & 0x4000)
            if pushes_lr and start in self.call_targets:
                return start
        return None

    def calls_between(self, start, end):
        return [target for address, target in self.calls if start <= address < end]

    def ram_literal(self, load_address):
        """The RAM address loaded by the 'ldr rX, [pc, #n]' at load_address."""
        pool = ((load_address + 4) & ~3) + (self.u16(load_address) & 0xFF) * 4
        if pool + 4 > self.base + len(self.body):
            return None
        literal = self.u32(pool)
        return literal if APP_RAM[0] <= literal < APP_RAM[1] else None

    def pointer_getter(self, function):
        """If function is 'ldr rX, =LIT ; ldr r0, [rX] ; bx lr', return LIT."""
        load = self.u16(function)
        reg = (load >> 8) & 7
        if (load & 0xF800 != 0x4800 or self.u16(function + 2) != 0x6800 | reg << 3
                or self.u16(function + 4) != 0x4770):
            return None
        return self.ram_literal(function)

    def seven_bit_setters(self):
        """RAM bytes written by 'ldr rX, =LIT ; ldrb rY, [rX] ;
        bfi rY, r0, #0, #7 ; strb rY, [rX]': a setter of a 7-bit field."""
        hw = self.halfwords
        literals = set()
        for i in range(len(hw) - 4):
            if hw[i] & 0xF800 == 0x4800 and hw[i + 2] == 0xF360:
                x, y = (hw[i] >> 8) & 7, hw[i + 1] & 7
                if (hw[i + 1] == 0x7800 | x << 3 | y and hw[i + 3] == y << 8 | 0x0006
                        and hw[i + 4] == 0x7000 | x << 3 | y):
                    literal = self.ram_literal(self.base + 2 * i)
                    if literal is not None:
                        literals.add(literal)
        return literals


def find_state(image):
    """Return (found, problems): found maps a name to its value, problems
    lists what could not be located and why."""
    found, problems = {}, []

    # bt_start: the one function that loads the advertising-name format.
    bt_start = name_load = None
    string = image.string_address(NAME_FORMAT)
    if string is None:
        problems.append("bt_start: its format string is missing or not unique")
    else:
        loads = image.literal_loads(string)
        starts = {image.function_start(load) for load in loads}
        if len(starts) != 1 or None in starts:
            problems.append("bt_start: %d code references to its format string, in %d functions"
                            % (len(loads), len(starts)))
        else:
            bt_start, name_load = starts.pop(), loads[-1]

    if bt_start is not None:
        getters = {}
        for target in image.calls_between(bt_start, name_load):
            pointer = image.pointer_getter(target)
            if pointer is not None:
                getters.setdefault(pointer, []).append(target)
        if getters:
            found["GLOBAL_STATE"] = max(getters, key=lambda p: len(getters[p]))
        else:
            problems.append("GLOBAL_STATE: bt_start calls no pointer getter")
        # ldrb.w rT, [r0, #imm12] straight after a call: fields of the state.
        offsets = {image.u16(a + 2) & 0xFFF for a in range(bt_start, name_load, 2)
                   if image.u16(a) == 0xF890}
        triples = [o for o in sorted(offsets) if o >= 0x100 and {o + 1, o + 2} <= offsets]
        if len(triples) == 1:
            found["master_mac"] = triples[0]
        else:
            problems.append("master_mac offset: %d candidate byte triples in bt_start" % len(triples))

    setters = image.seven_bit_setters()
    if len(setters) == 1:
        found["case_battery"] = setters.pop()
    else:
        problems.append("case battery byte: %d candidate setters" % len(setters))
    return found, problems


def locate(path):
    """find_state for the app image in a zip or .bin file."""
    try:
        return find_state(Image(read_app_image(path)))
    except ValueError as error:
        return {}, [str(error)]


def main():
    parser = argparse.ArgumentParser(
        description="Find where a G1 app-core image keeps the state the browser "
                    "front end shows.")
    parser.add_argument("firmware", metavar="FIRMWARE", help="an update zip or the app-core .bin")
    args = parser.parse_args()

    found, problems = locate(args.firmware)
    for name, form in (("GLOBAL_STATE", "0x%08x"), ("master_mac", "+0x%x"),
                       ("case_battery", "0x%08x")):
        print("%-14s %s" % (name, form % found[name] if name in found else "not found"))
    for problem in problems:
        print("g1_state: " + problem, file=sys.stderr)
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
