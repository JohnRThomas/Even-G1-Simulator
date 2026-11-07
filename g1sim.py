#!/usr/bin/env python3
"""Run prebuilt Even Realities G1 firmware in the Renode nRF5340 emulator.

    g1sim.py application_update.zip
    g1sim.py app_update.bin net_core_app_update.bin
    g1sim.py master_update.zip slave_update.zip

A firmware is either an update zip holding the two images, or the two .bin
files themselves (in either order). Nothing is built or flashed: the images
are loaded into the emulated cores as they are.

The firmware runs on both lenses of the pair, the master and the slave. If
two firmwares are given, the first runs on the master lens and the second on
the slave lens.

Without --seconds the firmware runs until you type "quit" at the Renode
monitor prompt; UART output of both cores of both lenses is printed as log
lines ("g1/uart0" is the master's app core and "g1/uart_net" its net core;
the slave's are "g1_slave/uart0" and "g1_slave/uart_net").

Each lens keeps its external flash in a file, data/g1_extstore_left.bin for
the master (left) lens and data/g1_extstore_right.bin for the slave (right)
lens, so what the firmware stores there is still there on the next run.
Delete a file to give that lens a factory-fresh flash again.

Each lens has its own MAC, which --mac-master and --mac-slave change; the
last three bytes end the name the lens advertises under.

Needs Python 3 (standard library only) and Renode. Renode is taken from
--renode, $RENODE, the copy that setup.sh puts into this folder, or PATH, in
that order.
"""
import argparse
import fcntl
import hashlib
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
RESC = os.path.join(HERE, "g1.resc")
LENS_RESC = os.path.join(HERE, "g1_lens.resc")
MASTER, SLAVE = "g1", "g1_slave"      # machine names of the two lenses
LENS_NAMES = {MASTER: "master", SLAVE: "slave"}

# Each lens's MAC unless --mac-master / --mac-slave give another. The firmware
# reads it from the net core's FICR and ends its advertising name with the
# last three bytes: "Even G1_34_R_2970D9" and "Even G1_34_L_5BEB85" (34 is
# the ESB channel in the settings seed).
DEFAULT_MAC = {MASTER: "E9:A4:9E:29:70:D9", SLAVE: "F2:65:0B:5B:EB:85"}

# The file holding each lens's external flash, a raw image of the 32 MiB
# chip: the master is the left lens, the slave the right one. Renode creates
# a missing file from the settings seed and writes every change through.
EXTSTORE = {MASTER: os.path.join(HERE, "data", "g1_extstore_left.bin"),
            SLAVE: os.path.join(HERE, "data", "g1_extstore_right.bin")}
LOCAL_RENODE = os.path.join(HERE, ".renode", "renode")

# The emulation's time quantum: how far a core runs before the cores are
# brought back in step. Within a quantum a core's timers can run late by up to
# that long while another core is busy. The SoftDevice Controller checks its
# radio timing against a timer and stops (the lens then reboots) if it finds
# itself late by more than some tens of microseconds, which a connection with
# a phone, with its 150 us turnarounds, would run into at Renode's default of
# 100 us. A lens that only advertises does not, so the shorter quantum, which
# costs about a third of the speed, is only used while a phone is switched on.
QUANTUM = "0.0001"
PHONE_QUANTUM = "0.00005"

# G1 internal-flash layout. It is fixed by the bootloaders on the device, so
# an image that does not fit it would not boot on real glasses either.
MCUBOOT_MAGIC = 0x96F3B83D
APP_HEADER_SIZE = 0x200          # app slot 0xC000, vector table at 0xC200
APP_CODE = (0x0000C200, 0x00100000)
APP_RAM = (0x20000000, 0x20080000)
NET_CODE = (0x01008800, 0x01040000)
NET_RAM = (0x21000000, 0x21010000)


def die(message):
    sys.exit("g1sim: " + message)


def read_images(paths):
    """Return [(name, bytes)] for every candidate firmware image named on the
    command line: the .bin members of a zip, or the files themselves."""
    images = []
    for path in paths:
        if not os.path.isfile(path):
            die("no such file: " + path)
        if zipfile.is_zipfile(path):
            with zipfile.ZipFile(path) as archive:
                for member in archive.namelist():
                    if member.lower().endswith(".bin"):
                        images.append((member, archive.read(member)))
        else:
            with open(path, "rb") as f:
                images.append((os.path.basename(path), f.read()))
    return images


def split_header(data):
    """Split an image into (MCUboot header, rest). The header is empty for a
    raw image that starts with its vector table."""
    if len(data) >= 32 and struct.unpack_from("<I", data)[0] == MCUBOOT_MAGIC:
        header_size = struct.unpack_from("<H", data, 8)[0]
        return data[:header_size], data[header_size:]
    return b"", data


def which_core(body):
    """Identify an image by the initial SP and reset vector at its start."""
    if len(body) < 8:
        return None
    sp, pc = struct.unpack_from("<II", body)
    for core, ram, code in (("app", APP_RAM, APP_CODE), ("net", NET_RAM, NET_CODE)):
        if ram[0] < sp <= ram[1] and code[0] <= pc < code[1]:
            return core
    return None


def sort_images(paths):
    """Return the firmwares named on the command line, one or two, each as
    {"app": (name, header, body), "net": ...}. The first image of each core
    belongs to the first firmware, the next one to the second."""
    firmwares = [{}, {}]
    for name, data in read_images(paths):
        header, body = split_header(data)
        core = which_core(body)
        if core is None:
            die("%s is not a G1 app-core or net-core image "
                "(its vector table does not match the G1 flash layout)" % name)
        for firmware in firmwares:
            if core not in firmware:
                firmware[core] = (name, header, body)
                break
        else:
            die("%s is a third %s-core image; give one firmware, or one for "
                "each of the two lenses" % (name, core))
    first, second = firmwares
    for core in ("app", "net"):
        if core not in first:
            die("no %s-core image given; pass an update zip or both .bin files" % core)
    if not second:
        return [first]
    for core, other in (("app", "net"), ("net", "app")):
        if core not in second:
            die("the second firmware has no %s-core image, only the %s-core image %s"
                % (core, other, second[other][0]))
    return [first, second]


def prepare_firmware(paths, workdir):
    """Write the images named on the command line into workdir in the form
    g1_lens.resc expects. Returns the firmware of each lens, {MASTER: lens,
    SLAVE: lens}, a lens being {"app_bin": path, "net_bin": path, "app":
    one-line description, "net": the same}. With one firmware given, both
    lenses run it."""
    firmwares = sort_images(paths)
    lenses = {}
    for machine, firmware, prefix in zip((MASTER, SLAVE), firmwares, ("", "slave_")):
        # Which lens, in what is printed: nothing while both run the same.
        who = "" if len(firmwares) == 1 else LENS_NAMES[machine] + " lens, "
        lens = {}
        name, header, body = firmware["app"]
        if len(header) > APP_HEADER_SIZE:
            die("%s: MCUboot header is 0x%x bytes, the G1 app slot has room for 0x%x"
                % (name, len(header), APP_HEADER_SIZE))
        lens["app_bin"] = os.path.join(workdir, prefix + "app.bin")
        with open(lens["app_bin"], "wb") as f:
            f.write(b"\xff" * (APP_HEADER_SIZE - len(header)) + header + body)
        lens["net_bin"] = os.path.join(workdir, prefix + "net.bin")
        with open(lens["net_bin"], "wb") as f:
            f.write(firmware["net"][2])
        for core in ("app", "net"):
            name, header, body = firmware[core]
            lens[core] = "%s (%d bytes, sha256 %s)" % (
                name, len(header) + len(body), hashlib.sha256(header + body).hexdigest()[:16])
            print("g1sim: %s%s core: %s" % (who, core, lens[core]))
        lenses[machine] = lens
    lenses.setdefault(SLAVE, lenses[MASTER])
    return lenses


def find_renode(explicit):
    for candidate in (explicit, os.environ.get("RENODE"),
                      LOCAL_RENODE if os.path.isfile(LOCAL_RENODE) else None,
                      shutil.which("renode")):
        if candidate:
            if not os.path.isfile(candidate):
                die("Renode not found at " + candidate)
            return candidate
    die("Renode not found. Run ./setup.sh, or install Renode 1.16+ and put it "
        "on PATH or in $RENODE.")


def extstore(machine):
    """The file holding a lens's external flash. Says what the lens starts
    from, and refuses to start while another simulator has the file open:
    two of them writing one flash would leave neither with what it wrote."""
    path, who = EXTSTORE[machine], LENS_NAMES[machine]
    if os.path.isfile(path) and os.path.getsize(path):
        with open(path, "rb") as f:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                die("%s, the %s lens's flash, is in use by another running simulator"
                    % (path, who))
        print("g1sim: %s lens flash: %s, kept from an earlier run" % (who, path))
    else:
        print("g1sim: %s lens flash: %s, new" % (who, path))
    return path


def monitor_path(path):
    """A file path as a Renode monitor token."""
    return "@" + os.path.abspath(path).replace("\\", "/").replace(" ", "\\ ")


ANALYZERS = [
    "showAnalyzer sysbus.uart0 Antmicro.Renode.Analyzers.LoggingUartAnalyzer",
    "showAnalyzer sysbus.uart_net Antmicro.Renode.Analyzers.LoggingUartAnalyzer",
]


def for_lens(command):
    """Monitor commands for a --execute/--at command. A command starting with
    "slave:" is run on the slave lens's machine instead of the master's."""
    if command.startswith("slave:"):
        return ['mach set "%s"' % SLAVE, command[len("slave:"):].strip(), 'mach set "%s"' % MASTER]
    return [command]


def setup_lines(firmware, args):
    """Monitor commands that create the two lenses with their firmware (see
    prepare_firmware) and apply the options every front end shares, up to
    the point where the run starts."""
    if args.mac_master == args.mac_slave:
        die("the two lenses need different MACs (--mac-master, --mac-slave)")
    lines = ["i " + monitor_path(RESC)]
    for machine, address in ((MASTER, args.mac_master), (SLAVE, args.mac_slave)):
        lens = firmware[machine]
        # Set after "mach create", so that they are the lens's own.
        lines += ['mach create "%s"' % machine,
                  "$app_bin=" + monitor_path(lens["app_bin"]),
                  "$net_bin=" + monitor_path(lens["net_bin"]),
                  "$extstore=" + monitor_path(extstore(machine)),
                  "$slave=%s" % ("true" if machine == SLAVE else "false"),
                  "$deviceaddr0=0x%08X" % (address & 0xFFFFFFFF),
                  "$deviceaddr1=0x%08X" % (0xFFFF0000 | address >> 32),
                  "i " + monitor_path(LENS_RESC)] + ANALYZERS
    lines.append('mach set "%s"' % MASTER)
    if args.phone:                       # as the phone app: a link to each lens
        lines += ['emulation SetGlobalQuantum "%s"' % PHONE_QUANTUM,
                  "vcentral Enabled true"]
        lines += for_lens("slave: vcentral Enabled true")
    for command in args.execute:
        lines += for_lens(command)
    return lines


def write_session(firmware, workdir, args):
    lines = setup_lines(firmware, args)
    if args.seconds is not None:
        # Nobody is watching a timed run: let it go faster than real time,
        # which Renode otherwise holds the emulation back to.
        lines.append("emulation SetAdvanceImmediately true")
    now = 0.0
    for time, command in sorted(args.at, key=lambda event: event[0]):
        if time > now:
            lines.append('emulation RunFor "%s"' % (time - now))
            now = time
        lines += for_lens(command)
    if args.seconds is None:
        lines.append("start")
    else:
        if args.seconds > now:
            lines.append('emulation RunFor "%s"' % (args.seconds - now))
        if args.frame:
            lines.append("spim_a.jbd DumpFrame " + monitor_path(args.frame))   # the master's
        lines.append("quit")
    session = os.path.join(workdir, "session.resc")
    with open(session, "w") as f:
        f.write("\n".join(lines) + "\n")
    return session


def timed_command(text):
    """argparse type for --at: 'SECONDS COMMAND'."""
    time, _, command = text.strip().partition(" ")
    try:
        time = float(time)
    except ValueError:
        time = -1.0
    if time < 0 or not command.strip():
        raise argparse.ArgumentTypeError("expected 'SECONDS COMMAND', got %r" % text)
    return time, command.strip()


PACKET_WORD_RE = re.compile(r'"([^"]*)"|([^\s,]+)')


def parse_packet(text):
    """The bytes of a protocol packet as it is typed:

        0x3e - get burial point data
        26 06 00 00 06 01
        260600000601
        0x4b "some text" 0x00

    Hex bytes, each with or without 0x, separated by spaces or commas or
    run together; text in double quotes stands for its UTF-8 bytes; a lone
    "-" and all that follows it is a comment. Raises ValueError."""
    packet = bytearray()
    for quoted, word in PACKET_WORD_RE.findall(text):
        if word == "-":
            break
        if not word:
            packet += quoted.encode("utf-8")
            continue
        digits = word[2:] if word[:2].lower() == "0x" else word
        if len(digits) == 1:
            digits = "0" + digits
        try:
            packet += bytes.fromhex(digits)
        except ValueError:
            raise ValueError("not hex bytes: %r" % word)
    if not packet:
        raise ValueError("no bytes in %r" % text)
    return bytes(packet)


def mac_address(text):
    """A MAC written as E9:A4:9E:29:70:D9, as a number."""
    octets = text.split(":")
    try:
        if len(octets) != 6 or any(len(octet) != 2 for octet in octets):
            raise ValueError
        return int.from_bytes(bytes.fromhex("".join(octets)), "big")
    except ValueError:
        raise argparse.ArgumentTypeError("expected a MAC like E9:A4:9E:29:70:D9, got %r" % text)


def add_setup_arguments(parser):
    """The firmware and the options every front end shares (see setup_lines)."""
    parser.add_argument("firmware", nargs="+", metavar="FIRMWARE",
                        help="an update zip, or the app-core and net-core .bin files; "
                             "both lenses run it. A second firmware, given the same "
                             "way, runs on the slave lens instead")
    parser.add_argument("--phone", action="store_true",
                        help="switch the virtual phone on: it connects to each of "
                             "the two lenses over BLE and pairs with it")
    parser.add_argument("--execute", action="append", default=[], metavar="COMMAND",
                        help="Renode monitor command to run after the firmware is "
                             "loaded and before it starts (repeatable); prefix it "
                             "with 'slave:' to run it on the slave lens")
    for machine, option in ((MASTER, "--mac-master"), (SLAVE, "--mac-slave")):
        parser.add_argument(option, type=mac_address, metavar="MAC",
                            default=DEFAULT_MAC[machine],
                            help="MAC of the %s lens (default %s). The firmware sets "
                                 "the top two bits of its first byte, and ends the "
                                 "name it advertises under with the last three bytes"
                                 % (LENS_NAMES[machine], DEFAULT_MAC[machine]))
    parser.add_argument("--renode", metavar="PATH", help="Renode executable to use")


def main():
    parser = argparse.ArgumentParser(
        description="Run prebuilt Even Realities G1 firmware in the Renode nRF5340 emulator.")
    add_setup_arguments(parser)
    parser.add_argument("--seconds", type=float, metavar="N",
                        help="run for N seconds of emulated time, then exit "
                             "(default: run until 'quit' at the monitor prompt)")
    parser.add_argument("--frame", metavar="OUT.ppm",
                        help="with --seconds: save the master lens's display "
                             "framebuffer on exit")
    parser.add_argument("--at", action="append", default=[], type=timed_command,
                        metavar="'SECONDS COMMAND'",
                        help="Renode monitor command to run at that emulated time "
                             "(repeatable); 'slave:' prefix as for --execute")
    args = parser.parse_args()
    if args.frame and args.seconds is None:
        parser.error("--frame needs --seconds")
    if args.seconds is not None and any(time >= args.seconds for time, _ in args.at):
        parser.error("an --at time is not before the end of the run (--seconds)")

    renode = find_renode(args.renode)
    with tempfile.TemporaryDirectory(prefix="g1sim-") as workdir:
        firmware = prepare_firmware(args.firmware, workdir)
        session = write_session(firmware, workdir, args)
        sys.stdout.flush()
        process = subprocess.Popen(
            [renode, "--disable-xwt", "--console", "--plain", session])
        while True:
            try:
                return process.wait()
            except KeyboardInterrupt:      # Ctrl-C reaches Renode too; wait for it
                pass


if __name__ == "__main__":
    sys.exit(main())
