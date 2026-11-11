#!/usr/bin/env python3
"""Browser front end for the G1 simulator.

    g1_sim_web.py application_update.zip
    g1_sim_web.py app_update.bin net_core_app_update.bin
    g1_sim_web.py master_update.zip slave_update.zip

Starts the same emulation as g1sim.py (same firmware arguments and options)
and serves a control page on http://127.0.0.1:8340/ with pause/resume/reset
and one column for each lens of the pair, the master and the slave: its live
display, touch bar, state, both UART consoles, the virtual phone with a box
for the protocol packets it is to send, sensor inputs, a monitor command box
and the GPIO pins.

The server owns one headless Renode and talks to its monitor over a local
TCP socket; UART output is read from Renode's log. It listens on the loopback
interface only: the page can run any Renode monitor command.

Python 3, standard library only. Uses g1sim.py and g1_state.py next to it.
"""
import argparse
import json
import os
import re
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import g1sim
import g1_state

ANSI_RE = re.compile(rb"\x1b\[[0-9;?]*[A-Za-z]")
IAC_SUBNEG_RE = re.compile(rb"\xff\xfa.*?\xff\xf0", re.S)
IAC_RE = re.compile(rb"\xff[\xfb-\xfe].|\xff[\xf1-\xf9]")
PROMPT_RE = re.compile(rb"\([^)\n]{1,64}\) $")
# The log names the lens's machine: "g1/uart0", "g1_slave/uart0".
UART_LINE_RE = re.compile(
    r"\[INFO\]\s+(?:(\w+)/)?(?:sysbus\.)?(uart0|uart_net):\s+(?:\[[^\]]*\]\s?)?(.*)$")
# What a lens's virtual phone logs: "g1/vcentral: ..." (G1 protocol) or
# "g1/vcentral_g2: ..." (G2 protocol).
PHONE_LINE_RE = re.compile(r"\[(?:INFO|WARNING|ERROR)\]\s+(\w+)/vcentral(?:_g2)?: (.*)$")
VIRT_TIME_RE = re.compile(r"Elapsed Virtual Time:\s*([0-9:.,]+)")
BYTE_RE = re.compile(r"0x([0-9A-Fa-f]{2})\b")
COLOUR_RE = re.compile(r"\x1b?\[(?:\d+;)*\d+m")       # log colour codes in UART text

APP_RAM = (0x20000000, 0x20080000)

STRAP_PINS = (25, 26, 27, 28)
TOUCH = "sysbus.twim2.touch"
# The phone the page controls: the G1-protocol one unless the simulator was
# started with --phone-g2 (set in main).
PHONE = "sysbus." + g1sim.PHONE_G1
LENSES = {"master": g1sim.MASTER, "slave": g1sim.SLAVE}
CONSOLES = ("uart0", "uart_net", "phone", "slave_uart0", "slave_uart_net", "slave_phone")


class MonitorError(Exception):
    pass


def monitor_number(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    return repr(float(value))


def parse_scalar(text):
    """Best-effort parse of a value the monitor printed."""
    text = text.strip()
    if text in ("True", "true"):
        return True
    if text in ("False", "false"):
        return False
    for convert in (lambda t: int(t, 0), float):
        try:
            return convert(text)
        except ValueError:
            pass
    return text


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Renode:
    """The Renode process and its monitor session."""

    def __init__(self, executable, session, log_path):
        self.executable = executable
        self.session = session
        self.log_path = log_path
        self.port = free_port()
        self.process = None
        self.sock = None
        self.lock = threading.RLock()          # one monitor command at a time
        self.log_lock = threading.Lock()
        self.log_offset = 0
        self.log_partial = ""
        self.uart_lines = {name: [] for name in CONSOLES}

    def start(self, timeout=180.0):
        self.log_file = open(self.log_path, "wb")
        # With --port the monitor lives on the socket and the session script
        # given on the command line does the loading. stdin is kept open: a
        # closed one can make Renode's console reader see EOF and exit.
        self.process = subprocess.Popen(
            [self.executable, "--disable-xwt", "--plain", "--port", str(self.port), self.session],
            stdin=subprocess.PIPE, stdout=self.log_file, stderr=subprocess.STDOUT)
        deadline = time.time() + timeout
        while self.sock is None:
            if self.process.poll() is not None:
                raise MonitorError("Renode exited during startup")
            if time.time() > deadline:
                raise MonitorError("could not connect to the Renode monitor")
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=2)
                self.sock.settimeout(0.2)
            except OSError:
                time.sleep(0.2)
        self._read_until_prompt(60.0)          # banner and session echo
        while time.time() < deadline:          # until the machine answers
            try:
                if isinstance(self.get("sysbus.cpuapp", "PC"), int):
                    return
            except MonitorError:
                if not self.alive():
                    raise MonitorError("Renode exited during startup")
            time.sleep(0.5)
        raise MonitorError("the machine did not come up")

    def alive(self):
        return self.process is not None and self.process.poll() is None

    def shutdown(self):
        with self.lock:
            if self.process is None:
                return
            if self.alive() and self.sock is not None:
                try:
                    self.sock.sendall(b"quit\n")
                except OSError:
                    pass
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait()
            if self.sock is not None:
                self.sock.close()
                self.sock = None
            self.log_file.close()

    @staticmethod
    def _clean(raw):
        raw = IAC_SUBNEG_RE.sub(b"", raw)
        raw = IAC_RE.sub(b"", raw)
        raw = ANSI_RE.sub(b"", raw)
        return raw.replace(b"\r", b"")

    def _read_until_prompt(self, timeout):
        buffer = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise MonitorError("monitor socket closed")
                buffer += chunk
            except socket.timeout:
                pass
            if PROMPT_RE.search(self._clean(buffer[-256:])):     # the tail is enough
                return self._clean(buffer)
        raise MonitorError("timed out waiting for the monitor prompt")

    def command(self, command, timeout=20.0):
        """Run one monitor command and return what it printed."""
        if "\n" in command or "\r" in command:
            raise MonitorError("multi-line commands are not allowed")
        with self.lock:
            if not self.alive() or self.sock is None:
                raise MonitorError("Renode is not running")
            self.sock.settimeout(0.0)          # drop unsolicited output
            try:
                while self.sock.recv(65536):
                    pass
                raise MonitorError("monitor socket closed")
            except (BlockingIOError, socket.timeout):
                pass
            finally:
                self.sock.settimeout(0.2)
            self.sock.sendall(command.encode("utf-8", "replace") + b"\n")
            out = self._read_until_prompt(timeout)
        text = re.sub(r"\([^)\n]{1,64}\) $", "", out.decode("utf-8", "replace"))
        lines = text.split("\n")
        if lines and lines[0].strip() == command.strip():      # the echo
            lines = lines[1:]
        return "\n".join(lines).strip()

    def on(self, lens):
        """Context in which commands go to that lens's machine ("master" or
        "slave"); the monitor is on the master's machine otherwise."""
        return _LensContext(self, lens)

    def get(self, path, prop):
        return parse_scalar(self.command("%s %s" % (path, prop)))

    def set(self, path, prop, value):
        self.run("%s %s %s" % (path, prop, monitor_number(value)))

    def run(self, command):
        """A command that prints nothing when it succeeds."""
        out = self.command(command)
        if out:
            raise MonitorError(out)

    def try_get(self, path, prop):
        try:
            return self.get(path, prop)
        except MonitorError:
            return None

    def read_bytes(self, address, count):
        try:
            found = BYTE_RE.findall(self.command("sysbus ReadBytes 0x%08X %d" % (address, count)))
        except MonitorError:
            return None
        return [int(h, 16) for h in found] if len(found) == count else None

    def read_word(self, address):
        value = self.try_get("sysbus", "ReadDoubleWord 0x%08X" % address)
        return value if isinstance(value, int) else None

    # UART output: the session attaches Renode's logging analyzer to both
    # UARTs, so every completed line shows up in the log. So does what the
    # virtual phones report.
    def poll_log(self):
        with self.log_lock:
            with open(self.log_path, "r", errors="replace") as f:
                f.seek(self.log_offset)
                data = self.log_partial + f.read()
                self.log_offset = f.tell()
            lines = data.split("\n")
            self.log_partial = lines.pop()
            for line in lines:
                match = UART_LINE_RE.search(line)
                if match:
                    name = ("slave_" if match.group(1) == g1sim.SLAVE else "") + match.group(2)
                    self.uart_lines[name].append(COLOUR_RE.sub("", match.group(3)))
                    continue
                match = PHONE_LINE_RE.search(line)
                if match:
                    name = ("slave_" if match.group(1) == g1sim.SLAVE else "") + "phone"
                    self.uart_lines[name].append(match.group(2))

    def uart_since(self, cursor):
        self.poll_log()
        with self.log_lock:
            new_cursor, lines = {}, {}
            for name, seen in self.uart_lines.items():
                start = min(max(cursor.get(name, 0), 0), len(seen))
                lines[name] = seen[start:]
                new_cursor[name] = len(seen)
            return new_cursor, lines

    def mark(self, text):
        with self.log_lock:
            for seen in self.uart_lines.values():
                seen.append(text)


class _LensContext:
    def __init__(self, renode, lens):
        if lens not in LENSES:
            raise ValueError("lens must be master or slave")
        self.renode, self.lens = renode, lens

    def __enter__(self):
        self.renode.lock.acquire()
        if self.lens != "master":
            self.renode.command('mach set "%s"' % LENSES[self.lens])

    def __exit__(self, *exc):
        try:
            if self.lens != "master":
                self.renode.command('mach set "%s"' % LENSES["master"])
        finally:
            self.renode.lock.release()


class Api:
    def __init__(self, renode, firmware):
        self.renode = renode
        # lens -> what it runs, as g1sim.prepare_firmware describes it
        self.firmware = {lens: firmware[machine] for lens, machine in LENSES.items()}
        self.frames = {}                    # lens -> (key, framebuffer)
        self.resets = 0

    def check_lens(self, lens):
        if lens not in LENSES:
            raise ValueError("no such lens: %r" % (lens,))
        return lens

    def images(self):
        """[label, description] of each firmware image that is running."""
        master, slave = self.firmware["master"], self.firmware["slave"]
        if master is slave:                 # one firmware on both lenses
            return [["App image", master["app"]], ["Net image", master["net"]]]
        return [["%s %s image" % (lens.capitalize(), core), self.firmware[lens][core]]
                for lens in LENSES for core in ("app", "net")]

    # ------------------------------ reading ------------------------------

    def state(self):
        rn = self.renode
        match = VIRT_TIME_RE.search(rn.command("machine ElapsedVirtualTime"))
        return {
            "paused": rn.get("machine", "IsPaused") is True,
            "virtual_time": match.group(1) if match else None,
            "firmware": self.images(),
            "log": rn.log_path,
        }

    def display(self, lens="master"):
        rn = self.renode
        with rn.on(self.check_lens(lens)):
            counter = rn.get("sysbus.spim_a.jbd", "FrameCounter")
            key = (self.resets, counter)
            if self.frames.get(lens, (None, None))[0] != key:   # something was drawn
                self.frames[lens] = (key, rn.command(
                    "sysbus.spim_a.jbd GetFramebufferBase64", timeout=30).strip().strip('"'))
        return {"frame": counter, "width": 640, "framebuffer": self.frames[lens][1]}

    def gpio(self, lens="master"):
        """The GPIO ports' OUT, IN and DIR registers of one lens."""
        rn = self.renode

        def port(base):
            return {"out": rn.read_word(base + 0x04) or 0,
                    "in": rn.read_word(base + 0x10) or 0,
                    "dir": rn.read_word(base + 0x14) or 0}
        with rn.on(self.check_lens(lens)):
            return {"p0": port(0x50842500), "p1": port(0x50842800)}

    def identity(self, lens):
        """The firmware's view of a lens, whose machine is the current one,
        read through the addresses g1_state.py located in its image; a value
        it could not locate is None."""
        rn = self.renode
        found = self.firmware[lens]["state"][0]
        gs = rn.read_word(found["GLOBAL_STATE"]) if "GLOBAL_STATE" in found else None
        if gs is not None and not APP_RAM[0] <= gs < APP_RAM[1]:
            gs = None                        # the firmware has not allocated it yet
        head = rn.read_bytes(gs, 5) if gs is not None else None
        macs = (rn.read_bytes(gs + found["master_mac"], 12)
                if gs is not None and "master_mac" in found else None)
        case = rn.read_bytes(found["case_battery"], 1) if "case_battery" in found else None

        def mac_text(octets):
            return ":".join("%02X" % b for b in reversed(octets))
        name = None
        if head and macs:
            # As bt_start composes it outside factory-test mode: the master
            # lens is "R" and uses the master MAC, the slave "L" and its own.
            pair = "" if head[2] == 0xFF else "%d_" % head[2]
            own = macs[:6] if head[0] == 1 else macs[6:]
            name = "Even G1_%s%s_%02X%02X%02X" % (
                pair, "R" if head[0] == 1 else "L", own[2], own[1], own[0])
        return {
            "booted": gs is not None,
            "name": name,
            "role": None if not head else {1: "master", 2: "slave"}.get(head[0], str(head[0])),
            "work_mode": head[1] if head else None,
            "esb_channel": head[2] if head else None,
            "esb_addresses": "0x%02X / 0x%02X" % (head[3], head[4]) if head else None,
            "master_mac": mac_text(macs[:6]) if macs else None,
            "slave_mac": mac_text(macs[6:]) if macs else None,
            "case_battery": case[0] & 0x7F if case else None,
            "touch_armed": rn.try_get(TOUCH, "EventsArmed"),
            "touches": rn.try_get(TOUCH, "PressesDelivered"),
        }

    def head(self):
        """Where the IMU model has the lens on the current machine: "flat" on
        a table, worn with the head "level", or worn with the head "up" at
        the display; None while a gesture is moving it."""
        rn, imu = self.renode, "sysbus.twim2.lsm6dso"
        if rn.try_get(imu, "GesturePlaying") is True:
            return None
        x, z = rn.try_get(imu, "AccelerationX"), rn.try_get(imu, "AccelerationZ")
        if not isinstance(x, (int, float)) or not isinstance(z, (int, float)):
            return None
        return "up" if x >= 0.9 else "flat" if z >= 0.9 else "level"

    def lens_state(self, lens):
        """What the firmware and the models say about a lens, whose machine
        is the current one."""
        rn, pmic = self.renode, "sysbus.twim1.npm1300"
        identity = self.identity(lens)
        return {
            "booted": identity["booted"],
            "not_located": self.firmware[lens]["state"][1],
            "identity": identity,
            "head": self.head(),
            "cores": {
                "app_pc": rn.try_get("sysbus.cpuapp", "PC"),
                "app_halted": rn.try_get("sysbus.cpuapp", "IsHalted") is True,
                "net_pc": rn.try_get("sysbus.cpunet", "PC"),
                "net_halted": rn.try_get("sysbus.cpunet", "IsHalted") is True,
            },
            "power": {
                "battery_mv": rn.try_get(pmic, "BatteryVoltageMilliVolts"),
                "watchdog_expired": rn.try_get(pmic, "WatchdogExpired"),
                "watchdog_resets": rn.try_get(pmic, "WatchdogResetCount"),
                "reset_on_watchdog": rn.try_get(pmic, "ResetOnWatchdogExpiry"),
                "ship_mode": rn.try_get(pmic, "ShipModeActive"),
                "touch_armed": identity["touch_armed"],
                "touches": identity["touches"],
            },
            "radio": {
                "phone_enabled": rn.try_get(PHONE, "Enabled"),
                "phone_connected": rn.try_get(PHONE, "Connected"),
                "phone_link": rn.try_get(PHONE, "Summary"),
                "phone_mtu": rn.try_get(PHONE, "Mtu"),
                "phone_authenticated": rn.try_get(PHONE, "Authenticated") if PHONE_IS_G2 else None,
                "packets_sent": rn.try_get(PHONE, "PacketsSent"),
                "packets_received": rn.try_get(PHONE, "PacketsReceived"),
                "data_events": rn.try_get(PHONE, "DataEventsAnswered"),
            },
            "straps": {str(pin): rn.try_get("sysbus.gpio0.strap_p0_%d" % pin, "State") is True
                       for pin in STRAP_PINS},
        }

    def lens(self):
        """The state of each lens."""
        result = {"lenses": {}}
        for lens in LENSES:
            with self.renode.on(lens):
                result["lenses"][lens] = self.lens_state(lens)
        return result

    # ------------------------------ acting -------------------------------

    def press(self, what, action, lens="master"):
        """Touch a lens's touch bar, or bring the charging case's NFC field
        up at it."""
        commands = {
            ("touch", "press"): TOUCH + " Press",
            ("touch", "release"): TOUCH + " Release",
            ("nfc", "press"): "sysbus.twim1.st25dv RfField true",
            ("nfc", "release"): "sysbus.twim1.st25dv RfField false",
        }
        if (what, action) not in commands:
            raise ValueError("expected touch or nfc, and press or release")
        with self.renode.on(self.check_lens(lens)):
            self.renode.command(commands[(what, action)])
        return {}

    def inject(self, body):
        """Set a sensor, an input or a strap of one lens."""
        with self.renode.on(self.check_lens(body.get("lens", "master"))):
            self.inject_here(body)
        return {}

    def inject_here(self, body):
        rn, what = self.renode, body.get("what")
        imu, pmic = "sysbus.twim2.lsm6dso", "sysbus.twim1.npm1300"
        if what in ("accel", "gyro"):
            prop = "Acceleration" if what == "accel" else "AngularRate"
            for axis in "xyz":
                if axis in body:
                    rn.set(imu, prop + axis.upper(), float(body[axis]))
        elif what == "gesture":
            name = str(body.get("name", "don"))
            if not re.match(r"^[A-Za-z0-9_]+$", name):
                raise ValueError("bad gesture name")
            rn.run('%s PlayGesture "%s"' % (imu, name))
        elif what == "lux":
            rn.set("sysbus.twim1.opt3001", "Lux", float(body["value"]))
        elif what == "ain":
            rn.run("sysbus.saadc SetAnalogInput 3 %d" % int(body["microvolts"]))
        elif what == "temperature":
            rn.set(imu, "Temperature", float(body["celsius"]))
        elif what == "battery":
            rn.set(pmic, "BatteryVoltageMilliVolts", int(body["mv"]))
        elif what == "vbus":
            rn.set(pmic, "VbusConnected", bool(body["connected"]))
        elif what == "strap":
            if int(body["pin"]) not in STRAP_PINS:
                raise ValueError("strap pin must be one of %s" % (STRAP_PINS,))
            rn.set("sysbus.gpio0.strap_p0_%d" % int(body["pin"]), "Set", bool(body["state"]))
        elif what == "pdm":
            source = body.get("source")
            if source == "silence":
                rn.run("sysbus.pdm0 ConfigureSilence")
            elif source == "constant":
                rn.run("sysbus.pdm0 ConfigureConstant %d" % int(body["value"]))
            elif source == "sine":
                rn.run("sysbus.pdm0 ConfigureSine %d %d"
                       % (int(body["amplitude"]), int(body["frequency"])))
            else:
                raise ValueError("pdm source must be silence, constant or sine")
        else:
            raise ValueError("unknown injection %r" % (what,))

    def phone(self, body):
        """Act on a lens's virtual phone: switch it on or off ("enabled"),
        set the ATT MTU it asks for at its next connection ("mtu"), send a
        protocol packet ("packet", as g1sim.parse_packet reads it; for the G2
        phone its first two bytes are the service id and the flag, the rest
        the protobuf payload), or one of the actions "reconnect" and "forget"
        (the bond)."""
        rn = self.renode
        lens = self.check_lens(body.get("lens", "master"))
        result = {}
        packet = None
        if "packet" in body:
            packet = g1sim.parse_packet(str(body["packet"]))
            if PHONE_IS_G2 and len(packet) < 2:
                raise ValueError("a G2 request is: service id, flag, protobuf bytes")
            result["sent"] = packet.hex(" ").upper()
        if "mtu" in body and not 23 <= int(body["mtu"]) <= 517:
            raise ValueError("an ATT MTU is between 23 and 517")
        actions = {"reconnect": "Disconnect", "forget": "ForgetBond"}
        if "action" in body and body["action"] not in actions:
            raise ValueError("unknown phone action %r" % (body["action"],))
        with rn.on(lens):
            if "mtu" in body:
                rn.set(PHONE, "Mtu", int(body["mtu"]))
            if "enabled" in body:
                rn.set(PHONE, "Enabled", bool(body["enabled"]))
            try:
                if "action" in body:
                    rn.run("%s %s" % (PHONE, actions[body["action"]]))
                if packet is not None and PHONE_IS_G2:
                    rn.run('%s SendG2 %d %d "%s"' % (PHONE, packet[0], packet[1], packet[2:].hex()))
                elif packet is not None:
                    rn.run('%s SendPacket "%s"' % (PHONE, packet.hex()))
            except MonitorError as error:
                # The phone's own refusal ("the phone is switched off") is
                # the last line of what the monitor prints.
                if not rn.alive():
                    raise
                raise ValueError("phone: " + str(error).strip().splitlines()[-1])
        if "enabled" in body:
            self.set_quantum()
        return result

    def set_quantum(self):
        """The shorter time quantum while a phone is on (see g1sim.QUANTUM)."""
        rn, phones = self.renode, 0
        for lens in LENSES:
            with rn.on(lens):
                phones += rn.try_get(PHONE, "Enabled") is True
        rn.run('emulation SetGlobalQuantum "%s"'
               % (g1sim.PHONE_QUANTUM if phones else g1sim.QUANTUM))

    def control(self, action):
        rn = self.renode
        if action == "pause":
            rn.command("pause")
        elif action == "resume":
            rn.command("start")
        elif action == "reset":
            rn.command("pause")
            for lens in LENSES:
                with rn.on(lens):
                    rn.command("machine Reset")
                    rn.command("runMacro $reset")    # reloads the images, re-points the cores
            self.resets += 1
            rn.mark("----- reset -----")
            rn.command("start")
        elif action == "quit":
            signal.raise_signal(signal.SIGINT)   # os.kill would terminate outright on Windows
        else:
            raise ValueError("unknown action %r" % (action,))
        return {}


class Handler(BaseHTTPRequestHandler):
    api = None
    renode = None
    hosts = ()

    def log_message(self, fmt, *args):
        pass

    def reply(self, obj, code=200):
        payload = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def guarded(self, work):
        # Only requests addressed to this server by its loopback name: keeps
        # another site's page from reaching the monitor through the browser.
        if self.headers.get("Host") not in self.hosts:
            return self.reply({"ok": False, "error": "forbidden"}, 403)
        try:
            result = work()
            if result is not None:
                self.reply(dict(ok=True, **result))
        except MonitorError as e:
            self.reply({"ok": False, "error": "monitor: %s" % e},
                       502 if self.renode.alive() else 503)
        except (ValueError, KeyError, TypeError) as e:
            self.reply({"ok": False, "error": str(e)}, 400)

    def do_GET(self):
        path, _, query = self.path.partition("?")
        params = dict(part.split("=", 1) for part in query.split("&") if "=" in part)

        def work():
            api = self.api
            if path == "/":
                page = PAGE.replace("__PHONE_G2__", "true" if PHONE_IS_G2 else "false").encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return None
            if path == "/api/uart":
                cursor, lines = self.renode.uart_since(
                    {name: int(params.get(name, 0)) for name in CONSOLES})
                return {"cursor": cursor, "lines": lines}
            if path == "/api/display":
                return api.display(params.get("lens", "master"))
            if path == "/api/gpio":
                return api.gpio(params.get("lens", "master"))
            routes = {"/api/state": api.state, "/api/lens": api.lens}
            if path not in routes:
                self.reply({"ok": False, "error": "not found"}, 404)
                return None
            return routes[path]()
        self.guarded(work)

    def do_POST(self):
        def work():
            api = self.api
            # A JSON content type cannot be sent cross-site without the
            # server's consent, which it never gives.
            if self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json":
                raise ValueError("expected Content-Type: application/json")
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length") or 0)) or b"{}")
            if not isinstance(body, dict):
                raise ValueError("expected a JSON object")
            if self.path == "/api/cmd":
                if not isinstance(body.get("cmd"), str) or not body["cmd"].strip():
                    raise ValueError("expected a monitor command")
                with self.renode.on(api.check_lens(body.get("lens", "master"))):
                    return {"output": self.renode.command(body["cmd"], timeout=60)}
            routes = {
                "/api/control": lambda: api.control(body.get("action")),
                "/api/press": lambda: api.press(body.get("id"), body.get("action"),
                                                body.get("lens", "master")),
                "/api/inject": lambda: api.inject(body),
                "/api/phone": lambda: api.phone(body),
            }
            if self.path not in routes:
                self.reply({"ok": False, "error": "not found"}, 404)
                return None
            return routes[self.path]()
        self.guarded(work)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        # A page that went away mid-request is not worth a traceback.
        if not isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
            super().handle_error(request, client_address)


def main():
    parser = argparse.ArgumentParser(
        description="Run prebuilt Even Realities G1 firmware in Renode with a browser front end.")
    g1sim.add_setup_arguments(parser)
    parser.add_argument("--port", type=int, default=8340, metavar="N",
                        help="HTTP port on 127.0.0.1 (default 8340)")
    parser.add_argument("--paused", action="store_true",
                        help="load the firmware but do not start it")
    args = parser.parse_args()

    global PHONE, PHONE_IS_G2
    PHONE_IS_G2 = g1sim.phone_peripheral(args) == g1sim.PHONE_G2
    PHONE = "sysbus." + (g1sim.PHONE_G2 if PHONE_IS_G2 else g1sim.PHONE_G1)

    executable = g1sim.find_renode(args.renode)
    with tempfile.TemporaryDirectory(prefix="g1simweb-") as workdir:
        firmware = g1sim.prepare_firmware(args.firmware, workdir)
        for lens in firmware.values():
            # Where each image keeps the state the page shows.
            lens["state"] = g1_state.locate(lens["app_bin"])
        lines = g1sim.setup_lines(firmware, args)
        if not args.paused:
            lines.append("start")
        session = os.path.join(workdir, "session.resc")
        with open(session, "w") as f:
            f.write("\n".join(lines) + "\n")

        renode = Renode(executable, session, os.path.join(workdir, "renode.log"))
        print("g1_sim_web: starting Renode ...", flush=True)
        try:
            renode.start()
        except MonitorError as e:
            renode.shutdown()
            with open(renode.log_path, errors="replace") as f:
                sys.stderr.write("".join(f.readlines()[-25:]))
            sys.exit("g1_sim_web: %s (end of the Renode log is above)" % e)

        Handler.api = Api(renode, firmware)
        Handler.renode = renode
        Handler.hosts = ("127.0.0.1:%d" % args.port, "localhost:%d" % args.port)
        try:
            server = Server(("127.0.0.1", args.port), Handler)
        except OSError as e:
            renode.shutdown()
            sys.exit("g1_sim_web: cannot listen on port %d: %s" % (args.port, e))

        def stop(signum=None, frame=None):
            threading.Thread(target=server.shutdown, daemon=True).start()
        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        print("g1_sim_web: http://127.0.0.1:%d/   (Ctrl-C to stop; Renode log: %s)"
              % (args.port, renode.log_path), flush=True)
        try:
            server.serve_forever()
        finally:
            renode.shutdown()
            server.server_close()
    return 0


PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Even G1 simulator</title>
<style>
:root {
  --bg: #14171c; --panel: #1c2129; --panel-edge: #2a313c; --fg: #d7dde6;
  --dim: #7d8794; --accent: #4fb3ff; --ok: #3ecf6f; --warn: #e0a63c;
  --err: #e05c5c; --mono: "SF Mono", Menlo, Consolas, monospace;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--bg); color: var(--fg);
       font: 14px/1.45 -apple-system, "Segoe UI", Roboto, sans-serif; }
header { padding: 8px 16px; background: var(--panel);
         border-bottom: 1px solid var(--panel-edge); position: sticky; top: 0; z-index: 5; }
#top-bar { display: flex; align-items: center; justify-content: space-between; }
#machine { display: flex; flex-wrap: wrap; gap: 2px 22px; margin-top: 4px; }
#images { display: contents; }
h1 { font-size: 18px; margin: 0; }
h2 { font-size: 15px; margin: 0; display: flex; align-items: baseline; gap: 8px; }
h3 { font-size: 12px; margin: 12px 0 6px; color: var(--dim);
     text-transform: uppercase; letter-spacing: .05em; }
.panel > h3:first-child, .lens-grid h3 { margin-top: 0; }
.sub { font-size: 12px; margin: 8px 0 2px; color: var(--dim); }
.dim { color: var(--dim); font-weight: normal; }
.right { margin-left: auto; }
.mono { font-family: var(--mono); }
.small { font-size: 11px; word-break: break-all; }
.hidden { display: none !important; }
#control-bar { display: flex; align-items: center; gap: 10px; }
.dot { width: 10px; height: 10px; border-radius: 50%; background: var(--err); display: inline-block; }
.dot.ok { background: var(--ok); }
.dot.paused { background: var(--warn); }
button { background: #2a3340; color: var(--fg); border: 1px solid #3a4657;
         border-radius: 4px; padding: 4px 12px; cursor: pointer; font: inherit; }
button:hover { background: #344052; }
button:active { background: var(--accent); color: #08121c; }
button.warn { border-color: var(--warn); }
#btn-run { min-width: 5.5em; }
button.btn-hw { padding: 8px 16px; border-color: var(--accent); user-select: none; }
button.btn-hw.held { background: var(--accent); color: #08121c; }
/* One column per lens; on a narrow window the slave's goes below the master's. */
main { display: grid; grid-template-columns: repeat(auto-fill, minmax(min(100%, 672px), 1fr));
       gap: 12px; padding: 12px 16px; align-items: start; }
.lens-col { display: grid; gap: 12px; min-width: 0; }
.panel { background: var(--panel); border: 1px solid var(--panel-edge);
         border-radius: 6px; padding: 12px 14px; min-width: 0; }
.display-wrap { overflow-x: auto; }
/* No width here: fitDisplays() sets a whole-number multiple of the panel size. */
canvas.lens-display { box-sizing: content-box; image-rendering: pixelated;
                      background: #000; border: 1px solid var(--panel-edge); display: block; }
table.kv { border-collapse: collapse; width: 100%; }
table.kv th { text-align: left; color: var(--dim); font-weight: normal; width: 1%;
              padding: 2px 16px 2px 0; white-space: nowrap; vertical-align: top; }
table.kv td { padding: 2px 0; }
pre.console { background: #0d1014; border: 1px solid var(--panel-edge); border-radius: 4px;
              height: 220px; overflow-y: auto; margin: 0; padding: 8px;
              font-family: var(--mono); font-size: 12px; white-space: pre-wrap; word-break: break-all; }
pre.cmd-out { height: 120px; }
pre.phone-out { height: 160px; }
.row { display: flex; flex-wrap: wrap; gap: 12px; align-items: center; margin: 6px 0; }
.row label { display: flex; align-items: center; gap: 5px; }
.slider-row { display: grid; grid-template-columns: 70px 1fr 90px; gap: 10px;
              align-items: center; margin: 4px 0; }
.slider-row .lbl { color: var(--dim); font-size: 12px; text-align: right; }
input[type=range] { width: 100%; accent-color: var(--accent); }
input[type=number], input[type=text], select {
  background: #10141a; color: var(--fg); border: 1px solid #3a4657;
  border-radius: 4px; padding: 3px 6px; font-family: var(--mono); width: 100%; }
select { width: auto; }
.pdm-params input { width: auto; }
.lens-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(260px, 1fr)); gap: 14px 22px; }
.gpio-port { margin: 8px 0; }
.gpio-port .port-name { font-family: var(--mono); color: var(--dim); margin-right: 8px; }
.gpio-regs { font-family: var(--mono); font-size: 11px; color: var(--dim); }
.gpio-pins { display: grid; grid-template-columns: repeat(32, minmax(18px, 1fr)); gap: 2px; margin-top: 4px; }
@media (max-width: 700px) { .gpio-pins { grid-template-columns: repeat(16, minmax(18px, 1fr)); } }
.gpio-pin { text-align: center; border-radius: 3px; border: 1px solid #333c48;
            padding: 2px 0 1px; font-family: var(--mono); font-size: 10px; color: var(--dim); cursor: default; }
.gpio-pin .lvl { display: block; height: 6px; border-radius: 2px; margin: 2px 3px 0; background: #262d37; }
.gpio-pin.out { border-color: var(--accent); }
.gpio-pin.hi .lvl { background: var(--ok); }
.gpio-pin.named { color: var(--fg); }
.ind { width: 9px; height: 9px; border-radius: 50%; background: var(--dim);
       display: inline-block; vertical-align: middle; }
.ind.ok { background: var(--ok); }
.ind.off { background: var(--err); }
.ind-label { font-size: 12px; }
footer { padding: 6px 16px 14px; font-size: 12px; min-height: 2em; }
#status-line.err { color: var(--err); }
</style>
</head>
<body>
<header>
  <div id="top-bar">
    <h1>Even G1 <span class="dim">simulator</span></h1>
    <div id="control-bar">
      <span id="conn-dot" class="dot" title="server link"></span>
      <span id="virt-time" class="mono" title="emulated time">--:--:--</span>
      <button id="btn-run">pause</button>
      <button id="btn-reset" class="warn">reset</button>
    </div>
  </div>
  <div id="machine" class="small">
    <span id="images"></span>
    <span><span class="dim">Renode log</span> <span class="mono" id="log-path">—</span></span>
  </div>
</header>

<main id="columns"></main>

<footer><span id="status-line" class="dim"></span></footer>
<script>
"use strict";
const $ = id => document.getElementById(id);
const statusLine = $("status-line");

function setStatus(msg, isErr) {
  statusLine.textContent = msg || "";
  statusLine.className = isErr ? "err" : "dim";
}

async function api(path, body) {
  const opts = body === undefined ? {} :
    { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  const r = await fetch(path, opts);
  const data = await r.json().catch(() => ({ ok: false, error: "bad JSON" }));
  if (!data.ok) throw new Error(data.error || ("HTTP " + r.status));
  return data;
}

function post(path, body, okMsg) {
  return api(path, body)
    .then(d => { if (okMsg) setStatus(okMsg); return d; })
    .catch(e => { setStatus(e.message, true); throw e; });
}

/* ------------------------------ columns ------------------------------ */

// The page is one column per lens, both built from this template. Every id
// in a column ends in the lens: "display-master", "display-slave", ...
const LENSES = ["master", "slave"];

// Whether the phone speaks the G2 app's protocol (--phone-g2) or the G1 app's.
const PHONE_G2 = __PHONE_G2__;

// [label, id, cell classes]
const INFO_ROWS = [["Advertising name", "name", "mono"], ["Role", "role", ""],
                   ["Work mode", "workmode", "mono"], ["App core PC", "pc-app", "mono"],
                   ["Net core PC", "pc-net", "mono"], ["Touch configured", "touch-armed", ""],
                   ["Touches seen", "touches", "mono"]];
const POWER_ROWS = [["Battery", "battery", "mono"], ["Case battery", "case", "mono"],
                    ["PMIC watchdog", "watchdog", ""], ["Watchdog resets", "resets", "mono"],
                    ["Ship mode", "ship", ""]];
const ESB_ROWS = [["ESB channel", "channel", "mono"], ["ESB address bytes", "addresses", "mono"],
                  ["Master MAC", "mac-master", "mono small"], ["Slave MAC", "mac-slave", "mono small"]];
const PHONE_ROWS = [["Phone", "phone", ""], ["Link", "link", ""],
                    ["Packets", "packets", "mono"], ["Connection events", "dataev", "mono"]];

// Protocol packets offered in the phone's packet box; what is typed there is
// free. The bytes come first, a comment after " - ". For the G2 phone a
// packet is: service id, flag, protobuf bytes; the phone frames it.
const PACKETS_G2 = [
  "0x80 0x00 08 04 10 65 1a 04 08 01 10 04 - auth (magic 0x65)",
  "0x80 0x20 08 0e 10 66 6a 00 - heartbeat",
  "0x80 0x20 08 0c 10 67 - get device info",
  "0x80 0x20 08 80 01 10 68 82 08 04 08 00 10 00 - time sync: 1970, UTC",
  "0x09 0x20 08 02 10 69 22 02 08 01 - settings query: battery, firmware",
  "0x80 0x20 08 08 10 6a - disconnect",
];
const PACKETS = PHONE_G2 ? PACKETS_G2 : [
  "0x29 - get brightness",
  "0x2a - get anti-shake",
  "0x2b - get work mode",
  "0x2c 0x01 - get device info, phone is Android",
  "0x2c 0x02 - get device info, phone is iOS",
  "0x2d - get MAC",
  "0x2e - get notification whitelist",
  "0x32 - get head-up angle",
  "0x33 - get glasses serial number",
  "0x34 - get device serial number",
  "0x35 - get ESB channel",
  "0x36 - get notification count",
  "0x37 - get time since boot",
  "0x38 - get ANCS enable",
  "0x39 0x05 0x00 0x00 0x01 - get running app",
  "0x3a - get wear detection",
  "0x3b - get raster config",
  "0x3c - get notification auto display",
  "0x3e - get burial point data",
  "0x3f 0x05 - get hardware setting: double-tap action",
  "0x3f 0x06 - get hardware setting: luminance coefficient",
  "0x3f 0x07 - get hardware setting: long press",
  "0x3f 0x08 - get hardware setting: microphone on head-up",
  "0x25 0x06 0x00 0x01 0x04 0x01 - heartbeat",
  "0x27 0x01 - set wear detection on",
  "0x27 0x00 - set wear detection off",
  "0x4d 0xfb - set MTU, the app session is established",
  "0x47 - unpair: the lens drops its bond and disconnects",
];

function rows(lens, list) {
  return list.map(([label, id, cls]) =>
    `<tr><th>${label}</th><td class="${cls}" id="${id}-${lens}">—</td></tr>`).join("");
}

function lensColumn(lens) {
  const master = lens === "master";
  const uart = name => "con-" + (master ? "" : "slave_") + name;
  return `
<div class="lens-col" id="col-${lens}">
  <h2><span id="title-${lens}">${master ? "Master" : "Slave"} lens</span>
    <span class="right small mono dim" id="boot-${lens}"></span></h2>

  <section class="panel">
    <h3>Display <span class="dim mono" id="frames-${lens}">0 window writes</span></h3>
    <div class="display-wrap"><canvas class="lens-display" id="display-${lens}" width="640" height="480"></canvas></div>
    <div class="row" style="margin:10px 0 0">
      ${master ? `<button class="btn-hw" id="gesture-master" title="Moves the IMU model. Head-up: from a table, the 'don' motion (picked up, put on, head tilted up); when worn, the tilt alone. The firmware opens the dashboard on both lenses on the head-up. Head-down tilts the head back to level.">head-up</button>` : ""}
      <button class="btn-hw" id="touch-${lens}" title="Touches the touch bar while held: click for a tap, hold for a long press.">touch bar</button>
      <button class="btn-hw" id="nfc-${lens}" title="The charging case's NFC field: up while held. Each change interrupts the firmware; after ten it concludes the glasses are in the case.">NFC field</button>
    </div>
  </section>

  <section class="panel">
    <div class="lens-grid">
      <div><h3>Info</h3><table class="kv">${rows(lens, INFO_ROWS)}</table></div>
      <div><h3>Power</h3><table class="kv">${rows(lens, POWER_ROWS)}</table></div>
      <div><h3>Radio <span class="dim">lens pair, ESB</span></h3>
        <table class="kv">${rows(lens, ESB_ROWS)}</table></div>
      <div><h3>Radio <span class="dim">phone, BLE</span></h3>
        <table class="kv">${rows(lens, PHONE_ROWS)}</table>
        <div class="row">
          <label title="The virtual phone connects over BLE from the address the lens is bound to, ${PHONE_G2 ? "authenticates (G2 protocol, no pairing)" : "pairs with the lens"} and keeps the link alive. The emulation runs slower while a phone is on."><input type="checkbox" id="peer-phone-${lens}"> phone${PHONE_G2 ? " (G2)" : ""}</label>
          <label title="The ATT MTU the phone asks for when it connects. The G1 app uses 251, the G2 app 512; the G1 firmware takes 498 or more as the start of a firmware update.">MTU <input type="number" id="phone-mtu-${lens}" min="23" max="517" value="${PHONE_G2 ? 512 : 251}" style="width:5em"></label>
        </div>
        <div class="row">
          <button id="phone-reconnect-${lens}" title="The phone closes the link and connects again.">reconnect</button>
          <button id="phone-forget-${lens}" title="The phone forgets its bond with the lens: at its next connection it pairs again.">forget bond</button>
        </div></div>
    </div>
  </section>

  <section class="panel">
    <h3>App core console <span class="dim">uart0</span></h3>
    <pre class="console" id="${uart("uart0")}"></pre>
    <h3>Net core console <span class="dim">uart_net</span></h3>
    <pre class="console" id="${uart("uart_net")}"></pre>
    <h3>Phone <span class="dim">packets to and from the lens</span></h3>
    <pre class="console phone-out" id="${uart("phone")}"></pre>
    <div class="row">
      <input type="text" id="packet-${lens}" list="packets" autocomplete="off" spellcheck="false"
             placeholder="2b" style="flex:1"
             title="${PHONE_G2 ? "A G2 request for the phone to frame and write to the lens: service id, flag, protobuf bytes, e.g. 0x80 0x20 08 0e 10 66 6a 00; after &quot; - &quot; a comment. It is sent once the phone is connected with notifications on." : "A packet for the phone to write to the lens: hex bytes, with or without 0x, e.g. 0x3e or 26 06 00 00 06 01; &quot;text&quot; in double quotes for its bytes; after &quot; - &quot; a comment. It is sent once the phone is connected and paired."}">
      <button id="packet-send-${lens}">send</button>
    </div>
  </section>

  <section class="panel">
    <h3>Sensor input</h3>
    <div class="sub">Accelerometer, g</div>
    <div id="accel-${lens}"></div>
    <div class="sub">Gyroscope, dps</div>
    <div id="gyro-${lens}"></div>
    <div class="sub">Environment</div>
    <div id="env-${lens}"></div>
    <div class="sub">Battery</div>
    <div id="batt-${lens}"></div>
    <div class="row"><label><input type="checkbox" id="vbus-${lens}"> VBUS connected</label></div>
    <div class="sub">Microphone</div>
    <div class="row">
      <select id="pdm-source-${lens}">
        <option value="silence">silence</option>
        <option value="sine">sine</option>
        <option value="constant">constant</option>
      </select>
      <span id="pdm-sine-${lens}" class="pdm-params hidden">
        <input type="number" id="pdm-amp-${lens}" value="8000" title="amplitude (int16)" style="width:5.5em"> amp
        <input type="number" id="pdm-freq-${lens}" value="440" title="frequency Hz" style="width:5em"> Hz
      </span>
      <span id="pdm-const-${lens}" class="pdm-params hidden">
        <input type="number" id="pdm-value-${lens}" value="1000" title="constant sample (int16)" style="width:6em"> value
      </span>
      <button id="pdm-apply-${lens}">apply</button>
    </div>

    <h3>Monitor command</h3>
    <div class="row">
      <input type="text" id="cmd-${lens}" placeholder="sysbus.twim2.touch Tap 1500" style="flex:1">
      <button id="cmd-run-${lens}">run</button>
    </div>
    <pre class="console cmd-out" id="cmd-out-${lens}"></pre>

    <h3>GPIO <span class="dim">blue border = output, green = high</span></h3>
    <div id="gpio-p0-${lens}"></div>
    <div id="gpio-p1-${lens}"></div>
    <div class="sub">Straps, read at boot; P0.26 high = slave lens</div>
    <div class="row">
      ${[25, 26, 27, 28].map(pin =>
        `<label><input type="checkbox" id="strap-${pin}-${lens}"> P0.${pin}</label>`).join("")}
    </div>
  </section>
</div>`;
}

$("columns").innerHTML = LENSES.map(lensColumn).join("");

const packetList = document.createElement("datalist");
packetList.id = "packets";
packetList.append(...PACKETS.map(packet => {
  const option = document.createElement("option");
  option.value = packet;
  return option;
}));
document.body.append(packetList);

/* ------------------------------ display ------------------------------ */

function lensDisplay(lens) {
  const canvas = $("display-" + lens), ctx = canvas.getContext("2d");
  let imageData = null, lastFrame = null;
  return async function () {
    try {
      const d = await api("/api/display?lens=" + lens);
      $("frames-" + lens).textContent = d.frame + " window writes";
      if (d.framebuffer === lastFrame) return;
      lastFrame = d.framebuffer;
      const raw = atob(d.framebuffer);
      const height = Math.floor(raw.length * 2 / d.width);
      if (!imageData || imageData.width !== d.width || imageData.height !== height) {
        canvas.width = d.width; canvas.height = height;
        fitDisplays();
        imageData = ctx.createImageData(d.width, height);
        imageData.data.fill(255);
      }
      const px = imageData.data;
      // 4 bpp, two pixels per byte, left pixel in the high nibble; shown in green.
      for (let i = 0; i < raw.length; i++) {
        const b = raw.charCodeAt(i), o = i * 8;
        px[o] = px[o + 2] = px[o + 4] = px[o + 6] = 0;
        px[o + 1] = ((b >> 4) & 0xF) * 17;
        px[o + 5] = (b & 0xF) * 17;
      }
      ctx.putImageData(imageData, 0, 0);
    } catch (e) { /* transient */ }
  };
}

// Each panel pixel is drawn as the same n x n block of screen pixels (n a whole
// number, as large as fits), so the browser never resamples the image. A lens
// that does not fit at n = 1 scrolls sideways instead of shrinking.
function fitDisplays() {
  const dpr = window.devicePixelRatio || 1;
  for (const canvas of document.querySelectorAll("canvas.lens-display")) {
    const room = canvas.parentElement.clientWidth - 2;      // less the border
    if (room <= 0) continue;                                // not laid out yet
    const n = Math.max(1, Math.floor(room * dpr / canvas.width));
    canvas.style.width = (canvas.width * n / dpr) + "px";
  }
}
for (const canvas of document.querySelectorAll("canvas.lens-display"))
  new ResizeObserver(fitDisplays).observe(canvas.parentElement);
window.addEventListener("resize", fitDisplays);     // browser zoom changes the ratio

const displayPollers = Object.fromEntries(LENSES.map(lens => [lens, lensDisplay(lens)]));

async function pollDisplay() {
  for (const lens of LENSES) await displayPollers[lens]();
}

/* ------------------------------ consoles ----------------------------- */

const consoles = ["uart0", "uart_net", "phone", "slave_uart0", "slave_uart_net", "slave_phone"];
const cursors = Object.fromEntries(consoles.map(name => [name, 0]));

async function pollUart() {
  try {
    const d = await api("/api/uart?" + consoles.map(name => `${name}=${cursors[name]}`).join("&"));
    for (const name of consoles) {
      const lines = d.lines[name];
      if (lines.length) {
        const el = $("con-" + name);
        const stick = el.scrollTop + el.clientHeight >= el.scrollHeight - 8;
        el.textContent += lines.join("\n") + "\n";
        if (el.textContent.length > 200000)
          el.textContent = el.textContent.slice(-150000);
        if (stick) el.scrollTop = el.scrollHeight;
      }
      cursors[name] = d.cursor[name];
    }
  } catch (e) { /* transient */ }
}

/* ------------------------------ machine ------------------------------ */

const connDot = $("conn-dot");
let paused = false;

// The firmware images in the header: one pair when both lenses run the same
// firmware, otherwise one pair for each lens.
let imagesShown = "";
function showImages(images) {
  if (JSON.stringify(images) === imagesShown) return;
  imagesShown = JSON.stringify(images);
  $("images").replaceChildren(...images.map(([label, description]) => {
    const item = document.createElement("span"), name = document.createElement("span"),
          value = document.createElement("span");
    name.className = "dim"; name.textContent = label;
    value.className = "mono"; value.textContent = description;
    item.append(name, " ", value);
    return item;
  }));
}

async function pollState() {
  try {
    const d = await api("/api/state");
    paused = d.paused;
    connDot.className = "dot " + (paused ? "paused" : "ok");
    connDot.title = paused ? "paused" : "running";
    $("btn-run").textContent = paused ? "resume" : "pause";
    $("virt-time").textContent = d.virtual_time || "?";
    showImages(d.firmware);
    $("log-path").textContent = d.log;
  } catch (e) {
    connDot.className = "dot";
    connDot.title = "no answer from the server";
  }
}

$("btn-run").onclick = () =>
  post("/api/control", { action: paused ? "resume" : "pause" }, paused ? "running" : "paused")
    .then(pollState).catch(() => {});
$("btn-reset").onclick = () => post("/api/control", { action: "reset" }, "machine reset");

/* ------------------------------ inputs ------------------------------- */

function debounced(onSet) {
  let t = null;
  return v => { clearTimeout(t); t = setTimeout(() => onSet(parseFloat(v)), 120); };
}

function slider(container, label, min, max, step, value, onSet) {
  const row = document.createElement("div");
  row.className = "slider-row";
  const lbl = document.createElement("span");
  lbl.className = "lbl"; lbl.textContent = label;
  const rng = document.createElement("input");
  rng.type = "range"; rng.min = min; rng.max = max; rng.step = step; rng.value = value;
  const num = document.createElement("input");
  num.type = "number"; num.min = min; num.max = max; num.step = step; num.value = value;
  row.append(lbl, rng, num);
  container.appendChild(row);
  const fire = debounced(onSet);
  rng.oninput = () => { num.value = rng.value; fire(rng.value); };
  num.onchange = () => { rng.value = num.value; fire(num.value); };
}

// The head-up button offers the opposite of where the master's head is:
// "flat" on a table, worn "level", or "up" at the display.
let head = "flat", headHeld = 0;
function showHead(pose) {
  head = pose;
  $("gesture-master").textContent = pose === "up" ? "head-down" : "head-up";
}

// Everything in one lens's column that the user can act on.
function wireLens(lens) {
  const el = id => $(id + "-" + lens);
  const inject = (body, msg) => post("/api/inject", { lens, ...body }, `${lens} lens: ${msg}`);

  if (lens === "master") {
    el("gesture").onclick = () => {
      const [name, next, msg] = head === "up" ? ["head_down", "level", "head down (1.7 s)"]
        : head === "level" ? ["head_up", "up", "head up (1.7 s)"]
        : ["don", "up", "putting the glasses on, head up (5.6 s)"];
      showHead(next);
      headHeld = Date.now() + 2500;       // until the model reports the gesture
      inject({ what: "gesture", name }, msg);
    };
  }

  // Held buttons: down while the pointer is.
  for (const id of ["touch", "nfc"]) {
    const btn = el(id);
    const send = action => post("/api/press", { id, action, lens },
      `${lens} lens: ${btn.textContent} ${action}`);
    btn.addEventListener("pointerdown", e => {
      btn.setPointerCapture(e.pointerId);
      btn.classList.add("held");
      send("press");
    });
    const up = () => {
      if (!btn.classList.contains("held")) return;
      btn.classList.remove("held");
      send("release");
    };
    btn.addEventListener("pointerup", up);
    btn.addEventListener("pointercancel", up);
  }

  const phone = el("peer-phone");
  phone.addEventListener("change", () =>
    post("/api/phone", { enabled: phone.checked, lens },
         `${lens} lens: phone ` + (phone.checked ? "on" : "off"))
      .catch(() => { phone.checked = !phone.checked; }));
  el("phone-mtu").addEventListener("change", () =>
    post("/api/phone", { mtu: +el("phone-mtu").value, lens },
         `${lens} lens: the phone asks for an MTU of ${el("phone-mtu").value} when it next connects`)
      .catch(() => {}));
  for (const action of ["reconnect", "forget"])
    el("phone-" + action).onclick = () =>
      post("/api/phone", { action, lens }, `${lens} lens: phone ` +
           (action === "forget" ? "forgot its bond" : "reconnecting")).catch(() => {});

  function sendPacket() {
    const packet = el("packet").value.trim();
    if (!packet) return;
    post("/api/phone", { packet, lens })
      .then(d => setStatus(`${lens} lens: phone sends ${d.sent}`))
      .catch(() => {});
  }
  el("packet-send").onclick = sendPacket;
  el("packet").addEventListener("keydown", e => { if (e.key === "Enter") sendPacket(); });

  for (const axis of ["x", "y", "z"]) {
    slider(el("accel"), "a" + axis, -4, 4, 0.01, axis === "z" ? 1 : 0,
      v => inject({ what: "accel", [axis]: v }, `accel ${axis} = ${v} g`));
    slider(el("gyro"), "g" + axis, -2000, 2000, 1, 0,
      v => inject({ what: "gyro", [axis]: v }, `gyro ${axis} = ${v} dps`));
  }
  slider(el("env"), "lux", 0, 10000, 1, 0,
    v => inject({ what: "lux", value: v }, `lux = ${v}`));
  slider(el("env"), "AIN3 mV", 0, 3300, 1, 0,
    v => inject({ what: "ain", microvolts: Math.round(v * 1000) }, `AIN3 = ${v} mV`));
  slider(el("env"), "temp °C", -40, 85, 0.5, 25,
    v => inject({ what: "temperature", celsius: v }, `IMU temperature = ${v} C`));
  slider(el("batt"), "batt mV", 2500, 4500, 10, 4500,
    v => inject({ what: "battery", mv: Math.round(v) }, `battery = ${v} mV`));

  el("vbus").addEventListener("change", () =>
    inject({ what: "vbus", connected: el("vbus").checked },
      "VBUS " + (el("vbus").checked ? "connected" : "removed")));

  el("pdm-source").addEventListener("change", () => {
    const s = el("pdm-source").value;
    el("pdm-sine").classList.toggle("hidden", s !== "sine");
    el("pdm-const").classList.toggle("hidden", s !== "constant");
  });
  el("pdm-apply").onclick = () => {
    const s = el("pdm-source").value;
    const body = { what: "pdm", source: s };
    if (s === "sine") {
      body.amplitude = +el("pdm-amp").value;
      body.frequency = +el("pdm-freq").value;
    } else if (s === "constant") {
      body.value = +el("pdm-value").value;
    }
    inject(body, "microphone source: " + s);
  };

  function runCommand() {
    const cmd = el("cmd").value.trim();
    if (!cmd) return;
    post("/api/cmd", { cmd, lens }).then(d => {
      const out = el("cmd-out");
      out.textContent += "> " + cmd + "\n" + (d.output ? d.output + "\n" : "");
      out.scrollTop = out.scrollHeight;
    }).catch(() => {});
  }
  el("cmd-run").onclick = runCommand;
  el("cmd").addEventListener("keydown", e => { if (e.key === "Enter") runCommand(); });

  for (const pin of [25, 26, 27, 28]) {
    const box = $(`strap-${pin}-${lens}`);
    box.addEventListener("change", () =>
      inject({ what: "strap", pin, state: box.checked },
        `strap P0.${pin} = ${box.checked ? "high" : "low"}`));
  }
}
LENSES.forEach(wireLens);

/* ------------------------------ GPIO --------------------------------- */

const P0_LABELS = {
  4: "I2C1 SCL", 5: "I2C1 SDA", 7: "AIN3 (NFC field)",
  8: "display SCK", 9: "display MOSI", 10: "display MISO", 11: "display CS",
  13: "QSPI IO0", 14: "QSPI IO1", 15: "QSPI IO2", 16: "QSPI IO3",
  17: "QSPI SCK", 18: "QSPI CSN",
  19: "panel control", 20: "UART0 TX", 21: "panel control", 22: "UART0 RX",
  23: "panel control", 24: "panel power",
  25: "strap", 26: "strap: lens role", 27: "strap", 28: "strap", 30: "touch power",
};
const P1_LABELS = {
  0: "net core", 1: "net core", 2: "I2C2 SDA", 3: "I2C2 SCL",
  9: "touch INT", 10: "NFC GPO", 13: "PDM CLK", 14: "PDM DIN",
};

function buildPort(elId, portName, labels, pinCount) {
  const root = $(elId);
  root.className = "gpio-port";
  const head = document.createElement("div");
  head.innerHTML = `<span class="port-name">${portName}</span><span class="gpio-regs"></span>`;
  const grid = document.createElement("div");
  grid.className = "gpio-pins";
  const cells = [];
  for (let i = 0; i < pinCount; i++) {
    const c = document.createElement("div");
    c.className = "gpio-pin" + (labels[i] ? " named" : "");
    c.title = `${portName}.${String(i).padStart(2, "0")}` + (labels[i] ? ": " + labels[i] : "");
    c.innerHTML = `${i}<span class="lvl"></span>`;
    grid.appendChild(c);
    cells.push(c);
  }
  root.append(head, grid);
  return { cells, regs: head.querySelector(".gpio-regs") };
}

const gpioUi = Object.fromEntries(LENSES.map(lens => [lens, {
  p0: buildPort("gpio-p0-" + lens, "P0", P0_LABELS, 32),
  p1: buildPort("gpio-p1-" + lens, "P1", P1_LABELS, 16),
}]));
const hex8 = v => "0x" + (v >>> 0).toString(16).toUpperCase().padStart(8, "0");

function updatePort(ui, regs) {
  ui.regs.textContent = ` OUT=${hex8(regs.out)} DIR=${hex8(regs.dir)} IN=${hex8(regs.in)}`;
  ui.cells.forEach((c, i) => {
    const isOut = (regs.dir >>> i) & 1;
    const level = isOut ? (regs.out >>> i) & 1 : (regs.in >>> i) & 1;
    c.classList.toggle("out", !!isOut);
    c.classList.toggle("hi", !!level);
  });
}

async function pollGpio() {
  for (const lens of LENSES) {
    try {
      const d = await api("/api/gpio?lens=" + lens);
      updatePort(gpioUi[lens].p0, d.p0);
      updatePort(gpioUi[lens].p1, d.p1);
    } catch (e) { /* transient */ }
  }
}

/* ------------------------------ lens state --------------------------- */

const hex = v => typeof v === "number" ? "0x" + v.toString(16).toUpperCase() : "?";

function indicator(id, on, text) {
  const td = $(id);
  td.textContent = "";
  const dot = document.createElement("span");
  dot.className = "ind " + (on === true ? "ok" : on === false ? "off" : "");
  const label = document.createElement("span");
  label.className = "ind-label";
  label.textContent = " " + text;
  td.append(dot, label);
}

function txt(id, v) { $(id).textContent = (v === null || v === undefined) ? "—" : v; }

// A checkbox follows the model unless the user is on it.
function follow(id, value) {
  if (document.activeElement !== $(id)) $(id).checked = !!value;
}

function fillLens(lens, d) {
  const id = d.identity, co = d.cores, po = d.power, ra = d.radio;
  const at = name => name + "-" + lens;
  // A column is titled by the role its firmware reports.
  if (id.role === "master" || id.role === "slave")
    txt(at("title"), id.role === "master" ? "Master lens" : "Slave lens");
  txt(at("boot"), d.not_located.length
    ? "not located in this image: " + d.not_located.join("; ")
    : d.booted ? "" : "firmware state not allocated yet");

  txt(at("name"), id.name);
  txt(at("role"), id.role);
  txt(at("workmode"), id.work_mode);
  txt(at("pc-app"), hex(co.app_pc) + (co.app_halted ? " (halted)" : ""));
  txt(at("pc-net"), hex(co.net_pc) + (co.net_halted ? " (held in reset)" : ""));
  indicator(at("touch-armed"), po.touch_armed, po.touch_armed ? "yes" : "no");
  txt(at("touches"), po.touches);

  txt(at("battery"), po.battery_mv === null ? null : po.battery_mv + " mV");
  txt(at("case"), id.case_battery === null ? null : id.case_battery + " %");
  indicator(at("watchdog"), po.watchdog_expired === null ? null : !po.watchdog_expired,
    po.watchdog_expired ? "expired" + (po.reset_on_watchdog ? "" : " (reset switched off)") : "fed");
  txt(at("resets"), po.watchdog_resets);
  indicator(at("ship"), po.ship_mode === null ? null : !po.ship_mode, po.ship_mode ? "entered" : "no");

  txt(at("channel"), id.esb_channel);
  txt(at("addresses"), id.esb_addresses);
  txt(at("mac-master"), id.master_mac);
  txt(at("mac-slave"), id.slave_mac);

  indicator(at("phone"), ra.phone_enabled ? !!ra.phone_connected : null,
    !ra.phone_enabled ? "off" : !ra.phone_connected ? "not connected"
    : PHONE_G2 ? (ra.phone_authenticated ? "connected, authenticated" : "connected, not authenticated")
    : "connected");
  txt(at("link"), ra.phone_link);
  txt(at("packets"), ra.packets_sent === null ? null
    : `${ra.packets_sent} sent, ${ra.packets_received} received`);
  txt(at("dataev"), ra.data_events);
  follow(at("peer-phone"), ra.phone_enabled);
  if (document.activeElement !== $(at("phone-mtu")) && ra.phone_mtu) $(at("phone-mtu")).value = ra.phone_mtu;
  if (lens === "master" && d.head && Date.now() > headHeld) showHead(d.head);
  for (const pin of [25, 26, 27, 28]) follow(`strap-${pin}-${lens}`, d.straps[pin]);
}

async function pollLens() {
  let d;
  try { d = await api("/api/lens"); } catch (e) { return; }
  for (const lens of LENSES) fillLens(lens, d.lenses[lens]);
}

/* ------------------------------ pollers ------------------------------ */

// Each poller waits for its answer before asking again, so a slow one
// (a changed frame is 150 kB) cannot pile requests up.
function every(ms, poll) {
  (async function loop() { await poll(); setTimeout(loop, ms); })();
}
every(500, pollDisplay);
every(500, pollUart);
every(1000, pollState);
every(1000, pollGpio);
every(1500, pollLens);
</script>
</body>
</html>
"""


if __name__ == "__main__":
    sys.exit(main())
