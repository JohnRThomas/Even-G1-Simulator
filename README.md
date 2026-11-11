# Even G1 Simulator

Runs a prebuilt Even Realities G1 firmware on an emulated pair of lenses: two
nRF5340s with the G1's board peripherals, one the master lens and one the slave.

## Setup

Requirements: Python 3 and [Renode](https://renode.io) 1.16 or newer.

```sh
./setup.sh
```

`setup.sh` installs Renode 1.16.1 for Linux x86_64, on other systems install Renode yourself.

On Windows x64, run `setup.bat` instead, and start the scripts with `python`, e.g.
`python g1sim.py dfu_application.zip`.

## Running firmware in cli

Using a zipped Zephyr DFU package with `app_update.bin`, `net_core_app_update.bin`, and `manifest.json`:

```sh
./g1sim.py dfu_application.zip
./g1sim.py dfu_application_master.zip dfu_application_slave.zip
```

If one image is specified, both lenses will be loaded with that image. If two images are specified, the first runs on
the master lens and the second on the slave lens.


Using raw bin files:
```sh
./g1sim.py app_core.bin net_core.bin
./g1sim.py app_core_master.bin net_core_master.bin app_core_slave.bin net_core_slave.bin
```

| Options | |
|---|---|
| `--seconds N` | run N seconds of emulated time, then exit |
| `--frame OUT.ppm` | with `--seconds`: save the master lens's display framebuffer on exit |
| `--phone` | switch the virtual phone on: it connects to each of the two lenses over BLE and pairs with it (G1 app protocol) |
| `--phone-g2` | switch on the virtual phone that speaks the G2 app's protocol instead (see below); not with `--phone` |
| `--execute "CMD"` | Renode monitor command to run after loading, before starting (repeatable) |
| `--at "T CMD"` | Renode monitor command to run at emulated time T seconds (repeatable) |
| | a command starting with `slave:` runs on the slave lens |
| `--mac-master MAC` | MAC of the master lens (default `E9:A4:9E:29:70:D9`) |
| `--mac-slave MAC` | MAC of the slave lens (default `F2:65:0B:5B:EB:85`) |
| | the firmware sets the top two bits of the first byte; the last three bytes end the advertised name |
| `--renode PATH` | Renode executable (otherwise `$RENODE`, `.renode/`, `PATH`) |

# CLI Input/Ouput
The script will drop you to the Renode monitor prompt on the master. Type `quit` to exit.

UART output of both cores of both lenses is printed as Renode log lines:
 - Master app: `g1/uart0`
 - Master net: `g1/uart_net`
 - Slave  app: `g1_slave/uart0`
 - Slave  net: `g1_slave/uart_net`

## Browser Simulator

The simulator has a local front end that can be viewed through a web browser. 
![alt text](web.png)
Start the simulator with:
```sh
./g1_sim_web.py application_update.zip       # then open 
```
Then navigate to: http://127.0.0.1:8340/

| Options | |
|---|---|
| `--port N` | HTTP port on 127.0.0.1 (default 8340) |
| `--paused` | load the firmware but do not start it |
| `--phone` | switch the virtual phone on: it connects to each of the two lenses over BLE and pairs with it (G1 app protocol) |
| `--phone-g2` | switch on the virtual phone that speaks the G2 app's protocol instead; the page's phone controls then drive that phone |
| `--execute "CMD"` | Renode monitor command to run after loading, before starting (repeatable) |
| | a command starting with `slave:` runs on the slave lens |
| `--mac-master MAC` | MAC of the master lens (default `E9:A4:9E:29:70:D9`) |
| `--mac-slave MAC` | MAC of the slave lens (default `F2:65:0B:5B:EB:85`) |
| | the firmware sets the top two bits of the first byte; the last three bytes end the advertised name |
| `--renode PATH` | Renode executable (otherwise `$RENODE`, `.renode/`, `PATH`) |

## Touch

`Press` and `Release` do the two halves separately. The firmware counts
taps: one, two and three are clicks, five reboot the lens. It ignores the
touch bar while it believes the glasses are not being worn.

## Simulated Phone

Each lens has a virtual phone, off until `--phone` (both lenses, as the phone app holds a link to each) or the lens's `phone` checkbox in the browser switches it on. It is a BLE central that talks to the lens over the emulated radio; the lens's controller, host and GATT server are the firmware's own. The phone

- connects from `66:55:44:33:22:11`, the address the settings seed (`g1_seed.py`) binds the lens to,
- answers the controller's link-layer procedures and asks for 251-byte PDUs,
- exchanges the ATT MTU (251, as the phone app does), finds the Nordic UART Service by its UUIDs and switches its notifications on,
- pairs: LE Secure Connections, Just Works, with bonding. Until then the firmware does not answer packets,
- writes a heartbeat packet after 10 seconds without any other write, because the firmware drops a phone it has not heard from for 30 seconds,
- has no services of its own: the lens's search for ANCS finds nothing, as on an Android phone,
- declines the lens's requests for other connection parameters.

The bond outlives the link and a reboot of the lens: the phone encrypts its next connection with the stored key. A simulator that is started again starts with neither side bonded. If the lens closes the link or goes silent, the phone connects again at its next advertisement.

In the browser, the phone has a console under the lens's UART consoles. It shows what the phone does and every packet, `tx` to the lens and `rx` from it, with the emulated time. The box under it sends a packet: pick one of the listed ones or type the bytes.

```
0x3e - get burial point data        hex bytes, with or without 0x; after " - " a comment
26 06 00 00 06 01
0x4b "some text" 0x00               text in double quotes stands for its bytes
```

A packet typed before the phone is connected and paired is sent as soon as it is.

`MTU` is what the phone asks for at its next connection; `reconnect` makes that happen now. A notification carries three bytes less than the MTU: the firmware cannot send a longer answer at all and logs `Failed to send data over BLE connection` on its console (the 286-byte answer to `0x3e` needs an MTU of 289). The firmware takes an MTU of 498 or more as the start of a firmware update. `forget bond` makes the phone pair afresh at its next connection.

From the Renode monitor the phone is `sysbus.vcentral`:

| | |
|---|---|
| `Enabled true` | the phone's Bluetooth; switching it off closes the link |
| `SendPacket "3E"` | write a packet, as plain hex |
| `Mtu 251`, `HeartbeatSeconds 10`, `PairOnConnect true` | settings; with `PairOnConnect false` the phone pairs only when the lens sends a Security Request |
| `Pair`, `ForgetBond`, `Disconnect` | |
| `Summary`, `Connected`, `Encrypted`, `Bonded`, `AttMtu` | state |

`logLevel 0 sysbus.vcentral` adds the link-layer control, L2CAP, ATT and SMP PDUs to the Renode log.

While a phone is on, the emulation runs with a time quantum of 50 us instead of Renode's 100 us, which makes it about a third slower. Within a quantum a core's timers can be late by up to its length while another core is busy, and the SoftDevice Controller stops when it finds its radio timing late by more than some tens of microseconds: at 100 us a lens with a connection asserts within seconds and reboots.

## Simulated G2 Phone

`--phone-g2` switches on a second virtual phone (`sysbus.vcentral_g2`, model `BLE_VirtualCentralG2`) instead of the one above. It is for firmware that speaks the protocol of the Even G2 app: the same link layer, L2CAP, ATT and pairing as the G1 phone, and on top of them

- the Even service, UUIDs `00002760-08c2-11e1-9073-0e8ac72eXXXX`. After discovery, as the app does: one byte `00` is written with response to characteristic `5403`; the lens refuses that until the link is encrypted (ATT error 5, 8, 12 or 15), which makes the phone pair and write again. Then notifications are switched on for `5402` (protocol replies), `0002` (OTA, `OtaNotifications`), `6402` (microphone audio, `AudioNotifications`) and `7402` (file service, `FileNotifications`). Requests are written to `5401`, file-service frames (services `0xC4` to `0xC7`, `SendG2 0xc4 ...`) to `7401`,
- frames `AA 21 seq len total num sid flag <data>` to the lens and `AA 12 ...` from it, the data being the protobuf payload followed by its CRC-16/CCITT-FALSE (little-endian), cut into fragments of at most `FragmentSize` (232) bytes and put together again on the way in; a header-only frame (length 0) is a control frame whose flag byte carries a status,
- pairing (LE Secure Connections, Just Works, bonded) when the `5403` write is refused, or on a Security Request from the lens; a lens without `5403` is paired at once if `PairOnConnect` is set,
- the Even app's bring-up as captured from it, in this order: auth (service `0x80` type 4, `AA 21 mm 0C 01 01 80 00 08 04 10 mm 1A 04 08 01 10 04 crc`, flag 0x00, expected answer type 4 with an empty field 3, retried up to `AuthRetries` times) to both lenses, then to the right lens only (`BringUp`): pipe role (`0x80` type 5), time sync (`0x80` type 128: seconds since 1970 and the time zone in quarter hours, `UnixTime`/`TimeZoneQuarterHours`, 0 = the host's clock, UTC), Even AI config (`0x07` type 10), QuickList full update (`0x0C` type 2), app sync (`0x0D` type 0), settings read (`0x09` type 2; the answer's firmware version and battery are logged), units (`0x09` type 1), onboarding finished (`0x10` type 1), wear status event (`0x10` type 3), watchface info (`0x1F` type 0), case info (`0x81` type 1), notification control (`0x04` type 1), gestures (`0x09` type 1); all with flag `RequestFlag` (0x20),
- a keep-alive (`0x80` type 14, flag 0x00) to both lenses after `HeartbeatSeconds` (120, the app's period) without a write,
- an ATT MTU of 247, as the app asks for (`Mtu`).

The master lens is the right one (the firmware names it `_R_`), so g1sim sets `Side "left"` on the slave lens's phone, which then only authenticates and sends heartbeats, as the app does. Every request carries a "magic" in protobuf field 2 which the lens echoes in its reply; the reply is matched to the request by it (the lens's own sequence byte is not). A request without a reply within `ReplyTimeoutSeconds` (1) is sent again with a new magic `RequestRetries` (1) times, then reported, and the bring-up goes on with the next step; a header-only control frame counts as the answer to the request waiting on its service. Every frame is logged, decoded; a frame with flag bit 0 set is a push from the lens, not a reply:

```
tx service 0x80 flag 0x00 seq 101 auth: 08 04 10 65 1A 04 08 01 10 04
rx service 0x80 flag 0x00 seq 7 reply: 08 04 10 65 1A 00 answers auth
auth: OK
tx service 0x80 flag 0x20 seq 102 pipe role: 08 05 10 66 22 02 08 01
```

From the Renode monitor the G2 phone is `sysbus.vcentral_g2`; `Enabled`, `Mtu`, `HeartbeatSeconds`, `PairOnConnect`, `Pair`, `ForgetBond`, `Disconnect`, `Summary`, `Connected` are as for the G1 phone, and

| | |
|---|---|
| `SendG2 0x80 0x20 "08 0e 10 65 6a 00"` | write a request: service id, flag byte, protobuf payload as hex (may be `""`); the phone adds header, sequence, fragments and CRC, and reports the reply that echoes the payload's field 2 |
| `SendPacket "AA21..."` | write bytes to characteristic `5401` as they are |
| `Authenticate` | run the auth handshake (and the bring-up) again |
| `Side "left"` | this phone talks to the left lens: auth and heartbeats only (default `"right"`) |
| `BringUp`, `OtaNotifications`, `AudioNotifications`, `FileNotifications`, `FragmentSize`, `RequestFlag`, `ReplyTimeoutSeconds`, `RequestRetries`, `AuthRetries`, `UnixTime`, `TimeZoneQuarterHours` | settings |
| `Authenticated`, `BringUpComplete`, `FramesSent`, `FramesReceived`, `CrcErrors`, `AudioNotificationsReceived`, `LensFirmware`, `LensBattery` | state |

`ai_helpers/g2_frame.py` builds and decodes these frames from the command line (`g2_frame.py pb '{1: 14, 2: 102, 13: {}}'` gives the hex for `SendG2`).

In the browser (`g1_sim_web.py ... --phone-g2`) the phone checkbox, MTU, reconnect and console are the G2 phone's; the packet box takes `service id, flag, protobuf bytes` (e.g. `0x80 0x20 08 0e 10 66 6a 00`) and offers a few requests.

With G1 firmware, which has no Even service, the G2 phone connects, finds no `5401`/`5402` and says so.

## Simulated Flash Storage

Each lens keeps its external QSPI flash in a file, so that the firmware can have persistent storage across reboots just like the real glasses.

```
data/g1_extstore_left.bin     the master (left) lens
data/g1_extstore_right.bin    the slave (right) lens
```

A file is a raw image of the whole 32 MiB chip. Every program and erase goes into it at once, so it is up to date however the simulator ends. A file that does not exist is created as a factory-fresh flash: erased, apart from the settings area from `g1_seed.py`. Delete a file to give that lens freshly seeded flash image.

Only one simulator can use these files at a time, so only one set of simulators can be run from the same directory at one time.

## Reboots

When the firmware reboots a lens, that lens's machine is power-cycled: both
of its images are reloaded and its firmware starts again, while the other
lens keeps running. QSPI contents survive; RAM is not cleared, unlike on
hardware. `--execute "sysbus.twim1.npm1300 ResetOnWatchdogExpiry false"`
makes a reboot only log a warning instead.

## Battery

The stock firmware treats a battery whose state of charge stays below 100% without changing for two minutes as a stuck battery and reboots. The emulated battery never changes voltage, so the reboot will happen if you set the value lower than 100%.
