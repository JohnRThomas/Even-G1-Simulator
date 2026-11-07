# Even G1 Simulator

Runs a prebuilt Even Realities G1 firmware on an emulated pair of lenses: two
nRF5340s with the G1's board peripherals, one the master lens and one the slave.

## Setup

Requirements: Python 3 and [Renode](https://renode.io) 1.16 or newer.

```sh
./setup.sh
```

`setup.sh` installs Renode 1.16.1 for Linux x86_64, on other systems install Renode yourself.

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
| `--phone` | switch the virtual phone on: it connects to each of the two lenses over BLE and pairs with it |
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
| `--phone` | switch the virtual phone on: it connects to each of the two lenses over BLE and pairs with it |
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
