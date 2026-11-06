#!/usr/bin/env python3
"""Generate data/g1_extstore_seed.bin, the settings area of the external
QSPI flash, which g1_lens.resc puts at chip offset 0x130000 of a lens's
flash when it creates the lens's flash file.

It holds the four records the firmware reads at boot, filled in as on a lens
that has been through the factory and paired with a phone, so that the
firmware does not take its first-boot path on an erased flash:

  0x130000  system (factory) settings   struct sys_settings, 0x98 bytes
  0x131000  notification whitelist      empty
  0x134000  user settings               struct usr_settings, 0x2c bytes
  0x13e000  app settings                magic, language, dashboard start mode

Field names, offsets and defaults are those of the v1.66 source
(src/storage/storage.h, settings_records.c, setting_store.c). Edit the values
below and run this script to change what an emulated lens is set up as, then
delete data/g1_extstore_left.bin and data/g1_extstore_right.bin: a lens
whose flash file exists keeps the settings in that file.

Python 3, standard library only.
"""
import os
import struct

BASE = 0x130000
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data", "g1_extstore_seed.bin")

# ESB pairing with the second lens. The channel is also the pair number in
# the advertising name ("Even G1_34_...").
ESB_CHANNEL = 34
ESB_MASTER_ADDR = 0x41
ESB_SLAVE_ADDR = 0x42

# The phone the lens is bound to: address type, then the address in on-air
# (least significant byte first) order. This is the address the virtual phone
# (models/BLE_VirtualCentral.cs) connects from; any other central is
# disconnected as "invalid mac". All 00 or all FF means "not bound", which
# makes the firmware show its pairing prompt instead of the dashboard.
BONDED_ADDR = bytes([0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66])


def sys_settings():
    s = bytearray(0x98)
    struct.pack_into("<I", s, 0x00, 1)              # valid: not a first boot
    struct.pack_into("<I", s, 0x04, 0)              # aging_discharge_time_s
    s[0x08] = ESB_CHANNEL                           # esb_channel
    struct.pack_into("<I", s, 0x0C, 0)              # imu_base
    struct.pack_into("<I", s, 0x10, 80000)          # lux_base_max
    s[0x14] = 0                                     # panel_voltage
    s[0x15:0x35] = b"\xff" * 32                     # sn: unset, keeps the built-in one
    s[0x35:0x55] = b"\xff" * 32                     # name: unset, keeps the built-in one
    s[0x55] = ESB_MASTER_ADDR                       # esb_master_addr
    s[0x56] = ESB_SLAVE_ADDR                        # esb_slave_addr
    s[0x57] = 0                                     # aging_test_count
    s[0x58] = 0                                     # aging_start_soc
    struct.pack_into("<I", s, 0x5C, 0)              # aging_charge_time_s
    struct.pack_into("<HH", s, 0x60, 0x20, 0x20)    # canvas_x, canvas_y
    struct.pack_into("<IIII", s, 0x64, 0, 0, 0, 0)  # lux_base[3], lux_base_spare
    struct.pack_into("<HHH", s, 0x74, 0x3F, 0x3F, 0x3F)   # lum_base[3]
    struct.pack_into("<I", s, 0x7C, 128000)         # luminance_curve
    struct.pack_into("<I", s, 0x80, 0)              # luminance_curve_a1
    # 0x84..0x97: aging-test bookkeeping, all zero
    return bytes(s)


def usr_settings():
    s = bytearray(0x2C)
    s[0x00] = 10                                    # display_mode
    s[0x01] = 0x15                                  # brightness_level
    # wakeup_angle 0 with imu_wear 0 puts the head-up window at a pitch of
    # -96.5..-66.5 degrees, which the IMU model's "don" gesture (ending at
    # about -81) is centred in. The factory values (20 and 1) give -83..-53.
    s[0x02] = 0                                     # wakeup_angle
    s[0x03] = 1                                     # dof3_enable
    struct.pack_into("<I", s, 0x04, 400)            # imu_div
    s[0x08] = 1                                     # auto_brightness
    s[0x09] = 0                                     # bonded
    # check_mode 0 makes the firmware treat the lens as worn whatever the
    # ESB signal strength is; with 1 a lens with a weak or no partner signal
    # counts as taken off and its display stays in standby.
    s[0x0A] = 0                                     # check_mode
    s[0x0B] = 3                                     # raster_height_gear
    s[0x0C:0x13] = BONDED_ADDR                      # bonded_addr
    s[0x13] = 0                                     # imu_wear
    s[0x14] = 0                                     # message_mode
    s[0x15] = 0                                     # charge_limit_upper
    s[0x16] = 0                                     # charge_limit_lower
    s[0x17] = 0                                     # test_mode: factory test off
    s[0x18] = 3                                     # canvas_distance_gear
    s[0x19] = 15                                    # message_stay_time
    s[0x1A] = 0                                     # double_tap_customize
    struct.pack_into("<I", s, 0x1C, 0)              # user_horiz_pitch
    s[0x20] = 0                                     # date_format
    struct.pack_into("<H", s, 0x22, 100)            # brightness_percent
    s[0x24:0x26] = b"\x00\x00"                      # longpress_head_up_mic
    return bytes(s)


RECORDS = (
    (0x130000, sys_settings()),
    (0x131000, bytes([0x00, 0x00])),                # whitelist: nothing enabled, no apps
    (0x134000, usr_settings()),
    # magic 0xAA, language 2 (the firmware's default), startup_mode 0
    # (dashboard on head-up)
    (0x13E000, bytes([0xAA, 0x02, 0x00])),
)


def main():
    end = max(address + len(data) for address, data in RECORDS)
    blob = bytearray(b"\xff" * (end - BASE))        # erased flash between the records
    for address, data in RECORDS:
        blob[address - BASE:address - BASE + len(data)] = data
    with open(OUT, "wb") as f:
        f.write(blob)
    print("wrote %s (%d bytes, chip offset 0x%x)" % (OUT, len(blob), BASE))


if __name__ == "__main__":
    main()
