# Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of Nordic Semiconductor ASA nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import enum
import struct
import intelhex
import sys
import json
import os

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from deviceutil.deviceutil import \
    get_application_limits, \
    make_bootloader_for_platforms, \
    set_softdevices_for_platforms, \
    load_softdevies, \
    load_platforms

WORD_SIZE = 4


class BLInfoType(enum.IntEnum):
    INVALID = 0x00
    ECDSA_PUBLIC_KEY = 0x01
    VERSION = 0x02
    JOURNAL = 0x03
    FLAGS = 0x04
    SEGMENT_SD = 0x10
    SEGMENT_BL = 0x11
    SEGMENT_APP = 0x12
    LAST = 0x7FFF


class DevicePageEntry(object):
    def __init__(self, bl_info_type, data):
        #print("Type: ", type(data).__name__)
        if not (isinstance(bl_info_type, BLInfoType) and
                (isinstance(data, bytearray) or isinstance(data, str))):
            raise TypeError("Invalid type %r" % (type(data)))

        self.bl_info_type = bl_info_type
        if (isinstance(data, str)):
           data = bytearray.fromhex(data)
        self.data = self.pad(data)

    @property
    def word_count(self):
        return (len(self.data) + WORD_SIZE) // WORD_SIZE

    def pad(self, data):
        pad_byte_count = (WORD_SIZE - len(data) % WORD_SIZE) % WORD_SIZE
        if pad_byte_count > 0:
            data += bytearray([0xFF] * pad_byte_count)
        return data

    def serialize(self):
        return (bytearray(struct.pack('<HH', self.word_count, self.bl_info_type)
                          + self.data))


class DevicePage(object):
    def __init__(self, platform, softdevice, bootloader_config):
        self.entries = []
        self.generate_entries(platform, softdevice, bootloader_config)
        self.platform = platform
        self.softdevice = softdevice

    def generate_entries(self, platform, softdevice, bootloader_config):
        public_key = bytearray.fromhex(bootloader_config["public_key"]) \
                     if "public_key" in bootloader_config else None
        if public_key:
            self.entries.append(
                DevicePageEntry(BLInfoType.ECDSA_PUBLIC_KEY, public_key))

        app_limits = get_application_limits(platform, softdevice)
        app_segment = bytearray(struct.pack("<II",
                                            app_limits["flash_start"],
                                            app_limits["flash_size"]))
        sd_segment = bytearray(struct.pack("<II",
                                           softdevice["flash_start"],
                                           softdevice["flash_size"]))
        bl_segment = bytearray(struct.pack("<II",
                                           platform["bootloader"]["flash_start"],
                                           platform["bootloader"]["flash_size"]))

        self.entries.append(
            DevicePageEntry(BLInfoType.SEGMENT_APP, app_segment))
        self.entries.append(
            DevicePageEntry(BLInfoType.SEGMENT_SD, sd_segment))
        self.entries.append(
            DevicePageEntry(BLInfoType.SEGMENT_BL, bl_segment))

        version_data = bytearray(struct.pack("<HBBIHI",
                                             int(softdevice["version"], 16),
                                             bootloader_config["bootloader_id"],
                                             bootloader_config["bootloader_version"],
                                             bootloader_config["company_id"],
                                             bootloader_config["application_id"],
                                             bootloader_config["application_version"]))

        #print(version_data)
        self.entries.append(DevicePageEntry(BLInfoType.VERSION,
                                            version_data))

        self.entries.append(DevicePageEntry(BLInfoType.FLAGS,
                                            bytearray(struct.pack('I', 0xFFFFFFFF))))

    def write_hex(self, hexfile):
        # Info page metadata
        raw_data = bytearray(struct.pack("<BBBB", 4, 1, 8, 8))
        raw_data += bytearray().join(map(DevicePageEntry.serialize, self.entries))
        raw_data += bytearray(struct.pack("<HH", 0xFFFF, BLInfoType.LAST))
        hex_output = intelhex.IntelHex()
        hex_output.frombytes(raw_data,
                             self.platform["flash_size"] - self.platform["page_size"])
        hex_output.tofile(hexfile, "hex")


def write_specific_page(platforms, softdevices, args):
    platform = next((p for p in platforms if args.device == p["name"]), None)
    if not platform:
        print("Unknown device: \"%s\" in list %r" % (args.device, [p["name"] for p in platforms]))
        sys.exit(1)

    softdevice = next((s for s in softdevices if args.softdevice == s["name"]), None)
    if not softdevice:
        print("Unknown SoftDevice: \"%s\"" % (args.softdevice))
        sys.exit(1)

    if args.softdevice not in platform["softdevices"]:
        print("Unknown SoftDevice \"%s\" for platform \"%s\"" %
              (args.softdevice, args.device))
        sys.exit(1)

    # Filter out the others...
    platforms = [p for p in platforms
                 if args.device.lower() in p["name"].lower()]

    make_bootloader_for_platforms(platforms)
    set_softdevices_for_platforms(platforms, softdevices)

    # Dict is updated by reference
    # platform = platforms[args.device]
    # softdevice = softdevices[args.softdevice]

    with open(args.bootloader_config, "r") as f:
        bootloader_config = json.load(f)["bootloader_config"]
    device_page = DevicePage(platform, softdevice, bootloader_config)
    if not args.output_file:
        args.output_file = "_".join(["device_page",
                                     self.platform["name"],
                                     self.softdevice["name"]]) + ".hex"

    device_page.write_hex(args.output_file)


def write_all(platforms, softdevices, args):
    make_bootloader_for_platforms(platforms)
    set_softdevices_for_platforms(platforms, softdevices)

    with open(args.bootloader_config, "r") as f:
        bootloader_config = json.load(f)["bootloader_config"]

    for platform in platforms:
        for softdevice in platform["softdevices"]:
            device_page = DevicePage(platform, softdevice, bootloader_config)
            device_page.write_hex(args.output_file)


def main():
    softdevices = load_softdevies(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../configuration/softdevices.json"))
    platforms = load_platforms(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../configuration/platforms.json"))
    sd_str = ''
    for sd in softdevices:
        sd_str += ''.join(sd["name"]) + "\n"
    plt_str = ''
    for plt in platforms:
        plt_str += ''.join(plt["name"]) + '\n'

    SOFTDEVICE = "s132_7.2.0"
    DEVICE = "nrf52832_xxAA"
    parser = argparse.ArgumentParser(description="Device Page Generator")
    parser.add_argument("-d", "--device", help="Select device: " + ''.join(plt_str),
                        default=DEVICE)
    parser.add_argument("-sd", "--softdevice", help="Select SoftDevice: "  + ''.join(sd_str),
                        default=SOFTDEVICE)
    parser.add_argument("-c", "--bootloader-config",
                        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "bootloader_config_default.json"),
                        help="Bootloader configuration file")
    parser.add_argument("-o", "--output-file",
                        help="Output hex file (default: bin/device_page_%s_%s.hex)." % ("<DEVICE>", "<SOFTDEVICE>"),
                        default=False)
    parser.add_argument("--all", default=False, action="store_true",
                        help=("Writes all known device page combinations to "
                              + "\'bin/\'"))
    args = parser.parse_args()
    if not args.output_file:
        args.output_file = "bin/device_page_%s_%s.hex" % (args.device, args.softdevice)

    dirname = os.path.dirname(args.output_file)
    if not os.path.exists(dirname):
        os.mkdir(dirname)

    if args.all:
        write_all(platforms, softdevices, args)
        print("Wrote for device pages for all devices.")
    elif args.softdevice and args.device:
        write_specific_page(platforms, softdevices, args)
        print("Wrote device page for %s with the %s SoftDevice to %s." %
              (args.device, args.softdevice, args.output_file))


if __name__ == "__main__":
    main()
