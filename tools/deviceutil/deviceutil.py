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

import json

# Constants
KB = 1024
BOOTLOADER_FLASH_SIZE = 22*KB
BOOTLOADER_RAM_SIZE = 768
RESERVED_FLASH_PAGES_COUNT = 2


def to_kb(v):
    return v // 1024


def round_to_nearest_page_size(size, page_size):
    return int(page_size * ((size + page_size - 1) // page_size))


def make_bootloader_for_platforms(platforms):
    for p in platforms:
        bl = {}
        bl["flash_size"] = round_to_nearest_page_size(
            BOOTLOADER_FLASH_SIZE, p["page_size"])
        bl["flash_start"] = (p["flash_size"]
                             - (bl["flash_size"]
                                + RESERVED_FLASH_PAGES_COUNT*p["page_size"]))
        bl["ram_size"] = BOOTLOADER_RAM_SIZE
        bl["ram_start"] = p["ram_start"] + p["ram_size"] - bl["ram_size"]
        p["bootloader"] = dict(bl)


def get_application_limits(platform, softdevice):
    mbr_scratch_area = 4*KB if platform["name"].startswith("nrf52") else 0
    return {"flash_start": softdevice["flash_size"],
            "flash_size": (platform["bootloader"]["flash_start"]
                           - softdevice["flash_size"]
                           - mbr_scratch_area),
            "ram_start": platform["ram_start"] + softdevice["ram_size"],
            "ram_size": ((platform["bootloader"]["ram_start"]
                          - softdevice["ram_size"])
                         - platform["ram_start"])}


def set_softdevices_for_platforms(platforms, softdevices):
    for p in platforms:
        p["softdevices"] = [sd for sd in softdevices
                            if sd["name"] in p["softdevices"]]


def load_softdevies(filename):
    with open(filename, "r") as f:
        d = json.load(f)
    return d["softdevices"]


def load_platforms(filename):
    with open(filename, "r") as f:
        d = json.load(f)["platforms"]

    # No support for nrf51422_xxAB yet.
    return [p for p in d
            if "nrf51422_xxAB" not in p["name"]]
