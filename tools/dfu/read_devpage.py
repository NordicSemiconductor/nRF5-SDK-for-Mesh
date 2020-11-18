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

import shlex
import sys
import subprocess
import argparse

PADSIZE = 12
infos = {}
infos["7FFFFFFF"] = ("END", 1)
infos["00010011"] = ("KEY", 17)
infos["00020005"] = ("VERSION", 5)
infos["00040002"] = ("FLAGS", 2)
infos["00100003"] = ("SEGMENT_SD", 3)
infos["00110003"] = ("SEGMENT_BL", 3)
infos["00120003"] = ("SEGMENT_APP", 3)
infos["001A0011"] = ("SIGN_SD", 17)
infos["001B0011"] = ("SIGN_BL", 17)
infos["001C0011"] = ("SIGN_APP", 17)
infos["00210006"] = ("BANK_SD", 6)
infos["00220006"] = ("BANK_BL", 6)
infos["00240006"] = ("BANK_APP", 6)
infos["00210016"] = ("BANK_SD_SIGNED", 22)
infos["00220016"] = ("BANK_BL_SIGNED", 22)
infos["00240016"] = ("BANK_APP_SIGNED", 22)

def nrfjprog(args):
    process = subprocess.Popen(shlex.split("nrfjprog "+args), stdout=subprocess.PIPE, universal_newlines=True)
    out, err = process.communicate()
    if process == None or process.returncode != 0:
        print("Error calling nrfjprog with arguments " + args + ".")
        print(out)
        exit(2)
    return str(out)


def read_device_page(device):
    device_to_address = {
        "nrf51422_xxAC": "0x3f000",
        "nrf52832_xxAA": "0x7f000",
        "nrf52833_xxAA": "0xff000",
        "nrf52840_xxAA": "0xff000"
    }
    family = "-f " + device[:len("nrf52")].upper()
    device_page_address = device_to_address[device]
    readout = nrfjprog("--memrd " + device_page_address + " --n 1024 " + family)
    words = ""
    for line in readout.splitlines(True):
        words += line[12:48]
    entry = ""
    step_count = 0
    valid_entry = False
    for word in words.split()[1:]:
        if step_count == 0:
            if word.startswith("0000"):
                step_count = int(word, 16) - 1
                valid_entry = False
                continue

            if word not in infos:
                print("\nUnknown entry " + word + ". Device page invalid!")
                break
            (entry, step_count) = infos[word]
            valid_entry = True
            if entry == "END":
                sys.stdout.write("\n" + entry + ".")
                break
            sys.stdout.write("\n" + entry + ": ")
            for i in range(0, PADSIZE - len(entry)):
                sys.stdout.write(" ")
        elif valid_entry:
            sys.stdout.write(word)
        step_count -= 1


if __name__ == "__main__":
    devices = ["nrf51422_xxAC", "nrf52820_xxAA", "nrf52832_xxAA", "nrf52833_xxAA", "nrf52840_xxAA"]
    p = argparse.ArgumentParser("Device Page Reader")
    p.add_argument("-d", "--device", default=devices[1], action="store", choices=devices, help="Select device")
    args = p.parse_args()
    read_device_page(args.device)

