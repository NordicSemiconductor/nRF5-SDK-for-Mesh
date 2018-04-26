# Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

import sys
import jinja2
import datetime
import os

PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

from deviceutil.deviceutil import \
    to_kb, \
    make_bootloader_for_platforms, \
    set_softdevices_for_platforms, \
    load_softdevies, \
    load_platforms, \
    get_application_limits


SCRIPTNAME = "tools/linker/linker_file_generator.py"
USAGE_STRING = """Python Linker Script Generator
Usage: python {}

This script generates a set of linker/scatter files for ARMCC and GCC.
Both application and bootloader linker files are generated.

NOTE: Run the script from the nRF5 SDK for Mesh root directory.
""".format(SCRIPTNAME)


def configure_linker_file(infile, outfile, config):
    with open(infile, "r") as f:
        t = jinja2.Template(f.read(),  trim_blocks=True, lstrip_blocks=True)
    t.globals['hex'] = hex
    t.globals['to_kb'] = to_kb
    with open(outfile, "w") as f:
        f.write(t.render(config))


def write_bootloader_linker_files(platforms, timestamp, outdir):
    for p in platforms:
        print("Writing bootloader linker files for: {p[name]} to {outdir}".format(p=p, outdir=outdir))

        d = {"platform": p,
             "timestamp": timestamp,
             "scriptname": SCRIPTNAME}

        configure_linker_file("tools/linker/templates/armcc_bootloader.sct.template",
                              "{}/bootloader_{}.sct".format(outdir, p["name"]),
                              d)

        configure_linker_file("tools/linker/templates/gccarmemb_bootloader.ld.template",
                              "{}/bootloader_{}.ld".format(outdir, p["name"]),
                              d)


def write_app_linker_files(platforms, softdevices, timestamp, outdirs):
    d = {}
    for p in platforms:
        for sd in p["softdevices"]:
            if "s110" in sd["name"].lower():
                continue
            print("Writing application linker files for: {p[name]} {sd[name]}".format(p=p, sd=sd))

            d["application"] = get_application_limits(p, sd)
            d["softdevice"] = sd
            d["platform"] = p
            d["scriptname"] = SCRIPTNAME
            d["timestamp"] = timestamp
            for outdir in outdirs:
                print("-- ", outdir)
                configure_linker_file(
                    "tools/linker/templates/armcc.sct.template",
                    "{outdir}/{platform}_{sd}.sct".format(
                        outdir=outdir, platform=p["name"], sd=sd["name"]),
                    d)
                configure_linker_file(
                    "tools/linker/templates/gccarmemb.ld.template",
                    "{outdir}/{platform}_{sd}.ld".format(
                        outdir=outdir, platform=p["name"], sd=sd["name"]),
                    d)


def main():
    softdevices = load_softdevies("tools/configuration/softdevices.json")
    platforms = load_platforms("tools/configuration/platforms.json")

    # No support for nrf52810 yet.
    platforms = [p for p in platforms if "nrf52810" not in p["name"].lower()]

    make_bootloader_for_platforms(platforms)
    set_softdevices_for_platforms(platforms, softdevices)
    timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    write_bootloader_linker_files(platforms, timestamp, "mesh/bootloader/linker")
    write_app_linker_files(platforms, softdevices, timestamp,
                           ["examples/beaconing/linker",
                            "examples/dfu/linker",
                            "examples/enocean_switch/linker",
                            "examples/light_switch/provisioner/linker",
                            "examples/light_switch/client/linker",
                            "examples/light_switch/proxy_server/linker",
                            "examples/light_switch/server/linker",
                            "examples/pb_remote/client/linker",
                            "examples/pb_remote/server/linker",
                            "examples/serial/linker"])


if __name__ == "__main__":
    if "-h" in sys.argv or not os.path.exists(SCRIPTNAME):
        print(USAGE_STRING)
        sys.exit(0)
    else:
        main()
        print("Done.")
