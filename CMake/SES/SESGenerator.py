#!/usr/bin/env python3

# Usage: python SESGenerator.py <target_configuration>.json <output_directory>
#
# <target_configuration>.json is a json file generated from CMake on the form:
# {
#     "target": {
#         "name": "light_control_client_nrf52832_xxAA_s132_5.0.0",
#         "sources": "main.c;provisioner.c;..",
#         "includes": "include1;include2;..",
#         "definitions":"NRF52;NRF52_SERIES;..",
#     },
#     "platform": {
#         "name": "nrf52832_xxAA",
#         "arch": "cortex-m4f",
#         "flash_size": 524288,
#         "ram_size": 65536,
#     },
#     "softdevice": {
#         "hex_file": "<path-to-s132_nrf52_5.0.0_softdevice.hex>",
#         "flash_size": 143360,
#         "ram_size": 12720
#     }
# }

import jinja2
import sys
import argparse
import json
import os
from collections import namedtuple
from shutil import copyfile

TEST_JSON_STR = """{
    "target": {
        "name": "light_control_client_nrf52832_xxAA_s132_5.0.0",
        "sources": "main.c;provisioner.c",
        "includes": "include1;include2",
        "defines":"NRF52;NRF52_SERIES"
    },
    "platform": {
        "name": "nrf52832_xxAA",
        "arch": "cortex-m4f",
        "flash_size": 524288,
        "ram_size": 65536
    },
    "softdevice": {
        "hex_file": "path-to/s132_nrf52_5.0.0_softdevice.hex",
        "flash_size": 143360,
        "ram_size": 12720
    }
}"""

# Constants
NRF51_BOOTLOADER_FLASH_SIZE = 24576
NRF51_BOOTLOADER_RAM_SIZE = 768
NRF52_BOOTLOADER_FLASH_SIZE = 32768
NRF52_BOOTLOADER_RAM_SIZE = 4096
RAM_ADDRESS_START = 536870912

def application_flash_limits_get(softdevice_flash_size,
                                 bootloader_flash_size,
                                 platform_flash_size):
    return (hex(softdevice_flash_size), hex(platform_flash_size - bootloader_flash_size))

def application_ram_limits_get(softdevice_ram_size,
                               bootloader_ram_size,
                               platform_ram_size):
    return (hex(RAM_ADDRESS_START + softdevice_ram_size), hex(platform_ram_size - bootloader_ram_size))



DataRegion = namedtuple("DataRegion", ["start", "size"])

Target = namedtuple("Target", ["name", "includes", "defines", "sources"])
Platform = namedtuple("Platform", ["name", "arch", "flash_size", "ram_size"])
SoftDevice = namedtuple("Softdevice", ["hex_file", "flash_size", "ram_size"])
Configuration = namedtuple("Configuration", ["target", "platform", "softdevice"])

File = namedtuple("File", ["path"])
Group = namedtuple("Group", ["name", "files", "match_string"])


GROUP_TEMPLATES = [
    Group(name="Application", files=[], match_string="examples"),
    Group(name="Core", files=[], match_string="mesh/core"),
    Group(name="Serial", files=[], match_string="mesh/serial"),
    Group(name="Mesh stack", files=[], match_string="mesh/stack"),
    Group(name="GATT", files=[], match_string="mesh/gatt"),
    Group(name="DFU", files=[], match_string="mesh/dfu"),
    Group(name="Toolchain", files=[File("$(StudioDir)/source/thumb_crt0.s")], match_string="toolchain"),
    Group(name="Access", files=[], match_string="mesh/access"),
    Group(name="Bearer", files=[], match_string="mesh/bearer"),
    Group(name="SEGGER RTT", files=[], match_string="rtt"),
    Group(name="uECC", files=[], match_string="micro-ecc"),
    Group(name="nRF5 SDK", files=[], match_string="$(SDK_ROOT"),
    Group(name="Provisioning", files=[], match_string="mesh/prov"),
    Group(name="Configuration Model", files=[], match_string="models/foundation/config"),
    Group(name="Health Model", files=[], match_string="models/foundation/health"),
    Group(name="Generic OnOff Model", files=[], match_string="models/model_spec/generic_onoff"),
    Group(name="Simple OnOff Model", files=[], match_string="models/vendor/simple_on_off"),
    Group(name="Remote provisioning Model", files=[], match_string="models/proprietary/pb_remote")]

def unix_relative_path_get(path1, path2):
    if not path1.startswith('$('):
        path1 = os.path.relpath(path1, path2)
    return path1.replace("\\", "/")

def load_config(input_file):
    with open(input_file, "r") as f:
        config = json.load(f)
    return config

def load_softdevice(sd_config):
    with open(sd_config["definition_file"], "r") as f:
        config = json.load(f)
    return [sd for sd in config["softdevices"] if sd["name"] == sd_config["name"]][0]

def load_platform(platform_config):
    with open(platform_config["definition_file"], "r") as f:
        config = json.load(f)
    return [platform for platform in config["platforms"] if platform["name"] == platform_config["name"]][0]

def create_file_groups(files, out_dir):
    other = Group(name="Other", files=[], match_string=None)
    groups = GROUP_TEMPLATES[:]
    for f in files:
        found_group = False
        if "gcc_startup" in f.lower() or "arm_startup" in f.lower():
            continue

        for g in groups:
            if g.match_string in f:
                f = unix_relative_path_get(f, out_dir)
                g.files.append(File(f))
                found_group = True
                break
        if not found_group:
            f = unix_relative_path_get(f, out_dir)
            other.files.append(File(f))

    groups.append(other)
    # Remove empty groups
    for g in groups[:]:
        if len(g.files) == 0:
            groups.remove(g)

    return groups

def calculate_flash_limits(config):
    bl_flash_size = NRF51_BOOTLOADER_FLASH_SIZE if "nrf51" in config["platform"]["config"]["name"].lower() else NRF52_BOOTLOADER_FLASH_SIZE
    bl_flash_size = bl_flash_size if "nrf52810_xxAA" not in config["platform"]["config"]["name"] else 0
    flash_limits = application_flash_limits_get(config["softdevice"]["config"]["flash_size"], bl_flash_size, config["platform"]["config"]["flash_size"])
    return DataRegion(*flash_limits)

def calculate_ram_limits(config):
    bl_ram_size = NRF51_BOOTLOADER_RAM_SIZE if "nrf51" in config["platform"]["config"]["name"].lower() else NRF52_BOOTLOADER_RAM_SIZE
    bl_ram_size = bl_ram_size if "nrf52810_xxAA" not in config["platform"]["config"]["name"] else 0
    ram_limits = application_ram_limits_get(config["softdevice"]["config"]["ram_size"], bl_ram_size, config["platform"]["config"]["ram_size"])
    return DataRegion(*ram_limits)

def generate_ses_project(config, out_dir="."):
    files = config["target"]["sources"].split(";")
    config["target"]["includes"] = [unix_relative_path_get(i, out_dir) for i in config["target"]["includes"].split(";")]
    config["target"]["heap_size"] = 1024
    config["target"]["stack_size"] = 2048
    config["target"]["groups"] = create_file_groups(files, out_dir)
    config["target"]["flash"] = calculate_flash_limits(config)
    config["target"]["ram"] = calculate_ram_limits(config)
    config["platform"]["fpu"] = config["platform"]["config"]["arch"] == "cortex-m4f"
    config["softdevice"]["hex_file"] = unix_relative_path_get(config["softdevice"]["hex_file"], out_dir)
    config["sdk_default_path"] = unix_relative_path_get('../../../nRF5_SDK_17.0.2_d674dde', out_dir)
    s = ""

    with open("ses.xml", "r") as f:
        s = f.read()

    t = jinja2.Template(s)
    s = t.render(config)

    return s

def generate_ses_session(out_dir):
    session_file_contents = ['<!DOCTYPE CrossStudio_Session_File>',
                             '<session>',
                             '\t<Files>',
                             '\t\t<SessionOpenFile path="{}"/>',
                             '\t</Files>',
                             '</session>']
    return '\n'.join(session_file_contents).format(unix_relative_path_get('../../doc/getting_started/SES.md', out_dir))



def test():
    config = json.loads(TEST_JSON_STR)

    print(config)
    s = generate_ses_project(config)
    with open("test.xml", "w") as f:
        f.write(s)
    print ("Done")

def main():
    input_file = sys.argv[1]
    out_dir = sys.argv[2]

    config = load_config(input_file)
    config["softdevice"]["config"] = load_softdevice(config["softdevice"])
    config["platform"]["config"] = load_platform(config["platform"])
    ses_project = generate_ses_project(config, out_dir)

    out_dir += "/"
    # SES doesn't support "." in filenames
    output_filename = out_dir + config["target"]["name"].replace(".", "_")
    project_file = output_filename + ".emProject"
    with open(project_file, "w") as f:
        f.write(ses_project)

    # Create session
    ses_session = generate_ses_session(out_dir)
    session_file = output_filename + ".emSession"
    with open(session_file, "w") as f:
        f.write(ses_session)

    # Generate flash placement:
    copyfile("flash_placement.xml", out_dir + "flash_placement.xml")

    print("Wrote: " +  project_file)

if __name__ == "__main__":
    main()

