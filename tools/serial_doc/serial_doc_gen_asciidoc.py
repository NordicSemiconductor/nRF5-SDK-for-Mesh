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

import serial_doc_gen as gen
import sys
import re
import os

TABLE_BORDER = '|=======================\n'

CMD_INTRO = """
== Serial Commands

Serial commands are messages sent from the host controller to the nRF5x. Each
serial command results in a _CMD RSP_ event, indicating whether the command was
successful, and returning any relevant data.

See the link:#cmd_overview[Serial Command Overview] section for a summary of the available
commands with opcodes for communication with the nRF5x SoC. Each command is
explained further in the link:#cmd_details[Serial Command Details] section.

The serial commands are broken into groups:

- *Device:* HW commands for device operation.
- *Application:* A single opcode made available to the application.
- *Configuration:* Configuration of various device parameters, like addresses
  and radio properties.
- *Provisioning:* Provisioning specific commands and operations.
- *Open Mesh:* The set of commands used by the nRF Open Mesh.
- *Mesh:* Bluetooth Mesh commands for controlling a device's behavior on the
  mesh.
- *DFU:* Commands controlling the behavior of the Device Firmware Upgrade part
  of the mesh stack.

"""

CMD_OVERVIEW = """
=== [[cmd_overview]] Serial Command Overview

See the tables below for a list of serial commands available for each command
group in the nRF5x Mesh serial interface, and their opcodes. Each entry links
to their respective "Details" section, where the parameters and effect of each
command will be described.

"""

OVERVIEW_TABLE_OPTS = '[cols="2h,m",options="header"]\n'
DETAILS_TABLE_OPTS = '[cols="m,2,m,m,3",options="header"]\n'

CMD_DETAILS = """
=== [[cmd_details]] Serial Command Details


"""

EVT_INTRO = """
== Serial events

Serial Events are messages sent from the nRF5x to the host controller. Messages
are either sent asynchronously as a result of some interaction in the mesh, or
as a response to a command.

"""


EVT_OVERVIEW = """
=== [[evt_overview]] Serial Event Overview

See the table below for an overiew over the various events pushed by the nRF5x
to the host. Each entry links to their respective "Details" section, where the
parameters and reason for each event will be described.

"""

EVT_DETAILS = """
=== [[evt_details]] Serial Event Details

"""


class AsciiDocGenerator(gen.DocGenerator):
    def __init__(self, basename):
        gen.DocGenerator.__init__(self, basename)
        self.cmd_filename = basename + '_cmd.adoc'
        self.evt_filename = basename + '_evt.adoc'

    def generate(self, parser):
        # generate command file
        with open(self.cmd_filename, 'w') as f:
            f.write(CMD_INTRO)
            f.write(CMD_OVERVIEW)

            # sort command groups based on their opcodes
            range_start = {}
            for cmd in parser.commands:
                if not cmd.group in range_start:
                    range_start[cmd.group] = cmd.opcode
            groups = list(gen.COMMAND_GROUPS)
            groups.sort(key=lambda g: range_start[g])

            for group in groups:
                if len([cmd for cmd in parser.commands if cmd.group == group]) == 0:
                    continue
                f.write('\n')
                f.write(OVERVIEW_TABLE_OPTS)
                f.write('.' + gen.COMMAND_GROUPS[group] + ' Commands\n')
                f.write(TABLE_BORDER)
                f.write('| Command                                 | Opcode\n')
                for cmd in parser.commands:
                    if cmd.group == group:
                        f.write('| link:#cmd_' + hex(cmd.opcode) + '[' + cmd.name + ']' + ' ' * (40 - len(cmd.name)) + '| ' + hex(cmd.opcode) + '\n')
                f.write(TABLE_BORDER)

            f.write(CMD_DETAILS)
            for cmd in parser.commands:
                f.write('==== [[cmd_' + hex(cmd.opcode) + ']]`' + hex(cmd.opcode) + '` ' +  gen.COMMAND_GROUPS[cmd.group] + ' ' + cmd.name + '\n\n')
                f.write('_Total length: ' + cmd.length() + ' byte')
                if cmd.length() != '1':
                    f.write('s')
                f.write('_\n\n')
                f.write(cmd.description + '\n\n')
                if len(cmd.params) is 0:
                    f.write(cmd.name + ' takes no parameters.\n\n')
                else:
                    f.write(DETAILS_TABLE_OPTS)
                    f.write('.' + cmd.name + ' Parameters\n')
                    f.write(TABLE_BORDER)
                    f.write('| Type          | Name                                    | Size | Offset | Description\n')
                    for param in cmd.params:
                        f.write('| ' + param.typerepr() + ' ' * (13 - len(param.typerepr())) + ' | ' + param.name + ' ' * (40-len(param.name)) + '| ' + str(param.lengthrepr()) + ' | ' + str(param.offset) + ' | ' + param.description + '\n')
                    f.write(TABLE_BORDER)
                    f.write('\n')
                # command response:
                f.write('===== Response:\n\n')
                if cmd.response:
                    if len(cmd.response.statuses) > 0:
                        f.write('Status codes: \n\n')
                        f.write('\n\n'.join(['* ' + status for status in cmd.response.statuses]))
                        f.write('\n\n')
                    if cmd.response.params:
                        f.write(DETAILS_TABLE_OPTS)
                        f.write('.' + cmd.name + ' Response Parameters\n')
                        f.write(TABLE_BORDER)
                        f.write('| Type          | Name                                    | Size | Offset | Description\n')
                        for param in cmd.response.params:
                            f.write('| ' + param.typerepr() + ' ' * (13 - len(param.typerepr())) + ' | ' + param.name + ' ' * (40-len(param.name)) + '| ' + str(param.lengthrepr()) + ' | ' + str(param.offset) + ' | ' + param.description + '\n')
                        f.write(TABLE_BORDER)
                        f.write('\n')
                    else:
                        f.write('The response has no parameters.\n')
                    f.write('\n')
                else:
                    f.write('This command does not yield any response.\n\n')
        # generate event file
        with open(self.evt_filename, 'w') as f:
            f.write(EVT_INTRO)
            f.write(EVT_OVERVIEW)
            f.write('\n')
            f.write(OVERVIEW_TABLE_OPTS)
            f.write('.Events\n')
            f.write(TABLE_BORDER)
            f.write('| Event                                   | Opcode\n')
            for evt in parser.events:
                f.write('| link:#evt_' + hex(evt.opcode) + '[' + evt.name + ']' + ' ' * (40 - len(evt.name)) + '| ' + hex(evt.opcode) + '\n')
            f.write(TABLE_BORDER)

            f.write(EVT_DETAILS)
            for evt in parser.events:
                f.write('==== [[evt_' + hex(evt.opcode) + ']]`' + hex(evt.opcode) + '` ' + evt.name + '\n\n')
                f.write('_Total length: ' + evt.length() + ' byte')
                if evt.length() != '1':
                    f.write('s')
                f.write('_\n\n')
                f.write(evt.description + '\n\n')
                if len(evt.params) is 0:
                    f.write(evt.name + ' has no parameters.\n\n')
                else:
                    f.write(DETAILS_TABLE_OPTS)
                    f.write('.' + evt.name + ' Parameters\n')
                    f.write(TABLE_BORDER)
                    f.write('| Type          | Name                                    | Size | Offset | Description\n')
                    for param in evt.params:
                        f.write('| ' + param.typerepr() + ' ' * (13 - len(param.typerepr())) + ' | ' + param.name + ' ' * (40-len(param.name)) + '| ' + str(param.lengthrepr()) + ' | ' + str(param.offset) + ' | ' + param.description + '\n')
                    f.write(TABLE_BORDER)
                    f.write('\n')


if __name__ == '__main__':
    parser = gen.SerialHeaderParser()
    outdir = '.'
    try:
        outarg = sys.argv.index('-o')
        sys.argv.pop(outarg)
        outdir = sys.argv.pop(outarg)
    except:
        pass
    try:
        print("Reading desc file...")
        parser.check_desc_file('serial_desc.json')
        for filename in sys.argv[1:]:
            print("Parsing " + os.path.relpath(filename) + "...")
            parser.parse(filename)
        print("Generating asciidoc...")
        AsciiDocGenerator(outdir + '/serial').generate(parser)
        print("Done.")
        #except Exception as e:
    finally:
        pass
        #print("Exception: " + str(e))
