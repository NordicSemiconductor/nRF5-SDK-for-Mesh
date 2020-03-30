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

CMD_INTRO = """
# Serial commands

@ingroup LIBRARIES
Serial commands are messages sent from the host controller to the nRF5. Most
serial commands result in a _CMD RSP_ event, indicating whether the command was
successful and returning any relevant data depending on the command type.

# Serial command groups {#serial-commands}

The serial commands are broken into the following groups:

"""

CMD_OVERVIEW = """

See the tables in the following subsections for lists of serial commands available for each command
group in the nRF5 mesh serial interface, along with their opcodes. Each entry links
to the respective details section, where the parameters and effects of each
command are described.

---

"""

CMD_DETAILS = """
## Serial command details {#serial-command-details}

This subsection contains detailed description of each serial command, including opcodes, total length,
description, and parameters (if any are taken).

The commands are listed in order corresponding to groups. The group name precedes the name of the command
in each case.

"""

EVT_INTRO = """
# Serial events
"""


EVT_OVERVIEW = """
# Serial event overview {#serial-events}

Serial Events are messages sent from the nRF5 to the host controller. Messages
are sent in one of the following ways:
- asynchronously as a result of some interaction in the mesh;
- as a response to a command.

See the following table for an overview of the various events pushed by the nRF5
to the host. Each entry links to the details section, where the
parameters and reason for each event are described.

"""

EVT_DETAILS = """
---

## Serial event details {#serial-event-details}

This subsection contains detailed description of each serial event, including opcodes, total length,
description, and parameters (if any exist).

"""


class MarkdownGenerator(gen.DocGenerator):
    def __init__(self, basename):
        gen.DocGenerator.__init__(self, basename)
        self.cmd_filename = basename + '_cmd.md'
        self.evt_filename = basename + '_evt.md'

    def generate(self, parser):
        # generate command file
        with open(self.cmd_filename, 'w') as f:
            f.write(CMD_INTRO)
            for group in parser.groups:
                f.write('- [' + group.name + '](#' + group.name.lower().replace(' ', '-') + '-commands' + '): ' + group.description + '\n\n')
            f.write(CMD_OVERVIEW)

            # sort command groups based on their opcodes
            range_start = {}
            for cmd in parser.commands:
                if not cmd.group in range_start:
                    range_start[cmd.group.shorthand] = cmd.opcode
            groups = list(parser.groups)
            groups.sort(key=lambda g: range_start[g.shorthand])

            for group in groups:
                if len([cmd for cmd in parser.commands if cmd.group == group]) == 0:
                    continue
                f.write('\n')
                f.write('## ' + group.name + ' commands {#' + group.name.lower().replace(' ', '-') + '-commands' + '}\n\n')
                f.write('Command                                 | Opcode\n')
                f.write('----------------------------------------|-------\n')
                for cmd in parser.commands:
                    if cmd.group == group:
                        f.write('[' + cmd.name + '](#' + cmd.full_name().lower().replace(' ', '-') + ')' + ' ' * (40 - len(cmd.full_name())) + '| `' + '0x' + format(cmd.opcode, '02x') + '`\n')
                f.write('\n')
                f.write('---')
                f.write('\n')

            f.write(CMD_DETAILS)
            for cmd in parser.commands:
                f.write('---')
                f.write('\n')
                f.write('### ' + cmd.full_name() + ' {#' + cmd.full_name().lower().replace(' ', '-') + '}\n\n')
                f.write('_Opcode:_ `' + '0x' + format(cmd.opcode, '02x') + '`\n\n')
                f.write('_Total length:_ ' + cmd.length() + ' byte')
                if cmd.length() != '1':
                    f.write('s')
                f.write('\n\n')
                f.write(cmd.description + '\n\n')
                if len(cmd.params) is 0:
                    f.write('_' + cmd.name + ' takes no parameters._\n\n')
                else:
                    f.write('_' + cmd.name + ' Parameters:_\n\n')
                    f.write('Type          | Name                                    | Size | Offset | Description\n')
                    f.write('--------------|-----------------------------------------|------|--------|------------\n')
                    for param in cmd.params:
                        f.write('`' + param.typerepr() + '`' + ' ' * (11 - len(param.typerepr())) + ' | '
                                + param.name + ' ' * (40-len(param.name)) + '| '
                                + str(param.lengthrepr()) + ' ' * (4 - len(str(param.lengthrepr()))) + ' | '
                                + str(param.offset) + ' ' * (6 - len(str(param.offset))) + ' | '
                                + param.description + '\n')
                    f.write('\n')
                # command response:
                f.write('#### Response\n\n')
                if cmd.response:
                    if len(cmd.response.statuses) > 0:
                        f.write('Potential status codes:\n\n')
                        f.write('\n\n'.join(['- `' + status + '`' for status in cmd.response.statuses]))
                        f.write('\n\n')
                    if len(cmd.response.params) > 0:
                        f.write('_' + cmd.name + ' Response Parameters_\n\n')
                        f.write('Type          | Name                                    | Size | Offset | Description\n')
                        f.write('--------------|-----------------------------------------|------|--------|------------\n')
                        for param in cmd.response.params:
                            f.write('`' + param.typerepr() + '`' + ' ' * (11 - len(param.typerepr())) + ' | '
                                    + param.name + ' ' * (40-len(param.name)) + '| '
                                    + str(param.lengthrepr()) + ' ' * (4 - len(str(param.lengthrepr()))) + ' | '
                                    + str(param.offset) + ' ' * (6 - len(str(param.offset))) + ' | '
                                    + param.description + '\n')
                        f.write('\n')
                    else:
                        f.write('_The response has no parameters._\n')
                    f.write('\n')
                else:
                    f.write('_This command does not yield any response._\n\n')
        # generate event file
        with open(self.evt_filename, 'w') as f:
            f.write(EVT_INTRO)
            f.write(EVT_OVERVIEW)
            f.write('\n')
            f.write('Event                                                                    | Opcode\n')
            f.write('-------------------------------------------------------------------------|-------\n')
            for evt in parser.events:
                evtname = '[' + evt.name + ']' + '(#' + evt.name.lower().replace(' ', '-') + ')'
                f.write(evtname + ' ' * (73 - len(evtname)) + '| ' + '0x' + format(evt.opcode, '02x') + '\n')

            f.write(EVT_DETAILS)
            for evt in parser.events:
                f.write('### ' + evt.name + '          {#' + evt.name.lower().replace(' ', '-') + '}\n\n')
                f.write('_Opcode:_ `' + '0x' + format(evt.opcode, '02x') + '`\n\n')
                f.write('_Total length:_ ' + evt.length() + ' byte')
                if evt.length() != '1':
                    f.write('s')
                f.write('\n\n')
                f.write(evt.description + '\n\n')
                if len(evt.params) is 0:
                    f.write('_' + evt.name + ' has no parameters._\n\n')
                    f.write('\n')
                    f.write('---')
                    f.write('\n')
                else:
                    f.write('_' + evt.name + ' Parameters:_\n\n')
                    f.write('Type              | Name                                    | Size  | Offset | Description\n')
                    f.write('------------------|-----------------------------------------|-------|--------|------------\n')
                    for param in evt.params:
                        f.write('`' + param.typerepr() + '`' + ' ' * (15 - len(param.typerepr())) + ' | '
                                + param.name + ' ' * (40-len(param.name)) + '| '
                                + str(param.lengthrepr()) + ' ' * (5 - len(str(param.lengthrepr()))) + ' | '
                                + str(param.offset) + ' ' * (6 - len(str(param.offset))) + ' | '
                                + param.description + '\n')
                    f.write('\n')
                    f.write('---')
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
        print("Verifying...")
        parser.verify()
        print("Generating markdown...")
        MarkdownGenerator(outdir + '/serial').generate(parser)
        print("Done.")
    finally:
        pass
