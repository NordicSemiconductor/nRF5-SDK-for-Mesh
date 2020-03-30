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
###############################################################################
# Serial documentation generator
#
# Will generate everything you need for the world's best serial interface!
#
# Assumptions and gotchas:
# - If the last argument in a parameter structure is named 'data', and is an
#   array, it's variable length, and will be reported with a length of
#   '0..length'. Therefore all variable length data arrays must be named data,
#   and put at the end of the command. If you need the last parameter to be
#   named data and not be variable length, you better find a non-intrusive way
#   of fixing it.
# - The command parameters for each opcode are stated after their opcode
#   #define, in the specific format shown on the existing opcodes. You'll get a
#   warning if you fail to adhere to this format.
# - All serial command opcodes start with SERIAL_OPCODE_CMD_, and all serial
#   event opcodes start with SERIAL_OPCODE_EVT_. All other #defines will be
#   ignored as regular defines.
# - The generator does not support bitwidth specifiers.
# - If your packet is using external types, their length must be specified in
#   the PARAM_LENGTHS dict.
# - If your packet is using external defines in their declarations, they must
#   be specified in the EXTERNAL_DEFINES dict.
# - If you get a warning, do not turn the reporting off, fix the problem.
#   You're doing it wrong.
#
###############################################################################
import sys
import re
import os
import json

PRINT_WARNINGS = True

PARAM_LENGTHS = {
    'uint8_t'               : 1,
    'int8_t'                : 1,
    'uint16_t'              : 2,
    'int16_t'               : 2,
    'uint32_t'              : 4,
    'int32_t'               : 4,

    'uint8_t*'              : 4,
    'int8_t*'               : 4,
    'uint16_t*'             : 4,
    'int16_t*'              : 4,
    'uint32_t*'             : 4,
    'int32_t*'              : 4,
    'nrf_mesh_fwid_t'       : 10,
    'nrf_mesh_dfu_role_t'   : 1,
    'nrf_mesh_dfu_type_t'   : 1,
    'nrf_mesh_dfu_state_t'  : 1,
    'nrf_mesh_dfu_packet_t' : 24,
    'nrf_mesh_tx_token_t'   : 4,
    'access_model_id_t'     : 4,
    'dsm_handle_t'          : 2,
    'access_model_handle_t' : 2,
}

EXTERNAL_DEFINES = {
    'NRF_MESH_UUID_SIZE'                : 16,
    'NRF_MESH_ECDH_KEY_SIZE'            : 32,
    'NRF_MESH_KEY_SIZE'                 : 16,
    'NRF_MESH_ECDH_PUBLIC_KEY_SIZE'     : 64,
    'NRF_MESH_ECDH_PRIVATE_KEY_SIZE'    : 32,
    'NRF_MESH_ECDH_SHARED_SECRET_SIZE'  : 32,
    'NRF_MESH_SERIAL_PAYLOAD_MAXLEN'    : 254,
    'BLE_GAP_ADDR_LEN'                  : 6,
    'NRF_MESH_DFU_SIGNATURE_LEN'        : 64,
    'NRF_MESH_DFU_PUBLIC_KEY_LEN'       : 64,
    'BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH' : 31,
    'NRF_MESH_SERIAL_PACKET_OVERHEAD'   : 1,
}

ENFORCED_CASING = [
'UUID',
'nRF',
'Open Mesh',
'Bluetooth',
'FW',
'RX',
'TX',
'SAR',
'DFU',
'OOB',
'ECDH',
'ID',
'TTL',
'SRC',
'DST',
'ms',
' IV'
]


def error(error):
    if PRINT_WARNINGS:
        print('ERROR: ' + error)
    exit(-1)

def warn(warning):
    if PRINT_WARNINGS:
        print('WARNING: ' + warning)

def namify(name):
    name = name.replace('_', ' ').title()
    for c in ENFORCED_CASING:
        name = re.sub(c, c, name, flags = re.I)
    return name

def sizeof(variable):
    if variable in PARAM_LENGTHS:
        return PARAM_LENGTHS[variable];
    else:
        warn("Trying to get sizeof(" + str(variable) + "), no applicable size found...")
        return 1


class Param(object):
    def __init__(self, typename, name, offset, description='', array_len=1):
        self.typename = typename
        self.name = namify(name)
        self.offset = offset
        self.description = description
        self.array_len = array_len
        self.length = PARAM_LENGTHS[typename] * array_len

    def typerepr(self):
        if self.array_len > 1:
            return self.typename + '[' + str(self.array_len) + ']'
        else:
            return self.typename

    def lengthrepr(self):
        if self.name == 'Data' and self.array_len > 1:
            return '0..' + str(self.length)
        else:
            return str(self.length)

    def __repr__(self):
        ret = '%s %s' % (self.typename, self.name)
        if self.array_len > 1:
            ret += '[%d]' % self.array_len
        return ret

class Packet(object):
    def __init__(self, opcode, name, param_struct_name='', description=''):
        self.opcode = opcode
        self.raw_name = name
        self.name = namify(name)
        self.param_struct_name = param_struct_name
        self.description = description
        self.params = []

    def full_name(self):
        return self.name

    def length(self):
        if len(self.params) == 0:
            return '1'
        if self.params[-1].name == 'Data' and self.params[-1].array_len > 1:
            return str(self.params[-1].offset + 1) + '..' + str(self.params[-1].offset + self.params[-1].length + 1)
        return str(self.params[-1].offset + self.params[-1].length + 1)

    def set_description(self, description):
        self.description = description.replace('\n ', '\n').strip()
        if len(self.description) > 0 and self.description[-1] != '.': # fix punctuation :)
            self.description += '.'
            warn('Added punctuation at the end of the description of ' + self.name + ". You're welcome, by the way.")


    def __repr__(self):
        ret = '0x%02x %s' % (self.opcode, self.name)
        if len(self.params) > 0:
            ret += ' {' + ', '.join([str(param) for param in self.params]) + '}'
        if len(self.description) > 0:
            ret += ' - %s' % self.description
        return ret


class CommandGroup(object):
    def __init__(self, shorthand, name, description):
        self.name = name
        self.shorthand = shorthand
        self.description = description

class CommandResponse(object):
    def __init__(self, params_struct_name, statuses):
        self.statuses = statuses
        self.params_struct_name = params_struct_name
        self.params = []

    def __repr__(self):
        ret = ""
        if self.statuses and len(self.statuses) > 0:
            ret += "[" + ", ".join(self.statuses) + "] "
        if self.params and len(self.params) > 0:
            ret += ' {' + ', '.join([str(param) for param in self.params]) + '}'
        return ret

class Command(Packet):
    def __init__(self, opcode, name, param_struct_name='', description=''):
        self.group = None
        self.response = None
        Packet.__init__(self, opcode, name, param_struct_name, description)

    def full_name(self):
        return ' '.join([self.group.name, self.name])

    def __repr__(self):
        ret = Packet.__repr__(self)
        ret += ' ' * (130-len(ret))
        ret += ' GROUP: ' + self.group.name
        if self.response:
            ret += ' ' * (170-len(ret))
            ret += ' RESPONSE: ' + str(self.response)
        return ret

class SerialHeaderParser(object):
    def __init__(self):
        self.implicit_status_responses = ['INVALID_LENGTH']
        self.groups = []
        self.commands = []
        self.events = []
        self.defines = EXTERNAL_DEFINES
        self.structs = {}
        self.param_lengths = PARAM_LENGTHS
        self.structregex = re.compile('struct\s+(__attribute\S+)*\s*{')
        self.unionregex = re.compile('union\s+(__attribute\S+)*\s*{')

    @staticmethod
    def _strip_comments(string, strip_doc=False):
        comment_formats = [
            ('//', '\n', '///'),
            ('/*', '*/', '/**')
        ]
        for (start, end, exception) in comment_formats:
            progress = 0
            while True:
                first = string.find(start, progress)
                if first == -1:
                    break
                last = string.find(end, first)
                if last == -1:
                    string = string[:first]
                    break
                if strip_doc or not string[first:last].startswith(exception):
                    string = string[:first] + ('\n' * string[first:last].count('\n')) + string[last + len(end):]
                progress = first + len(start)

        return string

    def _find_closing_brace(self, string, start=0):
        struct_start = string.find('{', start)
        nesting = 1
        progress = struct_start + 1
        try:
            while nesting != 0:
                opening = string.find('{', progress)
                closing = string.find('}', progress)
                if opening == -1: opening = 100000000000000000
                if closing == -1: closing = 100000000000000000
                if opening < closing:
                    nesting += 1
                    progress = opening + 1
                elif opening > closing:
                    nesting -= 1
                    progress = closing + 1
                else:
                    raise Exception('Non matching braces around line ' + str(string[:struct_start].count('\n') + 1) + ' in ' + string + '.')
        except KeyboardInterrupt as e:
            print(string)
            raise  e
        return progress

    def _evaluate(self, string):
        string = SerialHeaderParser._strip_comments(string, True)
        counter = 100
        while counter:
            oldstring = string
            sorted_defines = [item for item in self.defines.items()]
            sorted_defines.sort(key=lambda tup: tup[0], reverse=True)
            for define, value in sorted_defines:
                string = string.replace(define, str(value))
            if oldstring == string:
                break
            counter -= 1

        # replace all sizeof(x) functions with sizeof("x"), to treat x as a
        # string. That way, we can look it up.
        string = re.sub(r'sizeof\(([^)]*)\)', r'sizeof("\1")', string)

        try:
            return eval(string)
        except Exception as e:
            raise e
        return 0

    def _find_defines(self, string):
        progress = 0
        define_prefix = '\n#define '
        while True:
            start = string.find(define_prefix, progress)
            if start == -1:
                break
            name = string[start + len(define_prefix):].split()[0]
            progress = start + len(define_prefix)
            valuestart = string[start + len(define_prefix) + len(name) :]
            value = ''
            for line in valuestart.splitlines():
                value += line
                if len(line) == 0 or line[-1] != '\\':
                    break
            value = SerialHeaderParser._strip_comments(value.strip(), True)
            if len(value) is 0:
                value = '1'

            if not name in self.defines:
                self.defines[name] = value

    def _find_opcodes(self, string):
        cmd_prefix = '#define SERIAL_OPCODE_CMD_'
        evt_prefix = '#define SERIAL_OPCODE_EVT_'
        param_struct_name_prefix = '/**< Params: @ref '
        for line in string.splitlines():
            op = None
            if line.startswith(cmd_prefix) and not 'CMD_RANGE' in line and line.split()[2].startswith('(0x'):
                opcode = int(line.split()[2].strip('()'), 16)
                try:
                    group = sorted([group for group in self.groups if line[len(cmd_prefix):].startswith(group.shorthand)], key=lambda group: len(group.shorthand))[-1]
                except:
                    error('Call check_desc() before parse(). Line "' + line + '"')

                name = line[len(cmd_prefix) + len(group.shorthand) + 1:line.find(' ', len(cmd_prefix))]
                for cmd in self.commands:
                    if cmd.full_name() == group.name + ' ' + namify(name):
                        cmd.opcode = opcode
                        op = cmd
                        break
                else:
                    warn('Command ' + group.name + ' ' + name + ' missing entry in description file.')
                    op = Command(opcode, name)
                    op.group = group
                    self.commands.append(op)
            if line.startswith(evt_prefix) and line.split()[2].startswith('(0x'):
                opcode = int(line.split()[2].strip('()'), 16)
                name = line[len(cmd_prefix):line.find(' ', len(cmd_prefix))]
                for evt in self.events:
                    if evt.name == namify(name):
                        op = evt
                        op.opcode = opcode
                        break
                else:
                    warn('Event ' + name + ' missing entry in description file.')
                    op = Packet(opcode, name)
                    self.events.append(op)

            struct_name_index = line.find(param_struct_name_prefix)
            if struct_name_index != -1:
                op.param_struct_name = line[struct_name_index + len(param_struct_name_prefix): line.find(' ', struct_name_index + len(param_struct_name_prefix))]
            elif op and not 'None.' in line:
                warn('OPCODE ' + op.name + ' is missing a parameter reference.')


    def _parse_struct(self, string, struct_name):
        params = []
        total_size = 0
        # get rid of nesting
        while True:
            substruct_start = self.structregex.search(string, re.M)
            if not substruct_start:
                break
            first = substruct_start.start()
            last = self._find_closing_brace(string, first)
            subparams = self._parse_struct(string[first:last], struct_name + '::' + string[last + 1: string.find(';', last)])
            subparams_string = ''
            for param in subparams:
                subparams_string += str(param) + ';\n'

            string = string[:first] + subparams_string + string[string.find(';', last)+1:]
        while True:
            subunion_start = self.unionregex.search(string, re.M)
            if not subunion_start:
                break
            first = subunion_start.start()
            string = string[:first] + string[string.find('{', first):]
            last = self._find_closing_brace(string[first:]) + first
            subname = string[last + 1: string.find(';', last)]
            try:
                description = ''
                description = string[last + 1:].splitlines()[0].split(';')[1]
            except:
                pass

            subparams = self._parse_struct(string[first:last], struct_name + '::' + subname)
            maxparam = subparams[0]
            for param in subparams:
                if param.length > maxparam.length:
                    maxparam = param
            # inject:
            string = string[:first] + 'uint8_t ' + subname + '[' + str(maxparam.length) + ']; ' + description + '\n' + string[string.find(';', last)+len(description):]

        in_comment = False
        description = ''
        for statement in string.strip('{}').splitlines():
            statement = statement.strip()
            if 'union' in statement:
                raise Exception('Found undetected union ' + statement + 'in struct ' + struct_name + '\n' + string)
            elif 'struct' in statement:
                raise Exception('Found undetected struct ' + statement + 'in struct ' + struct_name + '\n' + string)
            elif '}' in statement:
                raise Exception('Found undetected closing brace ' + statement + 'in struct ' + struct_name + '\n' + string)
            elif ':' in statement and not statement.startswith('/**'):
                raise Exception('Bitwidth specifiers are not supported.')

            if statement.startswith('/**'):
                in_comment = True
            if in_comment:
                if statement.startswith('*'):
                    statement = statement[1:].strip()
                description += statement.replace('/**', '').replace('*/', '').strip()
                if statement.endswith('*/'):
                    in_comment = False
                else:
                    description += ' ' # force spacing between lines in block comment
            else:
                elems = statement.split()
                if len(elems) < 2:
                    continue
                datatype = elems[0]
                name = statement[statement.find(datatype) + len(datatype): statement.find(';')].strip()
                array_len = 1
                if '[' in name:
                    array_len = int(self._evaluate(name[name.find('[') + 1:name.find(']')]))
                    name = name.split('[')[0]
                if description == '':
                    try:
                        description = ''
                        description = statement.split(';')[1]
                        description = description.replace('/**<', '').replace('*/', '').strip()
                    except:
                        pass
                if len(description) == 0:
                   warn('No description found for parameter ' + struct_name + '::' + name + ', added default description.')
                   description = namify(name)
                param = Param(datatype, name, total_size, description, array_len)
                params.append(param)
                description = ''
                total_size += param.length
        PARAM_LENGTHS[struct_name] = total_size
        return params

    def _find_enums(self, string):
        enums = re.findall(r'typedef\s+enum\s+(__attribute\S+)*\s*{([^}]+)}\s*([a-zA-Z_]\w*)\s*;', string, re.M)
        for enum in enums:
            statements = SerialHeaderParser._strip_comments(enum[1], True).split(',')
            last_val = -1
            for statement in statements:
                if '=' in statement:
                    [name, value] = [s.strip() for s in statement.split('=')]
                    value = eval(value.replace('U', '').replace('L', ''))
                    last_val = value
                    if len(name) > 0:
                        self.defines[name.strip()] = value
                else:
                    name = statement.strip()
                    if len(name) > 0:
                        self.defines[name] = last_val + 1
                        last_val += 1

            self.param_lengths[enum[2].strip()] = 1



    def _find_structs(self, string):
        progress = 0
        while True:
            typedef = string.find('typedef struct __attribute((packed))', progress)
            if typedef is -1:
                break
            struct_start = string.find('{', typedef)
            struct_end = self._find_closing_brace(string, typedef)
            progress = struct_end

            struct_name = string[struct_end + 1:string.find(';', struct_end)].strip()

            cmd_prefix = 'serial_cmd_'
            evt_prefix = 'serial_evt_'
            params = self._parse_struct(string[struct_start:struct_end], struct_name)
            self.structs[struct_name] = params
            if struct_name.startswith(cmd_prefix):
                for cmd in self.commands:
                    if cmd.param_struct_name == struct_name:
                        cmd.params = params
            elif struct_name.startswith(evt_prefix):
                for evt in self.events:
                    if evt.param_struct_name == struct_name:
                        evt.params = params
            for cmd in self.commands:
                if cmd.response and cmd.response.params_struct_name == struct_name:
                    cmd.response.params = params

    def parse(self, filename):
        header = ''
        with open(filename, 'r') as f:
            header = f.read()
        header = SerialHeaderParser._strip_comments(header)
        self._find_opcodes(header)
        self._find_defines(header)
        self._find_enums(header)
        self._find_structs(header)

    def check_desc_file(self, filename):
        if os.path.exists(filename):
            with open(filename, 'r') as f:
                database = json.load(f)
                for group in database["command_groups"]:
                    group_desc = ""
                    if "description" in group:
                        group_desc = group["description"]
                    else:
                        warn("Group " + group["name"] + " is missing a description.")
                    group_obj = CommandGroup(group["shorthand"], group["name"], group_desc)
                    self.groups.append(group_obj)

                    for cmd in group["commands"]:
                        command_packet = Command(0, cmd["name"])
                        command_packet.group = group_obj
                        command_packet.set_description(cmd["description"])
                        if "response" in cmd:
                            if "params" in cmd["response"] and len(cmd["response"]["params"]) > 0:
                                rsp_params_name = "serial_evt_" + cmd["response"]["params"] + "_t"
                            else:
                                rsp_params_name = ''
                            command_packet.response = CommandResponse(rsp_params_name, cmd["response"]["status"] + self.implicit_status_responses)
                        self.commands.append(command_packet)

                for evt in database["events"]:
                    pkt = Packet(0, evt["name"])
                    pkt.set_description(evt["description"])
                    self.events.append(pkt)

    def verify(self):
        known_cmd_opcodes = []
        for cmd in self.commands:
            if len(cmd.description) == 0:
                warn("Command " + cmd.full_name() + " is missing a description.")
            # for param in cmd.params:
            #     if len(param.description) == 0:
            #         warn("Parameter " + param.name + " in " + cmd.full_name() + " is missing a description.")
            if len(cmd.param_struct_name) != 0 and (not cmd.params or len(cmd.params) == 0):
                warn("Can't find parameters " + cmd.param_struct_name + " for command " + cmd.full_name())
            if cmd.opcode == 0:
                warn("Command " + cmd.full_name() + " has opcode 0x00, is it missing in the header?")
            elif cmd.opcode in known_cmd_opcodes:
                warn("Command " + cmd.full_name() + " isn't the only command with opcode 0x" + format(cmd.opcode, '02x') + ".")
            if cmd.response and cmd.response.params_struct_name != '' and (not cmd.response.params or len(cmd.response.params) == 0):
                warn("Can't find parameters for response to command " + cmd.full_name())
            known_cmd_opcodes.append(cmd.opcode)

        known_evt_opcodes = []
        for evt in self.events:
            if len(evt.description) == 0:
                warn("Event " + evt.full_name() + " is missing a description.")
            # for param in evt.params:
            #     if len(param.description) == 0:
            #         warn("Parameter " + param.name + " in " + evt.full_name() + " is missing a description.")
            if len(evt.param_struct_name) != 0 and (not evt.params or len(evt.params) == 0):
                warn("Can't find parameters " + evt.param_struct_name + " for event " + evt.full_name())
            if evt.opcode == 0:
                warn("Event " + evt.full_name() + " has opcode 0x00, is it missing in the header?")
            elif evt.opcode in known_evt_opcodes:
                warn("Event " + evt.full_name() + " isn't the only event with opcode 0x" + format(evt.opcode, '02x') + ".")
            known_evt_opcodes.append(evt.opcode)

    def __repr__(self):
        ret = 'Commands:\n'
        for cmd in self.commands:
            ret += '\t' + str(cmd) + '\n'
        ret += 'Events:\n'
        for evt in self.events:
            ret += '\t' + str(evt) + '\n'
        return ret

class DocGenerator(object):

    def __init__(self, basename):
        self.basename = basename

    def generate(self, parser):
        raise Exception('Not implemented.')




def test_comment_strip():
    test_str = \
    """
    1 // a comment\n
    2 //another comment\n
    3 //CRLF\r\n
    4 //double \n 5 // trouble\n
    5 //CRLF double \r\nVisible 6 // CRLF trouble\r\n
    //nothing before this one\n
    /* C89-style comment */\n
    /**< Leave this one in */\n
    /**< Leave this one in \n*/\n
    /** Leave this one in \n*/\n
    /// Leave this one as well\n
    /* C89-style comment */Visible\n
    /* Multiline C89-style \n fdf \r comment */\n
    /* Multiline C89-style
       just multiline stuff...
       */\n
    Done
    """
    print('stripped version:')
    print(SerialHeaderParser._strip_comments(test_str))

if __name__ == '__main__':
    parser = SerialHeaderParser()
    for filename in sys.argv[1:]:
        parser.parse(filename)
    print(str(parser))

