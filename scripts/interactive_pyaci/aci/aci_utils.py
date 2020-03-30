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

STATUS_CODE_LUT = {
    0x00: {"code": "SUCCESS", "description": "The command completed successfully."},
    0x80: {"code": "ERROR_UNKNOWN", "description": "An unknown error occurred."},
    0x81: {"code": "ERROR_INTERNAL", "description": "An internal error occurred, as there's something wrong with the implementation of the command handling."},
    0x82: {"code": "ERROR_CMD_UNKNOWN", "description": "The command is unknown to the Mesh device, or not implemented in this build."},
    0x83: {"code": "ERROR_INVALID_STATE", "description": "The command occurred at a time where the state of the device made it unable to process it."},
    0x84: {"code": "ERROR_INVALID_LENGTH", "description": "The length of the serial command was wrong. Please refer to the detailed documentation of the command in question to get a valid length."},
    0x85: {"code": "ERROR_INVALID_PARAMETER", "description": "One or more of the command parameters were wrong."},
    0x86: {"code": "ERROR_BUSY", "description": "The Mesh device was busy processing a previous command, or a resource required to execute the command was currently occupied for some other purpose."},
    0x87: {"code": "ERROR_INVALID_DATA", "description": "The data given as part of the serial command parameters was invalid, and could not be used in execution of the command."},
    0x8e: {"code": "ERROR_REJECTED", "description": "The command was rejected by the Mesh device, either because of insufficient resources, or because the requested resource was in a state where it couldn't handle the command."},
    0x93: {"code": "ERROR_TIMEOUT", "description": "The command processing was interrupted by a timeout, causing it to abort the command."},
    0x98: {"code": "ERROR_INVALID_KEY_DATA", "description": "The Key data given as part of the command parameters could not be verified."}}

def value_to_barray(value, size=4, big_endian=False):
    barray = bytearray([(value >> i) & 0xFF for i in range(0, size*8, 8)])
    if big_endian:
        barray = barray[::-1]
    return barray


def iterable_to_barray(iterable):
    if isinstance(iterable, str):
        return bytearray(iterable, 'ascii')
    else:
        return bytearray(iterable)


def barray_pop(barray, size=4):
    value = 0
    for i in range(0, 8*size, 8):
        value |= (barray.pop(0) << i)
    return value


def prettify_data(data):
    def format_function(v):
        def prettifyBytearray(v):
            try:
                return v.decode()
            except:
                return v.hex()

        if isinstance(v, bytearray):
            return "'{}'".format(prettifyBytearray(v))
        else:
            return v

    return "{" + "".join("'{}': {}, ".format(k, format_function(v)) for k, v in data.items())[:-2] + "}"


class CommandPacket(object):
    def __init__(self, opcode, data):
        if not isinstance(data, bytearray):
            raise TypeError("Data should be a bytearray")
        else:
            self._opcode = opcode
            self._data = data

    def __str__(self):
        raw_data = self.serialize()
        return "".join("{:02X}".format(b) for b in raw_data)

    def __repr__(self):
        return str(self)

    def __len__(self):
        return len(self._data) + 1

    def serialize(self):
        return bytearray([len(self), self._opcode]) + self._data


class EventPacket(object):
    def __init__(self, event_name, opcode, data):
        if not isinstance(data, dict):
            raise TypeError("Data should be a dict")
        elif not isinstance(event_name, str):
            raise TypeError("Event name should be a string")
        else:
            self._event_name = event_name
            self._opcode = opcode
            self._data = data

    def __str__(self):
        return "{{event: {}, data: {}}}".format(self._event_name, prettify_data(self._data))

    def __repr__(self):
        return str(self)

    def __len__(self):
        return len(self._data)


class ResponsePacket(object):
    def __init__(self, command_name, opcode, data):
        if not isinstance(data, dict):
            raise TypeError("Data should be a dict")
        elif not isinstance(command_name, str):
            raise TypeError("Command name should be a string")
        else:
            self._command_name = command_name
            self._opcode = opcode
            self._data = data

    def __str__(self):
        return "{}: {}".format(self._command_name, prettify_data(self._data))


    def __repr__(self):
        return str(self)


if __name__ == "__main__":
    test_packet = CommandPacket(0x13, bytearray([0x12, 0x11, 0x00]))
    print(test_packet)

    test_event = EventPacket("Echo", 0x82, {"key": [0, 1], "index": 42})
    print(test_event)
