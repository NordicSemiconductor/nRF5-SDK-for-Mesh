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

import enum
import logging

from aci.aci_utils import value_to_barray
from aci.aci_evt import Event
import aci.aci_cmd as cmd


class AccessStatus(enum.Enum):
    SUCCESS = 0x00
    INVALID_ADDRESS = 0x01
    INVALID_MODEL = 0x02
    INVALID_APPKEY_INDEX = 0x03
    INVALID_NETKEY_INDEX = 0x04
    INSUFFICIENT_RESOURCES = 0x05
    KEY_INDEX_ALREADY_STORED = 0x06
    INVALID_PUBLISH_PARAMETERS = 0x07
    NOT_A_SUBSCRIBE_MODEL = 0x08
    STORAGE_FAILURE = 0x09
    FEATURE_NOT_SUPPORTED = 0x0A
    CANNOT_UPDATE = 0x0B
    CANNOT_REMOVE = 0x0C
    CANNOT_BIND = 0x0D
    TEMPORARILY_UNABLE_TO_CHANGE_STATE = 0x0E
    CANNOT_SET = 0x0F
    UNSPECIFIED_ERROR = 0x10
    INVALID_BINDING = 0x11


class Opcode(object):
    # Mask for the two most significant bits that determine the opcode size
    FORMAT_MASK = 0xC0
    FORMAT_1BYTE0 = 0x00        # 1 byte opcode on the form 0b00xx xxxx
    FORMAT_1BYTE1 = 0x40        # 1 byte opcode on the form 0b01xx xxxx
    FORMAT_2BYTE = 0x80
    FORMAT_3BYTE = 0xC0
    FORMAT_INVALID = 0x7F

    def __init__(self, opcode, company_id=None, name=""):
        self.opcode = opcode
        self.company_id = company_id
        self.name = name

        if company_id:
            self.length = 3
        elif opcode > 0x00FF:
            self.length = 2
        else:
            self.length = 1

    def serialize(self):
        if self.company_id:
            return (value_to_barray(self.opcode, 1) +
                    value_to_barray(self.company_id, 2, big_endian=False))
        else:
            return value_to_barray(self.opcode, self.length, big_endian=True)

    def __str__(self):
        return self.serialize().hex()

    def __repr__(self):
        return "{} ({})".format(self.name, str(self))

    def __eq__(self, other):
        return (self.serialize == other.serialize())

    def __neq__(self, other):
        return not self.__eq__()


def opcode_from_message_get(data):
    format_bits = (data[0] & Opcode.FORMAT_MASK)
    if format_bits == Opcode.FORMAT_1BYTE0 or \
       format_bits == Opcode.FORMAT_1BYTE1:
        return bytearray([data[0]])
    elif format_bits == Opcode.FORMAT_2BYTE and len(data) >= 2:
        return bytearray(data[0:2])
    elif format_bits == Opcode.FORMAT_3BYTE and len(data) >= 3:
        return bytearray(data[0:3])
    else:
        return None


class AccessMessage(object):
    def __init__(self, event):
        self.opcode_raw = opcode_from_message_get(event._data["data"])
        self.meta = {k: v for k, v in event._data.items() if k is not "data"}
        self.data = event._data["data"][len(self.opcode_raw):]

    def __str__(self):
        return "Opcode {}, Data {}".format(self.opcode_raw, self.data)

    def __repr__(self):
        return str(self)


class Model(object):
    DEFAULT_TTL = 8
    DEFAULT_FORCE_SEGMENTED = False
    DEFAULT_TRANSMIC_SIZE = 0
    # Use master security materials by default
    DEFAULT_CREDENTIALS_FLAG = 0

    def __init__(self, opcode_and_handler_tuple_list):
        self.handlers = {}
        for opcode, handler in opcode_and_handler_tuple_list:
            self.handlers[str(opcode)] = handler

        self.element = None
        self.key_handle = None
        self.address_handle = None

        # Use root logger for now
        self.logger = logging.getLogger("")
        self.ttl = self.DEFAULT_TTL
        self.force_segmented = self.DEFAULT_FORCE_SEGMENTED
        self.transmic_size = self.DEFAULT_TRANSMIC_SIZE
        self.friendship_credentials_flag = self.DEFAULT_CREDENTIALS_FLAG

    def publish_set(self, key_handle, address_handle):
        """Sets the publication state for the model.

        Parameters
        ----------
            key_handle:     application or device key handle
            address_handle: address handle
        """
        self.key_handle = key_handle
        self.address_handle = address_handle

    def send(self, opcode, data=bytearray()):
        if self.element is None or self.element.access is None:
            raise RuntimeError("This model is not bound to an element.")
        elif self.key_handle is None:
            raise RuntimeError("This model is not bound to a key.")
        elif self.address_handle is None:
            raise RuntimeError("This model is not publishing to a valid address")

        self.logger.debug("Sending opcode: %s, data: %s", opcode, data.hex())
        message = opcode.serialize()
        message += data

        self.element.access.aci.send(
            cmd.PacketSend(self.key_handle,
                           self.element.address,
                           self.address_handle,
                           self.ttl,
                           self.force_segmented,
                           self.transmic_size,
                           self.friendship_credentials_flag,
                           message))


class Element(object):
    def __init__(self, access, address):
        self.models = []
        self.access = access
        self.address = address

    def model_add(self, model):
        if not isinstance(model, Model):
            raise TypeError("Wrong model type")
        else:
            self.models.append(model)
            model.element = self
            model.logger = logging.getLogger("%s.%s" % (
                self.access.aci.acidev.device_name, model.__class__.__name__))


class Access(object):
    def __init__(self, aci, element_address, num_elements=1):
        self.aci = aci
        self.elements = [Element(self, element_address + i) for i in range(num_elements)]
        self.aci.acidev.add_packet_recipient(self.__event_handler)
        self.aci.event_filter_add([Event.MESH_MESSAGE_RECEIVED_UNICAST])

    def model_add(self, model, idx=0):
        self.elements[idx].model_add(model)

    def __event_handler(self, event):
        if event._opcode == Event.MESH_MESSAGE_RECEIVED_UNICAST:
            message = AccessMessage(event)
            element_index = event._data["dst"] - self.elements[0].address
            assert(element_index < len(self.elements) and element_index >= 0)
            for model in self.elements[element_index].models:
                try:
                    opcode = message.opcode_raw
                    handler = model.handlers[opcode.hex()]
                    handler(opcode, message)
                except KeyError:
                    self.aci.logger.debug("Message {} unknown for model {}.".format(message, self))
                    pass
