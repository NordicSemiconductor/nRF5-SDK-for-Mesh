# Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.serialization import \
    Encoding, PrivateFormat, PublicFormat, NoEncryption, \
    load_der_private_key, load_der_public_key
from binascii import hexlify

import aci.aci_cmd as cmd
from aci.aci_evt import Event


PRIVATE_BYTES_START = 36
PRIVATE_BYTES_END = PRIVATE_BYTES_START + 32

PROV_FAILED_ERRORS = [
    "INVALID ERROR CODE",
    "INVALID_PDU",
    "INVALID_FORMAT",
    "UNEXPECTED_PDU",
    "CONFIRMATION_FAILED",
    "OUT_OF_RESOURCES",
    "DECRYPTION_FAILED",
    "UNEXPECTED_ERROR",
    "CANNOT_ASSIGN_ADDR"
]


# Useful reading:
# https://security.stackexchange.com/questions/84327/converting-ecc-private-key-to-pkcs1-format
def raw_to_public_key(public_bytes):
    assert(len(public_bytes) == 64)
    # A public key in the DER format.
    # We'll simply replace the actual key bytes.
    DER_FMT = b'0Y0\x13\x06\x07*\x86H\xce=\x02\x01\x06\x08*\x86H\xce=\x03\x01\x07\x03B\x00\x044\xfa\xfa+E\xfa}Aj\x9e\x118N\x10\xc8r\x04\xa7e\x1d\xd2JdK\xfa\xcd\x02\xdb{\x90JA-\x0b)\xba\x05N\xa7E\x80D>\xa2\xbc"\xe3k\x89\xd1\x10*ci\x19-\xed|\xb7H\xea=L`'  # NOQA
    key = DER_FMT[:-64]
    key += public_bytes

    public_key = load_der_public_key(key, backend=default_backend())
    return public_key


def raw_to_private_key(private_bytes):
    assert(len(private_bytes) == 32)
    # A private key in PKCS8+DER format.
    # We'll simply replace the actual key bytes.
    PKCS8_FMT = b'0\x81\x87\x02\x01\x000\x13\x06\x07*\x86H\xce=\x02\x01\x06\x08*\x86H\xce=\x03\x01\x07\x04m0k\x02\x01\x01\x04 \xd3e\xef\x9d\xbdc\x89\xe0.K\xc5\x84^P\r:\x9b\xfd\x038 _r`\x17\xac\xf2JJ\xff\x07\x9d\xa1D\x03B\x00\x044\xfa\xfa+E\xfa}Aj\x9e\x118N\x10\xc8r\x04\xa7e\x1d\xd2JdK\xfa\xcd\x02\xdb{\x90JA-\x0b)\xba\x05N\xa7E\x80D>\xa2\xbc"\xe3k\x89\xd1\x10*ci\x19-\xed|\xb7H\xea=L`'  # NOQA
    key = PKCS8_FMT[:PRIVATE_BYTES_START]
    key += private_bytes
    key += PKCS8_FMT[PRIVATE_BYTES_END:]
    private_key = load_der_private_key(
        key, password=None, backend=default_backend())
    return private_key


def public_key_to_raw(public_key):
    public_key_der = public_key.public_bytes(
        Encoding.DER, PublicFormat.SubjectPublicKeyInfo)
    # Public key is the last 64 bytes of the formatted key.
    return public_key_der[len(public_key_der) - 64:]


def private_key_to_raw(private_key):
    private_key_pkcs8 = private_key.private_bytes(
        Encoding.DER, PrivateFormat.PKCS8, NoEncryption())
    # Key is serialized in the PKCS8 format, but we need the raw key bytestring
    # The raw key is found from byte 36 to 68 in the formatted key.
    return private_key_pkcs8[PRIVATE_BYTES_START:PRIVATE_BYTES_END]


class OOBMethod(object):
    NONE = 0x00
    STATIC = 0x01
    OUTPUT = 0x02
    INPUT = 0x03


class ProvisioneeList(object):
    def __init__(self):
        self.__list = []

    def __contains__(self, item):
        return len([x for x in self.__list if x["uuid"] == item["uuid"]]) > 0

    def add(self, unprov_event):
        if unprov_event not in self:
            self.__list.append(unprov_event)

    def pop(self, index=None):
        if index:
            return self.__list.pop(index)
        else:
            return self.__list.pop()


class ProvDevice(object):
    def __init__(self, interactive_device, context_id, auth_data,
                 event_handler, enable_event_filter):
        self.iaci = interactive_device
        self.iaci.acidev.add_packet_recipient(event_handler)
        self.logger = self.iaci.logger
        self.__context_id = context_id
        self.__private_key = None
        self.__public_key = None
        self.__auth_data = bytearray([0]*16)
        # Supress provisioning events
        if enable_event_filter:
            self.iaci.event_filter_add([Event.PROV_UNPROVISIONED_RECEIVED,
                                        Event.PROV_LINK_ESTABLISHED,
                                        Event.PROV_AUTH_REQUEST,
                                        Event.PROV_CAPS_RECEIVED,
                                        Event.PROV_COMPLETE,
                                        Event.PROV_ECDH_REQUEST,
                                        Event.PROV_LINK_CLOSED,
                                        Event.PROV_LINK_ESTABLISHED,
                                        Event.PROV_OUTPUT_REQUEST,
                                        Event.PROV_FAILED])
        self.set_key_pair()

    def set_key_pair(self):
        """Generates a new private-public key pair and sets it for the device.
        """
        self.__private_key = ec.generate_private_key(ec.SECP256R1(),
                                                     default_backend())
        self.__public_key = self.__private_key.public_key()

        private_key_raw = private_key_to_raw(self.__private_key)
        public_key_raw = public_key_to_raw(self.__public_key)

        assert(len(private_key_raw) == 32)
        assert(len(public_key_raw) == 64)

        self.iaci.send(cmd.KeypairSet(private_key_raw, public_key_raw))

    def default_handler(self, event):
        if event._opcode == Event.PROV_ECDH_REQUEST:
            self.logger.info("ECDH request received")
            public_key_peer = raw_to_public_key(event._data["peer_public"])
            private_key = raw_to_private_key(event._data["node_private"])
            shared_secret = private_key.exchange(ec.ECDH(), public_key_peer)
            self.iaci.send(cmd.EcdhSecret(event._data["context_id"],
                                          shared_secret))

        elif event._opcode == Event.PROV_AUTH_REQUEST:
            self.logger.info("Authentication request")
            if event._data["method"] == OOBMethod.NONE:
                pass
            elif event._data["method"] == OOBMethod.STATIC:
                self.logger.info("Providing static data")
                self.iaci.send(cmd.AuthData(self.__context_id,
                                            self.__auth_data))
            else:
                self.logger.error("Unsupported authetication method {}".format(
                    event._data["method"]))

        elif event._opcode == Event.PROV_LINK_ESTABLISHED:
            self.logger.info("Link established")

        elif event._opcode == Event.PROV_LINK_CLOSED:
            self.logger.info("Provisioning link closed")

        elif event._opcode == Event.PROV_LINK_ESTABLISHED:
            self.logger.info("Provisioning link established")

        elif event._opcode == Event.PROV_OUTPUT_REQUEST:
            self.logger.error("Unsupported output request")
        elif event._opcode == Event.PROV_FAILED:
            self.logger.error("Provisioning failed with error {} ({})".format(PROV_FAILED_ERRORS[int(event._data["error_code"])], event._data["error_code"]))

        else:
            pass


class Provisioner(ProvDevice):
    def __init__(self, interactive_device,
                 context_id=0,
                 provisioner_address=0x01,
                 network_key_index=0,
                 network_key=bytearray([0]*16),
                 auth_data=bytearray([0]*16),
                 enable_event_filter=True):
        super(Provisioner, self).__init__(
            interactive_device, context_id, auth_data, self.__event_handler,
            enable_event_filter)

        self.__unprov_list = ProvisioneeList()
        self.__prov_data = {
            "network_key": network_key,
            "network_key_index": network_key_index,
            "iv_index": 0,
            "address": 0x0010,
            "iv_update_flag": 0,
            "key_refresh_flag": 0}
        self.__caps = {}  # Holds the capabilites of provisionee temporarily
        self.__address = provisioner_address
        self.init()

    def init(self):
        self.iaci.send(cmd.AddrLocalUnicastSet(self.__address, 1))
        self.iaci.send(cmd.SubnetAdd(self.__prov_data["network_key_index"],
                                     self.__prov_data["network_key"]))

    def prov_data_set(self, **kwargs):
        for k, v in kwargs.items():
            if k not in self.__prov_data:
                raise ValueError("{} not part of the provisioning data. "
                                 + "Choose on of {}".format(
                                     k, self.__prov_data.keys()))
            self.__prov_data[k] = v

    def prov_data_get(self):
        return self.__prov_data

    def scan_start(self):
        self.iaci.send(cmd.ScanStart())

    def scan_stop(self):
        self.iaci.send(cmd.ScanStop())

    def provision(self, unprovisioned_device_number=None, context_id=0):
        """Starts provisioning of the given unprovisioned device."""
        unprov = self.__unprov_list.pop(unprovisioned_device_number)
        self.iaci.send(cmd.Provision(context_id, unprov["uuid"],
                                     **self.__prov_data))

    def __event_handler(self, event):
        if event._opcode == Event.PROV_UNPROVISIONED_RECEIVED:
            if event._data not in self.__unprov_list:
                uuid_fmt = str(hexlify(event._data["uuid"]), 'ascii').upper()
                self.logger.info(
                    "Received UUID {} with RSSI: -{} dB".format(
                        uuid_fmt, event._data["rssi"]))
                self.__unprov_list.add(event._data)

        elif event._opcode == Event.PROV_CAPS_RECEIVED:
            self.logger.info("Received capabilities")
            self.logger.info("Number of elements: {}".format(
                str(event._data["num_elements"])))
            self.__caps[event._data["context_id"]] = event._data
            self.iaci.send(cmd.OobUse(event._data["context_id"],
                                      OOBMethod.NONE, 0))

        elif event._opcode == Event.PROV_COMPLETE:
            caps = self.__caps[event._data["context_id"]]
            num_elements = caps["num_elements"]
            address_range = "{}-{}".format(hex(event._data["address"]),
                                           hex(event._data["address"]
                                               + num_elements - 1))
            self.logger.info("Provisioning complete")
            self.logger.info("\tAddress(es): " + address_range)
            self.logger.info("\tDevice key: {}".format(
                str(hexlify(event._data["device_key"]), "ascii").upper()))
            self.logger.info("\tNetwork key: {}".format(
                str(hexlify(event._data["net_key"]), "ascii").upper()))

            self.logger.info("Adding device key to subnet 0")
            # Devkey added to subnet 0.
            self.iaci.send(cmd.DevkeyAdd(event._data["address"], 0,
                                         event._data["device_key"]))

            self.logger.info("Adding publication address(es)")
            for i in range(0, num_elements):
                self.iaci.send(cmd.AddrPublicationAdd(event._data["address"]
                                                      + i))
            # Update address to the next in range
            self.__prov_data["address"] += num_elements

        else:
            self.default_handler(event)


class Provisionee(ProvDevice):
    def __init__(self, interactive_device, context_id=0,
                 auth_data=bytearray([0]*16), enable_event_filter=True):
        super(Provisionee, self).__init__(
            interactive_device, context_id, auth_data, self.__event_handler,
            enable_event_filter)
        self.__num_elements = interactive_device.CONFIG.ACCESS_ELEMENT_COUNT
        self.iaci.send(cmd.CapabilitiesSet(self.__num_elements,
                                           0, 0, 0, 0, 0, 0,))

    def listen(self):
        self.iaci.send(cmd.Listen())

    def __event_handler(self, event):
        if event._opcode == Event.PROV_COMPLETE:
            address_range = "{}-{}".format(hex(event._data["address"]),
                                           hex(event._data["address"]
                                               + self.__num_elements - 1))
            self.logger.info("Provisioning complete")
            self.logger.info("\tAddress(es): " + address_range)
            self.logger.info("\tDevice key: {}".format(
                str(hexlify(event._data["device_key"]), "ascii").upper()))
            self.logger.info("\tNetwork key: {}".format(
                str(hexlify(event._data["net_key"]), "ascii").upper()))

            self.logger.info("Adding network key (subnet)")
            self.iaci.send(cmd.SubnetAdd(event._data["net_key_index"],
                                         event._data["net_key"]))

            self.logger.info("Adding device key to subnet 0")
            self.iaci.send(cmd.DevkeyAdd(event._data["address"], 0,
                                         event._data["device_key"]))
            self.logger.info("Setting the local unicast address range")
            self.iaci.send(cmd.AddrLocalUnicastSet(event._data["address"],
                                                   self.__num_elements))

        else:
            self.default_handler(event)
