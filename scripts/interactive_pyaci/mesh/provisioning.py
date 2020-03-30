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

import struct
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.serialization import \
    Encoding, PrivateFormat, PublicFormat, NoEncryption, \
    load_der_private_key, load_der_public_key

import aci.aci_cmd as cmd
from aci.aci_evt import Event
from mesh import types as mt


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


class OOBOutputAction(object):
    BLINK = 0x00
    BEEP = 0x01
    VIBRATE = 0x02
    DISPLAY_NUMERIC = 0x03
    ALPHANUMERIC = 0x04


class OOBInputAction(object):
    PUSH = 0x00
    TWIST = 0x01
    ENTER_NUMBER = 0x02
    ENTER_STRING = 0x03


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
                                        Event.PROV_INVITE_RECEIVED,
                                        Event.PROV_START_RECEIVED,
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
            self.logger.error("Provisioning failed with error {} ({})".format
                              (PROV_FAILED_ERRORS[int(event._data["error_code"])],
                               event._data["error_code"]))

        else:
            pass


class Provisioner(ProvDevice):
    def __init__(self, interactive_device, prov_db,
                 context_id=0,
                 auth_data=[0]*16,
                 enable_event_filter=True):
        super(Provisioner, self).__init__(
            interactive_device, context_id, auth_data, self.__event_handler,
            enable_event_filter)
        self.__address = self.iaci.local_unicast_address_start
        self.unprov_list = []
        self.prov_db = None
        self.__session_data = {}
        self.__next_free_address = None
        self.load(prov_db)

    def load(self, prov_db):
        """Loads the keys from the provisioning datase and sets the local unicast address.

            prov_db : Mesh database.
        """
        self.prov_db = prov_db

        self.__next_free_address = (
            self.prov_db.provisioners[0].allocated_unicast_range[0].low_address)

        for node in self.prov_db.nodes:
            self.__next_free_address = max(self.__next_free_address,
                                           node.unicast_address + len(node.elements))

        self.iaci.send(cmd.AddrLocalUnicastSet(self.__address, 1))

        for key in self.prov_db.net_keys:
            self.iaci.send(cmd.SubnetAdd(key.index,
                                         key.key))
        for key in self.prov_db.app_keys:
            self.iaci.send(cmd.AppkeyAdd(key.index, key.bound_net_key, key.key))

    def scan_start(self):
        """Starts scanning for unprovisioned beacons."""
        self.iaci.send(cmd.ScanStart())

    def scan_stop(self):
        """Stops scanning for unprovisioned beacons-"""
        self.iaci.send(cmd.ScanStop())

    def provision(self, uuid=None, key_index=0, name="", context_id=0, attention_duration_s=0):
        """Starts provisioning of the given unprovisioned device.

        Parameters:
        -----------
            uuid : uint8_t[16]
                16-byte long hexadcimal string or bytearray
            key_index : uint16_t
                NetKey index
            name : string
                Name to give the device (stored in the database)
            conext-id:
                Provisioning context ID to use. Normally no reason to change.
            attention_duration_s : uint8_t
                Time in seconds during which the device will identify itself using any means it can.
        """
        if not uuid:
            uuid = self.unprov_list.pop(0)
        elif isinstance(uuid, str):
            uuid = bytearray.fromhex(uuid)
        elif not isinstance(uuid, bytearray):
            raise TypeError("UUID must be string or bytearray")

        if len(uuid) != 16:
            raise ValueError("UUID must be 16 bytes long")

        netkey = None
        for key in self.prov_db.net_keys:
            if key_index == key.index:
                netkey = key
                break

        if not netkey:
            raise ValueError("No network key found for key index %d" % (key_index))

        self.iaci.send(cmd.Provision(context_id,
                                     uuid,
                                     netkey.key,
                                     netkey.index,
                                     self.prov_db.iv_index,
                                     self.__next_free_address,
                                     self.prov_db.iv_update,
                                     netkey.phase > 0,
                                     attention_duration_s))

        self.__session_data["UUID"] = uuid
        self.__session_data["name"] = name
        self.__session_data["net_keys"] = [netkey.index]
        self.__session_data["unicast_address"] = mt.UnicastAddress(self.__next_free_address)
        self.__session_data["config_complete"] = False
        self.__session_data["security"] = netkey.min_security

    def __event_handler(self, event):
        if event._opcode == Event.PROV_UNPROVISIONED_RECEIVED:
            uuid = event._data["uuid"]
            rssi = event._data["rssi"]
            if uuid not in self.unprov_list:
                self.logger.info(
                    "Received UUID {} with RSSI: {} dB".format(uuid.hex(), rssi))
                self.unprov_list.append(uuid)

        elif event._opcode == Event.PROV_CAPS_RECEIVED:
            element_count = event._data["num_elements"]
            self.logger.info("Received capabilities")
            self.logger.info("Number of elements: {}".format(element_count))

            self.iaci.send(cmd.OobUse(event._data["context_id"], OOBMethod.NONE, 0, 0))
            self.__session_data["elements"] = [mt.Element(i) for i in range(element_count)]

        elif event._opcode == Event.PROV_COMPLETE:
            num_elements = len(self.__session_data["elements"])
            address_range = "{}-{}".format(hex(event._data["address"]),
                                           hex(event._data["address"]
                                               + num_elements - 1))

            self.logger.info("Provisioning complete")
            self.logger.info("\tAddress(es): " + address_range)
            self.logger.info("\tDevice key: {}".format(event._data["device_key"].hex()))
            self.logger.info("\tNetwork key: {}".format(event._data["net_key"].hex()))
            self.logger.info("Adding device key to subnet %d", event._data["net_key_index"])
            # Devkey added to subnet 0.
            self.iaci.send(cmd.DevkeyAdd(event._data["address"], 0,
                                         event._data["device_key"]))

            self.logger.info("Adding publication address of root element")
            self.iaci.send(cmd.AddrPublicationAdd(event._data["address"]))

            self.__session_data["device_key"] = event._data["device_key"]
            self.store(self.__session_data)

            # Update address to the next in range
            self.__next_free_address += num_elements

        else:
            self.default_handler(event)

    def store(self, data):
        self.prov_db.nodes.append(mt.Node(**self.__session_data))
        self.prov_db.store()
        self.__session_data = {}


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
        if event._opcode == Event.PROV_INVITE_RECEIVED:
            attention_duration_s = event._data["attention_duration_s"]
            self.logger.info("Provisioning Invite received")
            self.logger.info("\tAttention Duration [s]: {}".format(attention_duration_s))

        elif event._opcode == Event.PROV_START_RECEIVED:
            self.logger.info("Provisioning Start received")

        elif event._opcode == Event.PROV_COMPLETE:
            address_range = "{}-{}".format(hex(event._data["address"]),
                                           hex(event._data["address"]
                                               + self.__num_elements - 1))
            self.logger.info("Provisioning complete")
            self.logger.info("\tAddress(es): " + address_range)
            self.logger.info("\tDevice key: {}".format(
                event._data["device_key"].hex()))
            self.logger.info("\tNetwork key: {}".format(
                event._data["net_key"].hex()))

            self.logger.info("Adding network key (subnet)")
            self.iaci.send(cmd.SubnetAdd(event._data["net_key_index"],
                                         event._data["net_key"]))

            self.logger.info("Adding device key to subnet 0")
            self.iaci.send(cmd.DevkeyAdd(event._data["address"], 0,
                                         event._data["device_key"]))
            self.logger.info("Setting the local unicast address range")
            self.iaci.send(cmd.AddrLocalUnicastSet(event._data["address"],
                                                   self.__num_elements))

            # A slight hack to update the access "layer" in case
            # anyone wants to try to run this as a provisionee
            for index, element in enumerate(self.iaci.access.elements):
                element.address = event._data["address"] + index

        else:
            self.default_handler(event)
