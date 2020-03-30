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

import json
import struct

from mesh.access import Model, Opcode, AccessStatus
import mesh.types as mt


def log2b(value):
    """Binary log2"""
    log_val = 0
    while value != 0:
        value >>= 1
        log_val += 1
    return log_val


class ConfigurationClient(Model):
    # We ignore some flake8 formatting errors to make the following contructs a bit more readable.
    _APPKEY_ADD                                   = Opcode(0x00)    # noqa: E501,E221
    _APPKEY_DELETE                                = Opcode(0x8000)  # noqa: E501,E221
    _APPKEY_GET                                   = Opcode(0x8001)  # noqa: E501,E221
    _APPKEY_UPDATE                                = Opcode(0x01)    # noqa: E501,E221
    _BEACON_GET                                   = Opcode(0x8009)  # noqa: E501,E221
    _BEACON_SET                                   = Opcode(0x800A)  # noqa: E501,E221
    _COMPOSITION_DATA_GET                         = Opcode(0x8008)  # noqa: E501,E221
    _MODEL_PUBLICATION_SET                        = Opcode(0x03)    # noqa: E501,E221
    _DEFAULT_TTL_GET                              = Opcode(0x800C)  # noqa: E501,E221
    _DEFAULT_TTL_SET                              = Opcode(0x800D)  # noqa: E501,E221
    _FRIEND_GET                                   = Opcode(0x800F)  # noqa: E501,E221
    _FRIEND_SET                                   = Opcode(0x8010)  # noqa: E501,E221
    _GATT_PROXY_GET                               = Opcode(0x8012)  # noqa: E501,E221
    _GATT_PROXY_SET                               = Opcode(0x8013)  # noqa: E501,E221
    _HEARTBEAT_PUBLICATION_GET                    = Opcode(0x8038)  # noqa: E501,E221
    _HEARTBEAT_PUBLICATION_SET                    = Opcode(0x8039)  # noqa: E501,E221
    _HEARTBEAT_SUBSCRIPTION_GET                   = Opcode(0x803A)  # noqa: E501,E221
    _HEARTBEAT_SUBSCRIPTION_SET                   = Opcode(0x803B)  # noqa: E501,E221
    _KEY_REFRESH_PHASE_GET                        = Opcode(0x8015)  # noqa: E501,E221
    _KEY_REFRESH_PHASE_SET                        = Opcode(0x8016)  # noqa: E501,E221
    _LOW_POWER_NODE_POLLTIMEOUT_GET               = Opcode(0x802D)  # noqa: E501,E221
    _MODEL_APP_BIND                               = Opcode(0x803D)  # noqa: E501,E221
    _MODEL_APP_UNBIND                             = Opcode(0x803F)  # noqa: E501,E221
    _MODEL_PUBLICATION_GET                        = Opcode(0x8018)  # noqa: E501,E221
    _MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET        = Opcode(0x801A)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_ADD                       = Opcode(0x801B)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_DELETE                    = Opcode(0x801C)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_DELETE_ALL                = Opcode(0x801D)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_OVERWRITE                 = Opcode(0x801E)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD       = Opcode(0x8020)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE    = Opcode(0x8021)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE = Opcode(0x8022)  # noqa: E501,E221
    _NETKEY_ADD                                   = Opcode(0x8040)  # noqa: E501,E221
    _NETKEY_DELETE                                = Opcode(0x8041)  # noqa: E501,E221
    _NETKEY_GET                                   = Opcode(0x8042)  # noqa: E501,E221
    _NETKEY_UPDATE                                = Opcode(0x8045)  # noqa: E501,E221
    _NETWORK_TRANSMIT_GET                         = Opcode(0x8023)  # noqa: E501,E221
    _NETWORK_TRANSMIT_SET                         = Opcode(0x8024)  # noqa: E501,E221
    _NODE_IDENTITY_GET                            = Opcode(0x8046)  # noqa: E501,E221
    _NODE_IDENTITY_SET                            = Opcode(0x8047)  # noqa: E501,E221
    _NODE_RESET                                   = Opcode(0x8049)  # noqa: E501,E221
    _RELAY_GET                                    = Opcode(0x8026)  # noqa: E501,E221
    _RELAY_SET                                    = Opcode(0x8027)  # noqa: E501,E221
    _SIG_MODEL_APP_GET                            = Opcode(0x804B)  # noqa: E501,E221
    _SIG_MODEL_SUBSCRIPTION_GET                   = Opcode(0x8029)  # noqa: E501,E221
    _VENDOR_MODEL_APP_GET                         = Opcode(0x804D)  # noqa: E501,E221
    _VENDOR_MODEL_SUBSCRIPTION_GET                = Opcode(0x802B)  # noqa: E501,E221

    _APPKEY_LIST                                  = Opcode(0x8002)  # noqa: E501,E221
    _APPKEY_STATUS                                = Opcode(0x8003)  # noqa: E501,E221
    _BEACON_STATUS                                = Opcode(0x800B)  # noqa: E501,E221
    _COMPOSITION_DATA_STATUS                      = Opcode(0x02)    # noqa: E501,E221
    _DEFAULT_TTL_STATUS                           = Opcode(0x800E)  # noqa: E501,E221
    _FRIEND_STATUS                                = Opcode(0x8011)  # noqa: E501,E221
    _GATT_PROXY_STATUS                            = Opcode(0x8014)  # noqa: E501,E221
    _HEARTBEAT_PUBLICATION_STATUS                 = Opcode(0x06)    # noqa: E501,E221
    _HEARTBEAT_SUBSCRIPTION_STATUS                = Opcode(0x803C)  # noqa: E501,E221
    _KEY_REFRESH_PHASE_STATUS                     = Opcode(0x8017)  # noqa: E501,E221
    _LOW_POWER_NODE_POLLTIMEOUT_STATUS            = Opcode(0x802E)  # noqa: E501,E221
    _MODEL_APP_STATUS                             = Opcode(0x803E)  # noqa: E501,E221
    _MODEL_PUBLICATION_STATUS                     = Opcode(0x8019)  # noqa: E501,E221
    _MODEL_SUBSCRIPTION_STATUS                    = Opcode(0x801F)  # noqa: E501,E221
    _NETKEY_LIST                                  = Opcode(0x8043)  # noqa: E501,E221
    _NETKEY_STATUS                                = Opcode(0x8044)  # noqa: E501,E221
    _NETWORK_TRANSMIT_STATUS                      = Opcode(0x8025)  # noqa: E501,E221
    _NODE_IDENTITY_STATUS                         = Opcode(0x8048)  # noqa: E501,E221
    _NODE_RESET_STATUS                            = Opcode(0x804A)  # noqa: E501,E221
    _RELAY_STATUS                                 = Opcode(0x8028)  # noqa: E501,E221
    _SIG_MODEL_APP_LIST                           = Opcode(0x804C)  # noqa: E501,E221
    _SIG_MODEL_SUBSCRIPTION_LIST                  = Opcode(0x802A)  # noqa: E501,E221
    _VENDOR_MODEL_APP_LIST                        = Opcode(0x804E)  # noqa: E501,E221
    _VENDOR_MODEL_SUBSCRIPTION_LIST               = Opcode(0x802C)  # noqa: E501,E221

    def __init__(self, prov_db):
        self.opcodes = [
            (self._APPKEY_LIST                      , self.__appkey_list_handler),                        # noqa: E501,E203
            (self._APPKEY_STATUS                    , self.__appkey_status_handler),                      # noqa: E501,E203
            (self._BEACON_STATUS                    , self.__beacon_status_handler),                      # noqa: E501,E203
            (self._COMPOSITION_DATA_STATUS          , self.__composition_data_status_handler),            # noqa: E501,E203
            (self._DEFAULT_TTL_STATUS               , self.__default_ttl_status_handler),                 # noqa: E501,E203
            (self._FRIEND_STATUS                    , self.__friend_status_handler),                      # noqa: E501,E203
            (self._GATT_PROXY_STATUS                , self.__gatt_proxy_status_handler),                  # noqa: E501,E203
            (self._HEARTBEAT_PUBLICATION_STATUS     , self.__heartbeat_publication_status_handler),       # noqa: E501,E203
            (self._HEARTBEAT_SUBSCRIPTION_STATUS    , self.__heartbeat_subscription_status_handler),      # noqa: E501,E203
            (self._KEY_REFRESH_PHASE_STATUS         , self.__key_refresh_phase_status_handler),           # noqa: E501,E203
            (self._LOW_POWER_NODE_POLLTIMEOUT_STATUS, self.__low_power_node_polltimeout_status_handler),  # noqa: E501,E203
            (self._MODEL_APP_STATUS                 , self.__model_app_status_handler),                   # noqa: E501,E203
            (self._MODEL_PUBLICATION_STATUS         , self.__model_publication_status_handler),           # noqa: E501,E203
            (self._MODEL_SUBSCRIPTION_STATUS        , self.__model_subscription_status_handler),          # noqa: E501,E203
            (self._NETKEY_LIST                      , self.__netkey_list_handler),                        # noqa: E501,E203
            (self._NETKEY_STATUS                    , self.__netkey_status_handler),                      # noqa: E501,E203
            (self._NETWORK_TRANSMIT_STATUS          , self.__network_transmit_status_handler),            # noqa: E501,E203
            (self._NODE_IDENTITY_STATUS             , self.__node_identity_status_handler),               # noqa: E501,E203
            (self._NODE_RESET_STATUS                , self.__node_reset_status_handler),                  # noqa: E501,E203
            (self._RELAY_STATUS                     , self.__relay_status_handler),                       # noqa: E501,E203
            (self._SIG_MODEL_SUBSCRIPTION_LIST      , self.__model_sig_subscription_list_handler),        # noqa: E501,E203
            (self._SIG_MODEL_APP_LIST               , self.__model_sig_app_list_handler),                 # noqa: E501,E203
            (self._VENDOR_MODEL_APP_LIST            , self.__vendor_model_app_list_handler),              # noqa: E501,E203
            (self._VENDOR_MODEL_SUBSCRIPTION_LIST   , self.__vendor_model_subscription_list_handler)]     # noqa: E501,E203

        self.prov_db = prov_db
        self.previous_command = None
        # FIXME: Hack to retain the virtual label UUID until we get an ack.
        self._tmp_address = None
        super(ConfigurationClient, self).__init__(self.opcodes)

    @staticmethod
    def _unpack_key_ind(packed_keys):
        keys = []
        if packed_keys:
            pairs_cnt, single_cnt = (len(packed_keys) // 3, len(packed_keys) % 3 // 2)
            keys = [k for i in range(pairs_cnt) for k in mt.KeyIndex.unpack(packed_keys[i * 3:(i + 1) * 3])]
            if single_cnt > 0:
                keys.append(mt.KeyIndex.unpack(packed_keys[3 * pairs_cnt:])[0])
        return keys

    def composition_data_get(self, page_number=0x00):
        self.send(self._COMPOSITION_DATA_GET, bytearray([page_number]))

    def appkey_add(self, appkey_index=0):
        key = self.prov_db.find_appkey(appkey_index)
        if not key:
            raise ValueError(
                "Could not find appkey with index %d" % (appkey_index))
        netkey_index = key.bound_net_key
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index, appkey_index)
        message += key.key
        self.previous_command = "add"
        self.send(self._APPKEY_ADD, message)

    def appkey_update(self, appkey_index=0):
        key = self.prov_db.find_appkey(appkey_index)
        if not key:
            raise ValueError(
                "Could not find appkey with index %d" % (appkey_index))
        netkey_index = key.bound_net_key
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index, appkey_index)
        message += key.key
        self.previous_command = "update"
        self.send(self._APPKEY_UPDATE, message)

    def appkey_delete(self, appkey_index=0):
        key = self.prov_db.find_appkey(appkey_index)
        if not key:
            raise ValueError(
                "Could not find appkey with index %d" % (appkey_index))
        netkey_index = key.bound_net_key
        key24 = mt.KeyIndex.pack(netkey_index, appkey_index)
        self.previous_command = "delete"
        self.send(self._APPKEY_DELETE, key24)

    def appkey_get(self, netkey_index=0):
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        self.send(self._APPKEY_GET, message)

    def netkey_add(self, netkey_index=0):
        key = self.prov_db.find_netkey(netkey_index)
        if not key:
            raise ValueError(
                "Could not find netkey with index %d" % (netkey_index))
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        message += key.key
        self.previous_command = "add"
        self.send(self._NETKEY_ADD, message)

    def netkey_update(self, netkey_index=0):
        key = self.prov_db.find_netkey(netkey_index)
        if not key:
            raise ValueError(
                "Could not find netkey with index %d" % (netkey_index))
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        message += key.key
        self.previous_command = "update"
        self.send(self._NETKEY_UPDATE, message)

    def netkey_delete(self, netkey_index=0):
        message = mt.KeyIndex.pack(netkey_index)
        self.previous_command = "delete"
        self.send(self._NETKEY_DELETE, message)

    def netkey_get(self):
        self.send(self._NETKEY_GET)

    def model_app_bind(self, element_address, appkey_index, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += mt.KeyIndex.pack(appkey_index)
        message += model_id.pack()
        self.previous_command = "bind"
        self.send(self._MODEL_APP_BIND, message)

    def model_app_unbind(self, element_address, appkey_index, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += mt.KeyIndex.pack(appkey_index)
        message += model_id.pack()
        self.previous_command = "unbind"
        self.send(self._MODEL_APP_UNBIND, message)

    def model_app_get(self, element_address, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += model_id.pack()
        if model_id.company_id:
            self.send(self._VENDOR_MODEL_APP_GET, message)
        else:
            self.send(self._SIG_MODEL_APP_GET, message)

    def model_publication_set(self, element_address, model_id, publish):

        # FIXME: Hack to retain address
        self._tmp_address = publish.address.to_json()

        message = bytearray()
        message += struct.pack("<H", element_address)
        message += publish.pack()
        message += model_id.pack()

        # If it's a bytearray, it's a virtual address.
        if isinstance(publish.address, mt.VirtualAddress):
            self.send(self._MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET, message)
        else:
            self.send(self._MODEL_PUBLICATION_SET, message)

    def model_publication_get(self, element_address, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += model_id.pack()
        self.send(self._MODEL_PUBLICATION_GET, message)

    def subscription_message_get(self, element_address, address, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)

        # FIXME: Hack to retain address
        if isinstance(address, int):
            self._tmp_address = "%04x" % (address)
            address = struct.pack("<H", address)
        else:
            self._tmp_address = address.hex()
        message += address
        message += model_id.pack()
        return message

    def model_subscription_add(self, element_address, address, model_id):
        message = self.subscription_message_get(
            element_address, address, model_id)
        self.previous_command = "add"
        # If it's a bytearray, it's a virtual address
        if isinstance(address, bytearray):
            self.send(self._MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD, message)
        else:
            self.send(self._MODEL_SUBSCRIPTION_ADD, message)

    def model_subscription_delete(self, element_address, address, model_id):
        message = self.subscription_message_get(
            element_address, address, model_id)
        self.previous_command = "delete"
        # If it's a bytearray, it's a virtual address
        if isinstance(address, bytearray):
            self.send(self._MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE, message)
        else:
            self.send(self._MODEL_SUBSCRIPTION_DELETE, message)

    def model_subscription_overwrite(self, element_address, address, model_id):
        message = self.subscription_message_get(
            element_address, address, model_id)
        self.previous_command = "overwrite"
        # If it's a bytearray, it's a virtual address
        if isinstance(address, bytearray):
            self.send(self._MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE, message)
        else:
            self.send(self._MODEL_SUBSCRIPTION_OVERWRITE, message)

    def model_subscription_delete_all(self, element_address, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += model_id.pack()
        self.previous_command = "delete_all"
        self.send(self._MODEL_SUBSCRIPTION_DELETE_ALL, message)

    def model_subscription_get(self, element_address, model_id):
        message = bytearray()
        message += struct.pack("<H", element_address)
        message += model_id.pack()
        if model_id.company_id:
            self.send(self._VENDOR_MODEL_SUBSCRIPTION_GET, message)
        else:
            self.send(self._SIG_MODEL_SUBSCRIPTION_GET, message)

    def key_refresh_phase_get(self, netkey_index):
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        self.send(self._KEY_REFRESH_PHASE_GET, message)

    def key_refresh_phase_set(self, netkey_index):
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        self.send(self._KEY_REFRESH_PHASE_GET, message)

    def node_reset(self):
        self.send(self._NODE_RESET)

    def beacon_get(self):
        self.send(self._BEACON_GET)

    def beacon_set(self, state):
        message = bytearray(struct.pack("<B", int(state)))
        self.send(self._BEACON_SET, message)

    def default_ttl_get(self):
        self.send(self._DEFAULT_TTL_GET)

    def default_ttl_set(self, ttl):
        message = bytearray(struct.pack("<B", ttl))
        self.send(self._DEFAULT_TTL_SET, message)

    def gatt_proxy_get(self):
        self.send(self._GATT_PROXY_GET)

    def gatt_proxy_set(self, state):
        message = bytearray(struct.pack("<B", int(state)))
        self.send(self._GATT_PROXY_SET, message)

    def relay_get(self):
        self.send(self._RELAY_GET)

    def relay_set(self, state, retransmit_count=0, retransmit_interval_steps=0):
        retransmit = (retransmit_count & 0x07 |
                      (retransmit_interval_steps & 0x1f) << 3)
        message = bytearray(struct.pack("<BB", int(state), retransmit))
        self.send(self._RELAY_SET, message)

    def friend_get(self):
        self.send(self._FRIEND_GET)

    def friend_set(self, state):
        message = bytearray()
        message += struct.pack("<B", int(state))
        self.send(self._FRIEND_SET, message)

    def heartbeat_publication_get(self):
        self.send(self._HEARTBEAT_PUBLICATION_GET)

    def heartbeat_publication_set(self, dst, count, period, ttl=64,
                                  feature_bitfield=0, netkey_index=0):
        message = bytearray()
        message += struct.pack(
            "<HBBBHH", dst, log2b(count), log2b(period), ttl, feature_bitfield, netkey_index)
        self.send(self._HEARTBEAT_PUBLICATION_SET, message)

    def heartbeat_subscription_get(self):
        self.send(self._HEARTBEAT_SUBSCRIPTION_GET)

    def heartbeat_subscription_set(self, src, dst, period):
        message = bytearray()
        message += struct.pack("<HHB", src, dst, log2b(period))
        self.send(self._HEARTBEAT_SUBSCRIPTION_SET, message)

    def low_power_node_polltimeout_get(self, lpn_address):
        message = bytearray()
        message += struct.pack("<H", lpn_address)
        self.send(self._LOW_POWER_NODE_POLLTIMEOUT_GET, message)

    def network_transmit_get(self):
        self.send(self._NETWORK_TRANSMIT_GET)

    def network_transmit_set(self, count=0, interval_steps=1):
        message = bytearray()
        message += struct.pack("<B", mt.NetworkTransmit(count, interval_steps).pack())
        self.send(self._NETWORK_TRANSMIT_SET, message)

    def node_identity_get(self, netkey_index):
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        self.send(self._NODE_IDENTITY_GET, message)

    def node_identity_set(self, netkey_index, state):
        message = bytearray()
        message += mt.KeyIndex.pack(netkey_index)
        message += struct.pack("<B", int(state))
        self.send(self._NODE_IDENTITY_SET, message)

    def node_get(self, src):
        for node in self.prov_db.nodes:
            if node.unicast_address == src:
                return node

        raise RuntimeError("Could not find node %04x" % (src))

    def model_get(self, element_address, model_id):
        for node in self.prov_db.nodes:
            beg = node.unicast_address
            end = beg + len(node.elements)
            if beg <= element_address < end:
                index = int(element_address) - int(node.unicast_address)
                element = node.elements[index]
                for model in element.models:
                    if model.model_id == model_id:
                        return model

        raise RuntimeError("Could not find model %r with element address %04x" %
                           (model_id, element_address))

    def db_save(self):
        self.prov_db.store()

    def __composition_data_status_handler(self, opcode, message):
        page = message.data[0]
        data = message.data[1:]
        data = mt.CompositionData().unpack(data)
        node = self.node_get(message.meta["src"])
        node.cid = data["cid"]
        node.pid = data["pid"]
        node.vid = data["vid"]
        node.crpl = data["crpl"]
        node.features = mt.NodeFeatures(**data["features"])
        node.elements = data["elements"]
        self.db_save()
        self.logger.info("Received composition data (page 0x%02x): %s",
                         page, json.dumps(data, indent=2))

    def __heartbeat_publication_status_handler(self, opcode, message):
        status, dst, count_log, period_log, ttl, features, netkey_index = struct.unpack(
            "<BHBBBHH", message.data)
        status = AccessStatus(status)
        self.logger.info("Heartbeat publication status: %s", status)
        if status == AccessStatus.SUCCESS:
            count = 0 if count_log == 0 else 2**(count_log - 1)
            period = 0 if period_log == 0 else 2**(period_log - 1)
            features = {
                "relay": (features & (1 << 0)) > 0,
                "proxy": (features & (1 << 1)) > 0,
                "friend": (features & (1 << 2)) > 0,
                "lowPower": (features & (1 << 3)) > 0
            }
            self.logger.info("Heartbeat publication state: " +
                             "dst: %04x, count: %d, period: %ds, features: %r, subnet: %d",
                             dst, count, period, features, netkey_index)

    def __appkey_status_handler(self, opcode, message):
        status = AccessStatus(message.data[0])
        netkey_index, appkey_index = mt.KeyIndex.unpack(message.data[1:4])
        self.logger.info("Appkey status: %s", status)
        if status in [AccessStatus.SUCCESS, AccessStatus.KEY_INDEX_ALREADY_STORED]:
            node = self.node_get(message.meta["src"])
            if self.previous_command == "add" and appkey_index not in node.app_keys:
                node.app_keys.append(appkey_index)
            elif self.previous_command == "update":
                pass
            elif self.previous_command == "delete" and appkey_index in node.app_keys:
                node.app_keys.remove(appkey_index)
            self.db_save()
            self.logger.info("Appkey %s %d succeded for subnet %d at node %04x",
                             self.previous_command, appkey_index, netkey_index,
                             message.meta["src"])

    def __appkey_list_handler(self, opcode, message):
        status, netkey_index = struct.unpack("<BH", message.data[0:3])
        status = AccessStatus(status)
        self.logger.info("Appkey list status: %s", status)
        if status == AccessStatus.SUCCESS:
            appkeys = ConfigurationClient._unpack_key_ind(message.data[3:])
            node = self.node_get(message.meta["src"])

            # Add newly discovered keys
            for index in appkeys:
                if ((self.prov_db.app_keys[index].bound_net_key == netkey_index and
                     index not in node.app_keys)):
                    node.app_keys.append(index)

            # Remove old dead keys
            for index in node.app_keys:
                if ((self.prov_db.app_keys[index].bound_net_key == netkey_index and
                     index not in appkeys)):
                    node.app_keys.remove(index)

            self.db_save()
            self.logger.info("Node %04x has appkeys: %r",
                             message.meta["src"], appkeys)

    def __beacon_status_handler(self, opcode, message):
        state = struct.unpack("<B", message.data)[0]
        state = bool(state)
        node = self.node_get(message.meta["src"])
        node.secure_network_beacon = mt.FeatureState(state)
        self.db_save()
        self.logger.info("Secure network beacon state: %s",
                         "on" if state else "off")

    def __default_ttl_status_handler(self, opcode, message):
        ttl = struct.unpack("<B", message.data)[0]
        node = self.node_get(message.meta["src"])
        node.default_TTL = mt.TTL(ttl)
        self.db_save()
        self.logger.info("Default TTL: %d", ttl)

    def __friend_status_handler(self, opcode, message):
        state = struct.unpack("<B", message.data)[0]
        node = self.node_get(message.meta["src"])
        node.features.friend = mt.FeatureState(state)
        self.db_save()
        self.logger.info("Friend state: %r", state)

    def __gatt_proxy_status_handler(self, opcode, message):
        state = struct.unpack("<B", message.data)[0]
        node = self.node_get(message.meta["src"])
        node.features.proxy = mt.FeatureState(state)
        self.db_save()
        self.logger.info("Proxy state: %r", state)

    def __key_refresh_phase_status_handler(self, opcode, message):
        status, netkey_index, phase = struct.unpack("<BHB", message.data)
        status = AccessStatus(status)
        self.logger.info("Key refresh status: %s", status)
        if status == AccessStatus.SUCCESS:
            node = self.node_get(message.meta["src"])
            for key in node.net_keys:
                if key.index == netkey_index:
                    key.phase = mt.KeyRefreshPhase(phase)
                    self.db_save()
                    self.logger.info("Netkey phase %r for subnet %r at node %04x",
                                     phase, netkey_index, node.unicastAddress)

    def __model_publication_status_handler(self, opcode, message):
        status, element_address = struct.unpack("<BH", message.data[0:3])
        status = AccessStatus(status)
        self.logger.info("Model publication status: %s", status)
        if status == AccessStatus.SUCCESS:
            publish = mt.Publish.unpack(message.data[3:10])
            if self._tmp_address:
                publish.address = mt.any_address(self._tmp_address)
                self._tmp_address = None
            model = self.model_get(element_address, mt.ModelId.unpack(message.data[10:]))
            model.publish = publish
            self.db_save()
            self.logger.info("Publication status for model %r at element %r to %r",
                             model.model_id, element_address, publish)

    def __model_subscription_status_handler(self, opcode, message):
        status, element_address = struct.unpack("<BH", message.data[0:3])
        status = AccessStatus(status)
        self.logger.info("Model subscription status: %s", status)
        if status == AccessStatus.SUCCESS:
            # address = struct.unpack("<H", message[3:5])
            address = self._tmp_address
            self._tmp_address = None
            model_id = mt.ModelId.unpack(message.data[5:])
            model = self.model_get(element_address, model_id)
            if address not in model.subscribe:
                model.subscribe.append(mt.group_address(address))

            self.db_save()
            self.logger.info("Added subscription %r to model %r at element %04x",
                             address, model_id, element_address)

    def __network_transmit_status_handler(self, opcode, message):
        node = self.node_get(message.meta["src"])
        node.network_transmit = mt.NetworkTransmit.unpack(message.data[0])
        self.db_save()
        self.logger.info("Network transmit state: %r", node.network_transmit)

    def __relay_status_handler(self, opcode, message):
        state, retransmit = struct.unpack("<BB", message.data)
        node = self.node_get(message.meta["src"])
        node.relay_retransmit = mt.RelayRetransmit.unpack(retransmit)
        node.features.relay = mt.FeatureState(state)
        self.db_save()
        self.logger.info(
            "Relay state: %r, count: %d, interval: %d",
            state, node.relay_retransmit.count, node.relay_retransmit.interval)

    def __heartbeat_subscription_status_handler(self, opcode, message):
        status, src, dst, period_log, count_log, min_hops, max_hops = struct.unpack(
            "<BHHBBBB", message.data)
        status = AccessStatus(status)
        self.logger.info("Heartbeat subscription status: %s", status)
        if status == AccessStatus.SUCCESS:
            if period_log == 0:
                period = 0
            else:
                period = 2**(period_log - 1)
            self.logger.info("Heartbeat subscription state: " +
                             "src: %04x, dst: %04x, period: %ds, count: %d, min/max: %d/%d",
                             src, dst, period, count_log, min_hops, max_hops)

    def __model_app_status_handler(self, opcode, message):
        status, element_address, appkey_index = struct.unpack(
            "<BHH", message.data[:5])
        status = AccessStatus(status)
        model_id = mt.ModelId.unpack(message.data[5:])
        element_address = mt.UnicastAddress(element_address)
        appkey_index = mt.KeyIndex(appkey_index)

        self.logger.info("Model app bind status: %s", status)
        if status == AccessStatus.SUCCESS:
            model = self.model_get(element_address, model_id)

            # Was last command a bind or unbind?
            if self.previous_command == "bind" and appkey_index not in model.bind:
                model.bind.append(appkey_index)
            elif appkey_index in model.bind:
                model.bind.remove(appkey_index)

            self.db_save()
            self.logger.info("Appkey %s %d to model %r at %04x",
                             self.previous_command, appkey_index, model_id, element_address)

    def __netkey_status_handler(self, opcode, message):
        status, netkey_index = struct.unpack("<BH", message.data)
        netkey_index = mt.KeyIndex(netkey_index)
        status = AccessStatus(status)
        self.logger.info("Netkey status: %s", status)
        if status == AccessStatus.SUCCESS:
            node = self.node_get(message.meta["src"])
            if netkey_index not in node.net_keys:
                node.net_keys.append(netkey_index)
            self.db_save()
            self.logger.info("Added subnet %d to node %04x",
                             netkey_index, message.meta["src"])

    def __netkey_list_handler(self, opcode, message):
        node = self.node_get(message.meta["src"])
        if len(message.data) > 0:
            node.net_keys = ConfigurationClient._unpack_key_ind(message.data)
            self.db_save()
        self.logger.info("Node %04x has subnets %r",
                         message.meta["src"], node.net_keys)

    def __node_identity_status_handler(self, opcode, message):
        status, netkey_index, identity = struct.unpack("<BHB", message.data)
        status = AccessStatus(status)
        self.logger.info("Node identity status: %s", status)
        if status == AccessStatus.SUCCESS:
            if identity == 0:
                state = "stopped"
            elif identity == 1:
                state = "running"
            else:
                state = "not supported"

            self.logger.info(
                "Current node identity for subnet %d is %s", netkey_index, state)

    def __node_reset_status_handler(self, opcode, message):
        self.logger.info("Node %04x was reset", message.meta["src"])

    def __model_sig_app_list_handler(self, opcode, message):
        status, element_address, model_id = struct.unpack(
            "<BHH", message.data[0:5])
        status = AccessStatus(status)
        self.logger.info("SIG Model App List status: %s", status)
        if status == AccessStatus.SUCCESS:
            appkeys = ConfigurationClient._unpack_key_ind(message.data[5:])
            model = self.model_get(element_address, mt.ModelId(model_id))
            model.bind = appkeys
            self.db_save()
            self.logger.info(
                "SIG model %04x has appkey(s) %r bound", model_id, appkeys)

    def __model_sig_subscription_list_handler(self, opcode, message):
        status, element_address, model_id = struct.unpack(
            "<BHH", message.data[0:5])
        status = AccessStatus(status)
        self.logger.info("SIG Model Subscription List status: %s", status)
        if status == AccessStatus.SUCCESS:
            if len(message.data) > 5:
                addresses = struct.unpack("<" + "H" * (len(message.data[5:]) // 2),
                                          message.data[5:])
                addresses = [mt.group_address(a) for a in addresses]
            else:
                addresses = []

            model = self.model_get(element_address, mt.ModelId(model_id))
            model.subscribe = addresses
            self.db_save()
            self.logger.info(
                "SIG model %04x has addresse(s) %r bound", model_id, addresses)

    def __vendor_model_app_list_handler(self, opcode, message):
        status, element_address, company_id, model_id = struct.unpack(
            "<BHHH", message.data[0:7])
        status = AccessStatus(status)
        self.logger.info("SIG Model App List status: %s", status)
        if status == AccessStatus.SUCCESS:
            appkeys = ConfigurationClient._unpack_key_ind(message.data[7:])
            model = self.model_get(element_address,
                                   mt.ModelId(model_id=model_id,
                                              company_id=company_id))
            model.bind = appkeys
            self.db_save()
            self.logger.info("Vendor model %04x, company ID %04x has appkey(s) %r bound",
                             model_id, company_id, appkeys)

    def __vendor_model_subscription_list_handler(self, opcode, message):
        status, element_address, company_id, model_id = struct.unpack(
            "<BHHH", message.data[0:7])
        status = AccessStatus(status)
        self.logger.info("SIG Model Subscription List status: %s", status)
        if status == AccessStatus.SUCCESS:
            if len(message.data) > 7:
                addresses = struct.unpack("<" + "H" * (len(message.data[7:]) / 2),
                                          message.data[7:])
                addresses = [mt.group_address(a) for a in addresses]
            else:
                addresses = []

            model = self.model_get(
                element_address, mt.ModelId(model_id=model_id, company_id=company_id))
            model.subscribe = addresses
            self.db_save()
            self.logger.info("Vendor model %04x, company ID %04x has addresse(s) %r bound",
                             model_id, company_id, addresses)

    def __low_power_node_polltimeout_status_handler(self, opcode, message):
        # We append a 0x00 to the bytearray to unpack the PollTimeout as a uint32_t
        lpn_address, poll_timeout = struct.unpack("<HI", message.data + bytearray(1))

        if poll_timeout > 0:
            # Multiply to get in units of milliseconds (ref. 3.6.5.3, PollTimeout is in 100ms units)
            poll_timeout *= 100
            self.logger.info("Low power node %04x poll timeout %d ms.", lpn_address, poll_timeout)
        else:
            self.logger.info("Node is not a Friend not or LPN address %04x not a known LPN address",
                             lpn_address)
