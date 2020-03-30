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
import datetime
import enum


# Monkey-patch json:
# https://stackoverflow.com/questions/18478287/making-object-json-serializable-with-regular-encoder
def _default(self, obj):
    return getattr(obj.__class__, "to_json", _default.default)(obj)


_default.default = json.JSONEncoder().default  # Save unmodified default.
json.JSONEncoder.default = _default            # replacement


def camelify(s):
    """Helper function to convert a snake_case name to camelCase."""
    start_index = len(s) - len(s.lstrip("_"))
    end_index = len(s.rstrip("_"))
    sub_strings = s[start_index:end_index].split("_")
    return (s[:start_index]
            + sub_strings[0]
            + "".join([w[0].upper() + w[1:] for w in sub_strings[1:]])
            + s[end_index:])


def camelify_object(obj):
    """Returns a dict where all the class members have been camelified,
if their value is set."""
    return {camelify(k): v for k, v in obj.__dict__.items()
            if (v is not None and
                not k.startswith("_") and
                not (hasattr(v, "__len__") and len(v) == 0))}


def unpack(cls, obj):
    if obj is None:
        return None
    elif isinstance(obj, cls):
        return obj
    elif isinstance(obj, tuple):
        return cls(*obj)
    elif isinstance(obj, dict):
        return cls(**obj)
    else:
        return cls(obj)


class LimitedInt(int):
    MIN = 0
    MAX = 0

    def __new__(cls, value):
        if isinstance(value, str):
            value = int(value, 16)
        elif not isinstance(value, int):
            raise TypeError("Invalid type %r for %s" % (type(value), cls.__name__))

        if not (cls.MIN <= value <= cls.MAX):
            raise ValueError("Invalid %s: %d not in range [%d, %d]" % (
                cls.__name__, value, cls.MIN, cls.MAX))

        return super(LimitedInt, cls).__new__(cls, value)

    def __sub__(self, other):
        return self.__class__(int(self) - int(other))

    def __add__(self, other):
        return self.__class__(int(self) + int(other))

    def __radd__(self, other):
        return self.__class__(int(other) + int(self))

    def __rsub__(self, other):
        return self.__class__(int(other) - int(self))

    def to_json(self):
        return self


class Address(LimitedInt):
    def __str__(self):
        return "%04x" % (self)

    def __repr__(self):
        return self.__str__()

    # FIXME: The json encoder is happy to "help" us ignoring this method for
    # classes that subclass int.
    def to_json(self):
        return self.__str__()


class FixedBytearray(bytearray):
    LENGTH = 0

    def __init__(self, value):
        if isinstance(value, str):
            value = bytearray.fromhex(value)
        elif not isinstance(value, bytearray):
            raise TypeError("%s must be bytearray or string of hexadcimal numbers, not %r" %
                            (self.__class__.__name__, type(value)))

        if len(value) != self.LENGTH:
            raise ValueError("Invalid %s length: len(%r) = %d != %d" %
                             (self.__class__.__name__, value, len(value), self.LENGTH))

        super(FixedBytearray, self).__init__(value)

    def to_json(self):
        return self.hex()


class UnicastAddress(Address):
    MIN = 1
    MAX = 0x7FFF


class GroupAddress(Address):
    MIN = 0xC000
    MAX = 0xFEFF


class VirtualAddress(FixedBytearray):
    LENGTH = 16


class ReservedAddress(enum.IntEnum):
    UNASSIGNED = 0
    ALL_PROXIES = 0xfffc
    ALL_FRIENDS = 0xfffd
    ALL_RELAYS = 0xfffe
    ALL_NODES = 0xffff

    def to_json(self):
        return "%04x" % (self.value)


class Timestamp(str):
    def __new__(cls):
        return super(Timestamp, cls).__new__(cls, datetime.datetime.now().isoformat(' '))

    def to_json(self):
        return str(self)


class FriendshipCredentials(enum.IntEnum):
    DISABLED = 0
    ENABLED = 1

    def to_json(self):
        return self.value


class TTL(LimitedInt):
    MIN = 0
    MAX = 127

    # Overload to return integer and not string.
    def to_json(self):
        return self.value


class FeatureState(enum.IntEnum):
    DISABLED = 0
    ENABLED = 1
    NOT_SUPPORTED = 2

    def to_json(self):
        return self.value


class Retransmit(object):
    COUNT_MIN = 0
    COUNT_MAX = 7
    INTERVAL_STEPS_MIN = 0
    INTERVAL_STEPS_MAX = 31
    INTERVAL_STEP_SIZE = 1

    def __init__(self, count, interval_steps=None, interval=None):
        """If neither `interval_steps` nor `interval` is set, `interval_steps` will default to zero.
        If `interval` and not `interval_steps` is set, the `interval_steps` is calculated from the
        `interval`.
        """

        if self.COUNT_MIN <= count <= self.COUNT_MAX:
            self.count = count
        else:
            raise ValueError("Count %d is not in range [%d, %d]" % (
                count, self.COUNT_MIN, self.COUNT_MAX))

        # if interval is set (e.g. load from DB) we calculate the interval_steps from it
        if interval and (interval_steps == None):
            interval_steps = round(interval / self.INTERVAL_STEP_SIZE) - 1
        elif interval_steps == None:
            interval_steps = 0

        if self.INTERVAL_STEPS_MIN <= interval_steps <= self.INTERVAL_STEPS_MAX:
            self.interval_steps = interval_steps
        else:
            raise ValueError("Interval steps %d is not in range [%d, %d]" % (
                interval_steps, self.INTERVAL_STEPS_MIN, self.INTERVAL_STEPS_MAX))

        self.interval = (interval_steps + 1) * self.INTERVAL_STEP_SIZE

    def pack(self):
        return (self.count | (self.interval_steps << 3) & 0xFF)

    @classmethod
    def unpack(cls, val):
        count = val & 0x07
        interval_steps = ((val >> 3) & 0x1f)
        return cls(count, interval_steps)

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return {"count": self.count, "interval": self.interval}


class PublishRetransmit(Retransmit):
    INTERVAL_STEP_SIZE = 50


class NetworkTransmit(Retransmit):
    INTERVAL_STEP_SIZE = 10


class RelayRetransmit(Retransmit):
    INTERVAL_STEP_SIZE = 10


class KeyIndex(LimitedInt):
    MIN = 0
    MAX = 4095

    @staticmethod
    def pack(index1, index2=None):
        b = bytearray(3)
        b[0] = index1 & 0xFF
        b[1] = ((index1 >> 8) & 0x0F)
        if index2 is not None:
            b[1] |= ((index2 << 4) & 0xF0)
            b[2] = (index2 >> 4) & 0xFF
            return b
        else:
            return b[:2]

    @classmethod
    def unpack(cls, data):
        if len(data) not in [2, 3]:
            raise ValueError("Packed key index buffer should be 2 or 3 bytes, not %d."
                             % (len(data)))
        key1 = data[0] | ((data[1] & 0x0F) << 8)
        if len(data) == 3:
            key2 = ((data[1] & 0xF0) >> 4) | (data[2] << 4)
            return (cls(key1), cls(key2))
        else:
            return (key1,)


class PublishPeriod(LimitedInt):
    """PublishPeriod is given in milliseconds."""
    MIN = 0
    MAX = 3780000
    MULTIPLIERS = [100, 1000, 10000, 10*60*1000]

    @staticmethod
    def resolution_multiplier(res):
        if res < len(PublishPeriod.MULTIPLIERS):
            return PublishPeriod.MULTIPLIERS[res]
        else:
            raise ValueError("Invalid Step Resolution %d" % (res))

    def pack(self):
        max_steps = 0x3f
        res = -1
        for i in range(len(self.MULTIPLIERS)):
            if self <= self.MULTIPLIERS[i] * max_steps:
                res = i
                break
        if res < 0:
            raise ValueError("Invalid value %d" % (self))

        step = round(self / self.MULTIPLIERS[res])
        return (res << 6 | step & 0x3f) & 0xff

    @classmethod
    def unpack(cls, period):
        step_resolution = (period >> 6) & 0xff
        step_count = (period & 0x3f)
        return cls(step_count * cls.resolution_multiplier(step_resolution))


def any_address(address, allow_unassigned=False):
    if isinstance(address, str):
        address = int(address, 16)
    try:
        a = ReservedAddress(address)
    except (ValueError, TypeError) as e:
        a = None

    if not allow_unassigned and a == ReservedAddress.UNASSIGNED:
        raise ValueError("Unassigned address not allowed")
    elif a is not None:
        return a

    try:
        return UnicastAddress(address)
    except (ValueError, TypeError) as e:
        pass
    try:
        return GroupAddress(address)
    except (ValueError, TypeError) as e:
        pass
    try:
        return VirtualAddress(address)
    except Exception as e:
        raise ValueError("Could not find any address type matching %r" % (address))


def group_address(address):
    if isinstance(address, str):
        address = int(address, 16)
    try:
        return ReservedAddress(address)
    except (ValueError, TypeError) as e:
        pass
    try:
        return GroupAddress(address)
    except (ValueError, TypeError) as e:
        pass
    try:
        return VirtualAddress(address)
    except Exception as e:
        raise ValueError("Could not find any address type matching %r" % (address))


class Publish(object):
    def __init__(self, address, index=0, ttl=0, period=0, retransmit=(0, 0), credentials=0):
        self.address = any_address(address, allow_unassigned=True)
        self.index = KeyIndex(index)
        self.ttl = TTL(ttl)
        self.period = PublishPeriod(period)
        self.retransmit = unpack(PublishRetransmit, retransmit)
        self.credentials = FriendshipCredentials(credentials)

    def pack(self):
        b = bytearray()
        if isinstance(self.address, VirtualAddress):
            b += self.address
        else:
            b += struct.pack("<H", self.address)

        b += struct.pack("<H", self.index)
        if self.credentials == FriendshipCredentials.ENABLED:
            b[-1] |= (1 << 4)

        b += struct.pack("<BBB",
                         self.ttl,
                         self.period.pack(),
                         self.retransmit.pack())
        return b

    @classmethod
    def unpack(cls, data):
        if not isinstance(data, bytearray):
            raise TypeError("Invalid type for upack, was %s expected bytearray.", type(data))
        elif (len(data) != 7):
            raise ValueError("Invalid length of publication state %d", len(data))

        if len(data) > 11:
            address = data[:16]
            data = data[16:]
        else:
            address, = struct.unpack("<H", data[0:2])
            data = data[2:]
        index, ttl, period, retransmit = struct.unpack("<HBBB",
                                                       data[:5])
        credentials = (index >> 12) & 0x01
        index &= 0x3ff
        return cls(address, index, ttl, PublishPeriod.unpack(period),
                   PublishRetransmit.unpack(retransmit), credentials)

    def to_json(self):
        return camelify_object(self)

    def __repr__(self):
        return str(self.__dict__)


class ModelId(object):
    def __init__(self, model_id, company_id=None):
        self.model_id = model_id
        self.company_id = company_id

    def pack(self):
        if self.company_id:
            return struct.pack("<HH", self.company_id, self.model_id)
        else:
            return struct.pack("<H", self.model_id)

    @classmethod
    def unpack(cls, data):
        if not isinstance(data, bytearray):
            raise TypeError("Invalid data type \"%r\", should be bytearray." % (type(data)))

        if len(data) == 2:
            model_id, = struct.unpack("<H", data)
            return cls(model_id)
        elif len(data) == 4:
            company_id, model_id = struct.unpack("<HH", data)
            return cls(model_id, company_id)
        else:
            raise ValueError("Data length not 2 or 4 bytes: %r" % (len(data)))

    def to_json(self):
        if self.company_id:
            return "%04x%04x" % (self.company_id, self.model_id)
        else:
            return "%04x" % (self.model_id)

    def __eq__(self, other):
        return self.to_json() == other.to_json()

    def __neq__(self, other):
        return not self.__eq__(other)

    def __repr__(self):
        return self.to_json()

    @classmethod
    def from_json(cls, s):
        if len(s) == 4:
            return cls(int(s, 16))
        elif len(s) == 8:
            return cls(int(s[4:], 16), int(s[:4], 16))


class Identifier(object):
    def __init__(self, identifier, name=""):
        if isinstance(identifier, str):
            identifier = int(identifier, 16)
        self.identifier = identifier
        self.name = name

    def pack(self):
        return struct.pack("<H", self.identifier)

    @classmethod
    def unpack(cls, data, name=""):
        if not isinstance(data, bytearray):
            raise TypeError("Invalid type %r, should be bytearray." % (type(data)))
        _id, = struct.unpack("<H", data)
        return cls(_id, name)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "%04x" % (self.identifier)

    def to_json(self):
        return self.__str__()


class Group(object):
    def __init__(self, name, address, parent_address=ReservedAddress.UNASSIGNED):
        self.name = name
        self.address = group_address(address)
        self.parent_address = group_address(parent_address)

    def pack(self):
        return self.address.pack()

    def __eq__(self, other):
        return self.address == other.address

    def __neq__(self, other):
        return not self.__eq__(other)

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class _UUID(FixedBytearray):
    LENGTH = 16


class Key(FixedBytearray):
    LENGTH = 16


class KeyRefreshPhase(enum.IntEnum):
    NORMAL = 0
    DISTRIBUTING = 1
    SWITCHING = 2

    def to_json(self):
        return self.value


class SecurityLevel(enum.Enum):
    LOW = "low"
    HIGH = "high"

    def to_json(self):
        return self.value


class Appkey(object):
    def __init__(self, name, index, bound_net_key, key, old_key=None):
        self.name = name
        self.index = KeyIndex(index)
        self.bound_net_key = KeyIndex(bound_net_key)
        self.key = Key(key)
        if old_key:
            self.old_key = Key(old_key)

    def __repr__(self):
        return self.key.hex()

    def to_json(self):
        return camelify_object(self)


class NetkeyState(object):
    def __init__(self, index, phase=None):
        self.index = KeyIndex(index)
        if phase:
            self.phase = KeyRefreshPhase(phase)

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return self.index


class Netkey(NetkeyState):
    def __init__(self, name, index, key, min_security, phase, old_key=None):
        self.name = name
        self.index = KeyIndex(index)
        self.key = Key(key)
        if old_key:
            self.old_key = Key(old_key)
        self.min_security = SecurityLevel(min_security)
        self.phase = KeyRefreshPhase(phase)

    def __repr__(self):
        return self.key.hex()

    def to_json(self):
        return camelify_object(self)


class Model(object):
    def __init__(self, model_id,
                 subscribe=[], publish=None, bind=[], company_id=None):
        if isinstance(model_id, str):
            self.model_id = ModelId.from_json(model_id)
        elif company_id is not None:
            self.model_id = ModelId(model_id, company_id)
        else:
            self.model_id = unpack(ModelId, model_id)

        self.subscribe = [any_address(a, allow_unassigned=False) for a in subscribe]
        self.publish = unpack(Publish, publish)
        self.bind = [KeyIndex(i) for i in bind]

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class Element(object):
    def __init__(self, index, location=0, models=[], unicast_address=None, name=""):
        self.index = index
        self.location = Identifier(location)
        self.models = [unpack(Model, m) for m in models]
        if unicast_address:
            self.unicast_address = UnicastAddress(unicast_address)
        self.name = name

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class NodeFeatures(object):
    def __init__(self, relay, proxy, friend, low_power):
        self.relay = FeatureState(relay)
        self.proxy = FeatureState(proxy)
        self.friend = FeatureState(friend)
        self.low_power = FeatureState(low_power)

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class Node(object):
    def __init__(self, UUID, device_key, unicast_address, net_keys, config_complete, security, name="",
                 cid=None, vid=None, pid=None, crpl=None, network_transmit=None, relay_retransmit=None,
                 features=None, elements=[], app_keys=[], secure_network_beacon=None,
                 default_TTL=None):
        self.UUID = _UUID(UUID)
        self.device_key = Key(device_key)
        self.unicast_address = unpack(UnicastAddress, unicast_address)
        self.net_keys = [unpack(NetkeyState, keystate) for keystate in net_keys]
        self.config_complete = config_complete
        self.name = name
        self.security = SecurityLevel(security)
        self.cid = unpack(Identifier, cid)
        self.vid = unpack(Identifier, vid)
        self.pid = unpack(Identifier, pid)
        self.crpl = unpack(Identifier, crpl)
        self.features = unpack(NodeFeatures, features)
        self.elements = [unpack(Element, e) for e in elements]
        self.app_keys = [KeyIndex(index) for index in app_keys]
        self.secure_network_beacon = secure_network_beacon
        self.network_transmit = unpack(NetworkTransmit, network_transmit)
        self.relay_retransmit = unpack(RelayRetransmit, relay_retransmit)
        self.default_TTL = default_TTL

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class AddressRange(object):
    def __init__(self, cls, low_address, high_address):
        self.low_address = cls(low_address)
        self.high_address = cls(high_address)
        if self.low_address > self.high_address:
            raise ValueError("Low address > High address: %r > %r" % (
                self.low_address, self.high_address))

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class UnicastRange(AddressRange):
    def __init__(self, low_address, high_address):
        super(UnicastRange, self).__init__(UnicastAddress, low_address, high_address)


class GroupRange(AddressRange):
    def __init__(self, low_address, high_address):
        super(GroupRange, self).__init__(GroupAddress, low_address, high_address)


class Provisioner(object):
    def __init__(self, name, UUID, allocated_group_range, allocated_unicast_range):
        self.name = name
        self.UUID = _UUID(UUID)
        if isinstance(allocated_unicast_range, UnicastRange):
            self.allocated_unicast_range = allocated_unicast_range
        else:
            self.allocated_unicast_range = [unpack(UnicastRange, r)
                                            for r in allocated_unicast_range]

        if isinstance(allocated_group_range, GroupRange):
            self.allocated_group_range = allocated_group_range
        else:
            self.allocated_group_range = [unpack(GroupRange, r) for r in allocated_group_range]

    def __repr__(self):
        return str(self.__dict__)

    def to_json(self):
        return camelify_object(self)


class CompositionData(object):
    HEADER_LENGTH = 10
    ELEMENT_HEADER_LENGTH = 4

    def check_feature_bit(self, features, bit):
        """Checks the given feature bit in the features bitfield"""
        # Note: We return DISABLED here since we have to query
        # the actual state at a later point.
        if (features & (1 << bit)) > 0:
            return FeatureState.DISABLED
        else:
            return FeatureState.NOT_SUPPORTED

    def pack_features(self, features):
        features_mask = 0
        if ((features["relay"] == self.FEATURE_SUPPORTED_DISABLED or
             features["relay"] == self.FEATURE_SUPPORTED_ENABLED)):
            features_mask |= (1 << 0)
        if ((features["proxy"] == self.FEATURE_SUPPORTED_DISABLED or
             features["proxy"] == self.FEATURE_SUPPORTED_ENABLED)):
            features_mask |= (1 << 1)
        if ((features["friend"] == self.FEATURE_SUPPORTED_DISABLED or
             features["friend"] == self.FEATURE_SUPPORTED_ENABLED)):
            features_mask |= (1 << 2)
        if ((features["lowPower"] == self.FEATURE_SUPPORTED_DISABLED or
             features["lowPower"] == self.FEATURE_SUPPORTED_ENABLED)):
            features_mask |= (1 << 3)
        return features_mask

    def unpack(self, data):
        if not isinstance(data, bytearray):
            raise TypeError("Invalid type for unpack(), was %s expected bytearray.", type(data))
        header = data[:self.HEADER_LENGTH]
        body = data[self.HEADER_LENGTH:]
        cid, pid, vid, crpl, features = struct.unpack("<HHHHH", header)
        d = {
            "cid": Identifier(cid),
            "pid": Identifier(pid),
            "vid": Identifier(vid),
            "crpl": crpl,
            "features": {
                "relay": self.check_feature_bit(features, 0),
                "proxy": self.check_feature_bit(features, 1),
                "friend": self.check_feature_bit(features, 2),
                "low_power": self.check_feature_bit(features, 3)
            },
            "elements": []
        }
        index = 0
        while len(body) > 0:
            loc, num_s, num_v = struct.unpack(
                "<HBB", body[:self.ELEMENT_HEADER_LENGTH])
            idx = self.ELEMENT_HEADER_LENGTH
            model_s = [ModelId(mid) for mid in
                       struct.unpack("<" + "H"*num_s, body[idx:idx + num_s*2])]
            idx += num_s * 2
            model_v = struct.unpack("<" + "HH"*num_v, body[idx:idx + num_v*4])
            model_v = zip(model_v[::2], model_v[1::2])
            model_v = [ModelId(model_id=mid, company_id=cid) for cid, mid in model_v]
            idx += num_v * 4

            d["elements"].append(Element(index=index, location=loc, models=model_s+model_v))
            body = body[idx:]
            index += 1
        return d
