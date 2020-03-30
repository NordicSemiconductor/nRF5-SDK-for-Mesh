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


class TransitionTime(object):

    STEP_RESOLUTION_100MS = 0x00
    STEP_RESOLUTION_1S = 0x40
    STEP_RESOLUTION_10S = 0x80
    STEP_RESOLUTION_10M = 0xC0

    STEP_MASK = 0xC0
    STEP_100MS_FACTOR = 100
    STEP_1S_FACTOR = 1000
    STEP_10S_FACTOR = 10*1000
    STEP_10M_FACTOR = 10*60*1000

    STEP_100MS_MAX = 6200
    STEP_1S_MAX = 1000 * 62
    STEP_10S_MAX = 1000 * 620
    STEP_10M_MAX = 60 * 620 * 1000
    MAX = 0x3E
    UNKNOWN = 0x3F
    MAX_MS = 60 * 620 * 1000


    @classmethod
    def encode(cls, transition_time_ms):
        enc_time = 0
        if transition_time_ms <= cls.STEP_100MS_MAX:
            enc_time = (transition_time_ms // cls.STEP_100MS_FACTOR) | cls.STEP_RESOLUTION_100MS
        elif transition_time_ms <= cls.STEP_1S_MAX:
            enc_time = (transition_time_ms // cls.STEP_1S_FACTOR) | cls.STEP_RESOLUTION_1S
        elif (transition_time_ms <= cls.STEP_10S_MAX):
            enc_time = (transition_time_ms // cls.STEP_10S_FACTOR) | cls.STEP_RESOLUTION_10S
        elif transition_time_ms <= cls.STEP_10M_MAX:
            enc_time = (transition_time_ms // cls.STEP_10M_FACTOR) | cls.STEP_RESOLUTION_10M

        return enc_time


    @classmethod
    def decode(cls, enc_time):
        if (enc_time & cls.STEP_MASK) == cls.STEP_RESOLUTION_100MS:
            return (enc_time & ~cls.STEP_MASK) * cls.STEP_100MS_FACTOR
        elif (enc_time & cls.STEP_MASK) == cls.STEP_RESOLUTION_1S:
            return (enc_time & ~cls.STEP_MASK) * cls.STEP_1S_FACTOR
        elif (enc_time & cls.STEP_MASK) == cls.STEP_RESOLUTION_10S:
            return (enc_time & ~cls.STEP_MASK) * cls.STEP_10S_FACTOR
        elif (enc_time & cls.STEP_MASK) == cls.STEP_RESOLUTION_10M:
            return (enc_time & ~cls.STEP_MASK) * cls.STEP_10S_FACTOR
        else:
            return 0


    @classmethod
    def pack(cls, transition_time_ms, delay_ms):
        return struct.pack("<BB", cls.encode(transition_time_ms), (delay_ms // 5))

    @classmethod
    def unpack(cls, raw):
        enc_transition_time, enc_delay = struct.unpack("<BB", raw)
        return cls.decode(enc_transition_time), enc_delay*5
