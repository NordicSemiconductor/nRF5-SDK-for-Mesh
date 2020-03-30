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

from mesh.access import Model, Opcode
from models.common import TransitionTime
import struct


class GenericOnOffClient(Model):
    GENERIC_ON_OFF_SET = Opcode(0x8202, None, "Generic OnOff Set")
    GENERIC_ON_OFF_SET_UNACKNOWLEDGED = Opcode(0x8203, None, "Generic OnOff Set Unacknowledged")
    GENERIC_ON_OFF_GET = Opcode(0x8201, None, "Generic OnOff Get")
    GENERIC_ON_OFF_STATUS = Opcode(0x8204, None, "Generic OnOff Status")

    def __init__(self):
        self.opcodes = [
            (self.GENERIC_ON_OFF_STATUS, self.__generic_on_off_status_handler)]
        self.__tid = 0
        super(GenericOnOffClient, self).__init__(self.opcodes)

    def set(self, value, transition_time_ms=0, delay_ms=0, ack=True):
        message = bytearray()
        message += struct.pack("<BB", int(value > 0), self._tid)

        if transition_time_ms > 0:
            message += TransitionTime.pack(transition_time_ms, delay_ms)

        if ack:
            self.send(self.GENERIC_ON_OFF_SET, message)
        else:
            self.send(self.GENERIC_ON_OFF_SET_UNACKNOWLEDGED, message)

    def get(self):
        self.send(self.GENERIC_ON_OFF_GET)

    @property
    def _tid(self):
        tid = self.__tid
        self.__tid += 1
        if self.__tid >= 255:
            self.__tid = 0
        return tid

    def __generic_on_off_status_handler(self, opcode, message):
        logstr = "Present OnOff: " + "on" if message.data[0] > 0 else "off"
        if len(message.data) > 1:
            logstr += " Target OnOff: " + "on" if message.data[1] > 0 else "off"

        if len(message.data) == 3:
            logstr += " Remaining time: %d ms" % (TransitionTime.decode(message.data[2]))

        self.logger.info(logstr)
