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

import logging
import traceback
import threading
import collections
from serial import Serial
from aci.aci_cmd import CommandPacket
from aci.aci_evt import event_deserialize
import queue

EVT_Q_BUF = 128
SEGGER_UART_BYTES_MAX = 63


class Device(object):
    def __init__(self, device_name):
        self.device_name = device_name
        self.logger = logging.getLogger(self.device_name)
        self._pack_recipients = []
        self._cmd_recipients = []
        self.lock = threading.Event()
        self.events = list()
        self.__write_queue = queue.Queue()
        self.writer_alive = True
        threading.Thread(target=self.__writer).start()

    def __del__(self):
        self.kill_writer()

    def kill_writer(self):
        self.writer_alive = False
        self.__write_queue.put(None)

    def __wait(self, timeout=2):
        if len(self.events) == 0:
            self.lock.wait(timeout)
        self.lock.clear()

        if len(self.events) == 0:
            return None
        else:
            event = self.events[:]
            self.events.clear()
            return event

    def add_packet_recipient(self, function):
        self._pack_recipients.append(function)

    def remove_packet_recipient(self, function):
        if function in self._pack_recipients:
            self._pack_recipients.remove(function)

    def add_command_recipient(self, function):
        self._cmd_recipients.append(function)

    def process_packet(self, packet):
        self.events.append(packet)
        self.lock.set()
        for fun in self._pack_recipients[:]:
            try:
                fun(packet)
            except:
                self.logger.error('Exception in pkt handler %r', fun)
                self.logger.error('traceback: %s', traceback.format_exc())

    def process_command(self, command):
        for fun in self._cmd_recipients[:]:
            try:
                fun(command)
            except:
                self.logger.error('Exception in pkt handler %r', fun)
                self.logger.error('traceback: %s', traceback.format_exc())

    def __writer(self):
        while self.writer_alive:
            cmd = self.__write_queue.get()
            if cmd is None:
                return
            cmd.logger = self.logger
            self.write_data(cmd.serialize())
            retval = self.__wait()
            if retval == None:
                self.logger.info('cmd %s, timeout waiting for event' % (cmd.__class__.__name__))

    def write_aci_cmd(self, cmd):
        if isinstance(cmd, CommandPacket):
            self.__write_queue.put(cmd)
        else:
            self.logger.error('The command provided is not valid: %s\nIt must be an instance of the CommandPacket class (or one of its subclasses)', str(cmd))


class Uart(threading.Thread, Device):
    def __init__(self, port, baudrate=115200, device_name=None, rtscts=True):
        self.events_queue = collections.deque(maxlen=EVT_Q_BUF)
        threading.Thread.__init__(self)
        if not device_name:
            device_name = port
        self.device_name = device_name
        self.logger = logging.getLogger(self.device_name)
        Device.__init__(self, self.device_name)

        self._write_lock = threading.Lock()

        self.logger.debug("log Opening port %s, baudrate %s, rtscts %s", port, baudrate, rtscts)

        # We change the baudrate around to reset the UART state.
        # This is a trick to force detection of flow control settings etc.
        _trick_baudrate = 9600
        if baudrate == _trick_baudrate:
            _trick_baudrate = 115200
        self.serial = Serial(port=port, baudrate=_trick_baudrate, rtscts=rtscts, timeout=0.1)
        self.serial.baudrate = baudrate

        self.keep_running = True
        self.start()

    def __del__(self):
        self.stop()

    def stop(self):
        self.keep_running = False
        self.kill_writer()

    def get_packet_from_uart(self):
        tmp = bytearray([])
        while self.keep_running:
            tmp += bytearray(self.serial.read())
            tmp_len = len(tmp)
            if tmp_len > 0:
                pkt_len = tmp[0]
                if tmp_len > pkt_len:
                    data = tmp[:pkt_len+1]
                    yield data
                    tmp = tmp[pkt_len+1:]

    def run(self):
        for pkt in self.get_packet_from_uart():
            self.logger.debug("RX: %s", pkt.hex())
            try:
                if len(pkt) < 2:
                    self.logger.error('Invalid packet: %r', pkt)
                    continue
                parsed_packet = event_deserialize(pkt)
                if not parsed_packet:
                    self.logger.error("Unable to deserialize %s", pkt.hex())

            except Exception:
                self.logger.error('Exception with packet %s', pkt.hex())
                self.logger.error('traceback: %s', traceback.format_exc())
                parsed_packet = None

            if parsed_packet:
                self.events_queue.append(parsed_packet)
                self.logger.debug('parsed_packet %r', parsed_packet)
                self.process_packet(parsed_packet)

        self.serial.close()
        self.logger.debug("exited read event")

    def write_data(self, data):
        with self._write_lock:
            if self.keep_running:
                self.logger.debug("TX: %s", bytearray(data).hex())
                while len(data) > 0:
                    self.serial.write(bytearray(data[:SEGGER_UART_BYTES_MAX]))
                    data = data[SEGGER_UART_BYTES_MAX:]
                self.process_command(data)

    def __repr__(self):
        return '%s(port="%s", baudrate=%s, device_name="%s")' % (self.__class__.__name__, self.serial.port, self.serial.baudrate, self.device_name)
