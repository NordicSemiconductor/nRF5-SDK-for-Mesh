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

import re


class ApplicationConfig(object):
    def __init__(self, header_path=("../../../examples/serial/include/"
                                    + "nrf_mesh_config_app.h")):

        self.data = {}

        # Regular expression that finds all #define pairs
        # NOTE: This will not work for bitfield types, e.g., (1 << 3)
        r = re.compile("#define\s+([A-Za-z0-9_]+)[\s(]+([A-Za-z0-9_]+)")
        fdata = ""

        # Read the header into a string
        with open(header_path, "r") as f:
            fdata = f.read()

        matches = r.findall(fdata)
        for match in matches:
            self.data[match[0]] = self.define_parse(match[1])

        # Setting this allows the user to access the configuration data
        # as, e.g., ApplicationConfig.DEVICE_ID
        self.__dict__ = self.data

    def define_parse(self, define):
        DEFINE_LUT = {"ACCESS_COMPANY_ID_NONE": 0xFFFF,
                      "ACCESS_COMPANY_ID_NORDIC": 0x0059,
                      "ACCESS_TTL_USE_DEFAULT": 0xFF,
                      "CONFIG_FEATURE_RELAY_BIT": 1}

        if define.lower().startswith("0x"):
            return int(define, 16)
        elif define.isdigit():
            return int(define, 10)
        elif define in DEFINE_LUT.keys():
            return DEFINE_LUT[define]
        elif define in self.data.keys():
            return self.data[define]
        else:
            return None
