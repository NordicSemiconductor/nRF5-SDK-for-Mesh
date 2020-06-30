# Copyright (c) 2010 - 2019, Nordic Semiconductor ASA
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

import sys
import subprocess
import multiprocessing


USAGE_STRING = """Select one ('0'), several ('0, 1, 2') or all ('a') devices.
To quit, enter 'q': """


def list_devices():
    return [int(d) for d in subprocess.check_output(['nrfjprog', '-i']).splitlines()]


def select_devices(devices):
    device_range = list(range(len(devices)))
    number = None
    while number is None:
        try:
            number = str(input(USAGE_STRING))
            if number.lower() == "q":
                return []
            elif number.lower() == "a":
                return devices
            else:
                ids = set([int(n) for n in number.split(",")])
                if ids.issubset(set(device_range)):
                    return [devices[i] for i in ids]
        except ValueError:
            pass

        print("Invalid input \"%s\"" % (number))
        number = None


def flash(args):
    device, hexfile, mode = args
    device = str(device)
    cmd = ["nrfjprog", "-s", device, "--program", hexfile, "%s" % mode]
    print("# Programming ...")
    print(' '.join(cmd))
    subprocess.check_output(cmd)
    cmd = ["nrfjprog",  "-s", device, "--reset"]
    print("# Resetting ...")
    print(' '.join(cmd))
    subprocess.check_output(cmd)


def main():
    devices = list_devices()
    if devices and len(devices) > 0:
        device_range = list(range(len(devices)))
        print("Connected devices:")
        print("".join(["%d: %s\n" % (i, devices[i]) for i in device_range]))
    else:
        print("No devices connected")
        sys.exit(0)

    devices = select_devices(devices)

    if len(devices) == 0:
        print("No device(s) selected")
        sys.exit(0)

    hexfile = sys.argv[1]
    mode = sys.argv[2]
    with multiprocessing.Pool(len(devices)) as p:
        p.map(flash, [(d, hexfile, mode) for d in devices])

    sys.exit(0)


if __name__ == "__main__":
    main()
