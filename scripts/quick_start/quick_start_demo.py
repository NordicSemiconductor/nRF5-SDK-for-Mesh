# Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

from __future__ import print_function
import argparse
import subprocess
import sys
import os
import os.path
import functools

#---------------------------------------------------------------------------------------------------
class CmdExecutor(object):
    def run(self, cmd, verbose=False):
        if (verbose == False):
            cmd.append('-q')

        print (' '.join(cmd))

        result = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        log = result.communicate()[0].decode("ascii")
        while (result.returncode == None):
            log += result.communicate()[0].decode("ascii")
            continue

        retval = result.returncode

        if (verbose):
            print(log)

        return (log, retval)

#---------------------------------------------------------------------------------------------------
class ProgramDevice(object):
    cmd = CmdExecutor()

    def __init__(self, verbose=False):
        self.verbose = verbose

    def program(self, soft_dev_hex, app_hex, device_id, type="NRF52"):
        error_count = []
        print ('\n# Programming the SoftDevice on: ', device_id)
        (log, ec) = cmd.run(['nrfjprog', '--program', soft_dev_hex, "-s", device_id, "-f", type,], self.verbose)
        error_count.append(ec)

        print ('# Programming hex ')
        (log, ec) = cmd.run(['nrfjprog', '--program', app_hex, "-s", device_id, "-f", type,], self.verbose)
        error_count.append(ec)
        return error_count

    def erase(self, device_id):
        print ('\n# Erasing the device: ', device_id)
        (log, ec) = cmd.run(['nrfjprog', '--eraseall', "-s", device_id], self.verbose)
        return [ec]

    def reset(self, device_id):
        print ('\n# Resetting the device: ', device_id)
        (log, ec) = cmd.run(['nrfjprog', '--reset', "-s", device_id], self.verbose)
        return [ec]

#---------------------------------------------------------------------------------------------------
def device_type_get(device_id):
    if (len(device_id) < 9):
        return "00"
    elif (device_id[2] == "2" or device_id[2] == "3" or device_id[2] == "4"):
        return "52"
    elif (device_id[2] == "0" or device_id[2] == "1"):
        return "51"
    else:
        return "00"

#---------------------------------------------------------------------------------------------------
def get_board_index(msg, devs):
    printBoards(devs)
    while(1):
        try:
            if (sys.version_info[0] == 2):
                getinput = raw_input
            else:
                getinput = input

            lst_range = list(range(len(devs)))
            inp = getinput('\n' + msg + str(lst_range) + ' [Enter `s`: To skip]: ')

            if (len(inp) > 1 or len(inp) == 0):
                raise ValueError

            inp = str(inp.lower())

            if (not (inp == 's' or (inp in map(str, lst_range)))):
                raise ValueError
            elif (inp == 's'):
                print ("Warning: Skipping this firmware.")
                return 0

            num = int(inp)
        except  ValueError:
            print ("Error: Please enter valid input")
            continue

        return(devs[num])

#---------------------------------------------------------------------------------------------------
def printBoards(devices):
    print ("\nBoards with nRF52 devices: \n(Board index : Segger ID)")
    for i in range(len(devices)):
        print(str(i) + ' : ' + devices[i])


#---------------------------------------------------------------------------------------------------
class errorList(object):
    def __init__(self):
        self.error_lst = []

    def addErrors(self,ec):
        for e in ec:
            self.error_lst.append(e)
    def getErrors(self):
        return self.error_lst

#---------------------------------------------------------------------------------------------------
cwd = os.getcwd()
print ("Current directory: " + cwd)

parser = argparse.ArgumentParser(description="Mesh quick start demo - Firmware flashing script")
parser.add_argument('-p', '--provisioner', default=1, help="Provide the provisioner device ID")
parser.add_argument('-c', '--client', default=2, help="Provide the client device ID")
parser.add_argument('-v', '--verbose', action='store_true', help="Print nrfjprog output")
args = parser.parse_args()

prov_loaded = False
client_loaded = False
server_loaded = False
ec = 0
eList = errorList()
server_devices = []

soft_dev_hex52    =  cwd + "./bin/softdevice/s132_nrf52_6.0.0_softdevice.hex"
provisioner_hex52 =  cwd + "./bin/otime/light_switch_provisioner_nrf52832_xxAA_s132_6.0.0.hex"
client_hex52      =  cwd + "./bin/otime/light_switch_client_nrf52832_xxAA_s132_6.0.0.hex"
server_hex52      =  cwd + "./bin/otime/light_switch_server_nrf52832_xxAA_s132_6.0.0.hex"

# All files must be valid
for f in [soft_dev_hex52, provisioner_hex52, client_hex52, server_hex52]:
    if (not os.path.isfile(soft_dev_hex52)):
        print("Error: File ", f, " does not exist")
        quit()

# Filter nRF52 boards
cmd = CmdExecutor()
(log, ec) = cmd.run(['nrfjprog', '-i'])
devices = log.splitlines()
devices = [d for d in devices if device_type_get(d) == "52"]


# Ask user input if required.
device_selection = devices[:]
if (args.provisioner == 1):
    ret = get_board_index('# Enter board index you want to use as a Provisioner ', device_selection)
    if (ret):
        args.provisioner = ret
        device_selection.remove(args.provisioner)

if (args.client == 2):
    ret = get_board_index('# Enter board index you want to use as a Client ', device_selection)
    if (ret):
        args.client = ret


# Argument checks
if (len(str(args.provisioner)) < 9):
    print("Warning: Segger ID for Provisioner is not provided")
if (len(str(args.client)) < 9):
    print("Warning: Segger ID for Client is not provided")
if (len(str(args.provisioner)) == 9 and str(args.provisioner) == str(args.client)):
    print("Error: Provisioner and Client boards cannot be the same")
    quit()
if (len(str(args.client)) >= 9 and args.provisioner not in devices):
    print("Warning: Segger ID %s could not be found. Skipping." % args.provisioner)
if (len(str(args.client)) >= 9 and args.client not in devices):
    print("Warning: Segger ID %s could not be found. Skipping." % args.client)

# Action: Erase devices
programmer = ProgramDevice(args.verbose)
for d in devices:
    ec = programmer.erase(d)
    eList.addErrors(ec)

if (sum(err > 0 for err in eList.getErrors()) > 0):
    print('Error: While erasing the devices, please check if boards are connected')
    quit()

# Action: Program SoftDevice and firmware
for d in devices:
    # PROVISIONER - 52xx
    if (d == str(args.provisioner)):
            ec = programmer.program(soft_dev_hex52, provisioner_hex52, d)
            eList.addErrors(ec)
            if (all(e == 0 for e in ec)):
                prov_loaded = True
            else:
                print ("Error: Provisioner could not be programmed on: %s, please retry." % args.provisioner)
                quit()

    # CLIENT - 52xx
    elif (d == str(args.client)):
            ec =  programmer.program(soft_dev_hex52, client_hex52, d)
            eList.addErrors(ec)
            if (all(e == 0 for e in ec)):
                client_loaded = True
            else:
                print ("Error: Client could not be programmed on: %s, please retry." % args.client)
                quit()

    # SERVERs - 52xx
    else:
            ec =  programmer.program(soft_dev_hex52, server_hex52, d)
            if (all(e == 0 for e in ec)):
                server_loaded = True
                server_devices.append(d)
            else:
                print ("Warning: Server could not be programmed on: %s" % d)
                eList.addErrors(ec)

# Action: Reset all devices
for d in devices:
    ec = programmer.reset(d)
    eList.addErrors(ec)

print("\n# Summary:")
print("Errors occurred: %d" % sum(err > 0 for err in eList.getErrors()))
if (prov_loaded):
    print(" Provisioner ID: %s" % args.provisioner)
    devices.remove(args.provisioner)
else:
    print(" Provisioner ID: Not programmed")

if (client_loaded):
    print("      Client ID: %s" % args.client)
    devices.remove(args.client)
else:
    print("      Client ID: Not programmed")

if (server_loaded):
    print("     Server IDs: " + ", ".join(server_devices))
else:
    print("     Server IDs: Not programmed")

