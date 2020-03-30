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

from pynrfjprog import MultiAPI, Hex
from argparse import ArgumentParser
import os
import errno


PARAMS = {'__initial_sp': 0, 'Stack_Mem': 0}
PAINT_SIZE = 4 #Size of PAINT_COLOUR in Bytes
PAINT_COLOUR = 0xCABACABA
RESULT_FILENAME = 'stack_depth_check_results.txt'

terminal_colour = {
        'DARKMAGENTA' : '\033[0;35m',
        'OKBLUE' : '\033[94m',
        'OKGREEN' : '\033[92m',
        'DARKYELLOW' : '\033[0;33m',
        'FAIL' : '\033[91m',
        'ENDC' : '\033[0m'
    }

def read_map_file_for_params(filename):
    in_file = open(filename, 'r')
    file_data = in_file.readlines()
    in_file.close()
    for line in file_data:
        split_line = line.rstrip('\n\r').split()
        for k in PARAMS.keys():
            if k in split_line:
                if split_line[1].find('0x') > -1:
                    PARAMS[k] = int(split_line[1],16)

def write_result(mapFileName, result):
    testName = os.path.basename(mapFileName).split('.')[0]
    scriptDir = os.path.dirname(os.path.realpath(__file__))
    outFileName = os.path.join(scriptDir, RESULT_FILENAME)
    try:
        resultsFile = open(outFileName,'r+')
    except FileNotFoundError:
        print('%s does not exist, creating a brand new one, it should be part of the repository!!\n'%outFileName)
        resultsFile = open(outFileName,'w+')
    resultsData = resultsFile.readlines()

    newRecordEntry = testName + ': %d\n' %result
    recordFound = False
    existingResult = 0

    for i,record in enumerate(resultsData):
        if record.split(':')[0] == testName:
            existingResult = int(record.split()[1])
            print('Existing max stack_depth: %d, new max stack depth: %d' %(existingResult,result))
            if existingResult < result:
                print('Stack depth increased!\n')
                print('Recording new result in %s\n' %outFileName)
                resultsData[i] = newRecordEntry
            recordFound = True

    if not recordFound:
        print('New max stack depth: %d' %result)
        print('Recording new result in %s\n' %outFileName)
        resultsData.append(newRecordEntry)

    #Rewind file
    resultsFile.seek(0)
    #write its new contents
    resultsFile.writelines(resultsData)
    resultsFile.close()

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-s", "--serial", dest="serial", help="Device serial no", required=False)
    parser.add_argument("-m", "--mapfile", dest="file", help="Path to the application map file", required=True)
    options = parser.parse_args()

    #Read the parameters from the header file if possible, otherwise the default values will remain
    if not os.path.isfile(options.file):
        raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), options.file)

    read_map_file_for_params(options.file)
    if PARAMS['Stack_Mem'] == 0 or PARAMS['__initial_sp'] == 0:
        print("%sSommething went wrong with the map file reading...%s\nStack start: %d, Stack end: %d"
            %(terminal_colour['FAIL'], terminal_colour['ENDC'], PARAMS['Stack_Mem'], PARAMS['__initial_sp']))

    #Connect to device
    api = MultiAPI.MultiAPI(MultiAPI.API.DeviceFamily.NRF51)
    api.open()
    if options.serial:
        api.connect_to_emu_with_snr(int(options.serial))
    else:
        api.connect_to_emu_without_snr();

    #Read data in the stack area
    stack_size = PARAMS['__initial_sp'] - PARAMS['Stack_Mem']
    data = api.read(PARAMS['Stack_Mem'], stack_size)
    data32 = [data[x] | data[x+1]<<8 | data[x+2]<<16 | data[x+3]<<24 for x in range(0,len(data),PAINT_SIZE)]

    #Calculate max stack depth
    highest_untouched_stack_loc = PARAMS['Stack_Mem']

    for i,val in enumerate(data32):
        if val != PAINT_COLOUR:
            highest_untouched_stack_loc = PARAMS['Stack_Mem'] + i*PAINT_SIZE
            break
    max_stack_depth = PARAMS['__initial_sp'] - highest_untouched_stack_loc

    if highest_untouched_stack_loc == PARAMS['Stack_Mem']:
        print('%s***STACK OVERFLOW!***%s(or you did not run stack_depth_paint_stack in your firmware)'
            %(terminal_colour['FAIL'], terminal_colour['ENDC']))
    else:
        write_result(options.file, max_stack_depth)
