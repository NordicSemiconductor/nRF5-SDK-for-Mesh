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

import re
import sys
import os

USAGE_STRING = "python %s input-file output-file " % (sys.argv[0])

if len(sys.argv) != 3:
    sys.stderr.write("Invalid argv: %r. Usage: %s" % (sys.argv, USAGE_STRING))
    sys.exit(-1)
elif not os.path.exists(sys.argv[1]):
    sys.stderr.write("Invalid path \"%s\"" % (sys.argv[1]))
    sys.exit(-1)

INFILE = sys.argv[1]
OUTFILE = sys.argv[2]


if not os.path.exists(os.path.dirname(OUTFILE)):
    os.mkdir(os.path.dirname(OUTFILE))

with open(INFILE, "r") as f:
    indata = f.read()

regex = r"SVCALL\([A-Z_]+,\s+([a-z0-9_]+),\s+([a-z_0-9, (*;]+\))\);"
subst = "\\1 \\2;"
result = re.sub(regex, subst, indata, 0)


with open(OUTFILE, "w") as f:
    f.write(result)
