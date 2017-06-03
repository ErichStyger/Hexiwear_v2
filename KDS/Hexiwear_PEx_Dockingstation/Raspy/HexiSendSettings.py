# Python script to send settings to the Hexiwear
# Usage:
# python HexiSendSettings.py 00:29:40:08:00:01

import pexpect
import time
import sys # for command line argument list, see http://www.pythonforbeginners.com/system/python-sys-argv
from time import gmtime, strftime

#default address
DEVICE = "00:29:40:08:00:01"   # device #24
#DEVICE = "00:32:40:08:00:12"   # device #??

# use address from arguments (if any)
if len(sys.argv) == 2:
  DEVICE = str(sys.argv[1])

print("Hexiwear address:"),
print(DEVICE)

# Run gatttool interactively.
print("Running gatttool...")
child = pexpect.spawn("gatttool -I")

# Connect to the device.
print("Connecting to"),
print(DEVICE),
child.sendline("connect {0}".format(DEVICE))
child.expect("Connection successful", timeout=5)
print("Connected!")

# format of data:
# 02    : alert in setting type
# <size>: number of bytes
# <id>  : setting id
# <data>: setting data
command = "char-write-req 61 020501AABBCCDD00000000000000000000000000"
print(command)
child.sendline(command)
child.expect("Characteristic value was written successfully", timeout=5)

print("done!")

