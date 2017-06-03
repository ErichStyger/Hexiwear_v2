# Python script to send settings to the Hexiwear
# Usage:
# python HexiSendSettings.py <device> <in|out|err> text
# python HexiSendSettings.py 00:29:40:08:00:01 in "buzzer help"

import pexpect
import time
import sys # for command line argument list, see http://www.pythonforbeginners.com/system/python-sys-argv
from time import gmtime, strftime

#default address
DEVICE = "00:29:40:08:00:01"   # device #24
#DEVICE = "00:32:40:08:00:12"   # device #??

#defaults
id = 2    # 1: stdin, 2: stdout, 3: stderr
text = "hello"

# overwrite defaults from command line arguments (if any)
if len(sys.argv) >= 2:
  DEVICE = str(sys.argv[1])

if len(sys.argv) >= 3:
  if (str(sys.argv[2])=="in"):
    id = 1
  elif (str(sys.argv[2])=="out"):
    id = 2
  elif (str(sys.argv[2])=="err"):
    id = 3
  else: #default
    id = 2

if len(sys.argv) == 4:
  text = str(sys.argv[3])

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
# <id>  : setting id, 01 (stdin), 02 (stdout), 03 (stderr)
# <data>: setting data
# command = "char-write-req 61 02050268656C0A00000000000000000000000000"
# produce something like: "char-write-req 61 020702656C6C6F0A000000000000000000000000"
size = len(text)+2  # number of bytes, including id plus 0xa
command = "02" + "%0.2X"%size + "%0.2X"%id
for x in range(0,len(text)): # add characters in hex
  command = command + "%0.2X" % ord(text[x])
command = command + "%0.2X"%10  # line feed
for x in range(len(command),40):
  command = command + "0"  # fill up with zeros 
command = "char-write-req 61 " + command

print(command)
child.sendline(command)
child.expect("Characteristic value was written successfully", timeout=5)

print("done!")
