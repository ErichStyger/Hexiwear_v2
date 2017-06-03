# Python script to upate the RTC clock of all devices
# Usage:
# python RtcUpdateAll.py

import subprocess

devices = [
   # "00:32:40:08:00:12"
  "00:2A:40:0B:00:4E",  # device 23
  "00:29:40:08:00:01",  # device 24
   ]

for x in range(0,len(devices)):
  cmd = "python RtcUpdate.py " + devices[x]
  subprocess.call(cmd, shell=True)

print("finished updating all devices!")
