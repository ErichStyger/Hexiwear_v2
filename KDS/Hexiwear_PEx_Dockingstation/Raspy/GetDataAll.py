# Python script to get data from multiple devices
# Usage:
# python GetDataAll.py

import subprocess

# list of devices
devices = [
   # "00:32:40:08:00:12"
  "00:4A:40:0A:00:41", # device 20 ok
  "00:34:40:0A:00:4E", # device 21 display problem
  "00:2D:40:0A:00:09", # device 22 ok
  "00:2A:40:0B:00:4E", # device 23 ok
  "00:29:40:08:00:01", # device 24 ok
  "00:37:40:0B:00:02", # device 25 ok
   ]

for x in range(0,len(devices)):
  cmd = "python GetData.py " + devices[x]
  subprocess.call(cmd, shell=True)

print("finished all devices!")
