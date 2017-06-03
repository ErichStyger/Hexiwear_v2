# Using Hexiwear with Python
import pexpect
import time

DEVICE = "00:29:40:08:00:01"   # device #24
#DEVICE = "00:32:40:08:00:12"   # device #??

print("Hexiwear address:"),
print(DEVICE)

# Run gatttool interactively.
print("Running gatttool...")
child = pexpect.spawn("gatttool -I")

# Connect to the device.
print("Connecting to "),
print(DEVICE),
child.sendline("connect {0}".format(DEVICE))
child.expect("Connection successful", timeout=10)
print(" Connected!")

#print 'Press Ctrl-C to quit.'
# function to transform hex string like "0a cd" into signed integer
def hexStrToInt(hexstr):
    val = int(hexstr[0:2],16) + (int(hexstr[3:5],16)<<8)
    if ((val&0x8000)==0x8000): # treat signed 16bits
        val = -((val^0xffff)+1)
    return val

#while True:
# Accelerometer
child.sendline("char-read-hnd 0x30")
child.expect("Characteristic value/descriptor: ", timeout=10)
print(child.before)
child.expect("\r\n", timeout=10)
print(child.before)
print("Accel:  "),
print(child.before),
print(float(hexStrToInt(child.before[0:5]))/100),
print(float(hexStrToInt(child.before[6:11]))/100),
print(float(hexStrToInt(child.before[12:17]))/100)

# Accelerometer
child.sendline("char-read-hnd 0x34")
child.expect("Characteristic value/descriptor: ", timeout=10)
child.expect("\r\n", timeout=10)
print("Gyro:   "),
print(child.before),
print(float(hexStrToInt(child.before[0:5]))/100),
print(float(hexStrToInt(child.before[6:11]))/100),
print(float(hexStrToInt(child.before[12:17]))/100)

# Magnetometer
child.sendline("char-read-hnd 0x38")
child.expect("Characteristic value/descriptor: ", timeout=10)
child.expect("\r\n", timeout=10)
print("Magneto:"),
print(child.before),
print(hexStrToInt(child.before[0:5])),
print(hexStrToInt(child.before[6:11])),
print(hexStrToInt(child.before[12:17]))

#    time.sleep(5)
