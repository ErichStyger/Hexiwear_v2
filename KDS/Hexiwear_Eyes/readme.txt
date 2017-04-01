readme.txt
----------
Project for the Hexiwear with Kinetis Design Studio V3.2.0, implementing an electronic eye.

It includes the 'Electronic Animated Eyes' (thanks to Adafruit and Phil Burgess / Paint Your Dragon
for the incredible useful and amazing work!)


Links:
- https://github.com/adafruit/Adafruit-1.5inch-Color-OLED-PCB
- https://learn.adafruit.com/animated-electronic-eyes-using-teensy-3-1

To generate new image header files, use:
python tablegen.py defaultEye/sclera.png defaultEye/iris.png defaultEye/upper.png defaultEye/lower.png 50 > defaultEye.h