readme.txt
----------
Project with Processor Expert for Hexiwear using custom docking station.

Supported hardare:
- buzzer
- RGB LED
- touch buttons
- RTC
- accelerometer/magnetometer
- ambient light

- 3V3B_EN, PTB12: HI-Z disable, LOW enable: Hum&Temp, AmbLight
- LDO_EN, : Hi-Z disable, LOW enable: Heart rate pWR

Open points:
- left/right button group handling in UI
- UI for toggle adv mode/link state
- window border handling
- cleanup GUI component
- add widget for radio/check button (icon plus text)
- port other widgets
- transparent color
- touch button handling
- selections
- RTC sync needs to wait for second change
- humidity
- gyro
- ambient light
- tap detection
- I2C recovery
- battery voltage/status
- Accelerometer INT1, INT2 and RST
- low power mode
- mag returns all zero?
- show pairing code on connect
