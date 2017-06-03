readme.txt
----------
Tools:
- VNCViewer to connect to Raspy: https://mcuoneclipse.com/2016/12/27/vnc-server-on-raspberry-pi-with-autostart/
- IP address: https://mcuoneclipse.com/2015/12/23/raspberry-pi-tips-ip-address/
- WinSCP for file transfer

- Python with Raspy and Hexiwear: https://mcuoneclipse.com/2016/12/29/using-python-gatttool-and-bluetooth-low-energy-with-hexiwear/
- Pairing: https://mcuoneclipse.com/2016/12/19/tutorial-ble-pairing-the-raspberry-pi-3-model-b-with-hexiwear/

Device addresses:
#24: 00:29:40:08:00:01
#??: 



Quickstart:
bluetothctl  :should show all devices (new ones too)
power off    :power off
power on     :then power on again
agent on
default-agent
scan on
scan off
pair 00:32:40:08:00:12