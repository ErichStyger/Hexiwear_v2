readme.txt
----------
This is a port of the IAR Hexiwear KW40Z BLE project to Kinetis Design Studio.

The following symbols are defined in the IAR linker settings:
gUseNVMLink_d=1
__heap_size__=0
__stack_size__=384
gUseBootloaderLink_d=1
gUseInternalStorageLink_d=0
__ram_vector_table__=1
gNVMSectorCountLink_d=1


Raw image:
C:\nxp\KW40Z_Connectivity_Software_1.0.1\ConnSw\framework\Bootloader\Bootloader_OTAP_Serial\BootloaderOTAP_KW40Z4\Exe\BootloaderOTAP_KW40Z4.bin
Symbol: bootloader
Section: .bootloader
Align: 4


NOTE: 
- The project is still experimental.
- Currently I get a hard fault in the NXP BLE library after a device pairing

Links:
- https://mcuoneclipse.com/2016/12/30/building-the-nxp-ble-stack-with-open-source-gnu-and-eclipse-tools/


License:
The project contains portions from the 
- Freescale Connectivity Stack (KW40Z Connectivity Software 1.0.1) (http://www.nxp.com/products/wireless-connectivity/2.4-ghz-wireless-solutions/bluetooth-smart-bluetooth-low-energy:BLUETOOTH-LOW-ENERGY-NXP)
- Freescale Kinetis SDK V1.3 (http://www.nxp.com/ksdk)
