# ti_hci_test
UART/HCI connectivity verification test for TI dualband BT devices

This test program is intended to check the HW and device tree configurations 
on the Linux host for communicating with TI dual mode Bluetooth controller 
devices (WL18xx, CC256x).
build procedure: compile with appropriate cross compile tools or native tool chain:
ex: #  aarch64-none-linux-gnu-gcc -o ti_hci_test ti_hci_test.c

Usage: The test program takes three parameters, dev-name specifing the tty device 
correspoding to the UART used for BT/HCI communications, desired HCI baudrate and, 
bt_en_gpio, GPIO used for BT enable/disable.

root@am64xx-evm:~# ./ti_hci_test -h
usage : hci-test <dev-name> <baud rate> <bt_en_gpio> 

root@am64xx-evm:~# ./ti_hci_test /dev/ttyS6 115200 464
opening device: /dev/ttyS6 for final baudrate=115200 : bt_en_gpio=464 
echo 464 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio464/direction
echo 0 > /sys/class/gpio/gpio464/value
echo 1 > /sys/class/gpio/gpio464/value
Configuring for initial baudrate of 115200 with Hardware flow control 
UART initialized !!! 
Send HCI command : HCI_Read_Local_Version_Information 
Recived HCI Event/Response 
Found a Texas Instruments' chip!
Needed Firmware file : TIInit_11.8.32.bts
changing HCI baudrate to 115200 
Send HCI command : HCI_VS_Update_Uart_HCI_Baudrate 
Send baudrate change command to Controller 
Recived HCI Event/Response 
response byte[0] = 0x04 
response byte[1] = 0x0e 
response byte[2] = 0x04 
response byte[3] = 0x01 
response byte[4] = 0x36 
response byte[5] = 0xff 
response byte[6] = 0x00 
response byte[7] = 0x00 
response byte[8] = 0x00 
response byte[9] = 0xcf 
response byte[10] = 0x60 
response byte[11] = 0xf3 
response byte[12] = 0xff 
response byte[13] = 0xff 
response byte[14] = 0x00 
 
Changing host UART baudrate 
Send HCI command : HCI_Read_Local_Version_Information 
Recived HCI Event/Response 
Found a Texas Instruments' chip!
Needed Firmware file : TIInit_11.8.32.bts
Send HCI command : HCI_Read_BD_ADDR 
Recived HCI Event/Response 
response byte[0] = 0x04 
response byte[1] = 0x0e 
response byte[2] = 0x0a 
response byte[3] = 0x01 
response byte[4] = 0x09 
response byte[5] = 0x10 
response byte[6] = 0x00 
response byte[7] = 0x83 
response byte[8] = 0x5e 
response byte[9] = 0xcf 
response byte[10] = 0x02 
response byte[11] = 0xe2 
response byte[12] = 0x90 
response byte[13] = 0xff 
response byte[14] = 0xff 

