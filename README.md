This is an implementation of udp and bluetooth to send data from two sensors to other devices in the network. 

Hardware Requirements ESP32: The main microcontroller for interfacing with sensors and handling Zigbee communication. 
BME680 Sensor: Used to measure temperature, humidity, pressure, and gas resistance. 
BH1750 Sensor: Used to measure light intensity (lux).
I2C Connection: Ensure proper wiring between the ESP32 and the sensors using I2C protocol. 

Software Requirements ESP-IDF: The official development framework for the ESP32.
FreeRTOS: For task management and multitasking.

Sources: 
https://github.com/espressif/esp-idf/blob/master/examples/protocols/sockets/udp_server/README.md
https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md
