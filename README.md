# ESP32-Bike-Powermeter
Working esp32 power meter. Power calculates using HX711 with 4 BF350 sensors and MPU6050 rotation speed values. 
Fully compliant with the BLE Standard, visible by any phone bike app.
Now it broadcasts cadence with power service using BLE standart byte structure, no need different services.
Supports deep sleep with wake up on interrupt pin on MPU6050 module. In this mode it consumes about 4mA, in bluetooth connected mode ~= 50mA.

