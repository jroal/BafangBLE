I have an ebike with a Bafang M620 motor. Mine is NOT the later CAN version. I have the UART version.  I also have a Garmin fenix 6x solar watch that I track my bike rides with. I want to acquire the pedal cadence and torque data from the Bafang CAN and send it via BLE to my Garmin watch. This will enable enhanced health tracking by providing the human power data. 

I have this device: https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3B I plan to use for this project. It can read the UART data and send it over BLE. It can also display data.

<img width="773" height="1115" alt="BafangHarness" src="https://github.com/user-attachments/assets/55e24e31-ce58-4dcb-850c-e3053cf17a15" />

I plan to make a T harness to connect the device at the display connector. I bought this power step-down convertor: https://www.amazon.com/dp/B0FSDSVJC1 to support this project. 

I found the ESP32 does not natively support reading this Bafang UART. Ordered a PC817 module to read the UART and communicate this to input pins on the ESP32.
