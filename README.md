I have an ebike with a Bafang M620 motor. Mine is NOT the later CAN version. I have the UART version.  I also have a Garmin fenix 6x solar watch that I track my bike rides with. I want to acquire the pedal cadence and torque data from the Bafang CAN and send it via BLE to my Garmin watch. This will enable enhanced health tracking by providing the human power data. 

I have this device: https://docs.waveshare.com/ESP32-S3-Touch-LCD-4.3B I plan to use for this project. It can read the UART data and send it over BLE. It can also display data.

I made a T harness to connect the device at the display connector. I bought this power step-down convertor: https://www.amazon.com/dp/B0FSDSVJC1 to support this project. 

I found the ESP32 does not natively support reading this Bafang UART. Ordered a PC817 module https://www.amazon.com/dp/B0FWC82QF6 to read the UART and communicate this to input pins on the ESP32. That did not work so I switched to this https://www.amazon.com/dp/B08XLT21S6 TTL to RS485 module which did work. This module reads TTL (UART) and converts it to RS485 whicvh can be natively read by the Waveshare ESP32.

Update 10-7-2026: I finally got it reading data and parsing it to capture and show on the display. Reverse engineered motor power, assist level, speed, and battery percent (need more verification). After more research it appears the Bafang UART does not contain pedal orque or cadance. It also does not appear to communicate battery voltage on UART, likely because the display can sample this locally. That is just a guess at this point.

I ordered this: https://www.amazon.com/dp/B07VPFLSMX and plan to connect directly to the torque and cadence sensor in the Bafang motor. I will use 3 of the inputs as a differential input and scale it in the Waveshare device. Then I will need to calibrate it. If I need cadence, I will need to read that with one or both of the cadence sensors which are part of the torque sensor.
