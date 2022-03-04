# infrared_sensors
Infrared sensors for temperature measurements.

In order to measure the brake disc and tyre temperatures of the race car under various loads, a set of MLX90621 Infrared sensors was installed.

This script executes on the CAN node board, hosting an ATMega Microcontroller and is responsible for reading the sensor data using the I2C protocol. After extracting the data,
appropriate CAN messages are composed and published to the CAN Bus of the vehicle, in order to be collected by the Telemetry, ECU and Data Logger.

The system was part of a series of experiments aiming to enhance the design engineer's understanding of the race car's behavior on track and contribute to the decision making process.

![20140727_172455](https://user-images.githubusercontent.com/23053353/156832014-bb668bae-425c-445e-b8d6-5fcfde06ac61.jpg)

![005](https://user-images.githubusercontent.com/23053353/156832025-f3456273-6db8-4cfd-a400-4d2077534fd7.jpg)
