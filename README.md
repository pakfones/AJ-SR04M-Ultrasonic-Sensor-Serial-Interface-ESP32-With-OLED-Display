# Ultrasonic Sensor Distance Display with SH1106 OLED

This project demonstrates how to use an ultrasonic distance sensor (JSN-SR04T) with an ESP32 to measure and display the distance on an SH1106 OLED screen using UART communication.

## Features

- **Ultrasonic Sensor**: Sends a trigger command to the sensor and retrieves distance data over UART.
- **SH1106 OLED Display**: Displays the measured distance in centimeters on a 128x64 OLED screen.
- **Error Handling**: Displays error messages if the sensor fails to respond or returns invalid data.

## Hardware Requirements

- **ESP32**
- **JSN-SR04T Ultrasonic Sensor** (or compatible UART-based ultrasonic sensor)
- **SH1106 OLED Display** (128x64 resolution, I2C interface)

## Pin Connections

- **SH1106 OLED**
  - `SDA`: I2C Data
  - `SCL`: I2C Clock
- **JSN-SR04T Sensor**
  - `TX`: GPIO 10 (Trigger pin)
  - `RX`: GPIO 9 (Echo pin)

## Software Requirements

- **Arduino IDE** with the following libraries:
  - [Adafruit_SH110X](https://github.com/adafruit/Adafruit_SH110X)
  - Wire (Built-in I2C library)

## Setup and Usage

1. Connect the SH1106 OLED to the ESP32 via I2C.
2. Connect the JSN-SR04T sensor's TX pin to GPIO 10 and RX pin to GPIO 9 of the ESP32.
3. Upload the code to your ESP32 using the Arduino IDE.
4. The OLED screen will display the current distance in centimeters.
5. If there is a communication error or invalid data, an error message will be shown.

## Functions Overview

### `void requestDistance()`
Sends a trigger command to the sensor and waits for the response. If no response is received within a 100ms timeout, an error message is printed to the Serial Monitor.

### `void parseSensorData()`
Parses the sensor's UART response, validates the start byte, and extracts the distance in millimeters. The distance is then converted to centimeters and displayed on the OLED.

### `void displayDistance(float distanceCm)`
Displays the calculated distance in centimeters on the OLED.

### `void displayError(const char* errorMsg)`
Displays error messages on the OLED in case of communication failure or invalid data.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

- [Your Name]
