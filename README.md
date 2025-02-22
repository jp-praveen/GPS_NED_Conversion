# GPS NED Conversion

This project reads latitude, longitude, and altitude data from a NEO-M8N GPS module and converts it into a local NED (North-East-Down) frame. The NED frame is a common coordinate system used in navigation and robotics.

## Features
- **GPS Data Reading**: Reads latitude, longitude, altitude, and satellite count from the NEO-M8N GPS module.
- **NED Conversion**: Converts GPS coordinates into a local NED frame relative to a reference point.
- **Real-Time Output**: Displays the converted NED coordinates in real-time via the serial monitor.

## Hardware Requirements
- **NEO-M8N GPS Module**: For receiving GPS data.
- **Arduino**: Compatible with Arduino boards (e.g., Arduino Uno, Nano).
- **Wiring**: Connect the GPS module to the Arduino using SoftwareSerial (RX/TX pins).

## Software Requirements
- **Arduino IDE**: To upload and run the code.
- **TinyGPS++ Library**: Install the `TinyGPS++` library for Arduino.

## Code Overview
The code consists of the following components:
- **GPS Data Reading**: Uses the `TinyGPS++` library to read latitude, longitude, altitude, and satellite count.
- **NED Conversion**: Converts GPS coordinates into a local NED frame relative to a reference point (defined by `LAT_REF`, `LONG_REF`, and `ALT_REF`).
- **Real-Time Output**: Displays the GPS data and NED coordinates via the serial monitor.

