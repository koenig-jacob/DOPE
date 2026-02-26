#!/bin/bash

# This script is used to flash the compiled binary to the ESP32 microcontroller.

# Set the port and binary file variables
PORT="/dev/ttyUSB0"  # Change this to your ESP32's port
BAUD_RATE=115200
BINARY_FILE="build/DOPE.bin"  # Change this to the path of your compiled binary

# Check if the binary file exists
if [ ! -f "$BINARY_FILE" ]; then
    echo "Error: Binary file not found at $BINARY_FILE"
    exit 1
fi

# Flash the binary to the ESP32
echo "Flashing $BINARY_FILE to ESP32 on port $PORT..."
esptool.py --chip esp32 --port $PORT --baud $BAUD_RATE write_flash -z 0x1000 $BINARY_FILE

# Check if the flashing was successful
if [ $? -eq 0 ]; then
    echo "Flashing completed successfully."
else
    echo "Error: Flashing failed."
    exit 1
fi