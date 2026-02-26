# DOPE — Digital Optical Performance Engine

## Overview

The Digital Optical Performance Engine (DOPE) is a C++ library designed for ballistic trajectory calculations on the ESP32-P4 microcontroller. It ingests normalized sensor data, computes ballistic trajectories, applies atmospheric and Earth-rotation corrections, and produces structured firing solutions.

## Features

- **Platform-Agnostic**: Works across different hardware platforms.
- **Display-Agnostic**: Does not depend on specific display technologies.
- **UI-Agnostic**: Can be integrated into various user interfaces.
- **Vision-Agnostic**: Does not perform image processing or target selection.

## System Architecture

DOPE operates as a core engine that interacts with an application layer responsible for user interface, sensor management, and target selection. The architecture is designed to be cleanly layered, ensuring separation of concerns.

## Installation

To build and run the DOPE library, follow these steps:

1. **Clone the Repository**:
   ```
   git clone <repository-url>
   cd DOPE
   ```

2. **Build the Project**:
   Use CMake to configure and build the project:
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

3. **Flash to ESP32**:
   Use the provided script to flash the compiled binary to the ESP32 microcontroller:
   ```
   ./scripts/flash_esp32.sh
   ```

## Usage

To use the DOPE library in your application:

1. Include the necessary headers:
   ```cpp
   #include "bce/dope.h"
   #include "bce/firing_solution.h"
   ```

2. Initialize the library and set up sensor inputs.

3. Call the `DOPE_Update` function with the sensor data to compute the firing solution.

4. Retrieve the `FiringSolution` struct for the results.

## Testing

Unit tests are provided for various components of the library. To run the tests, navigate to the `test` directory and execute the test files.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.