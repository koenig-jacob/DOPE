# DOPE

## Digital Optical Performance Engine

DOPE is a deterministic embedded ballistic computation engine written in C++17 and designed for high-performance microcontrollers such as the ESP32-P4.

It is the core math and physics layer for a digital precision optic system. DOPE consumes normalized sensor data and produces a structured firing solution. It does not perform rendering, target detection, image processing, UI management, or communication. It computes trajectories. Nothing more.


## What DOPE Does

DOPE calculates a complete firing solution using real-time sensor input and ballistic parameters.

Core outputs include:

* Elevation hold (MOA)
* Windage hold (MOA)
* Time of flight
* Velocity at target
* Energy at target
* Coriolis correction
* Eötvös effect compensation
* Spin drift
* Cant correction
* Air density compensation

The engine computes one solution at a time and is designed to be called continuously in an embedded loop.

---

## Design Philosophy

DOPE is built around a few strict principles:

* Deterministic execution
* No dynamic memory allocation after initialization
* Static buffers only
* SI units internally
* Hardware abstraction at the boundary
* Clean separation between computation and presentation

The engine does not know anything about displays, reticles, target selection, or optical overlays. Those belong in the application layer.

---

## Architecture Overview

DOPE sits between low-level drivers and the higher-level application.

Sensor drivers provide:

* IMU data (accelerometer + gyroscope)
* Magnetometer data
* Barometric pressure and temperature
* Laser rangefinder measurements
* Optional encoder or optical data

The application layer handles:

* User profiles
* Rendering
* Reticle control
* Target selection
* Optional LiDAR or vision systems

DOPE only consumes numeric inputs and returns a firing solution structure.

---

## Ballistic Model

The engine uses a point-mass trajectory model with adaptive timestep integration.

Features include:

* Piecewise drag integration
* Ballistic coefficient correction for atmospheric conditions
* Manual wind decomposition into headwind and crosswind
* Earth rotation modeling using latitude input
* Spin drift using Litz approximation
* Cant compensation based on roll angle

Internal trajectory tables support ranges up to 2500 meters by default.

---

## Orientation System

DOPE includes an AHRS layer for orientation estimation.

* Mahony filter by default
* Optional Madgwick filter at compile time
* Quaternion-based fusion
* Magnetometer disturbance detection
* True heading computation
* Roll angle for cant correction

The orientation system is designed for precision stability in mostly static platforms.

---

## Atmospheric Handling

The engine supports:

* Pressure
* Temperature
* Humidity
* Altitude
* Latitude

If sensors are unavailable, ISA-consistent defaults are used. Defaults do not trigger faults. Only missing critical ballistic inputs will prevent a solution.

---

## Performance Targets

On an ESP32-P4 class device:

* 1000 m solution in under 8 ms
* 2500 m solution in under 15 ms
* AHRS update under 1 ms
* Zero heap allocations during runtime

The engine is designed for real-time embedded systems with strict timing requirements.

---

## What DOPE Is Not

DOPE does not implement:

* Image processing
* AI target detection
* Point cloud processing
* Rendering logic
* Wireless networking
* Data logging systems
* Full 6DOF projectile simulation
* Variable wind modeling along trajectory

It is a core physics engine.

---

## Intended Use

DOPE is intended for digital optical systems, embedded ballistic solvers, and precision instrumentation platforms that require deterministic, physically consistent trajectory computation.

It can serve as the core of a larger ecosystem, but remains deliberately minimal and focused.

---

## License

MIT License. (Open to change)

---

DOPE — Digital Optical Performance Engine
Embedded ballistic computation, cleanly separated from everything else.
