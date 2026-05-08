# BuggyProject

Arduino-based autonomous buggy project developed as part of a college engineering challenge.

The project involved controlling a small robotic buggy using embedded C/C++ on Arduino, with later work extending the system to use a HuskyLens computer vision module for object detection and environment visualisation.

## Project overview

The buggy was developed across multiple challenge stages, with the final stage involving computer vision. For the Gold Challenge, we used the HuskyLens camera module to identify objects in the environment and send information to a Processing visualisation running on a laptop.

The aim was to combine basic embedded control, sensing, and visual feedback rather than build a production-quality robotics stack.

## Features

- Arduino-based motor control
- Sensor-driven buggy behaviour
- PID control for smooth buggy movement
- HuskyLens-based computer vision
- Serial communication between Arduino and laptop
- Processing visualisation of the buggy environment
- Iterative testing through multiple challenge stages

## Hardware

- Arduino microcontroller
- Motor driver
- DC motors
- HuskyLens computer vision module
- Hall effect sensors for distance/speed sensing.
- Buggy chassis

## Software

- Arduino C/C++
- Processing
- Serial communication for telemetry / visualisation data

## Repository structure

```text
bronze_challenge/    Early Arduino control, Processing visualisation, and wiring
gold_challenge/      HuskyLens vision integration and final challenge code
experiments/         Standalone tests for camera, PID, sensors, and Processing gauges
```

## Notes

This was a college engineering project, so this repo shows the iterative development process rather than a polished software product.
The main value of the project was in integrating embedded control, sensors, computer vision, and visualisation under practical time constraints.
