---
layout: default
title: ME405 Romi Project
---

{# ME405 Romi Project

## Overview

This repository contains the software and documentation for a MicroPython Romi robot designed for autonomous navigation using cooperative multitasking, motor control, and onboard sensors. The robot was built using a Nucleo L476RG microcontroller. The line sensor and bump sensors were purchased through Pololu.

The project was completed collaboratively by **Eric Lee, Roman Ruettimann, and Jonathan Enrique Corvera**, with all members contributing equally to both hardware and software development.

---

## Project Goals and Features

The primary goal of this project was to design a Romi robot capable of reliably completing a multi-checkpoint course using only onboard sensing and state estimation. The robot must:

- Follow taped lines
- Navigate sections without lines
- Detect and react to physical obstacles
- Operate fully autonomously

A key design decision was to implement the entire system using **cooperative multitasking**. Each subsystem (line following, state estimation, bump sensing, mapping, and velocity control) is implemented as an independent task with shared variables. This keeps the system modular, debuggable, and easy to tune.

---

## Installation Instructions

All Python files in the `compile` folder are required to run the Romi system and should be copied directly onto the Nucleo board via USB. The `main.py` file serves as the entry point and manages all tasks and shared variables.

A terminal emulator such as **PuTTY** was used to interact with the robot. Use the following settings:

- Baud rate: **115200**
- `Ctrl + C`: Interrupt execution
- `Ctrl + D`: Reboot the system after code changes

---

## Map Course Description

### Course Layout

The course shown below is the track the Romi was required to navigate. Black dots represent checkpoints. Red cups placed inside dotted circles incur a time penalty if displaced.

![Map Course](media/Game_Track-1.png)

### Romi Robot

The Romi robot used in this project is shown below.

![Romi Robot](https://github.com/user-attachments/assets/ed25e055-c15b-46d2-824f-2e3d41bdcd17)

---

## Hardware Architecture Summary

The system is built around the **Nucleo L476RG**, which interfaces with:

- Integrated Romi motors and encoders
- 5-channel IR line sensor
- BNO055 IMU
- Bump switches
- Bluetooth module

Sensors are mounted to maximize visibility of the track and obstacles, with short wiring runs for reliability and ease of servicing.

---

## Component List

| Quantity | Component |
|---------:|----------|
| 4 | M2.5 × 8 mm Standoff |
| 4 | M2.5 × 10 mm Standoff |
| 4 | M2.5 × 30 mm Standoff |
| 4 | M2.5 × 6 mm Socket Head Cap Screw |
| 4 | M2.5 × 8 mm Socket Head Cap Screw |
| 4 | M2.5 × 10 mm Socket Head Cap Screw |
| 8 | M2.5 Nylon Lock Nuts |
| 8 | M2.5 Nylon Washers |
| 1 | Acrylic Romi-to-Shoe Adapter |
| 1 | BNO055 IMU Breakout Board |
| 2 | Nucleo L476RG |
| 1 | Romi Chassis (Motors, Encoders, Wheels, Casters) |
| 1 | 5-Channel IR Sensor |
| 2 | Bump Sensors |
| 1 | Bluetooth Module |

---

## Critical Parameters

| Parameter | Value |
|----------|-------|
| Chassis Diameter | 163 mm |
| Track Width | 141 mm |
| Wheel Radius | 35 mm |
| Gear Ratio | 119 25/33 ≈ 119.76 |
| Encoder Resolution | ≈ 1437 PPR |
| Motor Voltage | 4.5 V |
| Stall Torque | 177 N·mm @ 4.5 V / 283 N·mm @ 7.2 V |
| No-Load Speed | 150 RPM @ 4.5 V / 240 RPM @ 7.2 V |
| Max Speed | 550 mm/s @ 4.5 V / 880 mm/s @ 7.2 V |

---

## System Architecture

The Romi runs multiple cooperative tasks using `cotask.py` and `task_share.py`. Tasks include:

- PID velocity control
- Encoder processing
- Line sensing
- Line following
- IMU updates
- State estimation
- Map navigation

Each task runs as a generator function with a defined priority and execution period.

---

## Software Architecture Summary

Low-level tasks manage encoders, IMU updates, and wheel velocity control. Higher-level tasks handle navigation logic and state transitions. All data exchange occurs through shared variables to maintain deterministic timing.

State estimation uses a reduced-order Romi model integrated with an RK4 solver to estimate distance, heading, and wheel speeds. These estimates enable accurate straight-line and turning motions even in sections without line markings.

---

## Calibration and Tuning

Before each run:

1. Line sensors are calibrated on white and black surfaces
2. Encoder and IMU outputs are verified
3. Velocity PID gains and observer parameters are tuned using short test motions

This workflow ensures repeatability across runs and after hardware changes.

---

## Task Diagram

![Task Diagram](media/Task_Diagram.png)

---

## Results

A sample line-following run is shown in the video below:

[Line Following Demonstration](https://github.com/user-attachments/assets/12b63d06-b12a-43de-abc6-85b22fbb766f)

---

## Full Course Run

A typical run consists of booting the system, performing sensor calibration, and entering map-navigation mode. The robot then autonomously completes the course and stops at the finish point. Telemetry data can be streamed over serial for debugging and performance analysis.

---

## Troubleshooting

Early issues included line-following overshoot and distance errors during state-estimation segments. These were resolved through PID tuning and overshoot correction factors.

---

## Takeaways

This project emphasized the importance of task-based software structure, careful calibration, and systematic debugging. Cooperative multitasking proved to be an effective approach for managing complex autonomous behavior on a resource-constrained embedded platform.
}
