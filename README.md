# INF2004-Autonomous-Robot-Car
Embedded Systems Project — autonomous robotic car with line following, barcode decoding, motion control, terrain monitoring, obstacle avoidance, and WiFi telemetry.

# Autonomous Robotic Car Challenge

Embedded Systems Team Project

## Project Overview

The goal of this project is to design and develop an autonomous robotic
car capable of sensing its environment, making navigation decisions,
moving accurately, avoiding obstacles, and reporting telemetry.

The system consists of five specialised subsystems that are integrated
through a central Vehicle Controller.

## Main Objectives

The autonomous vehicle should be able to:

1. Follow a track reliably
2. Detect and decode navigation barcodes
3. Detect and measure humps
4. Detect and avoid obstacles
5. Publish system status and telemetry

------

## System Architecture

All subsystems communicate with the Vehicle Controller.

The Vehicle Controller is responsible for:

- Mission state management
- Coordination between subsystems
- Navigation decisions
- Sending commands to subsystem modules
- Receiving sensor and event information
