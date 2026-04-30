# 11348469_Individual_Project
Code Artifact Appendices 
# MPPT Buck Converter & BLE Telemetry System
**Student ID:** 11348469

This repository contains the software artefacts for my Final Year Project: a smart, MPPT-enabled buck converter with a real-time web dashboard.

## Repository Structure
As per the submission requirements, the code is divided into two main components:

* `/Firmware_STM32/`: Contains the C++ source code executed on the STM32F303RE Nucleo board. 
  * `main.cpp`: The core Finite State Machine, PI control loops, and Variable P&O algorithm.
* `/SPA/`: Contains the front-end Single Page Application (SPA).
  * `Web BLE Dashboard.html`: The user interface and BLE connection logic.

## Third-Party Code and Libraries
In compliance with academic integrity requirements, the following third-party libraries and frameworks were utilised:
* **Mbed OS:** Used as the core RTOS for the STM32 firmware.
* **Chart.js:** Utilised in the `/Web_Dashboard/` to render the live time-series graphs for voltage and current.
